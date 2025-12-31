#define NO_GLOBAL_SERIAL

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <LiquidCrystal_I2C.h>
#include <TM1637Display.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);   // по умолчанию
bool lcdInitialized = false;

TM1637Display* tm1637 = nullptr;
bool tm1637Initialized = false;

Servo servo;
bool servoAttached = false;
int servoPin = -1;

String logBuffer = "";

int usTrigPin = -1;
int usEchoPin = -1;
unsigned long usNextRead = 0;
float usLastDistance = 0.0;
bool usEnabled = false;

const char* ssid     = "esp32";
const char* password = "12345678";

WebServer server(80);

const int LED_PIN = 2;  // встроенный LED для большинства ESP32

// ---------- Глобальное хранилище программы ----------

DynamicJsonDocument programDoc(65536);

// Cache for function definitions between runs
DynamicJsonDocument functionsCache(32768);

JsonObject functionsObj;

// Кадр выполнения: массив шагов + текущий индекс + оставшиеся повторы
struct ExecFrame {
  JsonArray steps;
  size_t index;
  int repeatRemaining;  // 1 для обычного блока, >1 для repeat
};

std::vector<ExecFrame> execStack;
unsigned long nextActionTime = 0;
bool programRunning = false;

// Variables storage

std::map<String, float> variables;
std::map<String, String> stringVars;

// Forward declarations
void addLog(String s);

// Manage function definitions from incoming payloads and runtime definitions.
void refreshFunctionCache(JsonObject source) {
  if (!source.isNull() && source.size() > 0) {
    for (JsonPair kv : source) {
      functionsCache[kv.key()] = kv.value();
    }
  }
}

void ensureFunctionsObject() {
  JsonObject merged = programDoc["functions"].to<JsonObject>();
  if (merged.isNull()) {
    merged = programDoc.createNestedObject("functions");
  }

  // If we already cached definitions from a previous run, merge them back so
  // callFunc can resolve even when the current payload omitted the functions block

  if (functionsCache.size() > 0) {
    merged.set(functionsCache.as<JsonObject>());
  }

  functionsObj = merged;
}

void logFunctionDefinitions(const char* prefix) {
  if (functionsObj.isNull() || functionsObj.size() == 0) {
    return;
  }

  for (JsonPair kv : functionsObj) {
    const char* name = kv.key().c_str();
    JsonArray steps = kv.value().as<JsonArray>();
    size_t stepCount = steps.isNull() ? 0 : steps.size();
    addLog(String(prefix) + name + " (" + String(stepCount) + " steps)");
  }
}

// ---------- Вспомогательные функции ----------

float resolveValue(JsonVariant v);        
float resolveExpression(JsonObject expr); 

// Вычисление выражения вида { expr:true, left:..., op:"+", right:... }
float resolveExpression(JsonObject expr) {
  JsonVariant leftV = expr["left"];
  JsonVariant rightV = expr["right"];
  const char* op = expr["op"] | "+";

  float left = resolveValue(leftV);
  float right = resolveValue(rightV);

  if (strcmp(op, "+") == 0) return left + right;
  if (strcmp(op, "-") == 0) return left - right;
  if (strcmp(op, "*") == 0) return left * right;
  if (strcmp(op, "/") == 0) {
    if (right != 0.0f) return left / right;
    return 0.0f;
  }
  return 0.0f;
}

// Универсальное получение числа из JsonVariant:
// - число -> float
// - строка -> имя переменной -> значение
// - объект с {expr:true,...} -> вычислить выражение
float resolveValue(JsonVariant v) {
  if (v.is<float>()) {
    return v.as<float>();
  }
  if (v.is<int>()) {
    return (float)v.as<int>();
  }

  // выражение?
  if (v.is<JsonObject>()) {
    JsonObject obj = v.as<JsonObject>();
    if (obj.containsKey("expr")) {
      return resolveExpression(obj);
    }
  }

  // строка = имя переменной
  if (v.is<const char*>()) {
    String name = v.as<const char*>();
    auto it = variables.find(name);
    if (it != variables.end()) {
      return it->second;
    }
    return 0.0f;
  }

  return 0.0f;
}

void addLog(String s) {
    logBuffer += s + "\n";

    const int MAX_LOG = 4096;
    if (logBuffer.length() > MAX_LOG) {
        int cutPos = logBuffer.length() - MAX_LOG;
        logBuffer = logBuffer.substring(cutPos);  
    }
}

void updateUltrasonic() {
    if (!usEnabled) return;

    unsigned long now = millis();
    if (now < usNextRead) return;  // ждём 500 мс

    usNextRead = now + 500;  // следующее измерение через 500мс

    // TRIGGER impulse
    digitalWrite(usTrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(usTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(usTrigPin, LOW);

    // Measure echo
    long duration = pulseIn(usEchoPin, HIGH, 25000); // timeout 25ms

    if (duration == 0) {
        usLastDistance = -1;  // нет сигнала
    } else {
        usLastDistance = duration / 58.0;  // в сантиметрах
    }

    variables["distance"] = usLastDistance;  // обновляем глобальную переменную
}
// Выполнить один "атомарный" шаг (без раскрытия repeat/if — они создают новые фреймы)
void executeStep(JsonObject step) {
  const char* action = step["action"] | "";
  unsigned long now = millis();
  nextActionTime = now;  // по умолчанию следующую команду можно сразу

  if (strcmp(action, "ledOn") == 0) {
    digitalWrite(LED_PIN, HIGH);
    addLog("[STEP] ledOn");
  }

  else if (strcmp(action, "lcdPrintVar") == 0) {
      const char* name = step["name"] | "";

      if (!lcdInitialized) {
          addLog("[LCD] ERROR: lcdPrintVar before lcdSetup");
          return;
      }

      String s = stringVars[name];   // из stringVars

      if (s.length() > 16) s = s.substring(0, 16);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(s);

      addLog("[LCD PRINT VAR] " + String(name) + "=\"" + s + "\"");
  }

  else if (strcmp(action, "setVarString") == 0) {
      const char* name = step["name"] | "";
      const char* txt  = step["text"] | "";

      stringVars[name] = String(txt);

      addLog("[STEP] setVarString " + String(name) + " = \"" + String(txt) + "\"");
  }


  else if (strcmp(action, "servoSetup") == 0) {
      int pin = step["pin"] | 2;

      servo.attach(pin);
      servoAttached = true;
      servoPin = pin;

      addLog("[SERVO] setup on pin " + String(pin));
  }

  else if (strcmp(action, "servoWrite") == 0) {
      if (!servoAttached) {
          addLog("[SERVO] ERROR: servo not attached");
          return;
      }

      int angle = step["angle"] | 90;
      angle = constrain(angle, 0, 180);

      servo.write(angle);

      addLog("[SERVO] angle " + String(angle));
  }


  else if (strcmp(action, "tm1637Setup") == 0) {
      int clk = step["clk"] | 18;       // пины по умолчанию, подправь под свою схему
      int dio = step["dio"] | 19;
      int brightness = step["brightness"] | 7; // 0..7

      if (tm1637) {
          delete tm1637;
          tm1637 = nullptr;
      }

      tm1637 = new TM1637Display(clk, dio);
      tm1637->setBrightness(brightness, true);
      tm1637Initialized = true;

      addLog("[TM1637] setup CLK=" + String(clk) +
            " DIO=" + String(dio) +
            " BR=" + String(brightness));
  }

  else if (strcmp(action, "tm1637ShowNumber") == 0) {
      if (!tm1637Initialized || !tm1637) {
          addLog("[TM1637] ERROR: showNumber before setup");
          return;
      }

      float v = resolveValue(step["value"]);
      int iv = (int)v;

      // показываем число (4 разряда, с ведущими нулями)
      tm1637->showNumberDec(iv, false);

      addLog("[TM1637] showNumber " + String(iv));
  }

  else if (strcmp(action, "tm1637Clear") == 0) {
      if (tm1637Initialized && tm1637) {
          tm1637->clear();
          addLog("[TM1637] clear");
      } else {
          addLog("[TM1637] clear called, but not initialized");
      }
  }


  else if (strcmp(action, "ifButton") == 0) {
    int pin = step["pin"] | 0;
    pinMode(pin, INPUT);

    int val = digitalRead(pin);  // 1 = нажата (если кнопка подтянута к VCC)
    bool pressed = (val == HIGH);

    addLog("[BUTTON] pin " + String(pin) + " = " + String(val) + " -> " + (pressed ? "PRESSED" : "NOT PRESSED"));

    JsonArray chosen = pressed
      ? step["then"].as<JsonArray>()
      : step["else"].as<JsonArray>();

    if (!chosen.isNull()) {
      ExecFrame frame;
      frame.steps = chosen;
      frame.index = 0;
      frame.repeatRemaining = 1;
      execStack.push_back(frame);
    }
}

else if (strcmp(action, "analogWrite") == 0) {
    int pin = step["pin"] | 0;
    float v = resolveValue(step["value"]);
    int duty = constrain((int)v, 0, 255);

    int channel = pin % 16;  // простой канал = номер пина
    ledcAttachPin(pin, channel);
    ledcSetup(channel, 5000, 8);  // 5kHz, 8 бит (0–255)
    ledcWrite(channel, duty);

    addLog("[STEP] analogWrite pin " + String(pin) + " = " + String(duty));
}

else if (strcmp(action, "lcdInit") == 0) {
    int addr = step["addr"] | 0x27;
    lcd = LiquidCrystal_I2C(addr, 16, 2);
    lcd.init();
    lcd.backlight();
    lcdInitialized = true;
    addLog("[LCD] init addr=0x" + String(addr, HEX));
}

else if (strcmp(action, "lcdClear") == 0) {
    if (lcdInitialized) lcd.clear();
    addLog("[LCD] clear");
}

else if (strcmp(action, "lcdPrint") == 0) {
    if (!lcdInitialized) {
        addLog("[LCD] ERROR: lcdPrint before lcdSetup");
        return;
    }

    const char* txt = step["text"] | "";
    String s = txt;

    //Ошибка, если слишком длинно
    if (s.length() > 32) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("TEXT TOO LONG!!!");
        addLog("[LCD PRINT] ERROR: text > 32 chars");
        return;
    }

    //Готовим строки
    String line1 = s.substring(0, std::min(16U, s.length()));
    String line2 = (s.length() > 16) ? s.substring(16) : "";

    lcd.clear();

    //Первая_строка
    lcd.setCursor(0, 0);
    lcd.print(line1);

    //Вторая строка (если есть)
    if (line2.length() > 0) {
        lcd.setCursor(0, 1);
        lcd.print(line2);
    }

    addLog("[LCD PRINT] '" + s + "'");
}


else if (strcmp(action, "lcdSetup") == 0) {
    int sda = step["sda"] | 21;
    int scl = step["scl"] | 22;

    addLog("[LCD] setup SDA=" + String(sda) + " SCL=" + String(scl) + " addr=0x27");

    Wire.begin(sda, scl);

    lcd = LiquidCrystal_I2C(0x27, 16, 2);
    lcd.init();
    lcd.backlight();

    lcdInitialized = true;
}


else if (strcmp(action, "analogRead") == 0) {
    int pin = step["pin"] | 0;
    const char* var = step["var"] | "";

    pinMode(pin, INPUT);
    int raw = analogRead(pin);   // 0–4095 на ESP32

    float value = (float)raw;

    variables[var] = value;

    addLog("[STEP] analogRead pin " + String(pin) + " = " + String(raw) + " -> var " + String(var) + " (" + String(value, 2) + ")");
}

  else if (strcmp(action, "digitalWrite") == 0) {
      int pin = step["pin"] | 0;
      float v = resolveValue(step["value"]);
      int value = (v != 0.0f) ? HIGH : LOW;

      pinMode(pin, OUTPUT);
      digitalWrite(pin, value);

      addLog("[STEP] digitalWrite pin " + String(pin) + " = " + String(value));
  }

  else if (strcmp(action, "digitalRead") == 0) {
      int pin = step["pin"] | 0;
      const char* var = step["var"] | "";

      pinMode(pin, INPUT);
      int result = digitalRead(pin);

      variables[var] = (float)result;   // как float (0.0 или 1.0)

      addLog("[STEP] digitalRead pin " + String(pin) + " = " + String(result) + " -> var " + String(var));
  }

  else if (strcmp(action, "ledOff") == 0) {
    digitalWrite(LED_PIN, LOW);
    addLog("[STEP] ledOff");
  }

  else if (strcmp(action, "delay") == 0) {
    float msf = resolveValue(step["ms"]);
    if (msf < 0.0f) msf = 0.0f;
    unsigned long ms = (unsigned long)msf;
    nextActionTime = now + ms;
    addLog("[STEP] delay " + String(msf, 2) + " ms");
  }

  else if (strcmp(action, "setVar") == 0) {
    const char* name = step["name"] | "";
    float value = resolveValue(step["value"]);
    variables[name] = value;
    addLog("[STEP] setVar " + String(name) + " = " + String(value, 3));
  }

  else if (strcmp(action, "changeVar") == 0) {
    const char* name = step["name"] | "";
    float delta = resolveValue(step["delta"]);
    float before = variables[name];
    variables[name] = before + delta;
    addLog("[STEP] changeVar " + String(name) + ": " + String(before, 3) + " -> " + String(variables[name], 3));
  }

  else if (strcmp(action, "if") == 0) {
    JsonVariant leftV  = step["cond_left"];
    JsonVariant rightV = step["cond_right"];
    const char* op = step["cond_op"] | "==";

    float left  = resolveValue(leftV);
    float right = resolveValue(rightV);

    bool cond = false;
    if (strcmp(op, "==") == 0) cond = (left == right);
    else if (strcmp(op, "!=") == 0) cond = (left != right);
    else if (strcmp(op, ">")  == 0) cond = (left >  right);
    else if (strcmp(op, "<")  == 0) cond = (left <  right);
    else if (strcmp(op, ">=") == 0) cond = (left >= right);
    else if (strcmp(op, "<=") == 0) cond = (left <= right);

    addLog("[STEP] if (" + String(left, 3) + " " + String(op) + " " + String(right, 3) + ") => " + (cond ? "TRUE" : "FALSE"));

    JsonArray chosen = cond
      ? step["then"].as<JsonArray>()
      : step["else"].as<JsonArray>();

    if (!chosen.isNull()) {
      ExecFrame frame;
      frame.steps = chosen;
      frame.index = 0;
      frame.repeatRemaining = 1;
      execStack.push_back(frame);
    }
  }

  else if (strcmp(action, "whileCond") == 0) {

    JsonVariant leftV  = step["cond_left"];
    JsonVariant rightV = step["cond_right"];
    const char* op = step["cond_op"] | "==";

    float left  = resolveValue(leftV);
    float right = resolveValue(rightV);

    bool cond = false;
    if (strcmp(op, "==") == 0) cond = (left == right);
    else if (strcmp(op, "!=") == 0) cond = (left != right);
    else if (strcmp(op, ">")  == 0) cond = (left >  right);
    else if (strcmp(op, "<")  == 0) cond = (left <  right);
    else if (strcmp(op, ">=") == 0) cond = (left >= right);
    else if (strcmp(op, "<=") == 0) cond = (left <= right);

    addLog("[WHILE] condition (" + String(left,3) + " " + String(op) + " " + String(right,3) + ") => " + (cond ? "TRUE" : "FALSE"));

    if (cond) {
        ExecFrame frame;
        frame.steps = step["steps"].as<JsonArray>();
        frame.index = 0;
        frame.repeatRemaining = -2; // специальный код WHILE
        execStack.push_back(frame);
    }
  }


  else if (strcmp(action, "ultrasonicSetup") == 0) {
    usTrigPin = step["trig"] | -1;
    usEchoPin = step["echo"] | -1;

    pinMode(usTrigPin, OUTPUT);
    pinMode(usEchoPin, INPUT);

    usEnabled = true;
    usNextRead = 0;

    variables["distance"] = 0.0;

    addLog("[HC-SR04] started TRIG=" + String(usTrigPin) +
           " ECHO=" + String(usEchoPin));
  }

  else if (strcmp(action, "repeat") == 0) {
    int times = step["times"] | 1;
    if (times < 1) times = 1;
    JsonArray inner = step["steps"].as<JsonArray>();
    if (inner.isNull()) {
      addLog("[STEP] repeat: empty steps");
      return;
    }

    addLog("[STEP] repeat x" + String(times));

    ExecFrame frame;
    frame.steps = inner;
    frame.index = 0;
    frame.repeatRemaining = times;
    execStack.push_back(frame);
  }

  else if (strcmp(action, "callFunc") == 0) {
      const char* name = step["name"] | "";

      JsonArray funcSteps = functionsObj[name].as<JsonArray>();

      if (funcSteps.isNull()) {
          addLog(String("[FUNC] ERROR: function '") + name + "' not found or has no body");
          return;
      }

      if (funcSteps.size() == 0) {
          addLog(String("[FUNC] ERROR: function '") + name + "' is empty");
          return;
      }

      addLog(String("[FUNC] call ") + name);

      ExecFrame frame;
      frame.steps = funcSteps;
      frame.index = 0;
      frame.repeatRemaining = 1;

      execStack.push_back(frame);
    }

    else if (strcmp(action, "defineFunc") == 0) {
      const char* name = step["name"] | "";
      JsonArray body = step["steps"].as<JsonArray>();

      if (strlen(name) == 0) {
          addLog("[FUNC] ERROR: defineFunc without name");
          return;
      }

      if (body.isNull()) {
          addLog(String("[FUNC] ERROR: defineFunc '") + name + "' has no steps");
          return;
      }

      ensureFunctionsObject();

      functionsObj[name] = body;
      JsonArray arr = functionsObj[name].to<JsonArray>();
      arr.clear(); 
      for (JsonVariant v : body) {
        arr.add(v);
      }

    // cache update 
    functionsCache[name] = arr;
    addLog(String("[FUNC] defined ") + name + " (" + String(body.size()) + " steps)");
    }

  else {
    addLog("[STEP] unknown action: ");
    addLog(String(action));
  }
}

// Обработка всей программы асинхронно
// Обработка всей программы асинхронно
void processProgram() {
  if (!programRunning) return;

  unsigned long now = millis();
  if (now < nextActionTime) return;

  // Выполняем максимум ОДНО атомарное действие за вызов
  while (!execStack.empty()) {

    ExecFrame &frame = execStack.back();

    // Дошли до конца блока
    if (frame.index >= frame.steps.size()) {

      // ===== WHILE цикл =====
      if (frame.repeatRemaining == -2) {

        // Родительская рамка (где лежит объект whileCond)
        ExecFrame &parent = execStack[execStack.size() - 2];
        JsonObject whileObj = parent.steps[parent.index - 1];

        JsonVariant leftV  = whileObj["cond_left"];
        JsonVariant rightV = whileObj["cond_right"];
        const char* op     = whileObj["cond_op"] | "==";

        float left  = resolveValue(leftV);
        float right = resolveValue(rightV);

        bool cond = false;
        if (strcmp(op, "==") == 0) cond = (left == right);
        else if (strcmp(op, "!=") == 0) cond = (left != right);
        else if (strcmp(op, ">")  == 0) cond = (left >  right);
        else if (strcmp(op, "<")  == 0) cond = (left <  right);
        else if (strcmp(op, ">=") == 0) cond = (left >= right);
        else if (strcmp(op, "<=") == 0) cond = (left <= right);

        addLog("[WHILE] re-eval => " + String(cond ? "TRUE" : "FALSE"));

        if (cond) {
            frame.index = 0;      // повторяем тело
            continue;
        } else {
            execStack.pop_back(); // выходим из цикла
            continue;
        }
      }

      // ===== Обычный repeat =====
      if (frame.repeatRemaining > 1) {
        frame.repeatRemaining--;
        frame.index = 0;
        continue;
      }

      // ===== Завершение блока =====
      execStack.pop_back();
      continue;
    }

    // Взять шаг и выполнить
    JsonObject step = frame.steps[frame.index].as<JsonObject>();
    frame.index++;

    if (!step.isNull()) {
      executeStep(step);
      return;  // выполняем только один шаг за цикл
    }
  }

  // Программа закончилась
  programRunning = false;
  addLog("[PROGRAM] finished");
}

// ---------- HTTP обработчики ----------

void handleOptionsRun() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

void handleRun() {
  logBuffer = "";   

  if (!server.hasArg("plain")) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(400, "text/plain", "No body");
    return;
  }

  String body = server.arg("plain");

  // Очищаем старое состояние
  programDoc.clear();
  execStack.clear();
  variables.clear();
  programRunning = false;

  DeserializationError err = deserializeJson(programDoc, body);
  if (err) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(400, "text/plain", "JSON error");
    return;
  }

  JsonVariant functionsVariant = programDoc["functions"];
  JsonObject incomingFunctions = functionsVariant.is<JsonObject>()
                                     ? functionsVariant.as<JsonObject>()
                                     : JsonObject();
  bool gotFunctionsFromPayload = !incomingFunctions.isNull() && incomingFunctions.size() > 0;

  // Reuse cached definitions when request omits the functions block.
  refreshFunctionCache(incomingFunctions);
  ensureFunctionsObject();

  JsonArray steps = programDoc["steps"].as<JsonArray>();
  bool hasDefineSteps = false;
  if (!steps.isNull()) {
    for (JsonVariant v : steps) {
      JsonObject step = v.as<JsonObject>();
      const char* act = step["action"] | "";
      if (strcmp(act, "defineFunc") == 0) {
        hasDefineSteps = true;
        break;
      }
    }
  }

  if (functionsObj.size() > 0) {
    if (gotFunctionsFromPayload) {
      addLog("[FUNC] Loaded " + String(functionsObj.size()) + " function(s) from payload");
    } else {
      addLog("[FUNC] Using cached " + String(functionsObj.size()) + " function(s)");
    }
    logFunctionDefinitions("[FUNC]   - ");
  } else if (hasDefineSteps) {
    addLog("[FUNC] No preloaded definitions; will register from defineFunc steps");
  } else {
    addLog("[FUNC] No function definitions provided or cached");
  }

  if (steps.isNull()) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(400, "text/plain", "No steps");
    return;
  }

  ExecFrame root;
  root.steps = steps;
  root.index = 0;
  root.repeatRemaining = 1;

  execStack.push_back(root);
  programRunning = true;
  nextActionTime = millis();

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Async OK");
}

// ---------- setup / loop ----------

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/run", HTTP_OPTIONS, handleOptionsRun);
  server.on("/run", HTTP_POST, handleRun);

  server.on("/log", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", logBuffer);
  });

  server.on("/functions", HTTP_GET, []() {
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();

    ensureFunctionsObject(); // убедиться, что функции подгружены

    for (JsonPair kv : functionsObj) {
      const char* name = kv.key().c_str();
      JsonArray steps = kv.value().as<JsonArray>();
      root[name] = steps;
    }

    String out;
    serializeJson(root, out);

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
  });


  server.on("/vars", HTTP_GET, []() {
      JsonDocument doc;
      JsonObject root = doc.to<JsonObject>();

      for (auto &p : variables) {
        root[p.first] = p.second;
      }

      for (auto &p : stringVars) {
        root[p.first] = p.second;
      }


      String out;
      serializeJson(root, out);

      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "application/json", out);
  });

  server.on("/stop", HTTP_POST, []() {
    programRunning = false;
    execStack.clear();
    nextActionTime = 0;

    addLog("[PROGRAM] STOPPED by user");

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "Stopped");
});

  server.on("/clearlog", HTTP_POST, []() {
    logBuffer = "";  
    addLog("[LOG] Cleared by user");
    
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "Log cleared");
});

  server.begin();
}

void loop() {
  server.handleClient();
  processProgram();
  updateUltrasonic();
}
