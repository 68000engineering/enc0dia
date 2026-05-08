Scratch-like visual domain specific programming language/API with browser-based IDE to learn basics of coding and robotics with main purpose of giving basic understanding of variables, functions, algorithms and the programming itself for the beginners, like children and teenagers. Connection between PC and ESP32 is established via Wi-Fi.

Current functionality has: Digital and Analog pin output/input; work (declaring and modifying) with basic types of variables such as: strings, integers and floats; math operators; loops such as: if/else, for; logic operator while; ability to work with I2C 16x2 LCD, 7-digit display, servos, and ultrasonic sensor.

New functionality and external libraries can be easily added.

Important: the pins for the connected I2C-bus sensors can be easily modified already in IDE itself, thanks to the architecture of ESP32.

Setting up:
1. Upload firmware <main.cpp> on the ESP32 using PlatformIO extension for Visual Studio Code
2. Connect your PC to ESP32, SSID: ESP32 | Password: 12345678
3. Open <index.html> from <desktop>, which is inside the repository
4. Enjoy!

p.s special thanks for @nildanil for making a scratch for the browser IDE
