# deepC for arduino

deepC is an open source deep learning framework for on-device inference on embedded devices. This repository is adapted for arduino devices. It enables low-latency inference of on-device machine learning models with a small binary size with low memory and high performance.

# KPNS
### 1. add -fexceptions, and remove -fno-exceptions
to the compiler flags to the platform.txt of your Arduino IDE installation files given below:
  - ${ARDUINO_INSTALLATION}/.arduino15/packages/arduino/hardware/mbed/1.1.4/variants/ARDUINO_NANO33BLE/cxxflags.txt
  - ${ARDUINO_INSTALLATION}/arduino-1.8.12/hardware/arduino/avr/platform.txt
