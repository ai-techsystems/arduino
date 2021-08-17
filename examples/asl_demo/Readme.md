asl_demo_arduino_example.ino contains the application for American Sign Language on Arduino Nano 33 BLE Sense.

How to install visual_wake_word library for Arduino IDE?
 - Find the path for your arduino IDE installation. For me, it was ~/arduino
 - mkdir -p ~/arduino/libraries/asl_imu/src/cortex-m4
 - cp ndarray.h visual_wake_word.h ~/arduino/libraries/asl_imu/src
 - cp lib_asl_imu.a ~/arduino/libraries/asl_imu/src/cortex-m4/asl_imu.a
 - cp library.properties ~/arduino/libraries/asl_imu
