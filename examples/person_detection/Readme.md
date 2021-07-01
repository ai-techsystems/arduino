person_detect_arducam.ino contains the application for person detection on Arduino Nano 33 with ArduCAM.

How to install visual_wake_word library for Arduino IDE?
 - Find the path for your arduino IDE installation. For me, it was ~/arduino
 - mkdir -p ~/arduino/libraries/visual_wake_word/src/cortex-m4
 - cp ndarray.h visual_wake_word.h ~/arduino/libraries/visual_wake_word/src
 - cp lib_visual_wake_word.a ~/arduino/libraries/visual_wake_word/src/cortex-m4/visual_wake_word.a
 - cp library.properties ~/arduino/libraries/visual_wake_word
