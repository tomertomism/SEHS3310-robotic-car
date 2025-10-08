### Licensed by Apache license 2.0
# SEHS3310-robotic-car

* Demostration Video

[![Watch the video](https://img.youtube.com/vi/OXl6jhx9GE4/0.jpg)](https://youtube.com/shorts/OXl6jhx9GE4?feature=share)

* Front PCB

<img width="512" height="512" alt="0" src="https://github.com/tomertomism/SEHS3310-robotic-car/blob/main/1.png" />

* Back PCB

<img width="512" height="512" alt="1" src="https://github.com/tomertomism/SEHS3310-robotic-car/blob/main/2.png" />

## Please have a look if you are taking SEHS3310 as well.
* First, Yes, my code can do the task smoothly. Try It.
* But PLEASE! Write your own codes, this program should not be your final product, Those programmes are just for reference only. **Directly copy or modify without acknowledgement may result in plagiarism, aka ["academic dishonesty"](https://www.cpce-polyu.edu.hk/academic-registry/academic-integrity---student-conduct/academic-integrity).**
* Tasks are not difficult, try to solve it in a simple way.
* This is my friendly reminder for you, good luck!

* The only two files you may able to direct copy: [my_vl53l0x.c](https://github.com/tomertomism/SEHS3310-robotic-car/blob/main/ARM-Code-git/my_vl53l0x.c) and [my_vl53l0x.h](https://github.com/tomertomism/SEHS3310-robotic-car/blob/main/ARM-Code-git/my_vl53l0x.h).

## Here's Cantonese underneath
* 咪抄code，ching。阿sir有晒我code。我唔想你fail嘅，你淨抄coding style都夠你fail。自己做，真係唔難，試下唔好將tasks複雜化。Step by Step做。

## Introduction
+ I am not using Arduino Uno/Mega in this project. Thoughout the section, I am using ARM MCU.
* Programme in the OpenMV folder is only applicable for OV5640 sensor.
* Finish time: 41s
* Finish date: 20 July 2024
* Project update date: 21 July 2024
* Bugs exist in some circumstances

## How to download the programmes to MCU
* Keil uVision
* STM32F103Rx
* ST-Link/JTAG/USB to TTL

## How the car runs
* everytime plugged in the power cord, wait 3 - 5 seconds for initialisation. 
* in initialisation state, MCU will initial all component and reset the camera, the blue button on the evaluation board will be no response.
* after initialise, press the blue button, car will go.
