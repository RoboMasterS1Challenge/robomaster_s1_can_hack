# DJI RoboMaster S1 ROS Bridge Package

This package contains a ROS node package and MINI-M4 STM32 firmware for controlling your RoboMaster S1 through the ROS network.

## What is this?
You can control DJI RoboMaster S1 motion, LED, Blaster and buttle end command using ROS messages through the Network.

![robomasters1](https://user-images.githubusercontent.com/34103899/65610530-75044200-dfec-11e9-9dfc-d46c963ab85a.JPG)
It's old version.

## Preparation
You should prepare these items.
- DJI RoboMaster S1
- Ubuntu PC for development (16.04 or 18.04 LTS and STM32CubeIDE)
- MINI-M4 STM32
- Raspberry Pi (4 is recommended)
- Universal board for Raspberry Pi 4
- CAN Tranceiver breakout board (https://www.amazon.co.jp/dp/B076HHVZM1) 
- Wire harnes
- 2.54mm Socket and Pin etc.
- SONY PlayStation4(R) Controller

### PS4 Controller Key Map
 - Left Axis : Linear X and Linear Y
 - Right Axis : Gimbal Pitch and Gimbal Yaw
 - Right Axis push : Blaster
 - L2 : Enable control

## How to install
Install ds4drv for PlayStation4(R) Controller

$ sudo pip install ds4drv

$ git clone --recursive https://github.com/tatsuyai713/robomaster_s1_can_hack.git

Please make symbolic link from "ros" folder to your ROS workspace.

## How to make controller board
under construction...


