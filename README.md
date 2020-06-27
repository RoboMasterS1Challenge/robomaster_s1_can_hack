# DJI RoboMaster S1 ROS Bridge Package
****This repository will migrate to the following repository soon!!! ****
https://github.com/RoboMasterS1Challenge/robomaster_s1_ros_reference_design

This package contains a ROS node package and MINI-M4 STM32 firmware for controlling your RoboMaster S1 through the ROS network.
(Nucleo-F767ZI Version is discontined.)

## What is this?
You can control DJI RoboMaster S1 motion, LED, Blaster and some commands using ROS messages through ROS1 Network.

## Preparation
You should prepare these items.
- DJI RoboMaster S1
- Ubuntu PC with STM32CubeIDE
- MINI-M4 STM32
- Raspberry Pi 4B
- Universal board for Raspberry Pi
- CAN Tranceiver breakout board (Like this https://www.amazon.co.jp/dp/B076HHVZM1) 
- Wire harnes, 2.54mm Socket and Pins etc.
- DockerPi powerBoard (https://www.seeedstudio.com/DockerPi-PowerBoard-p-4100.html)
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


