# DJI RoboMaster S1 ROS Bridge Package

This package contains a ROS node package and Nucleo-F767ZI firmware for controlling your RoboMaster S1 through the ROS network.

## What is this?
You can control DJI RoboMaster S1 using geometry_msgs::Twist through the ROS Network.

## Preparation
You should prepare these items.
- DJI RoboMaster S1
- Ubuntu PC with WiFi (16.04 LTS and STM32CubeIDE)
- Nucleo-F767ZI
- Universal board (http://akizukidenshi.com/catalog/g/gP-09972/)
- CAN Tranceiver breakout board (https://www.amazon.co.jp/dp/B076HHVZM1) 
- Wire harnes
- 2.54mm Connector and Pin etc. (https://www.amazon.co.jp/gp/product/B071RK5STD/)

If you want to control via Wifi, please prepare these items.
- Ethernet-Wifi Converter (WLI-UTX-AG300/C)
- DCDC Converter (http://akizukidenshi.com/catalog/g/gK-09981/) or Mobile Battery for Wifi Converter

## How to install

Install ds4drv for PlayStation4(R) Controller

$ sudo pip install ds4drv

Install Ubuntu WiFi Access Point

$sudo apt install hostapd

$sudo vi /etc/hostapd/hostapd.conf

`interface=wlp61s0`

`driver=nl80211`

`ssid=RoboMasterS1Host`

`hw_mode=g`

`channel=7`

`wpa=2`

`wpa_passphrase=robomasters1`

`wpa_key_mgmt=WPA-PSK`

`rsn_pairwise=CCMP`

$ git clone --recursive https://github.com/tatsuyai713/robomaster_s1_can_hack.git

Please make symbolic link from "ros" folder to your ROS workspace.

## How to make controller board
under construction...
