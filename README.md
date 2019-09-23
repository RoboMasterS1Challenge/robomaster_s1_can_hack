# DJI RoboMaster S1 ROS Bridge Package (under developping)

This is a ROS node package and Nucleo-F767ZI Firmware for controlling your RoboMaster S1 through ROS network.

## Prepare
You should prepare these items.
- DJI RoboMaster S1
- Ubuntu PC with WiFi (16.04 LTS and STM32CubeIDE)
- Nucleo-F767Zi
- Universal board (like this http://akizukidenshi.com/catalog/g/gP-09972/)
- CAN Tranceiver breakout board (like this https://www.amazon.co.jp/dp/B076HHVZM1) 
- Wire harnes
- 2.54mm Connector and Pin etc. (like this https://www.amazon.co.jp/gp/product/B071RK5STD/)

If you want to control via Wifi, please prepare like these items.
- Ethernet-Wifi Converter (like this WLI-UTX-AG300/C)
- Mobile Battery for Wifi Converter.

## How to install

Install ds4drv for PlayStation4(R) Controller

$ sudo pip install ds4drv

Install Ubuntu WiFi Access Point

$sudo apt install hostapd

$sudo vi /etc/hostapd/hostapd.conf
`
interface=wlp61s0 # your WiFi device

driver=nl80211

ssid=RoboMasterS1Host

hw_mode=g

channel=7

wpa=2 # WPA2

wpa_passphrase=robomasters1 # your password

wpa_key_mgmt=WPA-PSK

rsn_pairwise=CCMP
`
$ git clone https://github.com/tatsuyai713/robomaster_s1_can_hack 

Please make symbolic link from "ros" folder to your ROS workspace.

## How to make controller board
under construnction...
