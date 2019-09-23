#!/bin/bash

SUBNET=192.168.0.0
NETMASK=255.255.255.0
AP_ADDR=192.168.0.1
WIFI_IF=wlp61s0


start() {
    service network-manager stop
    sleep 3
    ifconfig ${WIFI_IF} ${AP_ADDR} netmask ${NETMASK} up
    sleep 3
    service hostapd start
    if [ $? != 0 ]; then
        echo "failed to start hostapd."
        return 1
    fi

    echo "start access-point."
    return 0
}

stop() {
    service hostapd stop
    sleep 3    
    ifconfig ${WIFI_IF} down
    sleep 3
    service network-manager start

    echo "stop access-point."
    return 0
}

case $1 in
"start" )
    start
    ;;

"stop" )
    stop
    ;;

*)
    echo "usage: $0 start|stop"
    exit 1
esac
