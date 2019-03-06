#!/bin/sh

dpkg -s net-tools > /dev/null 2>&1

if [ -z "$1" ];then
    NAME=panda
else
    NAME=$1
fi

if [ $? -eq 0 ]; then
    ping -c 1 -l 1 -n -w 2 -q $NAME >/dev/null #populated arp cache
    MACDATA=$(arp -a panda)
    if [ "$(echo $MACDATA | awk '{print $1}')" = "arp:" ]; then
    	echo "No Panda found. Please connect panda to the network and start panda" >&2
    else
        PANDA_MAC="$(arp -a panda  | awk '{print $4}' | sed s/://g | tr -d '\n')"
        export PANDA_MAC #in case we got source'd
        echo "$PANDA_MAC"
    fi
else
    echo "Error while executing net-tools!
    
    Please make sure that net-tools is installed.
    try: sudo apt install net-tools"  >&2
fi
