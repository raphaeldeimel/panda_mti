#!/bin/sh

dpkg -s net-tools > /dev/null 2>&1

if [ -z "$1" ];then
    NAME=panda
else
    NAME=$1
fi

if [ $? -eq 0 ]; then
    ping -c 1 -l 1 -n -q $NAME >/dev/null #ensure ARP cache is populated
    arp -a $NAME  | awk '{print $4}' | sed s/://g | tr -d '\n'
else
    echo "dpkg-error"
fi

