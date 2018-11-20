#!/bin/sh

dpkg -s net-tools > /dev/null 2>&1

if [ $? -eq 0 ]; then
    arp -a $1  | awk '{print $4}' | sed s/://g | tr -d '\n'
else
    echo "dpkg-error"
fi
