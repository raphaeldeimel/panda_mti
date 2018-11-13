#!/bin/sh
arp -a $1 | awk '{print $4}'
