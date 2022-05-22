#!/bin/bash
SER2NET_IP=10.146.111.124
SER2NET_PORT=8888
DEV=$HOME/ttyV0

echo "Device $DEV created"
sudo socat pty,link=$DEV,echo=0,raw,mode=0777 tcp:$SER2NET_IP:$SER2NET_PORT 
