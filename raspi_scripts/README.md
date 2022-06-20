## Auto start ros + rosserial

A simple watchdog is used to restart rosserial if it dies because the stm32 is rebooted or reprogrammed

### Set IP/Hostname in noetic setup.bash

Edit /opt/ros/noetic/setup.bash and add the local ip/hostname (in case u need external ros comms) and add the following 2 ROS_XXX lines adjusted for your local network settings. This file is sourced by all workspaces and by the 3 sys scripts.

```
#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

export ROS_IP=10.146.111.222
export ROS_HOSTNAME=mowgli

<rest of file>
```

### Install UDEV rules

UDEV rule for /dev/gps (ublox) and /dev/mowgli

```
sudo cp 50-mowgli.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger
```

### Install roscore and rosserial services
```
sudo cp roscore.service /etc/systemd/system
sudo cp rosserial.service /etc/systemd/system 
sudo cp rosserial_watchdog.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl start roscore
sudo systemctl start rosserial
sudo systemctl start rosserial_watchdog
```
