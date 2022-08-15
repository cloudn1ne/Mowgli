### Install ROS Noetic on Ubuntu 20.04 Server on a Raspi

## Required Items

* Raspi 4
* 32GB or better SD Card
* USB Keyboard
* Mini HDMI to XXX adapter and a monitor
* WLAN credentials and internet connectivity

## Install Ubuntu 20.04 Server

* install ubuntu 20.04 with imager -> https://www.pragmaticlinux.com/2021/08/install-the-raspberry-pi-imager-on-ubuntu-debian-fedora-and-opensuse/
* boot up and set pw for user ubuntu 
* configure network so you can reach your rapi remotely -> https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line
* connect to your raspi via ssh

## Stop auto updates (from interfering)

```
systemctl stop --force unattended-upgrades.service
```

## Remove snapd

```
sudo snap remove --purge lxd
sudo snap remove --purge core20
sudo snap remove --purge snapd
sudo apt -y remove --autoremove snapd
```

## Update Ubuntu to latest version

```
sudo apt-get update
sudo apt-get -y upgrade
```

## Install ROS noetic -> http://wiki.ros.org/noetic/Installation/Ubuntu

when you reach point 1.4 Installation chose the ROS-Base (Bare Bones) option


## Add some alias helpers to \~/.bashrc

```
alias depit="rosdep install --from-paths src --ignore-src -r -y"
alias debug="picocom -b 115200 /dev/ttyAMA1"
```

## Configure ROS Core IP (in /opt/ros/noetic/setup.bash)
\<your-rover-ip> is your static ip address for the raspi

```
export ROS_IP=<your-rover-ip>
```

## Add sourceing the setup.bash for Mowgli to \~/.bashrc

```
source ~/MowgliRoverROS/devel/setup.bash
```

### Fetch Mowgli ROS and open_mower_ros software from github

## Clone MowgliROS Repo

```
cd ~
git clone git@github.com:cloudn1ne/MowgliRoverROS.git
```

## Clone submodules

```
cd ~/MowgliRoverROS/
git submodule update --init
```

## Install all ROS dependencies

```
cd ~/MowgliRoverROS/
depit
```

## Install some more dependencies that are not covered by rosdep

```
sudo apt-get -y install ros-noetic-tf2-eigen ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher ros-noetic-map-server ros-noetic-rosserial-server ros-noetic-gps-common picocom
```

## Install whats needed for ST Link V2

```
sudo apt-get -y install openocd
```

## Build ROS packages

```
cd ~/MowgliRoverROS/
./build_mowgli.sh
./build_om.sh
```

## Build str2str from RTKLIB
needs to be at least RTKLIB Branch 2.4.3

```
cd ~/MowgliRoverROS/src/RTKLIB/app/consapp/str2str/gcc/
make
sudo make install
```

## Plugin Mowgli via USB

## Install and start roscore and rosserial-server services

```
cd ~/MowgliRoverROS/raspi_scripts
./install.sh
systemctl start roscore
systemctl start rosserial
systemctl enable rosserial_watchdog
systemctl enable roscore
systemctl enable rosserial_watchdog
systemctl enable rosserial
```
