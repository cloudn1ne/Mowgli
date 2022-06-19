## Auto start ros + rosserial

A simple watchdog is used to restart rosserial if it dies because the stm32 is rebooted or reprogrammed

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
