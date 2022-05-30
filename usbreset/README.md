Handy tool to reset the usbport remotely after you flashed a new firmware update which upsets the USB stack on the raspi.

compile with:

```
gcc usbreset.c -o usbreset
```

Assuming you have your YF plugged into one of the USB 2.0 ports (black connector) on a raspi 4 run:

```
sudo ./usbreset /dev/bus/usb/001/002
```

then restart rosserial. This should help with rediscovering the CDC serial port after you have flashed new firmware onto the mainboard. The alternative is to unplug/replug the USB cable.


* [USB Reset Source](https://www.computerhilfen.de/info/usb-reset-am-raspberry-pi-usb-ports-zuruecksetzen.html)

