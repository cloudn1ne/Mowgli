# Mowgli

<img src="./images/gforce.jpg" width="60%"/>


## About

This repo tracks my progress in reverse engineering the Yardforce GForce mainbord as used in the Yardforce 500 (and others) Mower Robots.


## Whats done ?

- [Basic overview of the mainboard](./Kicad), [Datasheets](./Datasheets) for ICs
- Motor Drivers for both Drive Motors and Blade Motor can be controlled
- Firmware dump and [restore](./stm32/original_firmware) via an ST Link or other (e.g. JLink Segger) tools
- [Demo Python code](./playground/pay_joydrive.py) to control the Mower via a Joystick (needs STM32 code for Blade Motor control)

## In Progress
- Custom software for the STM32 main cpu, thats acts as a UART proxy to the components on the board
   - Drive Motor proxing working
   - Blade Motor proxing working

## Todo
- Serial protocol for the Control Panel, and why it needs to be plugged in for the drive motors to work
- Accelerometer (I2C), Flash (SPI) support in the proxy software
- Various ADCs and PWM signals available - Board voltage, charging states ..

## How to help

Check the Kicad drawings - note they contain hierachical subsheets (right click sheet, enter sheet).
Any missing pins or functions - please let me know or better a pull request.

## STM Code 

Highly experimental - and not stable, will change when new features are discovered on the mainboard.
Some custom protocol (as simple as possible) that can then be used by 3rd party software such as ROS to control the mower is the end goal

