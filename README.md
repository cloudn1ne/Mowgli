# Mowgli

<img src="./images/gforce.jpg" width="60%"/>


## About

This repo tracks my progress in reverse engineering the Yardforce GForce mainbord as used in the Yardforce 500 (and others) Mower Robots.

In the end in would like to use it with [OpenMower](https://github.com/ClemensElflein/OpenMower) but without having to use that projects mainboard, instead recycling as much hardware as possible from the original GForce mainboard. 
There should be no irreversible modifications to the GForce board required.

## Safety
<table border=1>
   <tr>
      <td>
The custom firmware has no protections - no tilt sensing, no stop buttons will work. If you stick your finger in the wrong place, you will lose it.<p>
         <ul>
<li>Remove the razor blades
<li>Dont be stupid
<li> Dont blame me for your lost finger
         </ul>
      </td>
      <td>
         <img src="./images/finger.png" align="right" width="150px"/>
      </td>
   </tr>
</table>

## Whats done ?

- [Basic overview of the mainboard](./Kicad), [Datasheets](./Datasheets) for ICs
- Motor Drivers for both Drive Motors and Blade Motor can be controlled
- Firmware dump and [restore](./stm32/original_firmware) via an ST Link or other (e.g. JLink Segger) tools
- [Demo Python code](./playground/) to control the Mower via a Joystick (needs STM32 code for Blade Motor control)

## Progress

Custom software for the STM32 main cpu, thats acts as a UART proxy to the components on the board

[![IMAGE ALT TEXT](http://img.youtube.com/vi/Jlm87qAKzOk/0.jpg)](http://www.youtube.com/watch?v=Jlm87qAKzOk "Drive Motors and Blade Motor with custom firmware")
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

