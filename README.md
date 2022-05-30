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

## rosserial node

-
- Accelerometer (I2C), Flash (SPI) support in the proxy software
- Various ADCs and PWM signals available - Board voltage, charging states ..

## Published Topics

- /battery_voltage - std_msgs::Float32 (current Battery Voltage)
- /charge_voltage - std_msgs::Float32 (PWM controlled Charge Voltage if plugged in)
- /charge_pwm - std_msgs::Int16 (PWM value for Charge Voltage)
- /charging_state - std_msgs::Bool (True/False depending on if the bot is charging)
- /odom - nav_msgs::Odometry (Odometry messages, from motor controller feedback)
## Planned (TODO) Topics
- /blade_state - std_msgs::Bool

## Subscribers
- /cmd_vel - geometry_msgs::Twist (compatible with teleop twist messages, so you can drive the bot)
- /cmd_blade_on - std_msgs::Bool (set True to turn on the Blade Motor)
- /cmd_blade_off - std_msgs::Bool (set True to turn off the Blade Motor)

