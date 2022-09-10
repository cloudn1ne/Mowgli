# Mowgli

<img src="./images/gforce.jpg" width="60%"/>


## About

This repo tracks my progress in reverse engineering the Yardforce GForce mainbord as used in the Yardforce 500 (and others) Mower Robots.

In the end in would like to use it with [OpenMower](https://github.com/ClemensElflein/OpenMower) but without having to use that projects mainboard, instead recycling as much hardware as possible from the original GForce mainboard. 
There should be no irreversible modifications to the GForce board required.

## Looking for MowgliRover ?

see [here](https://github.com/cloudn1ne/MowgliRover)

## Updates

see [Updates](UPDATES.md)


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
- Mainboard (GForce) Firmware [dump and restore](./stm32/mainboard_firmware) via an ST Link or other (e.g. JLink Segger) tools
- Panel (GForce) Firmware [dump and restore](./stm32/panel_firmware) via an ST Link or other (e.g. JLink Segger) tools
- [Demo Python code](./playground/) to control the Mower via a Joystick (needs "test_code" STM32 code flashed)
- [ROS Serial node](./stm32/ros_usbnode) via CDC USB Port with full hardware support
- Serial debugging on the unused (red) J18 header
- Software I2C on the unusued (red) J18 header - used for external IMU
- [Raspi, IMU, GPS Mounting Options](./Mounting)
- onboard Buzzer
- onboard IMU (accelerometer) for tilt protection
- Safety switches (Hall Sensors) for STOP button
- Rain Sensor

## TODO

- Move all UART functions use DMA as HAL_IT is a seriously broken API
- Publish Blade Motor state
- (Probably) Shift all the published topics to some common base such as /mowgli/

## rosserial node

- [See here how to use the ROS serial node](stm32/ros_usbnode)

## Published Topics

- /battery_voltage - std_msgs::Float32 (current Battery Voltage)
- /charge_voltage - std_msgs::Float32 (PWM controlled Charge Voltage if plugged in)
- /charge_current - std_msgs:Float32 (still a bit unclear how that works, needs more Kicad'ding)
- /charge_pwm - std_msgs::Int16 (PWM value for Charge Voltage)
- /charging_state - std_msgs::Bool (True/False depending on if the bot is charging)
- /odom - nav_msgs::Odometry (Odometry messages, from motor controller feedback)
- /left_encoder_ticks & /right_encoder_ticks - std_msgs::UInt21 (Accumulated Raw encoder values)
- /imu_onboard/data_raw - sensor_msgs::Imu - onboard accelerometer data (no gyro, no mag !)
- /imu_onboard/temp - sensor_msgs::Temperature - onboard accerlerometer temperature
- /buttonstates - on supported panels

### in case a supported I2C IMU is connected to J18

- /imu/data_raw - sensor_msgs::Imu - external IMU data (acceleration, gyro)
- /imu/mag - sensor_msgs::MagneticField - external IMU data (compass)

Currently only the [Pololu IMU 10v5](https://www.pololu.com/product/2739) is supported, but any I2C or SPI IMU should work.

## Planned (TODO) Topics
- /blade_state - std_msgs::Bool

## Subscribers
- /cmd_vel - geometry_msgs::Twist (compatible with teleop twist messages, so you can drive the bot)
- /cmd_blade_on - std_msgs::Bool (set True to turn on the Blade Motor)
- /cmd_blade_off - std_msgs::Bool (set True to turn off the Blade Motor)
- /cmd_reboot- std_msgs::Bool (set True to reboot the STM32)

## Serial Debugging

[Serial Debug for ros_usbnode](stm32/ros_usbnode#serial_debug)





