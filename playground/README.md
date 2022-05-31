## This code is now pretty much obsolete, better use the [ROS USB Node](../stm32/ros_usbnode) approach

## HowTo

- Flash the [Mowgli STM32 code](./stm32/Mowgli/) using Platform IO and an ST Link (Platform IO Project is setup for an ST Link already)
- You will need something that can run python3 and has pygame installed. I use a Mac or a Raspi will work fine too.
- Wire your serial adapter (or Raspi) to the serial port on the GForce board.
   I used the J18 (Red connector on the mainboard) because the connector from J5 (Signal will fit) and i dont need the signal sense board anymore.
   Maybe something like a JST PH (2mm pin pitch) fits as well, but i dont have them yet.
   As the pins are unfortunatly in the wrong place on the original J5 connector i used a sharp pick tool to relocate the pins as in the image below.
   
   <img src="/images/J18_serial.jpg" width="60%"/>
   
   You need to short out the connectors on the control(LED + Buttons) board and have that plugged into the mainboard.
   
   <img src="/images/bridged_connectors.jpg" height="200"/>
   
   You can then send messages (see Python Joystick example) to the STM32 which will pass them along to either the Blade Motor Controller or the Drive Motors Controller.
