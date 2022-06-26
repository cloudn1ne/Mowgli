## External (Debug) Serial & optionally IMU

## Refurbish the existing sensor board cable shown here:

<img src="serial_and_i2c_orig.jpg" width="60%">

## And refactor to an Debug and IMU (I2C) cable:

You will need to relocate the pins using a sharp pick tool (gently lift the small plastic tab, then you can extract the individual crimp pins, slide them into the new position until u hear an audible click)

<img src="serial_and_i2c_mowgli.png" width="100%">

If you do not need the I2C IMU you can obmit those cables, and just connect Black (GND), Blue (TX), Yellow (RX) to the Raspi GPIO header
