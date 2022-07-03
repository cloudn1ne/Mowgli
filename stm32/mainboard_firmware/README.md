# Preqrequisites

ST Link, and OpenOCD

# Firmware restore (ST Link + openocd)

Edit the restore.sh script, and update the filename to whatever your backup is called and run it.

```
Open On-Chip Debugger 0.11.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
Warn : Transport "hla_swd" was already selected
hla_swd
Info : clock speed 1000 kHz
Info : STLINK V2J28S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.239683
Info : stm32f1x.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : starting gdb server for stm32f1x.cpu on 3333
Info : Listening on port 3333 for gdb connections
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x08008834 msp: 0x2000c000
** Programming Started **
Info : device id = 0x10036414
Info : flash size = 256kbytes
** Programming Finished **
** Verify Started **
** Verified OK **
** Resetting Target **
shutdown command invoked
```

You should see the normal panel behaviour, plenty of LEDS active, panel locked etc ..

# Backing up your firmware

## Locating the connector

There is a 4-pin connector on the board (J9) - it has silkscreen that is labeled with GND, SWCL, SWDA, 3V3 pins.

<img src="../../images/mainboard_swd.jpg" width="60%"/>

## Backing up the firmware yourself (ST Link + openocd)

run the backup_firmware.sh script

```
Open On-Chip Debugger 0.11.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
Warn : Transport "hla_swd" was already selected
hla_swd
Info : clock speed 1000 kHz
Info : STLINK V2J28S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.239683
Info : stm32f1x.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : starting gdb server for stm32f1x.cpu on 3333
Info : Listening on port 3333 for gdb connections
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x08008834 msp: 0x2000c000
dumped 262144 bytes in 4.186105s (61.155 KiB/s)

shutdown command invoked
```

You should then find a firmware_backup.bin in the current folder

Check the SHA256 hashes at the bottom of the page to confirm a successfull backup, if they are listed already - if not please let me know your model and the fw details in the #reverse-engineering channel so i can update this list.

## Backing up the firmware yourself (JLink Segger + openocd)

Reference: https://medium.com/techmaker/reverse-engineering-stm32-firmware-578d53e79b3

(Assuming linux)

This step is easier with an interactive session because the length of your flash may be different per board

Start the openocd session. For j-link (and clones) 

    sudo openocd -c "adapter driver jlink" -c "adapter speed 2000" -c "transport select swd" -f "target/stm32f1x.cfg" 

Connect to the openocd session in a new terminal window

    telnet 127.0.0.1 4444

In that prompt try running these commands to connect to halt+reset it

    init 
    reset init

List flash banks available on the board - there should be one (size 0x00040000)

    flash banks 

Read the firmware and save it to firmware.bin, use the size you got from the previous step 

    flash read bank 0 firmware.bin 0 0x00040000

Exit the sessionr

    exit

Now you should have a firmware.bin file in the CWD you ran the openocd daemon

## Firmware SHA256 Hashes

| Model | PCB | PCB Version Markings | Controller | FW Version |FW Size | FW SHA256 Hash
|-------|-----|----------------------|------------|------------|--------|----------------
|YF500 Classic|Mainboard|20190715 RM-MB-V6.0.0|STM32F103 VCT6|V4.00_2018(191127)|256KB|a98ff567139f2d62c28c4271bea5a46546c7fc518d61985d90bfb66c626d5355
|YF500 Classic|Mainboard|20200616 RM-MB-V6.0.0|STM32F103 VCT6|V4.02_2020(201126)|256KB|b357bb83294f96a80247ece71a107e271768c9ba40efa532cf010af979e2c988
|YF500 Classic|Mainboard|20210427 RM-MB-V6.0.0|STM32F103 VCT6|V4.02_2020(210809)|256KB|875bedb32ccb62ee23453581c8af13e61a523bfe758120564d6bdb27cf641998
|YF500 Classic|Panel    |2020.05.08 RM-ECOW-V1.3.0|GD32F330 R8T6|EC4_V1.00_2020(210629)|64KB|b3b1142bf84e09fd506bf4b399632b60d2a6ae3a520c685d3fdbca08e2c1fb88
|YF500 ECO?   |Panel    |?|?|ECOW_V1.01_2019(201127)|64KB|00f75588237ef93e4afa7104f261ad112ecf7de2c436d5ac807f2985d8963e57
|YF900 ?      |Mainboard|?|STM32F103 VC??|V4.02_2020(201109)|256K|6ffa8130e60a0cd9cc4cc70488293e7436dfa909bf81ce09938abc9050f6ea96
|YF900 ?      |Panel.   |?|STM32F030 R8T6|ECOW_V1.00_2018(181102)|64K|316204837c3ebb85780dc929b4f2631bd4c77528af53aa7c6ea0945698412602
|RM1000 (Biltema)|Mainboard|?|?|V4.01_2019(191130)|256K|593684a5fdf4393c8959570a368361e729bd36272d3cbc66794be2bdff458bd2
	
