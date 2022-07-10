openocd -f yardforce500.cfg  -c " program ".pio/build/GD32F303/STM32F030/firmware.bin" 0x08000000 verify reset; shutdown;"
