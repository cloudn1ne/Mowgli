openocd -f yardforce500.cfg  -c " program "firmware.bin" 0x08000000 verify reset; shutdown;"
