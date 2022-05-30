openocd -f yardforce500.cfg  -c " program "panel_controller_GD32F330_R8T6_64k.bin" 0x08000000 verify reset; shutdown;"
