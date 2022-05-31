openocd -f yardforce500.cfg -c "init" -c "reset halt"   -c " dump_image firmware_backup.bin 0x08000000 0x40000" -c "reset" -c shutdown
