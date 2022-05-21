############################################################################
# PAC Joystrick Drive for YF500 Mower Robot V1.0 - cn@warp.at/CyberNet, 2022
###############))))#########################################################
# 
from re import I
import time
import serial
import pygame
import sys
import time
import argparse
import os

def set_bit(value, bit):
    return value | (1<<bit)

def clear_bit(value, bit):
    return value & ~(1<<bit)
    
def pac_crc(command):
    ########################################
    # calc CRC
    ########################################
    crc = 0x0
    for i in command[2:]:
        crc += i
    crc = crc % 256
    if (crc >0):
        crc -=1
    else:
        crc = 255
    crc_byte = crc.to_bytes(1, 'little')    
    return crc_byte

# Blade Motor
def pac5225_message(speed):
    ########################################
    # create message
    ########################################
    command = bytes([ 0x55, 0xAA, 0x3, 0x20, 0x80, speed ])
    #command = bytes([ 0x55, 0xAA, 0x3, 0x20, speed, 0x80 ])
    command += pac_crc(command)
    return command

# Drive Motor
def pac5210_message(left_dir, right_dir, left_speed, right_speed):
    ########################################
    # encode direction byte
    ########################################
    direction = 0
    if (left_dir.lower() == "r"):
        direction = set_bit(direction, 4)
        direction = set_bit(direction, 5)

    if (left_dir.lower()  == "f"):
        direction = clear_bit(direction, 4)
        direction = set_bit(direction, 5)

    if (right_dir.lower()  == "r"):
        direction = set_bit(direction, 6)
        direction = set_bit(direction, 7)

    if (right_dir.lower()  == "f"):
        direction = clear_bit(direction, 6)
        direction = set_bit(direction, 7)        

     # obey limits
    if (left_speed >= 255):
        left_speed = 255
                
    if (right_speed >= 255):
        right_speed = 255
                
    if (left_speed <= 0):
        left_speed = 0

    if (right_speed <= 0):
        right_speed = 0

    ########################################
    # create message
    ########################################
    command = bytes([ 0x55, 0xAA, 0x8, 0x10, 0x80, direction, right_speed, left_speed, 0x0, 0x0, 0x0 ])
    command += pac_crc(command)

    return command

def pac5210_sendidlecommand(ser):

    idle_command = pac5210_message("", "", 0,0)
    print("TX (STOP): ",['%02x' % int(i) for i in idle_command])
    ser.write(idle_command)    

def main():
    print("\n   /=======================================/")
    print("  /  PAC Drive for YF500 - CyberNet, 2022 /")
    print(" /=======================================/\n")

    parser = argparse.ArgumentParser()    
    parser.add_argument('--uart', type=str, help='UART device path (e.g. /dev/cu.XXXX)')

    args = parser.parse_args()
    if (args.uart == None):        
        print("error: no uart specified");        
        sys.exit()

    JOYSTICK = 0            # first
    LEFT_RIGHT_AXIS = 0     # left stick horiz
    FWD_REV_AXIS = 3        # left stick horiz
    DEADZONE = 0.05
    BUMP_STEER = 50          

    PREAMBLE_AA = bytes([0x55]);

    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.init()
    pygame.display.init()
    pygame.joystick.Joystick(JOYSTICK).init()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    print("found ",joysticks)
    joystick_count = pygame.joystick.get_count()
    print("number of joysticks = ",joystick_count)

    ########################################
    # open serial
    ########################################
    ser = serial.Serial(
        port=args.uart,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    if(ser.isOpen() == False):
        ser.open()

    pac5210_sendidlecommand(ser)

    # default blade motor is off
    blade_on = False

    done = False
    while not done:                
        #print(pygame.joystick.Joystick(0).get_axis(3))
        left_right_jval = pygame.joystick.Joystick(JOYSTICK).get_axis(LEFT_RIGHT_AXIS)
        front_rev_jval = pygame.joystick.Joystick(JOYSTICK).get_axis(FWD_REV_AXIS)

        if (front_rev_jval > DEADZONE or front_rev_jval < -DEADZONE):
            # DRIVE MODE
            front_rev = -front_rev_jval*255
            left_right = left_right_jval*255
        #    print(front_rev)

            left_speed = front_rev+(left_right/255*BUMP_STEER)-BUMP_STEER
            right_speed = front_rev-(left_right/255*BUMP_STEER)-BUMP_STEER

            if (front_rev < 0):
                left_dir = "r"
                right_dir = "r"               
            else:
                left_dir = "f"
                right_dir= "f"

            # obey limits
            if (left_speed >= 255):
                    left_speed = 255
                
            if (right_speed >= 255):
                    right_speed = 255

            if (left_speed <= -255):
                    left_speed = -255
                
            if (right_speed <= -255):
                    right_speed = -255            
    
            left_speed = int(abs(left_speed))
            right_speed = int(abs(right_speed))
#            print("DRIVE| left_speed = ", left_speed, "(",left_dir.upper(),") right_speed = ", right_speed, "(",right_dir.upper(),")")


        else:
            # TURN ONLY MODE
            turn_speed = 100

            if (left_right_jval > DEADZONE or left_right_jval < -DEADZONE):
                left_right = left_right_jval*255
                #print ("rotation only ", left_right)
                #print ("left_right  => ", left_right)
            else:
                left_right = 0
            
            left_speed = turn_speed*(left_right/255)            
            right_speed = -turn_speed*(left_right/255)
            if (left_speed < 0):
                left_dir = "r"
            else:
                left_dir = "f"

            if (right_speed < 0):
                right_dir = "r"
            else:
                right_dir = "f"                

            left_speed = int(abs(left_speed))
            right_speed = int(abs(right_speed))
#            print("TURN| left_speed = ", left_speed, "(",left_dir.upper(),") right_speed = ", right_speed, "(",right_dir.upper(),")")
            

        
        for event in pygame.event.get(): # User did something.
            if event.type == pygame.QUIT: # If user clicked close.
                done = True # Flag that we are done so we exit this loop.
            elif event.type == pygame.JOYBUTTONDOWN:
                    if pygame.joystick.Joystick(JOYSTICK).get_button(1): # B button                        
                        blade_on = True                        
                    elif pygame.joystick.Joystick(JOYSTICK).get_button(2):  # A button                    
                        blade_on = False                        
                    #elif pygame.joystick.Joystick(JOYSTICK).get_button(3):
                    #    print("y")
                    #elif pygame.joystick.Joystick(JOYSTICK).get_button(0):
                    #    print("x")

            #elif event.type == pygame.JOYBUTTONUP:
            #    print("Joystick button released.")



        # drive bot
        drive_command = pac5210_message(left_dir, right_dir, left_speed, right_speed)
        print("TX (MOVE): ",['%02x' % int(i) for i in drive_command])
        ser.write(drive_command)
        time.sleep(0.10)   

        # blade command
        if (blade_on):
            blade_command = pac5225_message(0x80);
        else:
            blade_command = pac5225_message(0x0);            

        print("TX (BLADE): ",['%02x' % int(i) for i in blade_command])
        ser.write(blade_command)
        time.sleep(0.10)   
    sys.exit()
 


if __name__ == "__main__":
    main()
