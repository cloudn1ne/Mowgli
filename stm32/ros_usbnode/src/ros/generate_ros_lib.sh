
# copy rosserial from our ROS host
scp -r ubuntu@10.146.111.222:/home/ubuntu/mowgli_ws/libraries/ros_lib ros_lib/

# overwrite arduino code with STM32 code
cp extra/* ros_lib
rm ros_lib/ArduinoHardware.h
rm ros_lib/ArduinoTcpHardware.h
