#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField

min_x = 0.0
min_y = 0.0
min_z = 0.0
max_x = 0.0
max_y = 0.0
max_z = 0.0
init = 1
f = open("calibration.txt", "w")

def callback(data):
    global min_x, min_y, min_z
    global max_x, max_y, max_z
    global init
    global f
    
#    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)

    x = data.magnetic_field.x
    y = data.magnetic_field.y
    z = data.magnetic_field.x
      
    if init == 1:
      min_x = max_x = x
      min_y = max_y = y
      min_z = max_z = z
      init = 0
      
    # find minimums
    min_x = min(min_x, x);
    min_y = min(min_y, y);
    min_z = min(min_z, z);
    
    # find maximums
    max_x = max(max_x, x);
    max_y = max(max_y, y);
    max_z = max(max_z, z);	

    print("---------------------------------------------")    
    print("min_x: %.10f max_x = %.10f " % (min_x, max_x))
    print("min_y: %.10f max_y = %.10f " % (min_y, max_y))
    print("min_z: %.10f max_z = %.10f " % (min_z, max_z))
    f.write("%.10f %.10f %.10f\n" % (x,y,z))

def calibrate():

    rospy.init_node('calibrate', anonymous=True)
    rospy.Subscriber("/imu/mag_calibration", MagneticField, callback)
    rospy.spin()

if __name__ == '__main__':
    calibrate()
