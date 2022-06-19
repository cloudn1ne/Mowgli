#!/usr/bin/env python
import rospy
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        
def callback_imu(data):    
#    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)

    quaternion = (
    data.orientation.x,
    data.orientation.y,
    data.orientation.z,
    data.orientation.w)
    
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0] * 180/math.pi
    pitch = euler[1] * 180/math.pi
    yaw = euler[2] * 180/math.pi

    print("---------------------------------------------")    
    print("roll: %.3f pitch: %.3f yaw: %.3f " % (roll, pitch, yaw))

def callback_mag(data):    
#    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)

    x = data.magnetic_field.x
    y = data.magnetic_field.y
    z = data.magnetic_field.x
      
    yxHeading = math.atan2(x, y);
    zxHeading = math.atan2(z, x);
    heading = yxHeading;
    declinationAngle = 0.0861;
    heading =  heading + declinationAngle;
    if (heading < 0):
        heading += 2*math.pi;
    if (heading > 2*math.pi):
        heading -= 2*math.pi;

    headingDegrees = heading * 180/math.pi; 

    print("---------------------------------------------")    
    print("heading: %.10f " % (headingDegrees))

def calibrate():

    rospy.init_node('calibrate', anonymous=True)
   # rospy.Subscriber("/imu/mag", MagneticField, callback_mag)
    rospy.Subscriber("/imu/data", Imu, callback_imu)
    rospy.spin()

if __name__ == '__main__':
    calibrate()
