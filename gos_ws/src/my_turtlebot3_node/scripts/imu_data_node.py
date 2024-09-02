#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
import tf.transformations as tft

def imu_callback(data):
    quaternion = data.orientation
    euler = tft.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    roll, pitch, yaw = euler

    yaw_degrees = yaw * (180.0 / 3.14159)
    
    rospy.loginfo(f"Yaw (degrees): {yaw_degrees}")

    yaw_pub.publish(yaw_degrees)

rospy.init_node('imu_quaternion_to_euler', anonymous=True)
rospy.Subscriber('/imu', Imu, imu_callback)
yaw_pub = rospy.Publisher('/imu/yaw_degrees', Float32, queue_size=10)

if __name__ == '__main__':
    try:
        # Your code here
        rospy.spin()
    except rospy.ROSInterruptException:
        pass