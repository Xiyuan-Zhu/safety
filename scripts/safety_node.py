#!/usr/bin/env python
import rospy
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

"""
The safety node subscribe from two topics:
'odom': a Odometry topic that contain information like speed, position and steering.
'scan': a LaserScan topic that contain data from Lidar.

The safty node publish to two topics:
'brake_bool': a Bool topic that denote if ABE should engage.
'brake': a AckermannDriveStamped to control the car when 'brake_bool' is true. The speed in it should be 0 to stop the car.

The program runs from "if __name__ == '__main__':" 
"""

class Safety(object):
    def __init__(self):
        self.speed = 0      # a parameter that contains current speed
        
        # subscribe and publish to topics
        rospy.Subscriber('odom',Odometry,self.odom_callback,queue_size=10)
        rospy.Subscriber('scan',LaserScan,self.scan_callback,queue_size=10)
        self.bool_topic = rospy.Publisher('brake_bool',Bool,queue_size=10)
        self.brake_topic = rospy.Publisher('brake',AckermannDriveStamped,queue_size=10)
    
    # when a message is received from 'odom', the following method will run
    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x  # update current speed

    # when a message is received from 'scan', the following method will run
    def scan_callback(self, scan_msg):
        # calculate TTC for each angle, see if brake is needed
        need_to_brake = False
        for idx in range(len(scan_msg.ranges)):
            ang = scan_msg.angle_min + idx * scan_msg.angle_increment
            if -math.pi/2 < ang < math.pi/2:        # only look at the front
                speed_prod = self.speed*math.cos(ang) + 0.1
                distance = scan_msg.ranges[idx]
                ttc = distance/speed_prod
                if ttc < 2:
                    need_to_brake = True
                    break
        
        # engage the brake if needed
        if need_to_brake:
            print("ABE engaged. ang: {:.2f} speed_prod: {:.3f} distance: {:.3f} TTC:{:.3f}".format(ang, speed_prod, distance, ttc))
            bool_msg = Bool()
            bool_msg.data = True
            self.bool_topic.publish(bool_msg)
            
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed=0
            self.brake_topic.publish(brake_msg)
        else:
            bool_msg = Bool()
            bool_msg.data = False
            self.bool_topic.publish(bool_msg)
        
        
def main():
    rospy.init_node('safety_node')          # init ros node with name 'safety_node'
    sn = Safety()           # init python class 'Safety', which handles emergency braking
    print("Safety node initialized.")
    rospy.spin()            # spin, so this python program will keep running.
    
if __name__ == '__main__':
    main()

