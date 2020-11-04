#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowing():
    def __init__(self):
        rospy.init_node("WallFollowing")
        self.laser = rospy.Subscriber("/scan", LaserScan, self.ls_callback)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.run()

    right_distance = float

    def ls_callback(self, msg):
        newDistance = 1
        for i in range(0, 20):
            if msg.ranges[260+i] < newDistance:
                newDistance = msg.ranges[225+i]
        self.right_distance = newDistance

    def run(self):
        while(True):
            if self.right_distance < 1:
                
                info = Twist()
                info.angular.x = 0
                info.angular.y = 0
                info.angular.z = ((1-self.right_distance)-0.5) * abs(((1-self.right_distance)-0.5))
                info.linear.y = 0
                info.linear.z = 0
                info.linear.x = 0.2

                #print(self.right_distance)
                print(info.angular.z)
                self.cmd_vel.publish(info)

if __name__ == '__main__':
    wallFollowing = WallFollowing()
