#!/usr/bin/env python
import tf
import rospy
import random
import numpy as np
import cv2, cv_bridge

from nav_msgs.msg import Odometry
from math import pi, sqrt, radians , ceil
from geometry_msgs.msg import Twist,Pose2D


class Workshop6:
   twist = Twist()
   def __init__(self):
      self.bridge = cv_bridge.CvBridge()

      #Subscribing and publishing to the relevant topics
      self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
      self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
      
      #Initialising all variable values
      self.pose = Pose2D()   
      self.r = rospy.Rate(30)



      try:
      	self.run()
      except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
       


   def run(self):
      self.twist = Twist()
      self.current_x = self.pose.x
      self.current_y = self.pose.y

      while not rospy.is_shutdown():
	  
      	  if(round(self.pose.x,2) !=round(self.current_x,2) or round(self.pose.y,2) !=round(self.current_y,2)):
          	print(self.pose.x , self.pose.y)
		self.to_grid(self.pose.x,self.pose.y,0,0,20,20,1)
		self.current_x = self.pose.x
	        self.current_y = self.pose.y

      self.stop
      rospy.loginfo("Action successfully.")


   #If you are initialising the bottom left grid cell to 1,1 not 0,0
   def to_grid(self, px, py, origin_x, origin_y, size_x, size_y, resolution):
      gx = ceil((px-origin_x)/resolution)
      gy = ceil((py-origin_y)/resolution)
      print (gy ,gx )
      #if(gx>(size_x/resolution) && gy>(size_y/resolution))

   #If you are initialising the bottom left grid cell to 1,1 not 0,0
   def to_world(self, gx, gy, origin_x, origin_y, size_x, size_y, resolution):
      px = (gx*resolution + origin_x) + (resolution/2)
      py = (gy*resolution + origin_y) + (resolution/2)
      print (py ,px )
      #if(gx>(size_x/resolution) && gy>(size_y/resolution))


   def stop(self):

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self._cmd_pub.publish(self.twist)
        rospy.sleep(1)




  
   #Odom Callback which returns values of pose orientation which we translate to self.pose
   def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

    


rospy.init_node('workshop6')
workshop6 = Workshop6()
rospy.spin()