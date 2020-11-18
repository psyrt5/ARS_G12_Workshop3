#!/usr/bin/env python
import tf
import rospy
import random
import numpy as np
import cv2, cv_bridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from math import pi, sqrt, radians
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Pose2D


class Follower:
   twist = Twist()
   def __init__(self):
      self.bridge = cv_bridge.CvBridge()

      #Subscribing and publishing to the relevant topics
      self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
      self._sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
      self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
      
      #Initialising all variable values
      self.pose = Pose2D()   
      self.r = rospy.Rate(30)
      self.obs_distance = 0
      self.LIDAR_ERR = 0.05

      #Setting Twist angular velocity to 0.2rad/s
      self.turn = Twist()
      self.stop_turn = Twist()
      self.stop_turn.angular.z = 0.0
      self.turn.angular.z = 0.2




   #Using Laser scanner callback to detect obstacles
   def scan_callback(self,msg):
         self.scan_filter = []

         for i in range(360):	
             if i <= 15 or i > 335:
                 if msg.ranges[i] >= self.LIDAR_ERR:
                     self.scan_filter.append(msg.ranges[i])
	
         self.obs_distance = min(self.scan_filter)
  
   #Using the image_callback function to obtain an image and convert it to hsv
   def image_callback(self, msg):

      #Getting the image and converting to hsv values
      image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

      #Processing the image data
      self.image_processing(image)




   #The main function which handles the image processing
   def image_processing(self,image):

      (h, w) = image.shape[:2]
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

      #Setting the lower and upper hsv values of green
      lower_green = np.array([0, 100, 0])
      upper_green = np.array([100, 255, 100])

      # find the colors within the specified green colour boundaries and apply the mask
      mask = cv2.inRange(hsv, lower_green, upper_green)

      #output = cv2.bitwise_and(hsv, hsv, mask = mask)
      #output = cv2.resize(output, (w/4,h/4))
      #cv2.imshow("Masked Image", output)
      #m_output = cv2.resize(mask, (w/4,h/4))
      #cv2.imshow("Mask", m_output)

      #Getting the height, width, depth of image shape.
      h, w, d = image.shape
      M = cv2.moments(mask)
      
      self.detect_green(image,M,h,w,d)


   #Function for producing movement based on what the robot can see
   def detect_green(self,image,M,h,w,d):

      #If a green shape is located
      if M['m00'] > 0:

	 print("Located Green Shape")

	 #Finding the center of the shape using pixel intensities moments calculations
         cx = int(M['m10']/M['m00'])
         cy = int(M['m01']/M['m00'])
         cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
         
	 #Calculating how far the robot is pointing off the center and adjusting the angular.z proportionately
	 err = cx - w/2
         self.twist.angular.z = -float(err) / 500
	 self.twist.linear.x = 0.2

         self.cmd_vel.publish(self.twist)

      
      #If there are no green objects in sight move until hitting an obstacle
      else:
	 
	 #If an object is <0.5m away, rotate 0.3rad/s
         if self.obs_distance < 0.5:
	 	self.twist.linear.x = 0
                self.twist.angular.z = 0.3
	        self.cmd_vel.publish(self.twist)
	        print("Obstacle Detected")
         else:
		self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
	        
		self.cmd_vel.publish(self.twist)
            
      
      #cv2.waitKey(0)
      self.stop
      rospy.loginfo("Action successfully.")
     


   #Stopping the robot
   def stop(self):

         self.twist.linear.x = 0
         self.twist.angular.z = 0
         self._cmd_pub.publish(self.twist)
         rospy.sleep(1)


rospy.init_node('follower')
follower = Follower()
rospy.spin()