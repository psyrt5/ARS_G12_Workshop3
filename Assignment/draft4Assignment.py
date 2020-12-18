#!/usr/bin/env python
import tf
import rospy
import random
import numpy as np
import time
from threading import Timer

from nav_msgs.msg import Odometry , OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from math import pi, sqrt, radians , ceil , floor , isinf , isnan , sin , cos , radians , degrees
from geometry_msgs.msg import Pose2D, Point,Twist
from actionlib_msgs.msg import *

import cv2, cv_bridge
from sensor_msgs.msg import Image, LaserScan


class draft2:
    
    def __init__(self):
	self.bridge = cv_bridge.CvBridge()
	self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
	self.depth_sub = rospy.Subscriber('camera/depth/image_raw', Image, self.depth_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        
        
        self.pose = Pose2D()   
        self.r = rospy.Rate(30)
        self.twist = Twist()

        self.size = 20
	self.origin = -10
        self.current_x = 0
        self.current_y = 0
        self.current_theta =0
	self.nullGoalCount = 0

	self.LIDAR_ERR = 0.05
        self.scan_filter = []
	self.obs_distance = []

	
        self.goalReached = False
	self.obstacleDetected = False
	self.goalSearchInProgress = False
	self.objectDetected = False
	self.notMoved = False

	#Initialising variables for object recognition
        #Fire Hydrant Upper and Lower Bounds
        self.fh_lower = np.array([0, 230, 0])
        self.fh_upper = np.array([0, 255, 100])

        #Green Box Upper and Lower Bounds
        self.gb_lower = np.array([40, 200, 0])
        self.gb_upper = np.array([80, 255, 255])

        #Mail Box Upper and Lower Bounds
        self.mb_lower = np.array([80, 150, 20])
        self.mb_upper = np.array([130, 170, 50])

        #Number 5 Upper and Lower Bounds
        self.n5_lower = np.array([0, 0, 100])
        self.n5_upper = np.array([0, 0, 255])
	self.n5_b_lower = np.array([0, 0, 0])
        self.n5_b_upper = np.array([0, 0, 0])
	
	#Object Thresholds
        self.scale = 4

        self.fh_threshold_lower = 10000
        self.fh_threshold_upper = 200000

        self.mb_threshold_lower = 20000
        self.mb_threshold_upper = 200000

        self.gb_threshold_lower = 30000
        self.gb_threshold_upper = 500000

        self.n5_threshold_lower = 10000
        self.n5_threshold_upper = 500000

        #Boolean variables for keeping track of which objects were found
        self.fh_found = False
        self.gb_found = False
        self.mb_found = False
        self.n5_found = False

        #Positional values locating the target on screen
        self.target_x = 0
        self.target_y = 0

	

	time.sleep(5)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")

    
            
    def run(self):
	
	#Intialsing grid and waiting for odometry variables to be initialised
        self.grid = [[0 for x in range(self.size)] for y in range(self.size)]
	
	#Define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")
        while not rospy.is_shutdown():
            self.explore(ac)




#****************************************************************************** EXPLORE FUNCTIONS *************************************************************************
    
    #The function responsible for exploring the map. Obtaining goal positions and sending the robot there.
    def explore(self,ac):
	goalReached = False

	if(self.nullGoalCount < 4):
		#Get the goal position as a grid coordinate and as a specific x, y coordinate
		xGridGoal,yGridGoal = self.explore_getGoal()
	else:
		print("Annealing ************************************************************************************")
		xGridGoal,yGridGoal = self.explore_annealingEscape()

	goalP_x , goalP_y = self.to_world(xGridGoal,yGridGoal,self.origin,self.origin,self.size,self.size,1)

	#print(xGridGoal , yGridGoal)
	
	goalReached = self.explore_moveToGoal(ac,goalP_x, goalP_y,xGridGoal,yGridGoal)

	#If the robot moved sucessfully to the goal
        if (goalReached):
            rospy.loginfo("Goal Met!")
	    self.update_grid(xGridGoal,yGridGoal,100)
        else:
            rospy.loginfo("Goal Not Met")
    

    #This function moves the robot to the goal and returns true/false if goal is met or not
    def explore_moveToGoal(self,ac,xGoal,yGoal,xGridGoal,yGridGoal):



	#Initialising the parameters of the move_base goal location
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)
	self.goalRunning = True
	self.timerRunning = False
	self.notMoved =False
	self.stopTimer =False
	self.objectDetected = False

        while (ac.get_state()<3 and self.goalRunning == True):
	    
	    self.explore_updateSearchGrid()

	    if(self.obstacle_detection() == False):
		    if(self.check_for_objects(ac)==False):
			    #If the timer is not running start a new timer
			    if(self.timerRunning ==False):
				my_Timer = Timer(2.5, self.explore_hasMoved)
				my_Timer.start()
				self.timerRunning =True
				self.stopTimer = False
		
			    #IF the timer needs to be stopped
			    if(self.stopTimer ==True):
				my_Timer.cancel()
				self.timerRunning = False
			    
		            #If the robot is not moving
			    if(self.notMoved == True):
				self.update_grid(xGridGoal, yGridGoal,999)
				self.nullGoalCount +=0.5
				return False
			  
			
			    
	    else:
		self.goalRunning = False
		ac.cancel_goal()
		self.obstacle_avoidance()
		self.grid[xGridGoal][yGridGoal] +=5
		self.nullGoalCount +=1
	
	if(self.timerRunning):
		my_Timer.cancel()
		self.timerRunning = False

	self.goalRunning = False

	if(self.grid[xGridGoal][yGridGoal]<15):
		print("Debugging message" , ac.get_state())
		if(ac.get_state() == 3):
			self.print_searchGrid()
		return False
	else:
		self.nullGoalCount = 0
		return True




    def explore_updateSearchGrid(self):
	laser_num =0
	laser_angle = 0

	if(abs(self.current_x -self.pose.x) >0.7 or abs(self.current_y - self.pose.y) >0.7 or abs(self.current_theta -self.pose.theta) >0.5):
		while(len(self.obs_distance)>2 and laser_num<3):                
		
			self.current_theta = self.pose.theta
			obs_dist = self.obs_distance


			if(obs_dist[laser_num] ==0):
				obs_dist[laser_num] = 0.1
	
			if(isinf(obs_dist[laser_num])):
				obs_dist[laser_num] = 3.5
	
			if(laser_num ==1):
				laser_angle = -0.523599 # 30 - THESE MUST BE SET TO THE SAME VALUE OF THE LASER CALLBACK
			elif(laser_num==2):
				laser_angle = 0.523599
	
			self.difference = self.pose.theta + 1.5708+laser_angle
			if (self.difference >3.14159):
				self.difference = - 3.14159 + (self.difference - 3.14159)

			self.difference = self.difference
			xDistanceToGoal = -sin(self.difference)*obs_dist[laser_num]
			yDistanceToGoal = cos(self.difference)*obs_dist[laser_num]


			while(obs_dist[laser_num]>0):
				xDistanceAlongLine = -sin(self.difference)*obs_dist[laser_num]
				yDistanceAlongLine = cos(self.difference)*obs_dist[laser_num]

				self.checkx = self.pose.x + xDistanceAlongLine
				self.checky = self.pose.y + yDistanceAlongLine
	
				gx,gy = self.to_grid(self.checkx,self.checky,self.origin,self.origin,self.size,self.size,1)
				self.update_grid(gx, gy,1)
				obs_dist[laser_num] = obs_dist[laser_num]-0.1
			
			laser_num = laser_num+1


    #This function is called every 3 seconds to check if the robot is staying in the same place
    def explore_hasMoved(self):
	if(self.objectDetected == True or self.obstacleDetected == True or self.goalSearchInProgress==True or abs(self.current_x -self.pose.x) >0.05 or abs(self.current_y - self.pose.y) >0.05):
		#print("MOVED")
		self.notMoved = False
	else:
		#print("NOT MOVED")
		self.notMoved = True
		self.print_searchGrid()
	self.stopTimer=True
	self.current_x = self.pose.x
	self.current_y = self.pose.y

    def explore_getGoal(self):
	gx , gy = self.to_grid(self.pose.x,self.pose.y,self.origin,self.origin,self.size,self.size, 1)
  	
	search_index = 0
	self.goalSearchInProgress =True
	
	print("Searching from - ", gx,gy , " pose = " , self.pose.x, self.pose.y)

	while(True):
		
		search_index = search_index +1
		gxMinus = gx-search_index
		gxPlus = gx+search_index
		gyMinus = gy-search_index
		gyPlus = gy+search_index

		if(gxMinus<0):
			gxMinus = 0
		if(gxPlus>19):
			gxPlus = 19
		if(gyMinus<0):
			gyMinus = 0
		if(gyPlus>19):
			gyPlus = 19


		x = gxMinus
		y = gyMinus
		
		x_change = 0
		y_change = 1
		
		ringSearched = False

		while(ringSearched == False):
			if(x ==gxMinus and y == gyPlus):
				x_change = 1
				y_change = 0
			elif(x ==gxPlus and y == gyPlus):
				x_change = 0
				y_change =-1
			elif(x ==gxPlus and y == gyMinus):
				x_change =-1
				y_change = 0
			elif(x==gxMinus and y == gyMinus and x_change == -1):
				x_change = 0
				ringSearched = True

			if(self.grid[x][y]<15 and x!=gx and y!=gy-1):
				self.goalSearchInProgress = False
				print(x,y)				
				return (x , y)		
				
			x = x + x_change
			y = y + y_change
		
		if(gxMinus==0 and gxPlus ==19 and gyMinus == 0 and gyPlus == 19):
			self.goalSearchInProgress = False
			print("Search Complete")
			return (gx,gy)
	

    def explore_annealingEscape(self):
	self.goalSearchInProgress =True
	maxSearchVal= 100
	print("Null Goal count = ",self.nullGoalCount)
	for i in range (5):
		for x in range(len(self.grid)):
			for y in range(len(self.grid[x])):
				if(self.grid[x][y]>0 and self.grid[x][y]<maxSearchVal):
					self.goalSearchInProgress =False
					return (x,y)
		maxSearchVal +=100

	return (self.explore_getGoal())

#****************************************************************************** OBSTACLE AVOIDANCE FUNCTIONS *******************************************************************************

    def obstacle_detection(self): 
 	   
	    if self.minDistance < 0.25:
		self.obstacleDetected =True
		return True
	    else:
		self.obstacleDetected =False
	        return False
	    
   

    def obstacle_avoidance(self):
	    while(self.obstacleDetected):

	 	    #Producing movement opposite to obstacle 
	            if self.minDistance < 0.4:
			self.obstacleDetected = True
			if(self.minDistanceAngle<=45 or self.minDistanceAngle >= 315):
		                self.twist.linear.x = -0.5
		                self.twist.angular.z = 0.0
			elif(self.minDistanceAngle>45 and self.minDistanceAngle <135):
		                self.twist.linear.x = 0.0
		                self.twist.angular.z = 0.2
			elif(self.minDistanceAngle>=135 and self.minDistanceAngle <=225):
		                self.twist.linear.x = 0.5
		                self.twist.angular.z = 0.0
			elif(self.minDistanceAngle>225 and self.minDistanceAngle <315):
		                self.twist.linear.x = 0.0
		                self.twist.angular.z = -0.2
	                self.cmd_pub.publish(self.twist)
        	        rospy.loginfo('Avoiding Obstacle!')

		    else:
			self.obstacleDetected = False
	        
	    

#****************************************************************************** OBJECT RECOGNITION FUNCTIONS ***********************************************************************************

    def check_for_objects(self,ac):
	(self.h, self.w) = self.image.shape[:2]
        self.resized_image = cv2.resize(self.image, (self.w/self.scale,self.h/self.scale))
        self.depth_resized = cv2.resize(self.depth, (self.w/self.scale,self.h/self.scale))

	if self.fh_found == False and self.detect_objects(self.mask_image(self.resized_image, self.fh_lower, self.fh_upper), self.fh_threshold_lower, self.fh_threshold_upper, ac):
            if self.move_to_object():
                self.fh_found = True
                print("Fire Hydrant Found")
		time.sleep(3)
                self.distance = 20
            return True
        elif self.mb_found == False and self.detect_objects(self.mask_image(self.resized_image, self.mb_lower, self.mb_upper), self.mb_threshold_lower, self.mb_threshold_upper, ac):
            if self.move_to_object():
                self.mb_found = True
                print("Mailbox Found")
		time.sleep(3)
                self.distance = 20
            return True
        elif self.gb_found == False and self.detect_objects(self.mask_image(self.resized_image, self.gb_lower, self.gb_upper), self.gb_threshold_lower, self.gb_threshold_upper, ac):
            if self.move_to_object():
                self.gb_found = True
                print("Green Box Found")
		time.sleep(3)
                self.distance = 20
            return True
        #if self.n5_found == False and self.check_for_box(self.resized_image):
            #if self.move_to_object():
                #self.n5_found = True
                #print("Number 5 Found")
		#time.sleep(3)
                #self.distance = 20
            #return True
	return False
          
    def move_to_object(self):
        # If the distance from the object to the robot is greater than 1 meter,
        # then we need to move the robot towards the direction of the object. 
        if self.distance >= 1.5:
            err = self.target_x - self.w/2
            self.twist.angular.z = -float(err) / 500
            self.twist.linear.x = 0.2
            self.cmd_pub.publish(self.twist)
            return False

        # The depth sensor can sometimes return NaN, which is smaller than 1. 
        # This is not the intended result so we ignore it. If the distance is
        # actually smaller than 1, then that means we have reach the vicinity of
        # the object.
        elif isnan(self.distance) == False:
            return True

        #Return false when the value is NaN
        else:
            return False

        
    #Given an image, a set of bounds, and a threshold,
    #give an estimation whether there is an object in view.
    def detect_objects(self, mask, lower_threshold, upper_threshold,ac):
        threshold = np.sum(mask == 255)

	if self.scale > 1:
            lower_threshold = lower_threshold / (self.scale * self.scale)
            upper_threshold = upper_threshold / (self.scale * self.scale)
        
        if threshold >= lower_threshold and threshold <= upper_threshold:
            #print(threshold)
            M = cv2.moments(mask)
            if M['m00'] > 0:
                #print(threshold)
                self.target_x = int(M['m10']/M['m00'])
                self.target_y = int(M['m01']/M['m00'])
                self.distance = self.depth_resized[self.target_y, self.target_x]
                if self.distance < 1:
                    return False
	    self.objectDetected = True
	    if(self.goalRunning):
		ac.cancel_goal()
		self.goalRunning = False
            return True
        return False
    
    #Given an image, and a set of bounds, mask an image
    #to retrieve a grayscale image.
    def mask_image(self, image, lower, upper):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        return mask
#****************************************************************************** CALLBACK FUNCTIONS AND UTILITY FUNCTIONS ***********************************************************************

   
    def to_grid(self, px, py, origin_x, origin_y, size_x, size_y, resolution):
        gx = int(floor((px-origin_x)/resolution))
        gy = int(floor((py-origin_y)/resolution))
	return gx , gy

    def to_world(self, gx, gy, origin_x, origin_y, size_x, size_y, resolution):
        px = (gx*resolution + origin_x) + (resolution/2.0)
        py = (gy*resolution + origin_y) + (resolution/2.0) 
        
	return px , py


   #Odom Callback which returns values of pose orientation which we translate to self.pose
    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
    
    #Using Laser scanner callback to detect obstacles around the robot
    def scan_callback(self,msg):
         self.obs_distance = []
	 self.obsDistanceArray = []

         for i in range(360):
		if msg.ranges[i] >= self.LIDAR_ERR:
			self.obsDistanceArray.append(msg.ranges[i])
             		if i==0 or i==30 or i==330 :
             	 		self.obs_distance.append(msg.ranges[i])

         self.minDistance = min(self.obsDistanceArray)
	 self.minDistanceAngle = self.obsDistanceArray.index(self.minDistance)

    def update_grid(self, gx, gy,increase):


	if(self.grid[gx][gy]+increase <=999):
            self.grid[gx][gy] = self.grid[gx][gy]+increase
	else:
	    self.grid[gx][gy] = 999

    #Image callback for retrieving the view from the front of the robot.
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #cv2.circle(self.image, (self.target_x, self.target_y), 20, (255,0,255), -1)

    #Depth callback for retrieving the depth of pixels
    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        

    #Print the map    
    def print_searchGrid(self):
        x = self.size-1

        while x>=0:
            y = self.size-1
            string = ""
            while y>=0:
		
		temp = str(self.grid[x][y])
		if(len(temp) ==1):
			temp = "00"+temp+" "
		elif(len(temp)==2):
			temp = "0"+temp+" "
		elif(len(temp)==3):
			temp = temp+" "
                string += temp
                y -= 1
            print(string)
            x -= 1

rospy.init_node('draft2')
draft2 = draft2()
rospy.spin()
