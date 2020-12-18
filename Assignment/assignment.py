#!/usr/bin/env python
from __future__ import print_function
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
from darknet_ros_msgs.msg import BoundingBoxes
import cv2, cv_bridge
from sensor_msgs.msg import Image, LaserScan


class observanceTest:
    
    def __init__(self):

	#Subscribing and Publishing to the correct topics
	self.bridge = cv_bridge.CvBridge()
	self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
	self.depth_sub = rospy.Subscriber('camera/depth/image_raw', Image, self.depth_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_callback)

        #Initialising the basic variables.
        self.pose = Pose2D()   
        self.twist = Twist()
	self.r = rospy.Rate(30)

        self.size = 20
	self.origin = -10
        self.current_x = 0
        self.current_y = 0
        self.current_theta =0
	self.nullGoalCount = 0

	#The laser callback variables
	self.LIDAR_ERR = 0.05
	self.searchLaserDistances = []

	#Yolo Callback Variables
        self.yoloname = ''
        self.center_y=0
        self.center_x=0

	#The process control variables
	self.goalSearchInProgress = False
	self.obstacleDetected = False	
	self.objectDetected = False
	self.goalReached = False
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
	
	#Object Thresholds
        self.scale = 4

        self.fh_threshold_lower = 10000
        self.fh_threshold_upper = 200000

        self.mb_threshold_lower = 20000
        self.mb_threshold_upper = 200000

        self.gb_threshold_lower = 30000
        self.gb_threshold_upper = 500000

        #Boolean variables for keeping track of which objects were found
        self.fh_found = False
        self.gb_found = False
        self.mb_found = False
        self.n5_found = False

        #Positional values locating the target on screen
        self.target_x = 0
        self.target_y = 0

	
	#Sleeping shortly to allow all values to set correctly before start.
	self.complete = False
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


	#Explore the scene whilst the user hasn't shut the robot down and the exploration hasnt been completed
        while not rospy.is_shutdown() and self.complete ==False:
            self.explore(ac)




#****************************************************************************** EXPLORE FUNCTIONS *************************************************************************
    
    #The function responsible for exploring the map. Obtaining goal positions and sending the robot there.
    def explore(self,ac):

	goalReached = False

	#Get a goal nearby unless the previous four goals have all been unsuccessful then get a goal which will escape the current position 
	if(self.nullGoalCount < 4):
		xGridGoal,yGridGoal = self.explore_getGoal()
	else:
		xGridGoal,yGridGoal = self.explore_escapeCurrentPosition()


	goalP_x , goalP_y = self.to_world(xGridGoal,yGridGoal,self.origin,self.origin,self.size,self.size,1)
	goalReached = self.explore_moveToGoal(ac,goalP_x, goalP_y,xGridGoal,yGridGoal)
	self.print_searchGrid()

	#If the robot moved sucessfully to the exploration goal
        if (goalReached):
	    self.update_searchGrid(xGridGoal,yGridGoal,100)
	    rospy.loginfo("Goal Met")
        else:
            rospy.loginfo("Goal Not Met")
    

    def explore_getGoal(self):

	#Get the current grid location of the robot gx and gy
	gx , gy = self.to_grid(self.pose.x,self.pose.y,self.origin,self.origin,self.size,self.size, 1)
  	
	search_index = 0
	self.goalSearchInProgress =True

	while(True):
		
		search_index = search_index +1
		gxMinus = gx-search_index
		gxPlus = gx+search_index
		gyMinus = gy-search_index
		gyPlus = gy+search_index

	
		#To ensure cells outside of the array size aren't checked
		if(gxMinus<0):
			gxMinus = 0
		if(gxPlus>self.size-1):
			gxPlus = self.size-1
		if(gyMinus<0):
			gyMinus = 0
		if(gyPlus>self.size-1):
			gyPlus = self.size-1

		
		#Start checking the bottom left cell from the current grid
		x = gxMinus
		y = gyMinus
		
		x_change = 0
		y_change = 1
		
		ringSearched = False

		#Loop until the ring of grid squares surrounding the current location have been searched.
		while(ringSearched == False):

			#This set of ifs is to establish the increase/decrease in the x/y values on the next iteration
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

			#If the grid cell has a value of less than 15 then use it as the goal location
			if(self.grid[x][y]<15 and x!=gx and y!=gy-1):
				self.goalSearchInProgress = False				
				return (x , y)		
				
			x = x + x_change
			y = y + y_change
		
		#When the robot has search every location these search parameters will be set to the corners of the map
		#Here if this is true, the search is complete and the self.complete parameter is set to true
		if(gxMinus==0 and gxPlus == self.size-1 and gyMinus == 0 and gyPlus == self.size-1):
			self.goalSearchInProgress = False
			print("Search Complete")
			self.complete = True
			return (gx,gy)
	
    #A method to pick a grid location for the robot which it has already explored, in order to get the robot to escape its current bad position
    def explore_escapeCurrentPosition(self):
	self.goalSearchInProgress =True
	maxGridVal= 100

	#print("Null Goal count = ",self.nullGoalCount)
	for i in range (5):
		for x in range(len(self.grid)):
			for y in range(len(self.grid[x])):
				if(self.grid[x][y]>0 and self.grid[x][y]<maxGridVal):
					self.goalSearchInProgress =False
					return (x,y)

		maxGridVal +=100 #If there were no avaliable locations. Increase the range of grid values accepted as a new location

	#If there are no options to escape then continue from the current position
	return (self.explore_getGoal())

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

        #rospy.loginfo("Sending goal location = " ,xGridGoal , yGridGoal)
        ac.send_goal(goal)

	self.goalRunning = True
	self.timerRunning = False
	self.notMoved =False
	self.stopTimer =False
	self.objectDetected = False

	#While the robot is making its way to the goal/exploring and that goal state is active
        while (ac.get_state()<3 and self.goalRunning == True):
	    
	    #Update the searchGrid
	    self.explore_updateSearchGrid()

	    #Continuously call obstacle Detection and objectRecognition to move the robot toward any detected object
	    while self.obstacle_detection() == False and self.objRec_checkForObjects(ac):
			if self.goalRunning:
				ac.cancel_all_goals()
				self.goalRunning = False
				print("I've found something")
			
		
	    if(self.obstacle_detection() == False):
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
			self.update_searchGrid(xGridGoal, yGridGoal,999)
			self.nullGoalCount +=0.5
			return False
			  
			
			    
	    else:
		#An obstacle has been detected. Cancel current goal and perform obstacle avoidance
		self.goalRunning = False
		ac.cancel_goal()
		self.obstacle_avoidance()
		self.grid[xGridGoal][yGridGoal] +=5
		self.nullGoalCount +=1
	

	#If the timer is running once the goal has finished, cancel the timer
	if(self.timerRunning):
		my_Timer.cancel()
		self.timerRunning = False

	self.goalRunning = False


	# If the goal grid hasn't been explored return false
	if(self.grid[xGridGoal][yGridGoal]<15):	
		return False
	else:
		self.nullGoalCount = 0
		return True


    #Updating the search grid to increase the cell value which the robot is looking at
    def explore_updateSearchGrid(self):

	#There are 3 lasers one straight forward (0), one to the left of the field of vision (1) and one to the right(2)
	laser_num =0
	laser_angle = 0

	#This if statement means the search grid is only updated periodically so we only get an update when the robot has made significant movement
	if(abs(self.current_x -self.pose.x) >0.5 or abs(self.current_y - self.pose.y) >0.5 or abs(self.current_theta -self.pose.theta) >0.5):
		self.current_theta = self.pose.theta

		#Whilst we are getting 3 distance values back and haven't exceeded the laser number	
		while(len(self.searchLaserDistances)>2 and laser_num<3):                
		
			obs_dist = self.searchLaserDistances[laser_num]

			#Accounting for edge cases
			if(obs_dist ==0):
				obs_dist = 0.1
			if(isinf(obs_dist)):
				obs_dist = 3.5
	
			#Adjusting laser_angle based on which laser is being used
			if(laser_num ==1):
				laser_angle = -0.523599 # 30 - THESE MUST BE SET TO THE SAME VALUE OF THE LASER CALLBACK
			elif(laser_num==2):
				laser_angle = 0.523599
	
			#Calibrating the angular offset to calculate the future distance in x/y correctly
			angleOffset = self.pose.theta + 1.5708+laser_angle
			if (angleOffset >3.14159):
				angleOffset = - 3.14159 + (angleOffset - 3.14159)

			#Using basic trigonometry to calculate the change in x/y from the current position to the furthest location the laser can reach
			xDistanceToGoal = -sin(angleOffset)*obs_dist
			yDistanceToGoal = cos(angleOffset)*obs_dist

			#Looping through 0.1m intervals down the laser and updating which grid each point is in.
			while(obs_dist>0):
	
				#Getting the x and y coordinate at this interval down the line
				xDistanceAlongLine = -sin(angleOffset)*obs_dist
				yDistanceAlongLine = cos(angleOffset)*obs_dist

				x = self.pose.x + xDistanceAlongLine
				y = self.pose.y + yDistanceAlongLine
	
				#Updating that grid location to add 1 to its value
				gx,gy = self.to_grid(x,y,self.origin,self.origin,self.size,self.size,1)
				self.update_searchGrid(gx, gy,1)
				obs_dist = obs_dist-0.1
			
			laser_num = laser_num+1


    #This function is repeated every several seconds to check if the robot is stuck in a position or the goal is ineffective
    def explore_hasMoved(self):

	if(self.objectDetected == True or self.obstacleDetected == True or self.goalSearchInProgress==True or abs(self.current_x -self.pose.x) >0.05 or abs(self.current_y - self.pose.y) >0.05):	
		self.notMoved = False #MOVED
	else:
		self.notMoved = True #NOT MOVED

	self.current_x = self.pose.x
	self.current_y = self.pose.y
	self.stopTimer=True
 

#****************************************************************************** OBSTACLE AVOIDANCE FUNCTIONS *******************************************************************************

    #Setting obstacleDetected variable and returning the relevant boolean
    def obstacle_detection(self): 
	    if self.minDistance < 0.25:
		self.obstacleDetected =True
		return True
	    else:
		self.obstacleDetected =False
	        return False
	    
   

    #The function whereby if an obstacle has been detected then the correct movement is made to move the robot away
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
        	        #rospy.loginfo('Avoiding Obstacle!')

		    else:
			self.obstacleDetected = False
	        
	    

#****************************************************************************** OBJECT RECOGNITION FUNCTIONS ***********************************************************************************

    def objRec_checkForObjects(self,ac):
	self.objRec_updateImages()

	#Attempt to detect the fire hydrant using those colour thresholds if the hydrant hasn't been detected already.
	if self.fh_found == False and self.objRec_detect(self.objRec_maskImage(self.resized_image, self.fh_lower, self.fh_upper), self.fh_threshold_lower, self.fh_threshold_upper, ac):
		print("Moving to FH", self.distance)
		#Move towards the object until a small distance away then stop the robot and print message.
		if self.objRec_moveToObject():
			self.fh_found = True
			print("Fire Hydrant Found")
			self.stop()
			time.sleep(3)
			self.distance = 20
		return True
        #Attempt to detect the mail box using those colour thresholds if the box hasn't been detected already.   
        elif self.mb_found == False and self.objRec_detect(self.objRec_maskImage(self.resized_image, self.mb_lower, self.mb_upper), self.mb_threshold_lower, self.mb_threshold_upper, ac):
		print("Moving to MB", self.distance)
		
		#Move towards the object until a small distance away then stop the robot and print message.
		if self.objRec_moveToObject():
			self.mb_found = True
			print("Mailbox Found")
			self.stop()
			time.sleep(3)
			self.distance = 20
		return True
 
        #Attempt to detect the green box using those colour thresholds if the box hasn't been detected already
        elif self.gb_found == False and self.objRec_detect(self.objRec_maskImage(self.resized_image, self.gb_lower, self.gb_upper), self.gb_threshold_lower, self.gb_threshold_upper, ac):
		print("Moving to GB", self.distance)
		
		#Move towards the object until a small distance away then stop the robot and print message.
		if self.objRec_moveToObject():
			self.gb_found = True
			print("Green Box Found")
			self.stop()
			time.sleep(3)
			self.distance = 20
		return True
	
	#Checking if the yolo callback has detected the number 5 box. if so move towards it.
        elif self.n5_found == False and self.yoloname == 'number 5':
            	self.target_x = self.center_x
            	self.target_y = self.center_y
            	self.distance = self.depth[self.target_y, self.target_x]
		
		#Move towards the object until a small distance away then stop the robot and print message.
            	if self.objRec_moveToObject():
                	self.n5_found = True
 	                print("Number 5 Found")
        	        self.stop()	
			time.sleep(3)
                	self.distance = 20
	    	return True

	#No object was detected
	return False

    def objRec_updateImages(self):
		(self.h, self.w) = self.image.shape[:2]
		self.resized_image = cv2.resize(self.image, (self.w/self.scale,self.h/self.scale))
		self.depth_resized = cv2.resize(self.depth, (self.w/self.scale,self.h/self.scale))

		resized_target = self.resized_image

		cv2.circle(resized_target, (self.target_x, self.target_y), 20, (255,0,255), -1)

		output = cv2.resize(resized_target, (480,270))
		cv2.imshow("Targeting", output)
		cv2.waitKey(1)

          
    def objRec_moveToObject(self):
        # If the distance from the object to the robot is greater than 1 meter,
        # then we need to move the robot towards the direction of the object. 
        if self.distance >= 1.5:
            err = self.target_x - (self.w/2)/self.scale
            self.twist.angular.z = -float(err) / 500
            print(self.twist.angular.z)
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
    def objRec_detect(self, mask, lower_threshold, upper_threshold,ac):
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
            return True
        return False
    
    #Given an image, and a set of bounds, mask an image
    #to retrieve a grayscale image.
    def objRec_maskImage(self, image, lower, upper):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        return mask
#****************************************************************************** CALLBACK FUNCTIONS AND UTILITY FUNCTIONS ***********************************************************************

    #A function to stop the movement of the robot 
    def stop(self):
	print("Stopping")
	self.twist.angular.z = 0
	self.twist.linear.x = 0
	self.cmd_pub.publish(self.twist)

    #Converting the coordinate position px/py to a grid position gx/gy
    def to_grid(self, px, py, origin_x, origin_y, size_x, size_y, resolution):
        gx = int(floor((px-origin_x)/resolution))
        gy = int(floor((py-origin_y)/resolution))
	return gx , gy

    #Converting a grid position gx/gy to a coordinate position px/py
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
         self.searchLaserDistances = []
	 self.obsDistanceArray = []

         for i in range(360):
		if msg.ranges[i] >= self.LIDAR_ERR:
			self.obsDistanceArray.append(msg.ranges[i])
             		if i==0 or i==30 or i==330 :
             	 		self.searchLaserDistances.append(msg.ranges[i])


	 #Getting the distance of the closest obstacle and getting the angle that nearest obstacle is from the robot
         self.minDistance = min(self.obsDistanceArray)
	 self.minDistanceAngle = self.obsDistanceArray.index(self.minDistance)

    
    #A function to update the searchGrid by a set numerical value "increase"
    def update_searchGrid(self, gx, gy,increase):

	#Capping the maximum value a cell can get to 999
	if(self.grid[gx][gy]+increase <=999):
            self.grid[gx][gy] = self.grid[gx][gy]+increase
	else:
	    self.grid[gx][gy] = 999


    #Image callback for retrieving the view from the front of the robot.
    def image_callback(self, msg):
		self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		#print("Image Updated")
        #cv2.circle(self.image, (self.target_x, self.target_y), 20, (255,0,255), -1)

    #Depth callback for retrieving the depth of pixels
    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # YOLO callback for detecting number 5 object
    def yolo_callback(self, msg):
        #print msg.bounding_boxes[0].Class
        if msg.bounding_boxes[0].Class == 'number 5':

            self.yoloname = msg.bounding_boxes[0].Class
            yolo_position = msg.bounding_boxes[0]
            xmin = yolo_position.xmin
            xmax = yolo_position.xmax
            ymin = yolo_position.ymin
            ymax = yolo_position.ymax
            self.center_x = xmin+(xmax-xmin)/2
            self.center_y = ymin+(ymax-ymin)/2


    #Print the colour coordinated version of the search grid.    
    def print_searchGrid(self):
        COLORS = ['\033[1;47m  \033[0m',  # White
		  '\033[1;42m  \033[0m',  # Green
                  '\033[1;46m  \033[0m',  # Cyan
                  '\033[1;44m  \033[0m',  # Blue
                  '\033[1;43m  \033[0m',  # Yellow
                  '\033[1;41m  \033[0m']  # Red
        for i in range(self.size):
            for j in range(self.size):

		#Printing the appropriate colour dependant on the grid value
		temp = int(self.grid[i][j])
		if temp == 0:
                	print('{}'.format(COLORS[0]),end="")
		elif 0<temp<=250:
			print('{}'.format(COLORS[1]),end="")
		elif 250<temp<=500:
			print('{}'.format(COLORS[2]),end="")
		elif 500<temp<=750:
			print('{}'.format(COLORS[3]),end="")
		elif 750<temp<=998:
			print('{}'.format(COLORS[4]),end="")
		else:
			print('{}'.format(COLORS[5]),end="")
            print(' ')

rospy.init_node('observanceTest')
observanceTest = observanceTest()
rospy.spin()
