#!/usr/bin/env python
import tf
import rospy
import random
import numpy as np

from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from math import pi, sqrt, radians , ceil
from geometry_msgs.msg import Pose2D, Point
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import *

class Workshop6:
    def __init__(self):
        #Subscribing and publishing to the relevant topics
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        
        #Initialising all pose variable values
        self.pose = Pose2D()   
        self.r = rospy.Rate(30)
        
        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")

    #Function which prints a menu option of where to set the next waypoint
    def choose_location(self):
        self.choice='q'

        rospy.loginfo("|-----------------------------------|")
        rospy.loginfo("|PRESS A KEY:")
        rospy.loginfo("|'0': Room 1-0 ")
        rospy.loginfo("|'1': Room 1-1 ")
        rospy.loginfo("|'2': Room 1-2 ")
        rospy.loginfo("|'3': Room 2-0 ")
        rospy.loginfo("|'4': Room 2-1 ")
        rospy.loginfo("|'5': Room 2-2 ")
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|WHERE TO GO?")

        #Setting the users choice to the variable self.choice
        self.choice = input()
            
    def run(self):

        # Declare the pairs of x,y coordinates for each room
        self.xRoom1_0 = -2.789
        self.yRoom1_0 = 3.141

        self.xRoom1_1 =-6.230
        self.yRoom1_1 = 3.998

        self.xRoom1_2 =-6.325
        self.yRoom1_2 = -0.511

        self.xRoom2_0 = 4.456
        self.yRoom2_0  = 1.533

        self.xRoom2_1 = 1.146
        self.yRoom2_1  = 4.568

        self.xRoom2_2 = 6.268
        self.yRoom2_2  = -1.240

	#Variable which is false until the goal is reached
        self.goalReached = False

	#Intialising variables for task 4
        self.current_x = self.pose.x
        self.current_y = self.pose.y
        self.current_gx = 0
        self.current_gy = 0
        
        self.size = 20
        array_size = self.size * self.size + self.size
        self.grid = [-1] * array_size


	#Running the initial choice menu
        self.choose_location()
        

        while not rospy.is_shutdown():
            self.move()
    
    #A function which continuously updates the goalReached variable
    def move(self):
        if (self.choice == 0):
            self.goalReached = self.moveToGoal(self.xRoom1_0, self.yRoom1_0)
            
        elif (self.choice == 1):
                self.goalReached = self.moveToGoal(self.xRoom1_1, self.yRoom1_1)

        elif (self.choice == 2):

                self.goalReached = self.moveToGoal(self.xRoom1_2, self.yRoom1_2)

        elif (self.choice == 3):

                self.goalReached = self.moveToGoal(self.xRoom2_0, self.yRoom2_0)

        elif (self.choice == 4):

                self.goalReached = self.moveToGoal(self.xRoom2_1, self.yRoom2_1)

        elif (self.choice == 5):

                self.goalReached = self.moveToGoal(self.xRoom2_2, self.yRoom2_2)

	#If no room selection was made
        if (self.goalReached):
            rospy.loginfo("Congratulations!")
            self.choose_location()
        #rospy.spin()
        else:
            rospy.loginfo("Hard Luck!")
            self.choose_location()
    
    #This function returns true/false if goal is met or not
    def moveToGoal(self,xGoal,yGoal):
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")


        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/
        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        #ac.wait_for_result(rospy.Duration(60))

        while (ac.get_state() != GoalStatus.SUCCEEDED):
            #If statement to limit consistently calling functions when the robot hasn't moved significantly
            if(round(self.pose.x,2) !=round(self.current_x,2) or round(self.pose.y,2) !=round(self.current_y,2)):
                self.to_grid(self.pose.x,self.pose.y,-8,-8,self.size,self.size,1)
                self.current_x = self.pose.x
                self.current_y = self.pose.y

        #SHOW VISUAL
        self.print_grid()
                
        rospy.loginfo("You have reached the destination")
        return True

   #If you are initialising the bottom left grid cell to 1,1 not 0,0
    def to_grid(self, px, py, origin_x, origin_y, size_x, size_y, resolution):
        gx = ceil((px-origin_x)/resolution)
        gy = ceil((py-origin_y)/resolution)

        #print(str(self.current_gx) + " : " + str(self.current_gy))
        #print(str(gx) + " : " + str(gy))

        if gy >= 0 and gx >= 0 and (gy != self.current_gy or gx != self.current_gx):
            self.update_grid(gx, gy)
            print("Updating")
            self.current_gx = gx
            self.current_gy = gy

    #Print the map    
    def print_grid(self):
        x = 0
        #Start with X axis
        while x < self.size:
            y = 0
            string = ""
            #Print Y axis
            while y < self.size:
                #If the Grid value = -1, then we haven't been there
                if self.grid[self.to_index(x,y,20)] == -1:
                    #Append the X character to the string message to be printed
                    string += "X"
                #If the Grid value isn't -1, then we have been there
                else:
                    #Append the space character to the string message to be printed
                    string += " "
                y += 1
            print(string)
            x += 1

   #If you are initialising the bottom left grid cell to 1,1 not 0,0
    def to_world(self, gx, gy, origin_x, origin_y, size_x, size_y, resolution):
        px = (gx*resolution + origin_x) + (resolution/2)
        py = (gy*resolution + origin_y) + (resolution/2)
        print (py ,px )

   #Odom Callback which returns values of pose orientation which we translate to self.pose
    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
    
    def update_grid(self, gx, gy):
        if self.grid[self.to_index(gx,gy,20)] == -1:
            #print(str(gx) + "x : " + str(gy) + "y - Is a new cell! :)")
            self.grid[self.to_index(gx,gy,20)] = 1
        else:
            pass
            #print(str(gx) + "x : " + str(gy) + "y - Is an old cell! :(")

    # Convert grid coordinate to map index
    # ------------------------------------------------------------------------------
    def to_index(self, gx, gy, size_x):
        return int(gy * size_x + gx)

rospy.init_node('workshop6')
workshop6 = Workshop6()
rospy.spin()