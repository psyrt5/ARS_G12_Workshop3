#!/usr/bin/env python

from math import pi, sqrt, radians

import rospy
import tf

import matplotlib.pyplot as plt
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)
        
	#Setting all values to 0 and setting angular velocity to 0.2rad/s
	self.pose = Pose2D()
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance = 0
	self.r = rospy.Rate(30)
        self.xpos=[]
        self.ypos=[]
        self.turn = Twist()
        self.stop_turn = Twist()
        self.stop_turn.angular.z = 0.0
        self.turn.angular.z = 0.2

        # self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.vel = Twist()
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
	self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        
        
        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # show trajectory 
            self.plotting()



    def run(self):

	#Loop through to complete single square trip.
        i = 0
        while i<4:
	
   	    #Travel 0.2m/s whilst odom distance traveled is less than 1m.
            while(round(self.distance,2)<1):
               
                move_cmd = Twist()
                move_cmd.linear.x = 0.2
                move_cmd.linear.y = 0
                move_cmd.linear.z = 0      
                self.vel_pub.publish(move_cmd)
                self.r.sleep()

            self.stop()
              
	    #Calculating the target theta value by adding rad90 to current pose
 	    current_rotation = self.pose.theta
	    new_rotation = current_rotation + radians(90)

            if i == 0 or i == 1 : 
                while (abs(round(self.pose.theta,2)) < round(new_rotation,3)):

		    #As pose.theta is capped at 3.14, if the target value is greater than this it must be adjusted.
		    if(new_rotation>3.149):
			if(abs(round(self.pose.theta,2))==3.14):
				new_rotation = 3.149 - (new_rotation-3.149)
			
                    self.vel_pub.publish(self.turn)
                    self.r.sleep()

                self.stop
                
            if  i==2 or i==3 :
                while (abs(round(self.pose.theta,2)) > abs(round(new_rotation,3))):

		    if(new_rotation>3.149):
			if(abs(round(self.pose.theta,2))==3.14):
				new_rotation = 3.149 - (new_rotation-3.149)		    

	            self.vel_pub.publish(self.turn)
                    self.r.sleep()

                self.stop

            i = i+1

        self.stop
        rospy.loginfo("Action Complete.")

    # Plotting the  route of the robot
    def plotting(self):
        for i in range(len(self.xpos)):
            
            #print(self.xpos[i])
            plt.plot(self.xpos[i], self.ypos[i],'rx')
       
        plt.savefig("workshop2task4.png")
        plt.show()    
                            

    #Stopping sequence which resets all velocities to 0
    def stop(self):
        self.current_x = self.pose.x
        self.current_y = self.pose.y
        
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

    
    #Odom Callback which returns values of pose orientation which we translate to self.pose
    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        
	#Calculating distance using pythagoras.
        self.distance = sqrt(((self.pose.x - self.current_x)**2)+((self.pose.y - self.current_y)**2))       

        self.xpos.append(self.pose.x)
        self.ypos.append(self.pose.y)  
    
if __name__ == '__main__':
    whatever = Turtlebot()