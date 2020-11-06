#!/usr/bin/env python

import tf
import rospy
import random

from nav_msgs.msg import Odometry
from math import pi, sqrt, radians
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Pose2D


class Obstacle():
    def __init__(self):
        #Initialising subscribers and publishers
        self._sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
      	self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
	
	#Initialising all variable values
	self.pose = Pose2D()
	self.r = rospy.Rate(30)
	self.LIDAR_ERR = 0.05
 	self.obs_distance = 0
        self.distance = 0
	self.wall_detect = True
   	self.right_distance = 0.0
	self.current_x =0.0
	self.current_y =0.0


	#Setting Twist angular velocity to 0.2rad/s
        self.turn = Twist()
        self.stop_turn = Twist()
        self.stop_turn.angular.z = 0.0
        self.turn.angular.z = 0.2


        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
       


    def run(self):

        self.twist = Twist()
        while not rospy.is_shutdown():
            
	    #Reset current_x/y to handle initial starting positions
	    self.r.sleep()
	    self.current_x = self.pose.x
            self.current_y = self.pose.y


	    #Travel 0.2m/s whilst odom distance traveled is less than 3m.
            while(round(self.distance,2)<3):
		
		#If an object is <0.5m away, rotate 0.3rad/s
            	if self.obs_distance < 0.5:
	                self.twist.linear.x = 0.0
	                self.twist.angular.z = 0.3
	                self._cmd_pub.publish(self.twist)
			
			#Updated current_x/y to handle new starting position
	      	        self.current_x = self.pose.x
	                self.current_y = self.pose.y
	                print('Obstacle Detected - Rotating')

		elif self.wall_detect ==True:
			print("Wall Detected")

			if self.right_distance < 0.5:
                    		self.twist.angular.z = ((1-self.right_distance)-0.5)*((1-self.right_distance)-0.5)*5
                	else:
                    		self.twist.angular.z = ((1-self.right_distance)-0.5)*abs(((1-self.right_distance)-0.5))*3
			
			self.twist.linear.x = 0.2			
			self._cmd_pub.publish(self.twist)

       	        else:
			self.twist.linear.x = 0.2
                	self.twist.angular.z = 0.0
			self._cmd_pub.publish(self.twist)

		
	    #Calculating the target theta value by adding random angle to current pose
	    new_rotation = self.pose.theta + radians(random.randint(0, 360))
		
	    
	    #print(new_rotation, "  -  ",self.pose.theta)
            while (abs(round(self.pose.theta,2)) < round(new_rotation,3)):
		print("Travelled 3m - Rotating")
 		#As pose.theta is capped at 3.14, if the target value is greater than this it must be adjusted.
		if(new_rotation>3.149):
			if(abs(round(self.pose.theta,2))==3.14):
		 		new_rotation = 3.149 - (new_rotation-3.149)
              
		self._cmd_pub.publish(self.turn)
                self.r.sleep()
                self.stop
            

        self.stop
        rospy.loginfo("Action successfully.")



    def stop(self):

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self._cmd_pub.publish(self.twist)
        rospy.sleep(1)



    def scan_callback(self,msg):
        self.scan_filter = []
	minRightDistance = 1
	count=0

        for i in range(360):
	    if i>=250 and i<=290:
		if msg.ranges[i] < 1:
			count=count+1
			if msg.ranges[i]<minRightDistance:
				minRightDistance = msg.ranges[i]
			
            if i <= 15 or i > 335:
                if msg.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter.append(msg.ranges[i])
	
	if(count>35):
		self.wall_detect = True
	else:
		self.wall_detect = False

        self.right_distance = minRightDistance
        self.obs_distance = min(self.scan_filter)

  
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
    


def main():
    rospy.init_node('turtlebot_scan')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
