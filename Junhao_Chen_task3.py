import rospy
from geometry_msgs.msg import Twist
from math import pi

class OutAndBack():
    def __init__(self):
        rospy.init_node('out_and_back', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rate =10
        r = rospy.Rate(rate)
        linear_speed = 0.2
        goal_distance = 1.0
        linear_duration = goal_distance / linear_speed

        
     
        for i in range(6):
            move_cmd = Twist()
            move_cmd.linear.x = linear_speed            
            # Move forward for a time to go the desired distance
            ticks = int(linear_duration * rate)            
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
            
            # Now rotate left roughly     
            move_cmd = Twist()
            move_cmd.angular.x = 0
            move_cmd.angular.y = 0
            move_cmd.angular.z = pi/2
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

        self.cmd_vel.publish(Twist())
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")
