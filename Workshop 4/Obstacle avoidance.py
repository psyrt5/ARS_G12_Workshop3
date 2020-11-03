import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


ranges = []
def scan_callback(msg):
    global ranges
    ranges = msg.ranges
 

class Obstacle():
    def __init__(self):
        self.LIDAR_ERR = 0.05
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
       

    def get_scan(self):
        # msg = rospy.wait_for_message("scan", LaserScan, self.scan_callback)
        # pich up the LaserScan data
        msg = rospy.wait_for_message("scan", LaserScan)
        self.scan_filter = []
        for i in range(360):
            if i <= 15 or i > 335:
                if msg.ranges[1] >= self.LIDAR_ERR:
                    self.scan_filter.append(msg.ranges[i])
                    
        
    def obstacle(self):
        self.twist = Twist()
        while not rospy.is_shutdown():
            self.get_scan()
            
            if min(self.scan_filter) < 0.5:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.8
                self._cmd_pub.publish(self.twist)
                rospy.loginfo('Stop!')
                
            

            else:
                self.twist.linear.x = 0.3
                self.twist.angular.z = 0.0
                rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))
                
            self._cmd_pub.publish(self.twist)

        self.stop
        rospy.loginfo("Action successfully.")


    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self._cmd_pub.publish(self.twist)
        rospy.sleep(1)


def main():
    rospy.init_node('turtlebot_scan')
    try:
        Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()