import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Obstacle():
    def __init__(self):
        self.LIDAR_ERR = 0.05
        self._sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # self.obstacle()
        self.distance = 0
        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
       


    def scan_callback(self,msg):
        # msg = rospy.wait_for_message("scan", LaserScan, self.scan_callback)
        rospy.loginfo('begin!')
        self.scan_filter = []
        for i in range(360):
            if i <= 15 or i > 335:
                if msg.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter.append(msg.ranges[i])
        self.distance = min(self.scan_filter)
    
    def run(self):
        self.twist = Twist()
        while not rospy.is_shutdown():
            # self.get_scan()
            
            if self.distance < 0.5:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.3
                self._cmd_pub.publish(self.twist)
                rospy.loginfo('Stop!')

            else:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
                rospy.loginfo('distance of the obstacle : %f', self.distance)

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
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
