"""
A LIMO robot autonomous mover
It goes straignt until it detect object, mainly wall around the map
within given range of +30 and -30 and within range of 0.5m distance
then turn right and check obstacle again and again
"""
# Python libs
import rclpy
from rclpy.node import Node

# ROS Messages 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        # Create publisher to control the robot
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        # Create a subscriber to Laserscan topic to listen laser scans
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)
    
    def laserscan_callback(self, data):
        # Callback called any time a new laser scan become available
        # Setup obstacle detection ranges
        min_dist = min(data.ranges[int(len(data.ranges)/2) -30 : int(len(data.ranges)/2) +30])
        t = Twist()
        if min_dist < 0.5:          # Obstacle detection range
            t.angular.z = -0.5       # Obstacle avoidance turning angle  
        else:
            t.linear.x = 0.3
        self.publisher.publish(t)


def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
