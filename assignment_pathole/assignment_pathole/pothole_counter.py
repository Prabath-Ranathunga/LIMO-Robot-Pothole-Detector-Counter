"""
Detected pothole counter
Get the x and y coordinates of the detected pothole by suscribing to marker topic
Save each coordinates and check whether it's in other coordinates threshold range
Keep only that not the each ones range
Count the pathole coordinate sets
Give x and y coordinates for each pothole
"""
# Python libs
import rclpy
from rclpy.node import Node

# ROS Messages
from visualization_msgs.msg import Marker

# Numpy
import numpy as np

class PotholeCounter(Node):
    def __init__(self):
        super().__init__('pothole_counter')
        # Create a subscriber to Marker to get object's coordinates
        self.subscription = self.create_subscription(Marker, '/marker', self.marker_callback, 10)

        self.coordinates = []
        # Setup threshold value to avoid repeat counting the same pothole again
        self.threshold = 0.08

    def marker_callback(self, msg):
        # The x and y coordinates from the marker message that published from pothole_detection 
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Check if the coordinate is within the threshold of any existing coordinate
        if self.is_within_threshold(x, y):
            return
        
        # Add the coordinate to the array
        self.coordinates.append((x, y))
        
        # Log the count of x and y sets in the array
        self.get_logger().info(f"Pothole Count : {len(self.coordinates)}")
        # Log the newly added coordinates
        self.get_logger().info(f"New Pothole Coordinates : {x, y}")

    def is_within_threshold(self, x, y):
        for coord in self.coordinates:
            if self.calculate_distance(coord[0], coord[1], x, y) < self.threshold:
                return True
        return False

    def calculate_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def main(args=None):
    rclpy.init(args=args)
    pothole_counter = PotholeCounter()
    try: 
        rclpy.spin(pothole_counter)
    except KeyboardInterrupt:
        # Save pothole coordinates to text file
        filename = "Pothole_Coordinates.txt"
        with open(filename, 'w') as file:
            for count, (x, y) in enumerate(pothole_counter.coordinates, start=1):
                file.write(f"Pothole {count} : ({x} {y})\n")
        print(f"Coordinates saved to {filename}")

    pothole_counter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()