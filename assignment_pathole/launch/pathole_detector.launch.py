from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "assignment_pathole",
            executable = "mover",
            name = 'mover_node'
        ),
        Node(
            package = "assignment_pathole",
            executable = "pothole_detector",
            name = 'detector_node'
        ),
        Node(
            package = "assignment_pathole",
            executable = "pothole_counter",
            name = 'counter_node',
            output = 'log'
        )
    ])