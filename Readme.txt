LIMO Robot pothole Detector/Counter


Installing ROS2 Humble desktop
    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Installing LIMO Simulation native
    https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup

Creating the ROS workspace and clone the repository
    1. Create new workspace(or use existing)
        mkdir eos2_ws
    2. Create 'src' folder
        cd ros2_ws
        mkdir src
    3. Clone the repository to src folder
        git clone https://github.com/Prabath-Ranathunga/CMP9767_Assignment.git
Build and Source the Packege
    1. Build the packege
        cd ~/ros2_ws
        colcon build --symlink-install
    2. Source the packege
        . install/setup.bash

Dependancies and Libraries that used 
    1. Libraries used
        opencv-python
        numpy
        math
    2. Dependancies that used
        rclpy
        CvBridge

Start the Gaazebo Simulator
    cd limo_ros2/
    source install/setup.bash

    1. Pothole simple world
        ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_gazebosim/worlds/potholes_simple.world 
    2. Pothole(realistic) worls
        ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_gazebosim/worlds/potholes.world 

Starting RVIZ2 graphical visualiser
    rviz2 -d src/limo_gazebosim/rviz/urdf.rviz 

ROS2 Commands

ROS2 launch all nodes at once
    $ cd ../ros2_ws/CMP9767_Assignment/assignment_pathole/launch
    ros2 launch pathole_detector.launch.py 

Or to Run the nodes one by one,
    ros2 run assignment_pathole mover
    ros2 run assignment_pathole pothole_detector
    ros2 run assignment_pathole pothole_detector_real
    ros2 run assignment_pathole pothole_counter

To visualise the pothole detection in real-time,
    1. Change the color image topic to '/limo/depth_camera_link/image_detect'
    2. Add the marker with the topic of '/marker'



Summary
1. Moving robot
    Using Laserscan subscriber to get lidar data to detect objects infront of the bobot and uses Twist to send robot movement commands. And it only uses 60 degree of detection range from the front and 0.5m obstacle detection distance. if robot detect obstacle it turns and if not it continue to move forward.

2. Detecting potholes
    Using subscribtion to color camera, depth camera, camera info to detect and get the pothole locations respect to the depth-link. First method is to use color
    segmentation (using upper and lower hsv color range) to detect potholes in the simple world and second method is to trained haar cascade model to detect potholes i the reallistic map. Then define a minimum detection
    area and get centroid of that pothole and use that to map out it from depth frame and get distance value to each potholes. Also calculate the approximate value of pothole area in cm^2 in certain distance range. 
    Using projectPixelTo3dRay get the actual coordinates of each detected pothole locations respect to the depth_link frame.
    Get the robot depth_link position and orientation data from the TF tree relative to the odeom frame since it's a fixed frame. Then using 'euler_from_quaternion' calculate the yaw value of the depth_link frame. With that and using some calculations 
    calculate the each detected pothole coordinates (only x and y). Then publish it using Marker for visualization in rviz and to calculate the total number of potholes.

3. Pothole counter
    Get the each pothole coordinates by subcribing to Marker topic and then calculating total number of potholes. Since it detect same pothole again and again and there are some coordinate of the centroid changes when it  detect from different angle, 
    It uses a threshold value to detect whether it's same pothole or not. If it's same pothole it only keep one coordinate and ignore other coordinates. If not it saves coordinates in to array and the log how many coordinates in the array. And it output each pothole coordinates once it added. And create text file with list of coordinates at the end.