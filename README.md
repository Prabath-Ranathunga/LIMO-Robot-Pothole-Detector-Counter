# LIMO Robot Pothole Detector/Counter
Simple LIMO robot autonomous drive, pothole detection and pothole counter


## Workspace and Simulation enviroment

### Installing ROS2 Humble
[ROS2 Humble instalation tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS Humble desktop 

### Installing LIMO simulation 
[Installing LIMO simulation](https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup) using native version

### Creating the ROS workspace and clone the repository
    * Create new workspace(or use existing)
        ```bash
        mkdir eos2_ws
        ```
    * Create 'src' folder
        ```bash    
        cd ros2_ws
        mkdir src
        ```
    * Clone the repository to src folder
        ```bash
        git clone https://github.com/Prabath-Ranathunga/CMP9767_Assignment.git
        ```

### Build and Source the Packege
    * Build the packege
        ```bash
        cd ~/ros2_ws
        colcon build --symlink-install
        ```
    * Source the packege
        ```bash
        . install/setup.bash
        ```

### Start the Gaazebo Simulator
    ```bash
    cd limo_ros2/
    source install/setup.bash
    ```

    * Pothole simple world
        ```bash
        ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_gazebosim/worlds/potholes_simple.world 
        ```
    * Pothole(realistic) worls
        ```bash
        ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_gazebosim/worlds/potholes.world 
        ```

## Starting RVIZ2 graphical visualiser
    ```bash
    rviz2 -d src/limo_gazebosim/rviz/urdf.rviz 
    ```

## ROS2 Commands

### ROS2 launch all nodes at once
    ```bash
    $ cd ../ros2_ws/CMP9767_Assignment/assignment_pathole/launch
    ros2 launch pathole_detector.launch.py 
    ```

### Or to Run the nodes one by one
    ```bash
    ros2 run assignment_pathole mover
    ros2 run assignment_pathole pathole_detector
    ros2 run assignment_pathole pathole_counter
    ```

### To visualise the pothole detection in real-time,
    * Change the color image topic to `/limo/depth_camera_link/image_detect`
    * Add the marker with the topic of `/marker`



## Summary

### Moving robot
    Using `Laserscan` subscriber to get lidar data to detect objects infront of the bobot and uses `Twist` to send robot movement commands. And it only uses 60 degree of detection range from the front and 0.5m obstacle detection distance. if robot detect obstacle it turns and if not it continue to move forward.

### Detecting potholes
    Using subscribtion to color camera, depth camera, camera info to detect and get the pothole locations respect to the depth-link. First method is to use color segmentation (using upper and lower hsv color range) to detect potholes in the simple world and second method is to trained haar cascade model to detect potholes i the realistic map. Then define a minimum detection area and get centroid of that pothole and use that to map out it from depth frame and get distance value to each potholes. Using `projectPixelTo3dRay` get the actual coordinates of each detected pothole locations respect to the depth_link frame.
    Get the robot depth_link position and orientation data from the TF tree relative to the odeom frame since it's a fixed frame. Then using `euler_from_quaternion` calculate the yaw value of the depth_link frame. With that and using some calculations calculate the each detected pothole coordinates (only x and y). Then publish it using `Marker` for visualization in rviz and to calculate the total number of potholes.

### Pothole counter
    Get the each pathole coordinates by subcribing to `Marker` topic and then calculating total number of potholes. Since it detect same pothole again and again and there are some coordinate of the centroid changes when it  detect from different angle, It uses a threshold value to detect whether it's same pothole or not. If it's same pathole it only keep one coordinate and ignore other coordinates. If not it saves coordinates in to array and the log how many coordinates in the array. And it output each pothole coordinates.