# LIMO Robot Pathole Detect/Counter
LIMO robot autonomous drive and pathole detection


## Workspace and Simulation enviroment

## Installing ROS2 Humble and LIMO-ROS2 Simulation
[ROS2 Humble instalation tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS Humble desktop 

[Installing LIMO simulation](https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup) using native version

### Source the ROS2 enviroment

```bash
$ source ../limo_ros2/setup.bash
```
## Running ROS packege nodes seperatly
To run the nodes seperatly,
```bash
ros2 run assignment_pathole <executable>
```
the executables are
```bash
    mover
    pathole_detector
    pathole_counter
```