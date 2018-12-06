# PC Setup for Gazebo Simulation

# F1tenth - PC Setup for Gazebo Simulation

The F1tenth platform can be simulated using Gazebo. 

## Install Dependencies

* Once all the required installations have been performed, install the following:

serial:

``` 
sudo apt-get install ros-kinetic-serial
```
controller_manager:

``` 
sudo apt-get install ros-kinetic-controller-manager
```
gazebo_ros_control:

``` 
sudo apt-get install ros-kinetic-gazebo-ros-control
```
joint_state_controller:

``` 
sudo apt-get install ros-kinetic-joint-state-controller 
```
effort_controllers

``` 
sudo apt-get install ros-kinetic-effort-controllers
```

## Build simulation packages
    # Copy race folder to the src folder under current workspace
    cp -r ~/F1tenth_BU/race_simulator ~/catkin_ws/src/
    # Build the pacakges
    source /opt/ros/kinetic/setup.bash
    cd ..
    catkin_make
    source devel/setup.bash

### Run the simulated car in Gazebo

Launch the simulation environment by

    source ~/catkin_ws/devel/setup.bash
    # Multiple environments with different suffix
    roslaunch racecar_gazebo racecar_cones.launch
    # or
    roslaunch racecar_gazebo racecar_tunnel.launch
    # or
    roslaunch racecar_gazebo racecar_walker.launch
<p float="left">
<img src="./env_pics/racecar_cones.jpg" alt="Cones Environment" width="250"/>
<img src="./env_pics/racecar_tunnel.jpg" alt="Tunnel Environment" width="250"/>
<img src="./env_pics/racecar_walker.jpg" alt="Walker Environment" width="250"/>
<br>
<em>Environment from left to right: cones, tunnel and walker</em>
</p>



    
Keyboard/Path_tracking control node

    <node pkg="racecar_control" type="keyboard_teleop.py" name="keyboard_teleop" output="screen" launch-prefix="xterm -e"/>
    <node pkg="racecar_control" type="path_tracking.py" name="path_tracking" output="screen" launch-prefix="xterm -e"/>
