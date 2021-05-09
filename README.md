# DifferentialDriveRobot_ROS
This repository contains a package that spawn a differential-drive-robot in a gazebo environment. 

You can use either a keybord-commands or impose a desired trajectory to move the robot

From the terminal run:

$ roslaunch ddr gazebo.launch

to launch Gazebo and the empty_world.world

You can spawn the robot by launching in a different terminal:

$ roslaunch ddr spawn_ddr.launch

In a different terminal, launch

$ roslaunch ddr controlKeyboard.launch

to control the robot by using the keyboard

NB: q -> stop

    w -> forward
    
    s -> backward
    
    a -> turn left
    
    d -> turn right
