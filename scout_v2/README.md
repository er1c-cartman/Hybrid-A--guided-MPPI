# ROS2 Packages for Scout Mobile Robot For Galactic

## Before start

* This repository is based on an official repository of [scout-ros2](https://github.com/agilexrobotics/scout_ros2). But, out team have written much more pkgs including simulation with gazebo and mapping and localization with nav2 package.
* This repository is optimized on ROS2-Galactic. (This repository can be built on ROS2-Foxy environment, but you should modify some variables in config and params to use scout navigation pkg.)

## Packages

This repository contains packages to control the scout robot on both real and simulation world. 

* scout_base: a ROS wrapper around [ugv_sdk](https://github.com/westonrobot/ugv_sdk) to monitor and control the scout robot.
* scout_description: URDF model for the mobile base for scout_base and scout_gazebo pkgs.
* scout_msgs: scout related message definitions.
* scout_gazebo: a Gazebo launcher with scout v2 model. An IMU and 2d Lidar are attached on top_plate of the robot.
* scout_navigation: a navigation launcher with nav2 pkg. To building a map, the launch scripts is written with slam_toolbox. For localization, we used amcl or slam_toolbox. 

## Supported Hardware

* Scout V2

## Basic usage of the ROS packages
1. [ROS-Installaion]

    See this page [click](https://docs.ros.org/en/galactic/Installation.html)

2. [Build] 

    (the following instructions assume your catkin workspace is at: ~/gal_ws/src)

    ```
    $ sudo apt-get install build-essential git cmake libasio-dev ros-galactic-nav2* ros-galactic-xacro ros-galactic-slam-toolbox ros-galactic-libg2o ros-galactic-joint-*
    $ mkdir -p ~/gal_ws/src
    $ cd ~/gal_ws/src
    $ git clone https://github.com/westonrobot/ugv_sdk.git --recursive
    $ git clone https://github.com/ggory15/scout_v2.git -b galactic-devel --recursive
    $ git clone https://github.com/rst-tu-dortmund/teb_local_planner -b ros2-master --recursive
    $ git clone https://github.com/rst-tu-dortmund/costmap_converter -b ros2 --recursive
    $ cd ..
    $ colcon build
    ```

3. [Hardware-Launch] 
 
* Start the base node for the Scout robot

    ```
    $ ros2 launch scout_base scout_base.launch.py
    ```

* Start the keyboard tele-op node

    ```
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

4. [Simulation-Launch]
 
* Start the gazebo node for the Scout robot

    ```
    $ ros2 launch scout_gazebo gazebo_launch.py
    ```

* Start the keyboard tele-op node (For Manual Control)

    ```
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

5. [MapBuilding-Launch]
 
* Start the gazebo node for the Scout robot

    ```
    $ ros2 launch scout_gazebo gazebo_launch.py
    ```

* Start the map building node 

    ```
    $ ros2 launch scout_navigation map_builnding.launch.py
    ```

* With Nav2 goal or manual control from keyboardtele-op node, you can get the current map. Then, The map can be saved with 'Save Map' and 'Serialize Map' buttons on SlamToolbox Plugin in Rviz2. The saved map is located at $HOME folder.

6. [Localization-Launch]
 
* Start the gazebo node for the Scout robot

    ```
    $ ros2 launch scout_gazebo gazebo_launch.py
    ```

* [Option1] Start the localization node with AMCL. The default map is from /maps folder. (Default map name is 'palypen_map')

    ```
    $ ros2 launch scout_navigation amcl_demo.launch.py 
    ```
* [Option2] Start the localization node with Slam Toolbox. The default map is from $HOME folder. (Default map name is 'test_map')

    ```
    $ ros2 launch scout_navigation localization.launch.py 
    ```



