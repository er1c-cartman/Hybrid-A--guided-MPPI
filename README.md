# benchmark_lunar
Benchmark framework for lunar navigation

This is a [ROS2] humble package developed for benchmark framework for lunar navigation. The software is designed for (local) navigation tasks with robots on lunar environment, which are equipped with a pose estimation and distance sensor. The goal of this software is to provide benchmark framework for lunar navigation.

### Building

In order to install the benchmark_lunar, clone the latest version from this repository into your workspace and compile the package using ROS2.

    cd your_workspace/src
    git clone https://github.com/er1c-cartman/benchmark_lunar.git
    cd ../
    colcon build
    
    


# Lunar simulator

First, set your model path to 
	
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/your_workspace/src/scout_v2/scout_gazebo/models
    
And run the simulation with
    
     ros2 launch scout_gazebo moon.launch.py
     
<img alt="Lunar Simulation Example" src="scout_v2/scout_gazebo/doc/lunar.png" width="700">



# Elevation mapping

Elevation mapping code is originated from ETH Zurich 
- [Elevation mapping](https://github.com/ANYbotics/elevation_mapping)

    
And run the elevation mapping and traversability estimation with
    
     ros2 launch elevation_mapping elevation_mapping.launch.py 

<img alt="Lunar Simulation Example" src="scout_v2/scout_gazebo/doc/elevation.png" width="700">
<img alt="Lunar Simulation Example" src="scout_v2/scout_gazebo/doc/traversability.png" width="700">

To edit parameters of traversability estimation, you can change values in following file

$HOME/your_workspace/src/elevation_mapping/elevation_mapping/config/grid_map_filters.yaml

# Navigation

Navigation code is originated from NAV2 
- [Navigation 2](https://github.com/ros-navigation/navigation2)
    
And run the navigation with
    
     ros2 launch scout_navigation mppi.launch.py 
     (dwa, teb, purepursuit. You can edit the config file and launch file to run what you want)

<img alt="Lunar Simulation Example" src="scout_v2/scout_gazebo/doc/navigation.png" width="700">


# Benchmark

Benchmark code is located at

$HOME/your_workspace/src/scout_v2/scout_navigation/result/benchmark.py

The result is saved at separate folder in same location. You can use python or matlab to benchmark the results



