# Optimal Rapidly exploring Random tree(RRT*) for path palnning in 3D space for Quadrotors

The aim of our project is to implement the path planning algorithm presented in the paper “RRT-based 3D Path Planning for Formation Landing of Quadrotor UAVs” 
and upgrade the path planning algorithm to an optimised RRT* algorithm and simulate the Quadrotor motion in Gazebo using ROS 

## Simulation Demo

![Visualisation](/Results/GAzebo-simulation-without-prunin.gif)


## Authors

- [Arjun Srinivasan](https://github.com/aarjunsrinivasan)
- [Pradeep](https://github.com/Pradeep-Gopal)


## Installation

Mavros installation

```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```
source this-> 
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
```
```
./install_geographiclib_datasets.sh
```

For initiating mavros to px4 connection 
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```
For PX4

Download  and source this script ->https://raw.githubusercontent.com/PX4/Devguide/v1.8.0/build_scripts/ubuntu_sim_ros_gazebo.sh

```
git clone https://github.com/PX4/Firmware.git
```
To launch PX4->       
```                     cd <Firmware_clone>
			make posix_sitl_default gazebo
			source ~/catkin_ws/devel/setup.bash    // (optional)
			source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
			export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
			export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
			roslaunch px4 posix_sitl.launch
```

Create a ROS package and run this python file 
```
python missionfly2.py

```







