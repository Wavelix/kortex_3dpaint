# kortex_3dpaint
Project for EE368 Robot Motion and Control, SUSTech

> [!WARNING]
> 项目施工中

Reference [Kinovarobotics/ros_kortex](https://github.com/Kinovarobotics/ros_kortex)

## Installation Steps
```
cd ~/catkin_workspace/src/ros_kortex
git clone git@github.com:Wavelix/kortex_3dpaint.git
cd ~/catkin_workspace
catkin_make
```
## Launch the Robot
```
source devel/setup.bash
```
Launching the Robot in Gazebo Simulation:
```
roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite
```
Launching the real robot:
```
roslaunch kortex_driver kortex_driver.launch arm:=gen3_lite
```

## Instructions
### aruco设置教程
First, to setup the realsense, please refer to the instructions in [CSDN](https://blog.csdn.net/wanghq2013/article/details/123325671).

确保安装以下包：
```
sudo apt-get install ros-noetic-aruco
sudo apt-get install ros-noetic-aruco-ros
sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-realsense2-description
sudo apt-get install ros-noetic-vision-opencv
catkin_make
```

连接相机，运行aruco：
```
cd ~/catkin_workspace
source devel/setup.bash
roslaunch kortex_3dpaint aruco_detector.launch
```
若运行出错：
```
sudo apt-get install ros-noetic-ddynamic-reconfigure
catkin_make clean
catkin_make -j$(nproc)
source devel/setup.bash
roslaunch kortex_3dpaint aruco_detector.launch
```

## Attention
To run ``pick_and_palce.py``, try ``roslaunch kortex_3dpaint pick_and_place.launch``


To run ``pick_and_place_trajectories``, which use **MoveIt** , try ``rosrun kortex_3dpaint pick_and_place_trajectories.py __ns:=my_gen3_lite``
> [!IMPORTANT]
> ``pick_and_palce.py`` may encounter problems in Gazebo Simulation.
