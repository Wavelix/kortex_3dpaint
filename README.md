# kortex_3dpaint

Project for EE368 Robot Motion and Control, SUSTech

> [!WARNING]
> 项目施工中

Reference [Kinovarobotics/ros_kortex](https://github.com/Kinovarobotics/ros_kortex)

## Installation

### Setup

```
cd ~/catkin_workspace/src/ros_kortex
git clone git@github.com:Wavelix/kortex_3dpaint.git
cd ~/catkin_workspace
catkin_make
```

### ArUco Setup

First, to setup the **RealSense**, please refer to the instructions in [CSDN](https://blog.csdn.net/wanghq2013/article/details/123325671).

Make sure the following packages are installed：

```
sudo apt-get install ros-noetic-aruco
sudo apt-get install ros-noetic-aruco-ros
sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-realsense2-description
sudo apt-get install ros-noetic-vision-opencv
```

```
cd ~/catkin_workspace
catkin_make
```

Connect to the **RealSense**, then launch the **ArUco**：

```
cd ~/catkin_workspace
source devel/setup.bash
roslaunch kortex_3dpaint aruco_detector.launch
```

If error occurs：

```
sudo apt-get install ros-noetic-ddynamic-reconfigure
cd ~/catkin_workspace
catkin_make clean
catkin_make -j$(nproc)
source devel/setup.bash
roslaunch kortex_3dpaint aruco_detector.launch
```

## Instructions

> [!important]
>
> Run the following commands after any new terminal opened:
>
> ```
> cd ~/catkin_workspace
> source devel/setup.bash
> ```

### Launch the Robot

Launch the robot in Gazebo simulation:

```
roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite
```

Launch the real robot:

```
roslaunch kortex_driver kortex_driver.launch arm:=gen3_lite
```
### Basic Commands

``pick_and_place_moveit.py`` -->  ``roslaunch kortex_3dpaint pick_and_place_moveit.launch __ns:=my_gen3_lite`` ( recommended )

``pick_and_place.py`` --> ``roslaunch kortex_3dpaint pick_and_place.launch`` ( not recommended )

> [!important]
>
> ``pick_and_place.py`` may encounter problems in Gazebo currently. 

``arm_control.py`` --> ``roslaunch kortex_3dpaint arm_control.launch __ns:=my_gen3_lite``

``aruco_detector.py`` --> ``roslaunch kortex_3dpaint aruco_detector.launch``
