# kortex_3dpaint
![](https://img.shields.io/badge/2025-Spring-green)
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-%23E95420.svg?&logo=ubuntu&logoColor=white)
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-%2322312D.svg?&logo=ros&logoColor=white)
![MoveIt](https://img.shields.io/badge/MoveIt-%2319356C.svg?&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNTAgMjUwIj48cGF0aCBmaWxsPSIjMTkzNTZDIiBkPSJNMTI1IDBDNTYuMSAwIDAgNTYuMSAwIDEyNXM1Ni4xIDEyNSAxMjUgMTI1IDEyNS01Ni4xIDEyNS0xMjVTMjQzLjkgMCAxMjUgMHoiLz48cGF0aCBmaWxsPSIjRkZGIiBkPSJNMTI1IDUwYy00MS40IDAtNzUgMzMuNi03NSA3NXMzMy42IDc1IDc1IDc1IDc1LTMzLjYgNzUtNzVTMjQzLjkgNTAgMTI1IDUweiIvPjwvc3ZnPg==&logoColor=white)
![Kinova Gen3 Lite](https://img.shields.io/badge/Kinova-Gen3_Lite-%23007DB8.svg?&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNTAgMjUwIj48cGF0aCBmaWxsPSIjMDA3REI4IiBkPSJNMTI1IDBDNTYuMSAwIDAgNTYuMSAwIDEyNXM1Ni4xIDEyNSAxMjUgMTI1IDEyNS01Ni4xIDEyNS0xMjVTMjQzLjkgMCAxMjUgMHoiLz48cGF0aCBmaWxsPSIjRkZGIiBkPSJNMTI1IDUwYy00MS40IDAtNzUgMzMuNi03NSA3NXMzMy42IDc1IDc1IDc1IDc1LTMzLjYgNzUtNzVTMjQzLjkgNTAgMTI1IDUweiIvPjwvc3ZnPg==&logoColor=white)

Project for EE368 Robot Motion and Control, SUSTech

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

``pick_and_place_moveit.py`` -->  ``roslaunch kortex_3dpaint pick_and_place_moveit.launch __ns:=my_gen3_lite``

``pick_and_place.py`` --> ``roslaunch kortex_3dpaint pick_and_place.launch``

> [!important]
>
> ``pick_and_place.py`` may encounter problems in Gazebo currently.
> 
> ``pick_and_place_moveit.py`` may ecounter problems when you need to use the gripper on the real robot. So please try ``pick_and_place.py`` first and then try ``pick_and_place_moveit.py``, which may solves the problem.

``arm_control.py`` --> ``roslaunch kortex_3dpaint arm_control.launch __ns:=my_gen3_lite``

``aruco_detector.py`` --> ``roslaunch kortex_3dpaint aruco_detector.launch``

``full_arm_control.py`` --> ``roslaunch kortex_3dpaint full_arm_control.launch`` (Please launch ``aruco_detector.py`` first)
