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
## More Instructions and Attention
To run ``pick_and_palce.py``, try ``roslaunch kortex_3dpaint pick_and_place.launch``


To run ``pick_and_place_trajectories``, which use **MoveIt** , try ``rosrun kortex_3dpaint pick_and_place_trajectories.py __ns:=my_gen3_lite``
> [!IMPORTANT]
> ``pick_and_palce.py`` may encounter problems in Gazebo Simulation.
