# task_manager_ros

One process manager to rule them all.

## Getting Started

We are in alpha with this at the moment.  Here are some instructions to help test the package.

```
mkdir -p task_manager_ws/src
cd task_manager_ws
catkin init
catkin build
source ./devel/setup.bash
roscore
roslaunch task_manager_ros task_master_ros_node.launch
roslaunch task_manager_ros task_minion_ros_node.launch
```