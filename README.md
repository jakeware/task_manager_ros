# task_manager_ros

One process manager to rule them all.

## Build

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

## Process Configuration

The process list can be found in task_manager_ros/cfg/task_config.yaml.  Each process is defined by the following components.

* Name: Each yaml entry is defined by a top-level process name.
* Command: Command for execution as if you were running the process on the command line.
* Group (optional): Group that the command belongs to.
* Dependencies (not yet implemented): List of dependencies that must be running before this process can be started.

## Interface

The GUI supports the following commands.  Note that you can start task groups by selected the group row and issuing the corresponding hotkey command.

* Ctrl-s: Start task or group.
* Ctrl-k: Stop task or group.