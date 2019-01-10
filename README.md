# task_manager_ros

This is a process manager package that allows for the execution of processes as well as a simple visualization of their resource consumption and standard output.  There are two main modules, task_minion and task_master.  The minion includes a Tkinter GUI that allows for the selection and interaction with a set of tasks from a predefined list.  Tasks can be processes or groups of processes.  The master is a python module responsible for executing commands.

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

* Up: Move task selection up one row.
* Down: Move task selection down one row.
* Ctrl-s: Start task or group.
* Ctrl-k: Stop task or group.

## Alternatives

Although there are several existing process managers, none of them satisfied our constraints.  Here is a list of those that we considered using.

* [rosmon] (http://wiki.ros.org/rosmon)
* [node_manager_fkie] (http://wiki.ros.org/node_manager_fkie)
* [procman] (https://github.com/ashuang/procman)
* [rqt_top] (https://github.com/ros-visualization/rqt_top)
* [rqt_launch] (https://github.com/ros-visualization/rqt_launch)
* [rosconstole] (https://github.com/ros/rosconsole)
