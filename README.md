# task_manager_ros

This is a process manager package that allows for the execution of processes as well as a simple visualization of their resource consumption and standard output.  There are two main modules, task_minion and task_master.  The minion includes a Tkinter GUI that allows for the selection and interaction with a set of tasks from a predefined list.  Tasks can be processes or groups of processes.  The master is a python module responsible for executing commands.

Note: The names of the minion and master modules should be switched.  The package evolved such that the original names no longer make sense.  That will be sorted out once everything is a little farther along.

## Quick Start

This project is in alpha at the moment.  However, if you decide you would like to experiment with it, here are some basic setup instructions.

```
mkdir -p task_manager_ws/src
cd task_manager_ws/src
git clone http://github.com/jakeware/task_manager_ros.git
cd ../
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

## Objectives

Here are the objectives for this package.

* Limited dependencies outside of those found in the default python version of Ubuntu LTS
* Capable of starting and stopping both individual and groups of processes
* Display process load for every process
* Display memory consumption for every process
* Display current state (running, stopped, etc.) for every process
* Display separate standard output stream for every process
* Enforce a dependency structure when starting a set of processes
* Use a simple YAML configuration file (No XML)
* Provide a ROS message passing framework
* Provide a non-GUI interface for headless operation on build servers
* [Maybe] Start and stop ROS launch files correctly

## Alternatives

Although there are several existing process managers, none of them satisfied the requirements listed above..  Here is a list of the other packages that we considered using.

* [rosmon](https://github.com/xqms/rosmon)
* [multimaster_fkie](https://github.com/fkie/multimaster_fkie)
* [procman](https://github.com/ashuang/procman)
* [rqt_top](https://github.com/ros-visualization/rqt_top)
* [rqt_launch](https://github.com/ros-visualization/rqt_launch)
* [rosconstole](https://github.com/ros/rosconsole)
* [rosspawn_gui](https://github.com/timn/rosspawn_gui)
