![GitHub release (latest by date)](https://img.shields.io/github/v/release/Task-Intellignet-Robotics-Research-Grp/aist_robotiq)
![GitHub](https://img.shields.io/github/license/Task-Intellignet-Robotics-Research-Grp/aist_robotiq)

| ROS 2 Distribution | Jazzy                                                                                                                                                                    |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Build Status       | [![jazzy-build](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/actions/workflows/jazzy-build.yaml/badge.svg)](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/actions/workflows/jazzy-build.yaml) |

aist_robotiq
==================================================

This repository contains two packages for controlling
[Robotiq](https://robotiq.com/) grippers under ROS2 environment.
It consists of two ROS2 packages;
- [aist_robotiq](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/tree/develop/aist_robotiq/): drivers and controllers
- [aist_robotiq_msgs](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/tree/develop/aist_robotiq_msgs/): definitions of ROS2 messages, services and actions used by `aist_robotiq`

## Installation
First, you should install a developer package of C++ library,  `nlohmann-json3`, for handling `JSON` messages in `C++` code;
```bash
sudo apt install nlohmann-json3-dev
```
Then download the [source code of this package](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq) as well as its dependencies into the ROS2 workspace;
```bash
cd ros2_ws/src
git clone https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq
vcs import . --input=aist_robotiq/dependencies.repos
```
Finally, you can compile the package by typing
```bash
source ros2_ws/install/setup.bash
colcon build
```

## Quick start
You can launch gripper with two or three fingers by issueing the following command,
```bash
ros2 launch aist_robotiq test.launch.py [gripper_name:=<gripper_name>] [driver_type:=<driver_type>]
```
where
- **gripper_type**: Type of the gripper. Possible choices are `robotiq_85`, `robotiq_140`, `robotiq_hande` and `robotiq_3f` (default: `robotiq_85`).
- **driver_type**: Type of the driver. Should be chosen accorrding to how the gripper hardware is electrically connected to the robot or PC. Possible choices are `rtu`, `tcp` and `urcap`.

You can launch EPick suction gripper by issueing the following command,
```bash
ros2 launch aist_robotiq test.launch.py gripper_name:=robotiq_epick [driver_type:=<driver_type>]
```
where
- **driver_type**: Type of the driver. Should be chosen accorrding to how the gripper hardware is electrically connected to the robot or PC. Possible choices are `rtu`, `tcp` and `urcap`.

## More info.
Please consult the following pages.
- [Documentation of aist_robotiq](https://task-intelligent-robotics-research-grp.github.io/aist_robotiq/index.html): Usage of controllers and drivers for `Robotiq` grippers. API of the controller clients.
- [Documentation of aist_robotiq_msgs](https://task-intelligent-robotics-research-grp.github.io/aist_robotiq/aist_robotiq_msgs/index.html): Definitions of ROS message/srvice/action used by controllers and drivers included in `aist_robotiq`.
