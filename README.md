![GitHub Release](https://img.shields.io/github/v/release/Task-Intelligent-Robotics-Research-Grp/aist_robotiq)
![GitHub License](https://img.shields.io/github/license/Task-Intelligent-Robotics-Research-Grp/aist_robotiq)

| ROS 2 Distribution | Jazzy                                                                                                                                                                    |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Build Status       | [![jazzy-build](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/actions/workflows/jazzy-build.yaml/badge.svg)](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/actions/workflows/jazzy-build.yaml) |

aist_robotiq
==================================================

## Overview
This repository provides ROS2 action controllers and drivers for [Robotiq](https://robotiq.com) two finger grippers, [2F-85, 2F-140](https://robotiq.com/products/2f85-140-adaptive-robot-gripper) and [Hand-E](https://robotiq.com/products/hand-e-adaptive-robot-gripper), three-finger gripper, [3F](https://robotiq.com/products/adaptive-grippers#Three-Finger-Gripper), and suction gripper, [EPick](https://robotiq.com/products/vacuum-grippers#EPick). The package is forked from the [robotiq package developed by CRI group](https://github.com/crigroup/robotiq). The URCap driver is borrowed from [the code by Felix von Drigalski](https://gist.github.com/felixvd/d538cad3150e9cac28dae0a3132701cf).

The repository contains the following two packages;
- [aist_robotiq](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/tree/develop/aist_robotiq/): drivers and controllers
- [aist_robotiq_msgs](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/tree/develop/aist_robotiq_msgs/): definitions of ROS2 messages, services and actions used by `aist_robotiq`

## Installation
First, you should install a developer package of C++ library,  `nlohmann-json3`, for handling `JSON` messages in `C++` code;
```bash
sudo apt install nlohmann-json3-dev
```
Then download [this package](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq) as well as its dependencies into the ROS2 workspace;
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
ros2 launch aist_robotiq test.launch.py [gripper_type:=<gripper_type>] [driver_type:=<driver_type>] [ip:=<ip>] [dev:=<dev>]
```
where
- **gripper_type**: Type of the gripper. Possible choices are `robotiq_85`, `robotiq_140`, `robotiq_hande` and `robotiq_3f` (default: `robotiq_85`).
- **driver_type**: Type of the driver. Should be chosen accorrding to how the gripper hardware is electrically connected to the robot or PC. Possible choices are `rtu`, `tcp` and `urcap` (default: `urcap`).
- **ip**: IP address of gripper or UR robot for `driver_type` of `tcp` or `urcap`, respectively (default: `192.168.1.11`).
- **dev**: TTY device name of Modbus for `driver_type` of `rtu` (default: `/dev/ttyUSB0`).


You can launch EPick suction gripper by issueing the following command,
```bash
ros2 launch aist_robotiq test.launch.py gripper_name:=robotiq_epick [driver_type:=<driver_type>] [ip:=<ip>] [dev:=<dev>]
```
where
- **driver_type**: Type of the driver. Should be chosen accorrding to how the gripper hardware is electrically connected to the robot or PC. Possible choices are `rtu`, `tcp` and `urcap` (default: `urcap`).
- **ip**: IP address of gripper or UR robot for `driver_type` of `tcp` or `urcap`, respectively (default: `192.168.1.11`).
- **dev**: TTY device name of Modbus for `driver_type` of `rtu` (default: `/dev/ttyUSB0`).

## More info.
Please refer tothe following pages.
- [Documentation of aist_robotiq](https://task-intelligent-robotics-research-grp.github.io/aist_robotiq/md_aist__robotiq_2README.html): Usage of controllers and drivers for `Robotiq` grippers. API of the controller clients.
- [Documentation of aist_robotiq_msgs](https://task-intelligent-robotics-research-grp.github.io/aist_robotiq/aist_robotiq_msgs/index.html): Definitions of ROS message/srvice/action used by controllers and drivers included in `aist_robotiq`.
