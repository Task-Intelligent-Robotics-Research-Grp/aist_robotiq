![GitHub release (latest by date)](https://img.shields.io/github/v/release/Task-Intellignet-Robotics-Research-Grp/aist_robotiq)
![GitHub](https://img.shields.io/github/license/Task-Intellignet-Robotics-Research-Grp/aist_robotiq)

| ROS 2 Distribution | Humble                                                                                                                                                                      | Jazzy                                                                                                                                                                    |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Build Status       | [![humble-build](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/actions/workflows/humble-build.yaml/badge.svg)](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/workflows/humble-build.yaml) | [![jazzy-build](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/actions/workflows/jazzy-build.yaml/badge.svg)](https://github.com/Task-Intelligent-Robotics-Research-Grp/aist_robotiq/actions/workflows/jazzy-build.yaml) |

aist_robotiq
==================================================

This repository contains two packages for controlling
[Robotiq](https://robotiq.com/) grippers under ROS2 environment.
It consists of two ROS2 packages;
- [aist_robotiq](./aist_robotiq/): drivers and controllers
- [aist_robotiq_msgs](./aist_robotiq_msgs/): definitions of ROS2 messages, services and actions used by `aist_robotiq`

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


## Usage
Please consult `README`s in each package for usage.





