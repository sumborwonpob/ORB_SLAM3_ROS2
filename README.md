# ORB_SLAM3_ROS2
A ROS2 implementation of ORB_SLAM3, originally https://github.com/UZ-SLAMLab/ORB_SLAM3.

Thank you very much to the authors and contributors of the original repo!

### Disclaimer: 
* This package is made for my own use. It is inteded to be an example repo. Please don't expect me to maintain it at all. There's a high chance that your request/question will never be answered, ever.

## Tested environment
* Ubuntu 22.04 with ROS2 Humble

## Dependencies
* Follow the instruction for installing ORB-SLAM3's and their dependencies in https://github.com/UZ-SLAMLab/ORB_SLAM3

## Installation
```
cd ros2_ws/src
git clone git@github.com:sumborwonpob/orb_slam3_ros2.git
cd ..
colcon build
```

## Getting started
* Copy the ORBvoc.txt from your ORB_SLAM3 directory to orb_slam3_ros2/config/ROS

* Edit the config file in orb_slam3_ros2/config/ROS/ros_config.yaml

* Everytime after editing any config, be sure to 'colcon build'

* Launch the launch file with

```
ros2 launch orb_slam3_ros2 orb_slam3_ros2.launch.py
```