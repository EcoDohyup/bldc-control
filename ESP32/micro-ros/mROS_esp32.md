# mROS on ESP32 board

## Installation and initial configuration
```bash
source /opt/ros/humble/setup.bash

# create ws and download tools
mkdir ws
cd ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

#build mROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# Testing installation
ros2 run micro_ros_agent micro_ros agent [parameters]

```

[A boilerplate for mROS for ESP32](https://gist.github.com/rasheeddo/5a6dd95b206233ad58bda8304ae2f30d)