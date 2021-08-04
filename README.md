# SVL Robot Bringup

`svl_robot_bringup` is a ROS2 package to assist with using [Nav2](https://navigation.ros.org/), the ROS 2 navigation stack, with the SVL Simulator.

The repository contains:
-  A ROS 2 node named `odom_tf_node` that subscribes to the `/odom` topic from the simulator and broadcasts the `tf` transform between the `basefootprint` and `odom` frames for the Cloi robot.
- A launch file which launches `odom_tf_node` as well as the `lgsvl_bridge` and the `pointcloud_to_laserscan` node and broadcasts a few static transforms for the sensor frames.
- A map of the LG Seocho environment available in the SVL Simulator to be used for localization and planning.
- Params for running Nav2.
- An rviz config file.

**Note**: this repository is currently only supported on ROS 2 Foxy.

Run the following to setup and launch:

```bash
mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/lgsvl/svl_robot_bringup.git
cd ..
rosdep update
rosdep install --from-path src -iy --rosdistro foxy
colcon build --symlink-install
source install/setup.bash
ros2 launch svl_robot_bringup robot_tf_launch.py
```