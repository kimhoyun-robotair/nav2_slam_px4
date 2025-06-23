# nav2_slam_px4

## Introduction & Notes

- This page covers the integration of `PX4`, `RTAB-MAP`, and `Nav2`.
- Due to the limitations of `Nav2`, only **2D autonomous flight** is implemented.
- The `PX4` version used is **1.15**. Functionality with other versions is **not guaranteed** (untested).
- **PX4 does not broadcast TF**, so various TF-related measures have been taken.

### Notice!!

RTAB-MAP is currently set to **localization mode**, which operates based on the rtabmap.db file in the `.ros` directory. **If you haven’t performed mapping with RTAB-MAP yet, open `rtabmap.launch.py`, disable localization mode, and then run mapping.**

```python
# 66th row in rtabmap.launch.py
DeclareLaunchArgument('localization', default_value='true', description='Launch in localization mode.'),
```

**And please change the topic name in the ros-gz bridge in `spawn_robot.launch.py` before use.**

## Basic Usage

### Installation

```bash
git clone https://github.com/kimhoyun-robotair/nav2_slam_px4.git
cd nav2_slam_px4
colcon build
# or colcon build --symlink-install
```

## Usage

```bash
# Terminal 1 
cd ~/path/to/PX4-Autopilot
PX4_GZ_WORLD=your_own_map make px4_sitl gz_x500_rtab

# Terminal 2 
MicroXRCEAgent udp4 -p 8888

# Terminal 3 (optional) 
cd ~/path/to/QGC
./QGroundControl.AppImage

# Terminal 4 
cd ~/path/to/nav2_slam_px4
source install/setup.bash
ros2 launch rtabmap_nav2_px4 bringup.launch.py
# or
# ros2 launch rtabmap_nav2_px4 autonomous_exploration.launch.py
# I recommend this one.
```

You can switch the mode for Drone (SLAM or Navigation) in `bringup.launch.py`
If you want to use the package for navigation, please uncomment the `navigation` line in `bringup.launch.py`.

- The following nodes will run:
1. TF publishing (odom ↔ base_link ↔ sensor_link etc)
2. Keyboard-based PX4 velocity control node (in a separate terminal)
3. Node for logging current velocity, position, etc. (in a separate terminal)
4. RTAB-MAP node
5. Nav2 node

### Setting up the Gazebo Development Environment (optional)
**It is recommended to use your own custom map**

1. Go to the `~/nav2_slam_px4/src/nav2_px4` package.

2. Copy the **lidar_2d_v2** directory from the internal model directory to `~/PX4-Autopilot/Tools/simulation/gz/models`.
   
3. Copy the **OakD-Lite** directory from the internal model directory to `~/PX4-Autopilot/Tools/simulation/gz/models`.

4. Copy the **turtlebot3_world** directory from the internal model directory to `~/PX4-Autopilot/Tools/simulation/gz/models`.

5. Copy the **x500_rtab** directory from the internal model directory to `~/PX4-Autopilot/Tools/simulation/gz/models`. This model is a quadcopter with a 2D lidar, RGB-D camera, and Visual Odometry.

6. Copy the **apt** directory from the internal model directory to `~/PX4-Autopilot/Tools/simulation/gz/models`.

7. Copy the **apt_world.sdf** world file directory from the internal model directory to `~/PX4-Autopilot/Tools/simulation/gz/worlds`.

8. Copy the **turtlebot3_world.sdf** file from the internal world directory to `~/PX4-Autopilot/Tools/simulation/gz/worlds`.

9. Add a file named **4015_gz_x500_rtab** to `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes` with the following content:

```bash
#!/bin/sh
#
# @name Gazebo x500 mono cam
#
# @type Quadrotor
#

PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_rtab}

. ${R}etc/init.d-posix/airframes/4001_gz_x500
```

- Then, add `4015_gz_x500_rtab` as a target in the `CMakeLists.txt` file in the same directory.

- Add the following line to your terminal or to your `~/.bashrc`:

```bash
export GZ_SIM_RESOURCE_PATH=/path/to/your/px4/directory/PX4-Autopilot/Tools/simulation/gz/models
```

- `GZ_SIM_RESOURCE_PATH` can only specify one directory at a time. If you use another gz model directory, make sure to comment/uncomment or export accordingly when using PX4 or not.

- Now, run PX4 as follows:

```bash
PX4_GZ_WORLD=turtlebot3_world make px4_sitl gz_x500_rtab
```

## Broadcasting TF from PX4 + Receiving PX4 SITL Topics

### Broadcasting base_link ↔ sensor_link TF for PX4 and ROS2 Integration

- By default, PX4 does not output TF.
- However, when handling SDF files and running simulations, frames broadcasted by GZ are always present. Thus, to properly use the topics in ROS2, you must align TF using the tf2 library in ROS2 to match the PX4 SITL frames.
- For this, the following URDF file is used:
    - `x500_rtab.urdf`
    - For smooth camera data handling, `camera_link_optical` and `depth_camera_link_optical` are defined.
    - All link names are set to match the frame names in the default PX4-Autopilot SDF files and the frames you see when echoing each GZ sensor topic. If not matched, sensor topics received in ROS2 won't visualize correctly in RViz.
- This allows TF broadcasting between `base_link` and `sensor_link`.

### Broadcasting odom ↔ base_link TF

- Use the following code to broadcast the odom ↔ base_link TF. All frame names are unified with those of the PX4 SITL visual odometry topic.
    - `odom_baselink_publisher.py`

### Changing Frame for Image Topics

- For image topics, since camera and ROS2 coordinate frames differ, frame correction is needed. A script is included that receives the image topic, changes the frame, and republishes.
    - `image_transform.py`

### Use Launch File for TF Broadcasting and PX4 SITL Topic Subscription

- The `spawn_robot.launch.py` launch file runs the bridges for receiving GZ topics from PX4-SITL as ROS2 topics, parses the URDF to publish TF via `robot_state_publisher`, and launches the additional TF publishing nodes above—all at once.

#### Example usage:

```bash
# Terminal 1
cd PX4-Autopilot
PX4_GZ_WORLD=turtlebot3_world make px4_sitl gz_x500_rtab

# Terminal 2
MicroXRCEAgent udp4 -p 8888

# Terminal 3
cd ~/nav2_slam_px4
source install/setup.bash
ros2 launch nav2_px4 spawn_robot.launch.py
```
