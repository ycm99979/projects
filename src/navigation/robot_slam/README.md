# Robot SLAM Package

ROS2 SLAM package for mobile manipulator robot using Cartographer and SLAM Toolbox.

## Features

- **Cartographer**: Google's 2D/3D SLAM solution
- **SLAM Toolbox**: Alternative SLAM solution (placeholder for future)

## Dependencies

- cartographer_ros
- slam_toolbox

## Configuration

### Cartographer (`config/cartographer.lua`)
- **Sensor Inputs**:
  - Odometry: `/diff_drive_controller/odom`
  - LiDAR: `/scan`
  - IMU: `/imu/data` (optional, currently disabled)
  
- **Key Settings**:
  - 2D SLAM enabled
  - Max range: 8.0m
  - Min range: 0.3m
  - Online correlative scan matching enabled
  - Optimization every 35 nodes

## Launch Files

### `cartographer.launch.py`

Launch Cartographer SLAM:

```bash
# With Gazebo simulation
ros2 launch robot_slam cartographer.launch.py use_sim_time:=true

# With real robot
ros2 launch robot_slam cartographer.launch.py use_sim_time:=false

# With RViz
ros2 launch robot_slam cartographer.launch.py use_sim_time:=true open_rviz:=true
```

**Launch Arguments**:
- `use_sim_time` (default: true) - Use simulation clock
- `open_rviz` (default: false) - Launch RViz2
- `configuration_basename` (default: cartographer.lua) - Config file name
- `resolution` (default: 0.05) - Occupancy grid resolution
- `publish_period_sec` (default: 0.5) - Map publishing period

## Topic Remapping

The launch file remaps the following topics:
- `odom` → `/diff_drive_controller/odom`
- `scan` → `/scan`
- `imu` → `/imu/data`

## RViz Configuration

Pre-configured RViz settings are available in `rviz/slam.rviz` for visualizing:
- Robot model
- LiDAR scans
- Occupancy grid map
- TF frames

## Usage with Mobile Manipulator

1. Launch Gazebo simulation:
```bash
ros2 launch robot_gazebo frbot_gz_sim.launch.py
```

2. In a new terminal, launch Cartographer:
```bash
source install/setup.bash
ros2 launch robot_slam cartographer.launch.py use_sim_time:=true
```

3. Drive the robot around to build the map using teleop or navigation commands.

4. Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## File Structure

```
robot_slam/
├── config/
│   └── cartographer.lua          # Cartographer configuration
├── launch/
│   └── cartographer.launch.py    # Main launch file
├── rviz/
│   └── slam.rviz                 # RViz configuration
├── CMakeLists.txt
├── package.xml
└── README.md
```
