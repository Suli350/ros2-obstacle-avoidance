# ROS2 Reactive Obstacle Avoidance

A self-contained 2D robot simulator with a **simulated LiDAR**, plus a **reactive
obstacle avoidance** controller that wanders without hitting anything. Runs anywhere
RViz runs, no Gazebo required.

![stack](https://img.shields.io/badge/ROS2-Humble-blue) ![python](https://img.shields.io/badge/python-3.10-green)

## Nodes

| Node | Subscribes | Publishes |
|---|---|---|
| `world_sim` | `/cmd_vel` | `/scan`, `/odom`, `/obstacles`, `/robot_marker`, TF `odom->base_link->laser` |
| `avoider` | `/scan` | `/cmd_vel` |

## What you learn

- `sensor_msgs/LaserScan` layout (angle_min, angle_increment, `inf` for no return)
- Ray casting against circles and line segments
- Sensor QoS (`qos_profile_sensor_data`) and why it matters
- Static vs dynamic TF broadcasters
- Visualization markers (`Marker`, `MarkerArray`)
- A sector-based reactive controller with hysteresis so the robot doesn't dither

## Build & run

```bash
cd ~/ros2_ws/src && git clone https://github.com/<you>/ros2-obstacle-avoidance.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch obstacle_avoidance avoidance.launch.py
```

Drive it yourself and see the collision checker stop you:

```bash
ros2 launch obstacle_avoidance avoidance.launch.py autonomous:=false
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Change the world in `config/world.yaml` (circles are `x, y, r` triples; walls are
`x1, y1, x2, y2`).

## Test

```bash
colcon test --packages-select obstacle_avoidance && colcon test-result --verbose
```

## Exercises

1. Replace the reactive controller with a Vector Field Histogram (VFH).
2. Add a wall-following mode (keep the right wall at 0.5 m with a PID).
3. Publish `/collisions` as a counter and plot it while tuning parameters.

The next project, **ros2-occupancy-mapping**, uses this simulator to build a map.
