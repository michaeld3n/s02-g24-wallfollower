# Wall Follower

MTRX3760 Project 2, Phase 1 — Section 02, Group 24.

Follows the right wall using laser scans. Stops and starts through a ROS service.

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Run

Source ROS and the relevant workspaces in each terminal. Start the standard Burger world:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In another terminal:

```bash
ros2 launch wall_follower wall_follower.launch.py use_sim_time:=true
```

The controller starts disabled. Start it with:

```bash
ros2 service call /set_enabled std_srvs/srv/SetBool '{data: true}'
```

Stop it with:

```bash
ros2 service call /set_enabled std_srvs/srv/SetBool '{data: false}'
```

Check its state with:

```bash
ros2 topic echo /state
```

## Interface

Node: `/wall_follower`. Commands and state publish at 20 Hz.

| Name | Type | Use |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Input |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Velocity output |
| `/state` | `std_msgs/msg/String` | State output |
| `/set_enabled` | `std_srvs/srv/SetBool` | Start/stop service |

Velocity uses `twist.linear.x` (m/s) and `twist.angular.z` (rad/s), with a ROS timestamp and `base_link` frame. Scan QoS is sensor data; command QoS is reliable/volatile, depth 10; state QoS is reliable/transient local, depth 1.

## Settings

Edit [config/wall_follower.yaml](config/wall_follower.yaml), then restart.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `wall_distance` | `0.25` | Target right distance, m |
| `forward_speed` | `0.10` | Forward speed, m/s |
| `turn_speed` | `0.65` | Maximum turn rate, rad/s |
| `front_stop_distance` | `0.25` | Start obstacle turn below this, m |
| `front_resume_distance` | `0.35` | Finish turn at this distance, m |
| `wall_lost_distance` | `0.90` | Search beyond this right distance, m |
| `distance_gain` | `2.0` | Distance correction gain |
| `heading_gain` | `1.2` | Heading correction gain |

Values must be finite and positive, except `heading_gain` may be zero. Resume distance must exceed stop distance; lost distance must exceed target distance. Invalid settings prevent movement.

`start_enabled` and `use_sim_time` default to `false`. Package settings are read-only at runtime; use `/set_enabled` for start/stop.

Launch options: `params_file` (defaults to the shipped YAML), `start_enabled`, `use_sim_time`, `namespace` (empty), `scan_topic` (`scan`), `cmd_vel_topic` (`cmd_vel`), `state_topic` (`state`) and `enable_service` (`set_enabled`). Names follow the namespace unless remapped to an absolute path.

## Behaviour

- Front: minimum range within 0° ±20°. Right and front-right: medians within -90° ±5° and -45° ±5°. Even-sized medians use the upper middle value. Scan angles must match the robot's forward/right directions.
- A close front obstacle causes a stationary left turn until the resume distance is reached. Losing the right wall causes a forward/right curve at half `turn_speed`. Normal following corrects distance and heading, slowing during stronger turns.
- Positive infinity becomes `range_max`. Mixed NaNs are ignored; an entirely NaN sector becomes `range_max`. Other out-of-range readings are discarded.
- Missing coverage, empty scans or invalid angle/range metadata can leave a sector unknown. If any required sector is unknown, reject the whole scan and retain the previous velocity and obstacle-turn flag. Input header timestamps are ignored.
- **No scan timeout:** missing scans retain the last command and state. Unusable scans retain the command with `invalid_scan`. This continues until usable input arrives or the controller is disabled.
- Disable sends zero immediately. Enable/disable transitions clear saved motion; restarting waits for usable new input. Repeating the existing enable value does not reset it.

| State | Meaning |
| --- | --- |
| `disabled` | Stopped through the service or startup setting |
| `waiting_for_scan` | Enabled, awaiting a new scan; zero velocity |
| `invalid_scan` | Unusable scan; hold the saved command, initially zero |
| `following_wall` | Following the right wall |
| `searching_wall` | Moving forward and curving right |
| `turning_left` | Turning left on the spot |
| `invalid_configuration` | Invalid settings; zero velocity, even if disabled |

## Test

From the built workspace:

```bash
colcon test
colcon test-result
```

The isolated [integration test](test/test_integration.py) checks velocity output for a right wall, a front obstacle and open space. It needs ROS, but no robot or Gazebo. The ROS integration test and Gazebo run have not yet been verified.

## Diagrams

![ROS Node Design Diagram](docs/node_design.svg)

![Simplified UML Class Diagram](docs/class_diagram.svg)

## Attribution

Adapted from [Project 1](https://github.com/Cod-Luo/MTRX3760-project-1), revision `05df1f5`, by Cody, royce, Marz and Paekinator. ROS adapter lineage: ROBOTIS CO., LTD., copyright 2019, Taehun Lim (Darby) and Ryan Shim. Package licence: [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0).
