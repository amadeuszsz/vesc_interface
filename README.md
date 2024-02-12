# vesc_interface
<!-- Required -->
<!-- Package description -->
The Autoware interface for VESC.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to vesc_interface
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch vesc_interface vesc_interface.launch.py
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name                             | Type                                                     | Description                 |
| -------------------------------- | -------------------------------------------------------- | --------------------------- |
| `sensors/core`                   | vesc_msgs::msg::VescStateStamped                         | VESC core.                  |
| `sensors/servo_position_command` | std_msgs::msg::Float64                                   | Servo position feedback.    |
| `/control/command/control_cmd`   | autoware_auto_control_msgs::msg::AckermannControlCommand | Autoware control command.   |
| `/control/command/gear_cmd`      | autoware_auto_vehicle_msgs::msg::GearCommand             | Autoware gear command.      |
| `/control/current_gate_mode`     | tier4_control_msgs::msg::GateMode                        | Autoware current gate mode. |
| `/control/command/emergency_cmd` | tier4_vehicle_msgs::msg::VehicleEmergencyStamped         | Autoware emergency command. |

### Output

| Name                              | Type                                               | Description                                |
| --------------------------------- | -------------------------------------------------- | ------------------------------------------ |
| `commands/motor/speed`            | std_msgs::msg::Float64                             | VESC speed (RPM), used in autonomous mode. |
| `commands/motor/current`          | std_msgs::msg::Float64                             | VESC accelertaion, used in manual mode.    |
| `commands/motor/brake`            | std_msgs::msg::Float64                             | VESC brake, used in manual mode.           |
| `commands/servo/position`         | std_msgs::msg::Float64                             | VESC servo position for steering.          |
| `/vehicle/status/control_mode`    | autoware_auto_vehicle_msgs::msg::ControlModeReport | Autoware control mode report.              |
| `/vehicle/status/gear_status`     | autoware_auto_vehicle_msgs::msg::GearReport        | Autoware gear report.                      |
| `/vehicle/status/steering_status` | autoware_auto_vehicle_msgs::msg::SteeringReport    | Autoware steering report.                  |
| `/vehicle/status/velocity_status` | autoware_auto_vehicle_msgs::msg::VelocityReport    | Autoware velocity report.                  |
| `/vehicle/status/battery_charge`  | tier4_vehicle_msgs::msg::BatteryStatus             | Autoware battery status.                   |

### Parameters

| Name                             | Type  | Description                     |
| -------------------------------- | ----- | ------------------------------- |
| `speed_to_erpm_gain`             | float | Speed (m/s) to ERPM gain.       |
| `speed_to_erpm_offset`           | float | Speed (m/s) to ERPM offset.     |
| `steering_angle_to_servo_gain`   | float | Steering angle to servo gain.   |
| `steering_angle_to_servo_offset` | float | Steering angle to servo offset. |
| `battery_cells`                  | int   | Number of battery cells.        |
| `accel_to_current_gain`          | float | Acceleration to current gain.   |

## References / External links
[VESC driver](https://github.com/f1tenth/vesc/tree/ros2)
