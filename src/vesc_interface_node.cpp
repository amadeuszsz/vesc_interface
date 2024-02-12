// Copyright 2023 Amadeusz Szymko
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vesc_interface/vesc_interface_node.hpp"


namespace vesc_interface
{

VescInterfaceNode::VescInterfaceNode(const rclcpp::NodeOptions & options)
:  Node("vesc_interface", options)
{
  VescParams vesc_params = {
    this->declare_parameter("speed_to_erpm_gain", 1.0),
    this->declare_parameter("speed_to_erpm_offset", 0.0),
    this->declare_parameter("steering_angle_to_servo_gain", 1.0),
    this->declare_parameter("steering_angle_to_servo_offset", 0.0),
    this->declare_parameter("accel_to_current_gain", 1.0),
    this->declare_parameter("battery_cells", 4)
  };
  auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  vesc_interface_ = std::make_unique<vesc_interface::VescInterface>(vesc_params, vehicle_info);

  // VESC subscribers
  vesc_state_sub_ = this->create_subscription<VescStateStamped>(
    "sensors/core", 1,
    std::bind(&VescInterfaceNode::vescStateCallback, this, std::placeholders::_1));
  vesc_servo_sub_ = this->create_subscription<Float64>(
    "sensors/servo_position_command", 1,
    std::bind(&VescInterfaceNode::vescServoCallback, this, std::placeholders::_1));
  // VESC publishers
  rpm_pub_ = this->create_publisher<Float64>("commands/motor/speed", 1);
  accel_amp_pub_ = this->create_publisher<Float64>("commands/motor/current", 1);
  brake_amp_pub_ = this->create_publisher<Float64>("commands/motor/brake", 1);
  servo_pub_ = this->create_publisher<Float64>("commands/servo/position", 1);
  // Autoware subscribers
  control_mode_cmd_sub_ =
    this->create_subscription<AckermannControlCommand>(
    "/control/command/control_cmd", 1,
    std::bind(&VescInterfaceNode::controlModeCmdCallback, this, std::placeholders::_1));
  gear_cmd_sub_ = this->create_subscription<GearCommand>(
    "/control/command/gear_cmd", 1,
    std::bind(&VescInterfaceNode::gearCmdCallback, this, std::placeholders::_1));
  gate_mode_cmd_sub_ = this->create_subscription<GateMode>(
    "/control/current_gate_mode", 1,
    std::bind(&VescInterfaceNode::gateModeCallback, this, std::placeholders::_1));
  emergency_cmd_sub_ = this->create_subscription<VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&VescInterfaceNode::emergencyCmdCallback, this, std::placeholders::_1));
  // Autoware publishers
  control_mode_rpt_pub_ =
    this->create_publisher<ControlModeReport>(
    "/vehicle/status/control_mode", 1);
  gear_rpt_pub_ = this->create_publisher<GearReport>(
    "/vehicle/status/gear_status", 1);
  steering_rpt_pub_ = this->create_publisher<SteeringReport>(
    "/vehicle/status/steering_status", 1);
  velocity_rpt_pub_ = this->create_publisher<VelocityReport>(
    "/vehicle/status/velocity_status", 1);
  battery_rpt_pub_ = this->create_publisher<BatteryStatus>(
    "/vehicle/status/battery_charge", 1);
}

void VescInterfaceNode::vescServoCallback(const Float64::SharedPtr msg)
{
  vesc_interface_->setSteering(msg->data); 
}

void VescInterfaceNode::vescStateCallback(const VescStateStamped::SharedPtr msg)
{
  // Velocity
  vesc_interface_->setVelocity(msg->state.speed);
  auto velocity_rpt_msg = VelocityReport();
  velocity_rpt_msg.header.stamp = msg->header.stamp;
  velocity_rpt_msg.header.frame_id = "base_link";
  velocity_rpt_msg.heading_rate = vesc_interface_->getHeadingRateRpt();
  velocity_rpt_msg.longitudinal_velocity = vesc_interface_->getLonVelocityRpt();
  velocity_rpt_msg.lateral_velocity = vesc_interface_->getLatVelocityRpt();
  velocity_rpt_pub_->publish(velocity_rpt_msg);

  // Steering
  auto steering_rpt_msg = SteeringReport();
  steering_rpt_msg.stamp = msg->header.stamp;
  steering_rpt_msg.steering_tire_angle = vesc_interface_->getSteeringRpt();
  steering_rpt_pub_->publish(steering_rpt_msg);

  // Battery
  vesc_interface_->setBatteryCharge(msg->state.voltage_input);
  auto battery_rpt_msg = BatteryStatus();
  battery_rpt_msg.stamp = msg->header.stamp;
  battery_rpt_msg.energy_level = vesc_interface_->getBatteryCharge();
  battery_rpt_pub_->publish(battery_rpt_msg);
}

void VescInterfaceNode::controlModeCmdCallback(const AckermannControlCommand::SharedPtr msg)
{
  // Driving
  auto control_msg = Float64();
  switch (vesc_interface_->getControlMode()) {
    case ControlModeReport::AUTONOMOUS:
      control_msg.data = vesc_interface_->getRpmCmd(msg->longitudinal.speed);
      rpm_pub_->publish(control_msg);
      break;
    case ControlModeReport::MANUAL:
      control_msg.data = vesc_interface_->getAccelCmd(msg->longitudinal.acceleration);
      if (control_msg.data >= 0.0 && vesc_interface_->getGear() == GearReport::REVERSE) {  // Reverse
        control_msg.data = -control_msg.data;
        accel_amp_pub_->publish(control_msg);
      } else if (control_msg.data < 0) {  // Brake
        control_msg.data = -control_msg.data;
        brake_amp_pub_->publish(control_msg);
      } else {  // Forward
        accel_amp_pub_->publish(control_msg);
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unhandled control mode %d", vesc_interface_->getControlMode());
  }

  // Steering
  auto control_lat_msg = Float64();
  control_lat_msg.data = vesc_interface_->getSteeringCmd(msg->lateral.steering_tire_angle);
  servo_pub_->publish(control_lat_msg);
}

void VescInterfaceNode::gearCmdCallback(const GearCommand::SharedPtr msg)
{
  vesc_interface_->setGear(msg->command);
  auto gear_rpt_msg = GearReport();
  gear_rpt_msg.stamp = this->now();
  gear_rpt_msg.report = vesc_interface_->getGear();
  gear_rpt_pub_->publish(gear_rpt_msg);
}

void VescInterfaceNode::gateModeCallback(const GateMode::SharedPtr msg)
{
  vesc_interface_->setControlMode(msg->data);
  auto control_mode_rpt_msg = ControlModeReport();
  control_mode_rpt_msg.stamp = this->now();
  control_mode_rpt_msg.mode = vesc_interface_->getControlMode();
  control_mode_rpt_pub_->publish(control_mode_rpt_msg);
}

void VescInterfaceNode::emergencyCmdCallback(
  const VehicleEmergencyStamped::SharedPtr msg)
{
  vesc_interface_->setEmergency(msg->emergency);
}

}  // namespace vesc_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_interface::VescInterfaceNode)
