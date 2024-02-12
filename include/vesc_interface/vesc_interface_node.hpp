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

#ifndef VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_
#define VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "vesc_interface/vesc_interface.hpp"


namespace vesc_interface
{
using VescInterfacePtr = std::unique_ptr<vesc_interface::VescInterface>;

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::ControlModeReport;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using std_msgs::msg::Float64;
using tier4_control_msgs::msg::GateMode;
using tier4_vehicle_msgs::msg::BatteryStatus;
using tier4_vehicle_msgs::msg::VehicleEmergencyStamped;
using vesc_msgs::msg::VescStateStamped;


class VESC_INTERFACE_PUBLIC VescInterfaceNode : public rclcpp::Node

{
public:
  explicit VescInterfaceNode(const rclcpp::NodeOptions & options);
  void vescServoCallback(const Float64::SharedPtr msg);
  void vescStateCallback(const VescStateStamped::SharedPtr msg);
  void controlModeCmdCallback(const AckermannControlCommand::SharedPtr msg);
  void gearCmdCallback(const GearCommand::SharedPtr msg);
  void gateModeCallback(const GateMode::SharedPtr msg);
  void emergencyCmdCallback(const VehicleEmergencyStamped::SharedPtr msg);

private:
  VescInterfacePtr vesc_interface_{nullptr};
  // VESC subscribers
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<Float64>::SharedPtr vesc_servo_sub_;
  // VESC publishers
  rclcpp::Publisher<Float64>::SharedPtr rpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr accel_amp_pub_;
  rclcpp::Publisher<Float64>::SharedPtr brake_amp_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  // Autoware subscribers
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr control_mode_cmd_sub_;
  rclcpp::Subscription<GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<GateMode>::SharedPtr gate_mode_cmd_sub_;
  rclcpp::Subscription<VehicleEmergencyStamped>::SharedPtr emergency_cmd_sub_;
  // Autoware publishers
  rclcpp::Publisher<ControlModeReport>::SharedPtr control_mode_rpt_pub_;
  rclcpp::Publisher<GearReport>::SharedPtr gear_rpt_pub_;
  rclcpp::Publisher<SteeringReport>::SharedPtr steering_rpt_pub_;
  rclcpp::Publisher<VelocityReport>::SharedPtr velocity_rpt_pub_;
  rclcpp::Publisher<BatteryStatus>::SharedPtr battery_rpt_pub_;

};
}  // namespace vesc_interface

#endif  // VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_
