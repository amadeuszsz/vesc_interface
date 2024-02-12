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

#ifndef VESC_INTERFACE__VESC_INTERFACE_HPP_
#define VESC_INTERFACE__VESC_INTERFACE_HPP_
#define MAX_LIPO_VOLTAGE 4.2
#define MIN_LIPO_VOLTAGE 3.2
#define MIN_VELOCITY_THRESHOLD 0.05

#include <cstdint>
#include <unordered_map>

#include <vehicle_info_util/vehicle_info.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>

#include "vesc_interface/visibility_control.hpp"


namespace vesc_interface
{
struct VescParams
{
  double speed_to_erpm_gain;
  double speed_to_erpm_offset;
  double steering_angle_to_servo_gain;
  double steering_angle_to_servo_offset;
  double accel_to_current_gain;
  long int battery_cells;
};
using vehicle_info_util::VehicleInfo;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using tier4_control_msgs::msg::GateMode;
using autoware_auto_vehicle_msgs::msg::ControlModeReport;

std::unordered_map<uint8_t, uint8_t> control_mode_mapping = {
  {GateMode::AUTO, ControlModeReport::AUTONOMOUS},
  {GateMode::EXTERNAL, ControlModeReport::MANUAL}
};

class VESC_INTERFACE_PUBLIC VescInterface
{
public:
  VescInterface(const VescParams vesc_params, const VehicleInfo vehicle_info);
  void setControlMode(const uint8_t control_mode);
  void setGear(const uint8_t gear);
  void setBatteryCharge(const double battery_charge);
  void setEmergency(const bool emergency);
  void setVelocity(const double rpm);
  void setSteering(const double servo_position);

  uint8_t getControlMode();
  uint8_t getGear();
  float getBatteryCharge();
  double getSteeringRpt();
  double getLonVelocityRpt();
  double getLatVelocityRpt();
  double getHeadingRateRpt();

  double getSteeringCmd(const double steering_tire_angle);
  double getRpmCmd(const float velocity);
  double getAccelCmd(const float accel);

private:
  VehicleInfo vehicle_info_;
  VescParams vesc_params_;
  uint8_t control_mode_;
  uint8_t gear_;
  double battery_charge_;
  bool emergency_;
  double steering_angle_rpt_;
  double velocity_rpt_;
};

}  // namespace vesc_interface

#endif  // VESC_INTERFACE__VESC_INTERFACE_HPP_
