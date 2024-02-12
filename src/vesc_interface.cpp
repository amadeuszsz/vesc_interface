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

#include "vesc_interface/vesc_interface.hpp"

#include <algorithm>
#include <iostream>

namespace vesc_interface
{

VescInterface::VescInterface(const VescParams vesc_params, const VehicleInfo vehicle_info)
{
  vesc_params_ = vesc_params;
  vehicle_info_ = vehicle_info;
}

void VescInterface::setControlMode(const uint8_t control_mode)
{
  control_mode_ = control_mode_mapping[control_mode];
}

void VescInterface::setGear(const uint8_t gear)
{
  gear_ = gear;
}

void VescInterface::setBatteryCharge(const double battery_charge)
{
  battery_charge_ = std::clamp(
    (battery_charge - vesc_params_.battery_cells * MIN_LIPO_VOLTAGE) /
    (vesc_params_.battery_cells * (MAX_LIPO_VOLTAGE - MIN_LIPO_VOLTAGE)),
    0.0, 1.0);
}

void VescInterface::setSteering(const double servo_position)
{
  steering_angle_rpt_ = (servo_position - vesc_params_.steering_angle_to_servo_offset) /
    vesc_params_.steering_angle_to_servo_gain;
  if (std::fabs(steering_angle_rpt_) < MIN_VELOCITY_THRESHOLD) {
    steering_angle_rpt_ = 0.0;
  }
}

void VescInterface::setVelocity(const double rpm)
{
  velocity_rpt_ = (rpm - vesc_params_.speed_to_erpm_offset) / vesc_params_.speed_to_erpm_gain;
}

void VescInterface::setEmergency(const bool emergency)
{
  emergency_ = emergency;
}

uint8_t VescInterface::getControlMode()
{
  return control_mode_;
}

uint8_t VescInterface::getGear()
{
  return gear_;
}

float VescInterface::getBatteryCharge()
{
  return static_cast<float>(battery_charge_);
}

double VescInterface::getSteeringRpt()
{
  return steering_angle_rpt_;
}

double VescInterface::getLonVelocityRpt()
{
  return velocity_rpt_ * cos(steering_angle_rpt_);
}

double VescInterface::getLatVelocityRpt()
{
  return velocity_rpt_ * sin(steering_angle_rpt_);
}

double VescInterface::getHeadingRateRpt()
{
  return velocity_rpt_ * tan(steering_angle_rpt_) / vehicle_info_.wheel_base_m;
}

double VescInterface::getSteeringCmd(const double steering_tire_angle)
{
  return steering_tire_angle * vesc_params_.steering_angle_to_servo_gain +
    vesc_params_.steering_angle_to_servo_offset;
}

double VescInterface::getRpmCmd(const float velocity)
{
  return emergency_ ? 0.0 : velocity * vesc_params_.speed_to_erpm_gain + vesc_params_.speed_to_erpm_offset;
}

double VescInterface::getAccelCmd(const float accel)
{
  return emergency_ ? 0.0 : accel * vesc_params_.accel_to_current_gain;
}

}  // namespace vesc_interface
