// Copyright 2025 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_data_transformer.hpp"

#include <cmath>
#include <iostream>

#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

PhidgetVelocityCommandDataTransformer::PhidgetVelocityCommandDataTransformer(
    const DrivetrainSettings & drivetrain_settings)
{
    radians_per_second_to_phidget_cmd_ = drivetrain_settings.gear_ratio * (1.0f / (2.0f * M_PI)) * 
                                         60.0f * (kMaxPhidgetCmdValue / drivetrain_settings.max_rpm_motor_speed);
}

float PhidgetVelocityCommandDataTransformer::convert(const float cmd) const
{
    return clampVelCmd(cmd * radians_per_second_to_phidget_cmd_);
}

PhidgetMotorStateTransformer::PhidgetMotorStateTransformer(const DrivetrainSettings & drivetrain_settings)
{
    phidget_pos_feedback_to_radians_ = (1.0f / drivetrain_settings.encoder_resolution) *
                                       (1.0f / drivetrain_settings.gear_ratio) * (2.0f * M_PI);

    phidget_vel_feedback_to_radians_per_second_ = (drivetrain_settings.max_rpm_motor_speed / 1000.0f) * 
                                                  (1.0f / drivetrain_settings.gear_ratio) *
                                                  (1.0f / 60.0f) * (2.0f * M_PI);

    phidget_current_feedback_to_newton_meters_ = (1.0f / 10.0f) * drivetrain_settings.motor_torque_constant * 
                                                 drivetrain_settings.gear_ratio * 
                                                 drivetrain_settings.gearbox_efficiency;
}

void PhidgetMotorStateTransformer::setData(const MotorDriverState & motor_state) 
{ 
    motor_state_ = motor_state;
};

float PhidgetMotorStateTransformer::getPosition() const
{ 
    return static_cast<float>(motor_state_.pos) * phidget_pos_feedback_to_radians_; 
}

float PhidgetMotorStateTransformer::getVelocity() const
{
    return motor_state_.vel * phidget_vel_feedback_to_radians_per_second_;
}

float PhidgetMotorStateTransformer::getTorque() const
{
    return motor_state_.current * phidget_current_feedback_to_newton_meters_;
}

PhidgetDriverDataTransformer::PhidgetDriverDataTransformer(const DrivetrainSettings & drivetrain_settings)
: channel_1_motor_state_(drivetrain_settings)
, channel_2_motor_state_(drivetrain_settings)
{

}

void PhidgetDriverDataTransformer::setMotorsStates(
    const MotorDriverState & channel_1_state, 
    const MotorDriverState & channel_2_state,
    const bool data_timed_out)
{
    channel_1_motor_state_.setData(channel_1_state);
    channel_2_motor_state_.setData(channel_2_state);
  
    motor_states_data_timed_out_ = data_timed_out;
}

void PhidgetDriverDataTransformer::setDriverState(
    const DriverState & state, 
    const bool data_timed_out)
{
    (void)state;
    
    // TODO: set driver state
    driver_state_data_timed_out_ = data_timed_out;
}

const PhidgetMotorStateTransformer & PhidgetDriverDataTransformer::getMotorState(const std::uint8_t channel) const
{
    if (channel == PhidgetDriver::motorChannel1) {
        return channel_1_motor_state_;
    } else if (channel == PhidgetDriver::motorChannel2) {
        return channel_2_motor_state_;
    }

    throw std::runtime_error("Invalid channel number");
}

const PhidgetDriverStateTransformer & PhidgetDriverDataTransformer::getDriverState() const 
{ 
    return driver_state_;
}

} // namespace rover_hardware_interface