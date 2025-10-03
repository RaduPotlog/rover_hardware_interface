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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_DRIVER_PHIDGET_DATA_TRANSFORMER_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_DRIVER_PHIDGET_DATA_TRANSFORMER_HPP_

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"
#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

struct DrivetrainSettings
{
    float motor_torque_constant;
    float gear_ratio;
    float gearbox_efficiency;
    float encoder_resolution;
    float max_rpm_motor_speed;
};

class PhidgetVelocityCommandDataTransformer
{

public:
    
    PhidgetVelocityCommandDataTransformer(const DrivetrainSettings & drivetrain_settings);
    
    float convert(const float cmd) const;

private:
    
    static constexpr float kMaxPhidgetCmdValue = 1.0f;

    inline float clampVelCmd(const float cmd) const
    {
        return std::clamp(cmd, -kMaxPhidgetCmdValue, kMaxPhidgetCmdValue);
    }

    float radians_per_second_to_phidget_cmd_;

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
};

class PhidgetDriverStateTransformer
{

public:
  
    PhidgetDriverStateTransformer() {}

    void SetTemperature(const std::int16_t temp) { temp_ = temp; };
    void SetHeatsinkTemperature(const std::int16_t heatsink_temp) { heatsink_temp_ = heatsink_temp; };
    void SetVoltage(const std::uint16_t voltage) { voltage_ = voltage; };
  
    void SetBatteryCurrent1(const std::int16_t battery_current_1)
    {
        battery_current_1_ = battery_current_1;
    };

    void SetBatteryCurrent2(const std::int16_t battery_current_2)
    {
        battery_current_2_ = battery_current_2;
    };

    float GetTemperature() const { return temp_; }
    float GetHeatsinkTemperature() const { return heatsink_temp_; }
    float GetVoltage() const { return voltage_ / 10.0; }
    float GetCurrent() const { return (battery_current_1_ + battery_current_2_) / 10.0; }

private:

    std::int16_t temp_ = 0;
    std::int16_t heatsink_temp_ = 0;
    std::uint16_t voltage_ = 0;
    std::int16_t battery_current_1_ = 0;
    std::int16_t battery_current_2_ = 0;
};

class PhidgetMotorStateTransformer
{

public:

    PhidgetMotorStateTransformer(const DrivetrainSettings & drivetrain_settings);

    void setData(const MotorDriverState & motor_state);

    float getPosition() const;
    float getVelocity() const;
    float getTorque() const;

private:

    float phidget_pos_feedback_to_radians_;
    float phidget_vel_feedback_to_radians_per_second_;
    float phidget_current_feedback_to_newton_meters_;

    MotorDriverState motor_state_ = {0, 0, 0};
};

class PhidgetDriverDataTransformer
{

public:

    PhidgetDriverDataTransformer(const DrivetrainSettings & drivetrain_settings);

    void setMotorsStates(
        const MotorDriverState & channel_1_state, 
        const MotorDriverState & channel_2_state,
        const bool data_timed_out);
  
    void setDriverState(const DriverState & state, const bool data_timed_out);

    const PhidgetMotorStateTransformer & getMotorState(const std::uint8_t channel) const;

    const PhidgetDriverStateTransformer & getDriverState() const;

private:
  
    PhidgetMotorStateTransformer channel_1_motor_state_;
    PhidgetMotorStateTransformer channel_2_motor_state_;

    PhidgetDriverStateTransformer driver_state_;

    bool motor_states_data_timed_out_ = false;
    bool driver_state_data_timed_out_ = false;
};

} // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_DRIVER_PHIDGET_DATA_TRANSFORMER_HPP_