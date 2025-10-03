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

#include "rover_hardware_interface/rover_system/rover_a1_system.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

#include "rover_hardware_interface/rover_driver/rover_a1_driver.hpp"

namespace rover_hardware_interface
{

RoverA1System::RoverA1System() 
: RoverSystem(joints_) 
{

}

void RoverA1System::defineRoverDriver()
{
    rover_driver_ = std::make_shared<RoverA1Driver>(drivetrain_settings_);
}

void RoverA1System::updateHwStates(const rclcpp::Time & time)
{
    const auto data = rover_driver_->getData(DriverNames::DEFAULT);

    const auto left = data.getMotorState(MotorChannels::LEFT);
    const auto right = data.getMotorState(MotorChannels::RIGHT);

    // hw_states_positions_[0] = left.getPosition();
    // hw_states_positions_[1] = right.getPosition();
    // hw_states_positions_[2] = left.getPosition();
    // hw_states_positions_[3] = right.getPosition();

    // hw_states_velocities_[0] = left.getVelocity();
    // hw_states_velocities_[1] = right.getVelocity();
    // hw_states_velocities_[2] = left.getVelocity();
    // hw_states_velocities_[3] = right.getVelocity();

    //TODO: Compile switch for open loop and closed loop
    rclcpp::Duration period = time - last_time_;
    last_time_ = time;
    double period_sec = period.seconds();

    for (size_t i = 0; i < hw_states_positions_.size(); ++i) {
        hw_states_positions_[i] += hw_commands_velocities_[i] * period_sec;
    }

    for (size_t i = 0; i < hw_states_velocities_.size(); ++i) {
        hw_states_velocities_[i] = hw_commands_velocities_[i];
    }

    hw_states_efforts_[0] = left.getTorque();
    hw_states_efforts_[1] = right.getTorque();
    hw_states_efforts_[2] = left.getTorque();
    hw_states_efforts_[3] = right.getTorque();
}

std::vector<float> RoverA1System::getSpeedCmd() const
{
    return { static_cast<float>(hw_commands_velocities_[0]), 
             static_cast<float>(hw_commands_velocities_[1]),
             static_cast<float>(hw_commands_velocities_[2]),
             static_cast<float>(hw_commands_velocities_[3])
           };
}

}  // namespace rover_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rover_hardware_interface::RoverA1System, hardware_interface::SystemInterface)
