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

void RoverA1System::initRobotDriver()
{
    // robot_driver_ = std::make_shared<RoverA1Driver>();
}

void RoverA1System::updateHwStates()
{
    hw_states_positions_[0] = 0;
    hw_states_positions_[1] = 0;
    hw_states_positions_[2] = 0;
    hw_states_positions_[3] = 0;

    hw_states_velocities_[0] = 0;
    hw_states_velocities_[1] = 0;
    hw_states_velocities_[2] = 0;
    hw_states_velocities_[3] = 0;

    hw_states_efforts_[0] = 0;
    hw_states_efforts_[1] = 0;
    hw_states_efforts_[2] = 0;
    hw_states_efforts_[3] = 0;
}

std::vector<float> RoverA1System::getSpeedCommands() const
{
    return {static_cast<float>(hw_commands_velocities_[0]), static_cast<float>(hw_commands_velocities_[1])};
}

}  // namespace rover_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rover_hardware_interface::RoverA1System, hardware_interface::SystemInterface)
