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

#include "rover_hardware_interface/rover_system/rover_system.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>

#include "rover_hardware_interface/rover_driver/rover_a1_driver.hpp"
#include "rover_hardware_interface/system_ros_interface/system_ros_interface.hpp"

namespace rover_hardware_interface
{

RoverSystem::RoverSystem(const std::vector<std::string> & joint_order)
: SystemInterface()
, joint_size_(joint_order.size())
, joint_order_(joint_order) 
, joints_names_sorted_(joint_size_)
{

}

CallbackReturn RoverSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    try {
        checkJointSize();
        sortAndCheckJointNames();
        setInitialValues();
        checkInterfaces();
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM(logger_, "An exception occurred while initializing: " << e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_configure(const rclcpp_lifecycle::State &)
{
    try {
        configureRobotDriver();
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to initialize motors controllers. Error: " << e.what());
        return CallbackReturn::ERROR;
    }

    std::fill(hw_commands_velocities_.begin(), hw_commands_velocities_.end(), 0.0);
    std::fill(hw_states_positions_.begin(), hw_states_positions_.end(), 0.0);
    std::fill(hw_states_velocities_.begin(), hw_states_velocities_.end(), 0.0);
    std::fill(hw_states_efforts_.begin(), hw_states_efforts_.end(), 0.0);

    system_ros_interface_ = std::make_unique<SystemROSInterface>("hardware_controller");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
    system_ros_interface_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_activate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
    system_ros_interface_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_error(const rclcpp_lifecycle::State &)
{
  
    system_ros_interface_.reset();

    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RoverSystem::export_state_interfaces()
{
    std::vector<StateInterface> state_interfaces;

    for (std::size_t i = 0; i < joint_size_; i++) {
        state_interfaces.emplace_back(StateInterface(
            joints_names_sorted_[i], hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        state_interfaces.emplace_back(StateInterface(
            joints_names_sorted_[i], hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
        state_interfaces.emplace_back(StateInterface(
            joints_names_sorted_[i], hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
    }

    return state_interfaces;
}

std::vector<CommandInterface> RoverSystem::export_command_interfaces()
{
    std::vector<CommandInterface> command_interfaces;
    
    for (std::size_t i = 0; i < joint_size_; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joints_names_sorted_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    }

    return command_interfaces;
}

return_type RoverSystem::read(const rclcpp::Time & time, const rclcpp::Duration & /* period */)
{
    (void)time;
    return return_type::OK;
}

return_type RoverSystem::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
    return return_type::OK;
}

void RoverSystem::checkJointSize() const
{
    if (info_.joints.size() != joint_size_) {
        throw std::runtime_error(
            "Wrong number of joints defined: " + std::to_string(info_.joints.size()) + ", " +
            std::to_string(joint_size_) + " expected.");
    }
}

bool checkIfJointNameContainValidSequence(const std::string & name, const std::string & sequence)
{
    const std::size_t pos = name.find(sequence);
    
    if (pos == std::string::npos) {
        return false;
    }

    if (pos >= 1) {
        const std::size_t id_before_sequence = pos - 1;
        
        if (name[id_before_sequence] != '_' && name[id_before_sequence] != '/') {
            return false;
        }
    }

    const std::size_t id_after_sequence = pos + sequence.length();
        
    if (id_after_sequence < name.length() && name[id_after_sequence] != '_') {
        return false;
    }

    return true;
}

void RoverSystem::sortAndCheckJointNames()
{
    for (std::size_t i = 0; i < joint_size_; i++) {
        std::size_t match_count = 0;

        for (std::size_t j = 0; j < joint_size_; j++) {
            if (checkIfJointNameContainValidSequence(info_.joints[j].name, joint_order_[i])) {
                joints_names_sorted_[i] = info_.joints[j].name;
                ++match_count;
            }
        }

        if (match_count != 1) {
            throw std::runtime_error(
                "There should be exactly one joint containing " + joint_order_[i] + ", " +
                std::to_string(match_count) + " found.");
        }
    }
}

void RoverSystem::setInitialValues()
{
    hw_commands_velocities_.resize(joint_size_, 0.0);

    hw_states_positions_.resize(joint_size_, std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(joint_size_, std::numeric_limits<double>::quiet_NaN());
    hw_states_efforts_.resize(joint_size_, std::numeric_limits<double>::quiet_NaN());
}

void RoverSystem::checkInterfaces() const
{
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
        // Commands
        if (joint.command_interfaces.size() != 1) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + std::to_string(joint.command_interfaces.size()) +
                " command interfaces. 1 expected.");
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + joint.command_interfaces[0].name +
                " command interface. " + hardware_interface::HW_IF_VELOCITY + " expected.");
        }

        // States
        if (joint.state_interfaces.size() != 3) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + std::to_string(joint.state_interfaces.size()) +
                " state  " + (joint.state_interfaces.size() == 1 ? "interface." : "interfaces.") +
                " 3 expected.");
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + joint.state_interfaces[0].name +
                " as first state interface. " + hardware_interface::HW_IF_POSITION + " expected.");
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + joint.state_interfaces[1].name +
                " as second state interface. " + hardware_interface::HW_IF_VELOCITY + " expected.");
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + joint.state_interfaces[2].name +
                " as third state interface. " + hardware_interface::HW_IF_EFFORT + " expected.");
        }
    }
}

void RoverSystem::configureRobotDriver()
{
    RCLCPP_INFO(logger_, "Successfully configured robot driver");
}

}  // namespace rover_ugv_hardware_interface
