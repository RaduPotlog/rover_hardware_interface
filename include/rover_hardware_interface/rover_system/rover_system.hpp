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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_SYSTEM_ROVER_SYSTEM_HPP_
#define ROVER_HARDWARE_INTERFACE_ROBOT_SYSTEM_ROVER_SYSTEM_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/hardware_component_interface.hpp"

#include "rover_hardware_interface/rover_driver/rover_driver.hpp"
#include "rover_hardware_interface/system_ros_interface/system_ros_interface.hpp"

namespace rover_hardware_interface
{

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class RoverSystem : public hardware_interface::SystemInterface
{

public:

    RCLCPP_SHARED_PTR_DEFINITIONS(RoverSystem)

    RoverSystem(const std::vector<std::string> & joint_order);
    
    virtual ~RoverSystem() = default;

    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<StateInterface> export_state_interfaces() override;
    std::vector<CommandInterface> export_command_interfaces() override;

    return_type read(const rclcpp::Time & time, const rclcpp::Duration & /* period */) override;
    return_type write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override;

    virtual void updateHwStates() = 0;

protected:

    void checkJointSize() const;
    void sortAndCheckJointNames();
    void setInitialValues();
    void checkInterfaces() const;

    void configureRobotDriver();

    const size_t joint_size_;

    const std::vector<std::string> joint_order_;
    std::vector<std::string> joints_names_sorted_;

    std::vector<double> hw_commands_velocities_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;
    std::vector<double> hw_states_efforts_;

    // ROS hardware interface 
    std::unique_ptr<SystemROSInterface> system_ros_interface_;

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROBOT_SYSTEM_ROVER_SYSTEM_HPP_
