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

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"

#include "rover_hardware_interface/rover_driver/driver.hpp"

namespace rover_hardware_interface
{

PhidgetDriver::PhidgetDriver()
{

}

void openWaitForAttachment(
    PhidgetHandle handle, 
    int32_t serial_number,
    int hub_port, 
    bool is_hub_port_device, 
    int channel)
{
    PhidgetReturnCode ret;

    ret = Phidget_setDeviceSerialNumber(handle, serial_number);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to set device serial number hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }

    ret = Phidget_setHubPort(handle, hub_port);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to set device hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }

    ret = Phidget_setIsHubPortDevice(handle, is_hub_port_device);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to set device is hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }

    ret = Phidget_setChannel(handle, channel);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to set device channel hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }

    ret = Phidget_openWaitForAttachment(handle, PHIDGET_TIMEOUT_DEFAULT);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to open device hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }
}

std::future<void> PhidgetDriver::initialize()
{
    std::lock_guard<std::mutex> lck(init_mtx_);
    init_promise_ = std::promise<void>();
    
    std::future<void> future = init_promise_.get_future();

    for (auto & [name, motor_driver] : motor_drivers_) {
        try {
            motor_driver->initialize();
        } catch (const std::runtime_error & e) {
            throw std::runtime_error(
                "Motor driver initilize exception on " + motorNamesToString(name) + 
                " motor: " + std::string(e.what()));
        }
    }
    
    return future;
}

std::future<void> PhidgetDriver::deinitialize()
{
    std::lock_guard<std::mutex> lck(init_mtx_);
    init_promise_ = std::promise<void>();
    
    std::future<void> future = init_promise_.get_future();

    return future;
}

DriverState PhidgetDriver::readState()
{
    DriverState state;
    
    state.fault_flags = 0;
    state.script_flags = 0;
    state.runtime_stat_flag_channel_1 = 0;
    state.runtime_stat_flag_channel_2 = 0;

    state.battery_current_1 = 0;
    state.battery_current_2 = 0;

    state.battery_voltage = 0;

    state.mcu_temp = 0;
    state.heatsink_temp = 0;

    return state;
}

void PhidgetDriver::turnOnEStop()
{

}

void PhidgetDriver::turnOffEStop()
{

}

void PhidgetDriver::addMotorDriver(
    const MotorNames name, 
    std::shared_ptr<MotorDriverInterface> motor_driver)
{
    if (std::dynamic_pointer_cast<PhidgetMotorDriver>(motor_driver) == nullptr) {
        throw std::runtime_error("Motor driver is not of type PhidgetMotorDriver");
    }
  
    motor_drivers_.emplace(name, motor_driver);
}

std::shared_ptr<MotorDriverInterface> PhidgetDriver::getMotorDriver(const MotorNames name)
{
    auto it = motor_drivers_.find(name);
    
    if (it == motor_drivers_.end()) {
        throw std::runtime_error("Motor driver with name '" + motorNamesToString(name) + "' does not exist");
    }

    return it->second;
}

PhidgetMotorDriver::PhidgetMotorDriver(
    std::weak_ptr<PhidgetDriver> driver, 
    const std::uint8_t channel, 
    const std::int32_t serial_number)
: driver_(driver)
, channel_(channel)
, serial_number_(serial_number)
{

}

void PhidgetMotorDriver::initialize()
{
    PhidgetReturnCode ret = PhidgetDCMotor_create(&motor_handle_);
    
    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to create Motor handle for channel " + std::to_string(channel_));
    }

    openWaitForAttachment(reinterpret_cast<PhidgetHandle>(motor_handle_), -1, channel_, false, 0);
    
    if (serial_number_ == -1)
    {
        ret = Phidget_getDeviceSerialNumber(reinterpret_cast<PhidgetHandle>(motor_handle_), &serial_number_);
        
        if (ret != EPHIDGET_OK) {
            throw std::runtime_error("Failed to get serial number for motor channel ");
        }
    }
}

void PhidgetMotorDriver::deinitialize() 
{

}

MotorDriverState PhidgetMotorDriver::readState()
{
    MotorDriverState state;

    state.vel = 0.0;
    state.pos = 0.0;
    state.current = 0.0;

    return state;
}

void PhidgetMotorDriver::sendCmdVel(const float cmd)
{
    if (auto driver = driver_.lock()) {
        PhidgetReturnCode ret = PhidgetDCMotor_setTargetVelocity(motor_handle_, cmd);
        
        // TODO: Process return value
        (void)ret;
    }
}

void PhidgetMotorDriver::turnOnSafetyStop()
{

}

}  // namespace rover_hardware_interface