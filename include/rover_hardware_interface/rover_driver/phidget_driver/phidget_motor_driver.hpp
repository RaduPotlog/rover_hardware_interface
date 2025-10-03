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

#ifndef ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_

#include <functional>
#include <vector>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include <libphidget22/phidget22.h>

#include "rover_hardware_interface/rover_driver/driver.hpp"

namespace rover_hardware_interface
{

class PhidgetDriver : public DriverInterface
{
    
public:

    static constexpr std::uint8_t motorChannel1 = 0;
    static constexpr std::uint8_t motorChannel2 = 1;
    static constexpr std::uint8_t motorChannel3 = 3;
    static constexpr std::uint8_t motorChannel4 = 4;

    PhidgetDriver();

    std::future<void> initialize() override;

    std::future<void> deinitialize() override;

    DriverState readState() override;

    void turnOnEStop() override;

    void turnOffEStop() override;

    void addMotorDriver(const MotorNames name, std::shared_ptr<MotorDriverInterface> motor_driver) override;

    std::shared_ptr<MotorDriverInterface> getMotorDriver(const MotorNames name) override;

private:

    std::mutex init_mtx_;
    std::promise<void> init_promise_;

    std::unordered_map<MotorNames, std::shared_ptr<MotorDriverInterface>> motor_drivers_;
};

class PhidgetMotorDriver : public MotorDriverInterface
{

public:

    PhidgetMotorDriver(std::weak_ptr<PhidgetDriver> driver, const std::uint8_t channel, const std::int32_t serial_number);

    void initialize() override;

    void deinitialize() override;

    MotorDriverState readState() override;

    void sendCmdVel(const float cmd) override;

    void turnOnSafetyStop() override;

private:
    
    std::weak_ptr<PhidgetDriver> driver_;

    const std::uint8_t channel_;
    std::int32_t serial_number_;

    PhidgetDCMotorHandle motor_handle_{nullptr};
};

} // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_