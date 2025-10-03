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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>

namespace rover_hardware_interface
{

enum class MotorNames {
    LEFT = 0,
    RIGHT,
};

enum class DriverNames {
    DEFAULT = 0,
    FRONT,
    REAR,
};

inline std::string motorNamesToString(const MotorNames motor_name)
{
    switch (motor_name) {
        case MotorNames::LEFT:
            return "left";
        case MotorNames::RIGHT:
            return "right";
        default:
            return "unknown";
    }
}

inline std::string driverNamesToString(const DriverNames driver_name)
{
    switch (driver_name) {
        case DriverNames::DEFAULT:
            return "default";
        case DriverNames::FRONT:
            return "front";
        case DriverNames::REAR:
            return "rear";
        default:
            return "unknown";
    }
}

}  // namespace rover_hardware_interface

namespace std
{

template <>
struct hash<rover_hardware_interface::MotorNames>
{
    std::size_t operator()(const rover_hardware_interface::MotorNames & motor_name) const noexcept
    {
        return static_cast<std::size_t>(motor_name);
    }
};

template <>
struct hash<rover_hardware_interface::DriverNames>
{
    std::size_t operator()(const rover_hardware_interface::DriverNames & driver_name) const noexcept
    {
        return static_cast<std::size_t>(driver_name);
    }
};

}  // namespace std

namespace rover_hardware_interface
{

struct MotorDriverState
{
    std::int32_t pos;
    std::int16_t vel;
    std::int16_t current;
};

struct DriverState
{
    std::uint8_t fault_flags;
    std::uint8_t script_flags;
    std::uint8_t runtime_stat_flag_channel_1;
    std::uint8_t runtime_stat_flag_channel_2;

    std::int16_t battery_current_1;
    std::int16_t battery_current_2;

    std::uint16_t battery_voltage;

    std::int16_t mcu_temp;
    std::int16_t heatsink_temp;
};

class MotorDriverInterface;

class DriverInterface
{

public:

    virtual std::future<void> initialize() = 0;

    virtual std::future<void> deinitialize() = 0;

    virtual DriverState readState() = 0;

    virtual void turnOnEStop() = 0;

    virtual void turnOffEStop() = 0;

    virtual void addMotorDriver(const MotorNames name, std::shared_ptr<MotorDriverInterface> motor_driver) = 0;

    virtual std::shared_ptr<MotorDriverInterface> getMotorDriver(const MotorNames name) = 0;

    using SharedPtr = std::shared_ptr<DriverInterface>;
};

class MotorDriverInterface
{

public:
    
    virtual void initialize() = 0;

    virtual void deinitialize() = 0;

    virtual MotorDriverState readState() = 0;

    virtual void sendCmdVel(const float cmd) = 0;

    virtual void turnOnSafetyStop() = 0;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_HPP_
