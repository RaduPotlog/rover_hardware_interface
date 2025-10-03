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

#include "rover_hardware_interface/rover_driver/rover_a1_driver.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"
#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_rover_driver.hpp"

namespace rover_hardware_interface
{

RoverA1Driver::RoverA1Driver(
    const DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time)
: PhidgetRoverDriver(drivetrain_settings, activate_wait_time)
{

}

void RoverA1Driver::sendSpeedCmd(const std::vector<float> & speeds)
{
    if (speeds.size() != 4) {
        throw std::runtime_error(
            "Invalid speeds vector size. Expected 4, got " + std::to_string(speeds.size()));
    }

    const auto speed_left = this->getCmdVelConverter().convert(speeds.at(0));
    const auto speed_right = this->getCmdVelConverter().convert(speeds.at(1));

    try {
        drivers_.at(DriverNames::DEFAULT)->getMotorDriver(MotorNames::LEFT)->sendCmdVel(speed_left);
        drivers_.at(DriverNames::DEFAULT)->getMotorDriver(MotorNames::RIGHT)->sendCmdVel(speed_right);
    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Driver send cmd failed: " + std::string(e.what()));
    }
}

void RoverA1Driver::attemptErrorFlagReset()
{ 
    sendSpeedCmd({0.0, 0.0});
}

void RoverA1Driver::defineDrivers()
{  
    auto driver = std::make_shared<PhidgetDriver>();

    auto left_motor_driver = std::make_shared<PhidgetMotorDriver>(
        std::dynamic_pointer_cast<PhidgetDriver>(driver), MotorChannels::LEFT, -1);
    auto right_motor_driver = std::make_shared<PhidgetMotorDriver>(
        std::dynamic_pointer_cast<PhidgetDriver>(driver), MotorChannels::RIGHT, -1);

    driver->addMotorDriver(MotorNames::LEFT, left_motor_driver);
    driver->addMotorDriver(MotorNames::RIGHT, right_motor_driver);

    drivers_.emplace(DriverNames::DEFAULT, driver);
}

}  // namespace rover_hardware_interface

