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

namespace rover_hardware_interface
{

RoverA1Driver::RoverA1Driver(const std::chrono::milliseconds activate_wait_time)
{
    (void)activate_wait_time;
    
    initDrivers();
}

void RoverA1Driver::setSpeed(const std::vector<float> & speeds)
{
    (void)speeds;
}

void RoverA1Driver::attemptErrorFlagReset()
{ 
    setSpeed({0.0, 0.0});
}

void RoverA1Driver::initDrivers()
{

}

}  // namespace rover_hardware_interface

