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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_MODBUS_TYPES_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_MODBUS_TYPES_HPP_

#include <string>

namespace rover_hardware_interface
{

enum class ContactStatus
{
    CONTACT_OFF = 0,
    CONTACT_ON,
};

enum class CoilCommand
{
    COIL_DISENGAGE = 0,
    COIL_ENGAGE,
};

enum class COIL_STATE
{
    COIL_NOT_ENGAGED = 0,
    COIL_ENGAGED,
};

struct ContactInfo
{
    uint8_t modbus_offset;
};

struct CoilInfo
{
    const bool default_coil_state;
    const bool is_coil_engage_allowed;
    const uint8_t modbus_offset;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_MODBUS_TYPES_HPP_
