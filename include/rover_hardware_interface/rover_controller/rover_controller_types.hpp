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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_TYPES_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_TYPES_HPP_

#include <map>
#include <string>

#include "rover_hardware_interface/rover_modbus/modbus_types.hpp"

namespace rover_hardware_interface
{

enum class RoverControllerContact
{
    CONTACT_HW_E_STOP_USER_BTN = 0,
};

enum class RoverControllerCoil
{
    COIL_MOTOR_CONTACTOR_ENGAGED        = 0,
    COIL_SW_E_STOP_CPU_WDG_TRIGGER      = 1, // sw_e_stop_cpu_wdg_trigger
    COIL_SW_E_STOP_USER_BUTTON          = 2, // sw_e_stop_user_button
    COIL_SW_E_STOP_MOTOR_DRIVER_FAULT   = 3, // sw_e_stop_motor_driver_faults
    COIL_SW_E_STOP_LATCH_RESET          = 4, // sw_e_stop_latch_reset
};

const std::map<RoverControllerContact, std::string> contacts_names_
{
    { RoverControllerContact::CONTACT_HW_E_STOP_USER_BTN, "CONTACT_HW_E_STOP_USER_BTN" },
};

const std::map<RoverControllerCoil, std::string> coils_names_
{
    { RoverControllerCoil::COIL_MOTOR_CONTACTOR_ENGAGED,        "COIL_MOTOR_CONTACTOR_ENGAGED"          },
    { RoverControllerCoil::COIL_SW_E_STOP_CPU_WDG_TRIGGER,      "COIL_SW_E_STOP_CPU_WDG_TRIGGER"        },
    { RoverControllerCoil::COIL_SW_E_STOP_USER_BUTTON,          "COIL_SW_E_STOP_USER_BUTTON"            },
    { RoverControllerCoil::COIL_SW_E_STOP_MOTOR_DRIVER_FAULT,   "COIL_SW_E_STOP_MOTOR_DRIVER_FAULT"     },
    { RoverControllerCoil::COIL_SW_E_STOP_LATCH_RESET,          "COIL_SW_E_STOP_LATCH_RESET"            },
};

struct RoverControllerContactInfo 
{
    ContactInfo contact_info;
};

struct RoverControllerCoilInfo 
{
    CoilInfo coil_info;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_TYPES_HPP_
