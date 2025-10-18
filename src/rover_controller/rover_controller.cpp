// Copyright 2024 Husarion sp. z o.o.
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

#include "rover_hardware_interface/rover_controller/rover_controller.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include "rover_hardware_interface/rover_controller/rover_controller_types.hpp"

namespace rover_hardware_interface
{

const std::vector<RoverControllerContactInfo> ContactCoilHandler::contacts_config_info_storage_ = {
    
    RoverControllerContactInfo
    {
        ContactInfo { static_cast<uint8_t>(RoverControllerContact::CONTACT_HW_E_STOP_USER_BTN) },
    }
};

const std::vector<RoverControllerCoilInfo> ContactCoilHandler::coils_config_info_storage_ = {

    RoverControllerCoilInfo { 
        CoilInfo { false, false, static_cast<uint8_t>(RoverControllerCoil::COIL_MOTOR_CONTACTOR_ENGAGED) }    
    },

    RoverControllerCoilInfo { 
        CoilInfo { true, true, static_cast<uint8_t>(RoverControllerCoil::COIL_SW_E_STOP_CPU_WDG_TRIGGER) }    
    },
    
    RoverControllerCoilInfo { 
        CoilInfo { true, true, static_cast<uint8_t>(RoverControllerCoil::COIL_SW_E_STOP_USER_BUTTON) }
    },
    
    RoverControllerCoilInfo { 
        CoilInfo { true, true, static_cast<uint8_t>(RoverControllerCoil::COIL_SW_E_STOP_MOTOR_DRIVER_FAULT) }    
    },

    RoverControllerCoilInfo { 
        CoilInfo { false, true, static_cast<uint8_t>(RoverControllerCoil::COIL_SW_E_STOP_LATCH_RESET) }    
    },
};

ContactCoilHandler::ContactCoilHandler(std::shared_ptr<RoverModbus> rover_modbus)
: rover_modbus_(rover_modbus)
{

}

ContactCoilHandler::~ContactCoilHandler()
{ 
    if (isContactCoilHandlerEnabled()) {
        contact_coil_handler_enabled_ = false;
        contact_coil_handler_thread_.join();
    }
}

bool ContactCoilHandler::start()
{
    if (isContactCoilHandlerEnabled()) {
        return true;
    }
    
    initCoils();

    contact_coil_handler_enabled_ = true;
    contact_coil_handler_thread_ = std::thread(&ContactCoilHandler::contactCoilHandlerThread, this);

    return isContactCoilHandlerEnabled();
}

bool ContactCoilHandler::isContactCoilHandlerEnabled() const 
{ 
    return contact_coil_handler_thread_.joinable(); 
}

// SW E-STOP USER BTN - sw_e_stop_user_button
void ContactCoilHandler::eStopUserBtnTrigger(const bool state)
{
    CoilInfo coil = { true, true, static_cast<uint8_t>(RoverControllerCoil::COIL_SW_E_STOP_USER_BUTTON)};
    rover_modbus_->writeDiscreteCoil(coil, state);
}

// SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
void ContactCoilHandler::eStopMotorDriverFaultTrigger(const bool state)
{
    CoilInfo coil = { true, true, static_cast<uint8_t>(RoverControllerCoil::COIL_SW_E_STOP_MOTOR_DRIVER_FAULT)};
    rover_modbus_->writeDiscreteCoil(coil, state);
}

// SW E-STOP LATCH RESET - sw_e_stop_latch_reset
void ContactCoilHandler::eStopLatchReset()
{
 
}

void ContactCoilHandler::initCoils() 
{
    for (size_t i = 0; i < coils_config_info_storage_.size(); ++i) {
        CoilInfo coil = { true, true, static_cast<uint8_t>(coil.modbus_offset)};
        rover_modbus_->writeDiscreteCoil(coil, coil.default_coil_state);
    }
}

void ContactCoilHandler::contactCoilHandlerThread()
{
    // ContactInfo contact_hw_btn;
    // contact_hw_btn.modbus_offset = 0;
    // CoilInfo coil_motor = { 
    //     true,
    //     false,
    //     0};     
    // uint16_t contact_state = rover_modbus_->readDiscreteContact(contact_hw_btn);
    // uint16_t coil_state = rover_modbus_->readDiscreteCoil(coil_motor);
    // RCLCPP_INFO(logger_, "Coil state: %d", coil_state);
    // RCLCPP_INFO(logger_, "Contact state: %d", contact_state);
    
    static bool wdg_state = false;
    CoilInfo coil_watchdog = { true, true, 1};

    while (contact_coil_handler_enabled_) {
        // Trigger watchdog        
        rover_modbus_->writeDiscreteCoil(coil_watchdog, wdg_state);
        wdg_state = !wdg_state;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

RoverController::RoverController()
{
    rover_modbus_ = std::make_shared<RoverModbus>("192.168.1.11", 502);
}

void RoverController::start()
{
    contactCoilHandler_ = std::make_unique<ContactCoilHandler>(rover_modbus_);
    contactCoilHandler_->start();
}

// SW E-STOP USER BTN - sw_e_stop_user_button
void RoverController::eStopUserBtnTrigger(const bool state)
{
    if (!contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopUserBtnTrigger(state);
}

// SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
void RoverController::eStopMotorDriverFaultTrigger(const bool state)
{
    if (!contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopMotorDriverFaultTrigger(state);
}

// SW E-STOP LATCH RESET - sw_e_stop_latch_reset
void RoverController::eStopLatchReset()
{
    if (!contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }
}

bool RoverController::waitFor(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lck(e_stop_cv_mtx_);

    should_abort_e_stop_reset_ = false;
    
    bool interrupted = e_stop_cv_.wait_for(
        lck, timeout, [&]() { 
            return should_abort_e_stop_reset_;
    });

    return !interrupted;
}

}  // namespace rover_hardware_interface
