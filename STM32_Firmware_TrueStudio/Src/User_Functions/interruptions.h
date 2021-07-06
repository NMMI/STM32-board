// ----------------------------------------------------------------------------
// BSD 3-Clause License

// Copyright (c) 2016, qbrobotics
// Copyright (c) 2017-2021, Centro "E.Piaggio"
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------

/**
* \file         interruptions.h
*
* \brief        Interruption handling and firmware core functions declaration.
* \date         July 6th, 2021
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2021 Centro "E.Piaggio". All rights reserved.
*/


#ifndef INTERRUPTIONS_H_INCLUDED
#define INTERRUPTIONS_H_INCLUDED

//==================================================================     include
#include "globals.h"

//=====================================================     function declaration

void function_scheduler();
void interrupt_manager(uint8_t data_to_process);
void chip_temperature();

// ----------------------------------------------------------------------------

#endif

//void function_scheduler(void);
//void encoder_reading(const uint8 index);
//void motor_control();
//void analog_read_end();
//void interrupt_manager();
//void pwm_limit_search();
//void overcurrent_control();
//void cycles_counter_update();
//void save_cycles_eeprom();





/* [] END OF FILE */
