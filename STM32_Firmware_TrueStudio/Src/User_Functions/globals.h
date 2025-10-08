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
* \file         globals.h
*
* \brief        Global definitions and macros are set in this file.
* \date         July 6th, 2021
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2021 Centro "E.Piaggio". All rights reserved.
*/


#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED
// -----------------------------------------------------------------------------

//=================================================================     includes
#include <stdlib.h>
#include <math.h>
#include "commands.h"
#include "utils.h"
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "dwt_delay.h"

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;  // Used for the IMU
extern SPI_HandleTypeDef hspi2;  // Used for the Encoders
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_hadc1;
extern FLASH_EraseInitTypeDef mem_settings;

//==============================================================================
//                                                                        DEVICE
//==============================================================================

#define VERSION                 "STM32 Board v 0.8 r2023 No LP Filter"

#define N_IMU_MAX               19 	// 18 CS pin + 1 IMU on the board
#define NUM_OF_DATA              5  // accelerometers, gyroscopes, magnetometers, quaternion and temperature data

#define N_ENCODER_MAX           18 // Max number of CS which can contain encoders
#define N_CYCLES_MAX            22

#define NUM_OF_SENSORS          3       /*!< Number of encoders.*/
#define NUM_OF_EMGS             2       /*!< Number of emg channels.*/
#define NUM_OF_ANALOG_INPUTS    4       /*!< Total number of analogic inputs.*/


//==============================================================================
//                                                               SYNCHRONIZATION
//==============================================================================

//Main frequency 1000 Hz
#define CALIBRATION_DIV         10     // 100 Hz
#define DIV_INIT_VALUE          1

//==============================================================================
//                                                                           DMA
//==============================================================================
    
#define DMA_BYTES_PER_BURST 2
#define DMA_REQUEST_PER_BURST 1
#define DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_DST_BASE (CYDEV_SRAM_BASE)
    
//==============================================================================
//                                                                     INTERRUPT
//==============================================================================
#define WAIT_START   0
#define WAIT_ID      1
#define WAIT_LENGTH  2
#define RECEIVE      3
#define UNLOAD       4

//==============================================================================
//                                                      SPI DELAY (MICROSECONDS)
//==============================================================================
#define SPI_DELAY_LOW       10
#define SPI_DELAY_HIGH      100
    
//==============================================================================
//                                                                         OTHER
//==============================================================================

#define FALSE                   0
#define TRUE                    1

#define DEFAULT_EEPROM_DISPLACEMENT 100   // in pages
#define FLASH_STARTING_ADDRESS 0x08060000

#define LOOKUP_DIM              6       // Dimension of the current lookup table
#define ANTI_WINDUP             1000    // Anti windup saturation
#define DEFAULT_CURRENT_LIMIT   1500    // Default Current limit, 0 stands for unlimited
#define RIGHT_HAND              -1
#define LEFT_HAND               1


//==============================================================================
//                                                        structures definitions
//==============================================================================
    
//==============================================================     data packet

struct st_data {
    uint8_t   buffer[128];            // CMD/DATA/CHECKSUM
    int16_t   length;                 // length
    int16_t   ind;                    // index
    uint8_t   ready;                  // Flag
};

//============================================     settings stored on the memory

struct st_mem {

	 uint16_t   flag;                       // Device has been configured               1
     uint16_t   id;                         // device id                                1
     uint16_t   baud_rate;                  // Baud Rate Setted                         1
     uint8_t	read_imu;
     uint8_t	read_encoders;
     uint8_t	read_motor;
     uint8_t	read_exp_port;
     uint16_t   IMU_conf[N_IMU_MAX][NUM_OF_DATA];     // IMU default configuration     85

     uint16_t   SPI_read_delay;             // delay on SPI reading
     uint8_t    control_mode;               // Motor Control mode                       1
     uint8_t    pos_lim_flag;               // Position limit active/inactive           1
     int32_t    pos_lim_inf;                // Inferior position limit for motor        4
     int32_t    pos_lim_sup;                // Superior position limit for motor        4
     uint8_t    res[NUM_OF_SENSORS];        // Angle resolution                         3
     uint8_t    maint_day;                  // Day of maintenance                       1
     uint8_t    maint_month;                // Month of maintenance                     1
     uint8_t    maint_year;                 // Year of maintenance                      1
     uint32_t   position_hist[10];          // Positions histogram - 10 zones          40
     int16_t    current_limit;              // Limit for absorbed current               2
     uint32_t   current_hist[4];            // Current histogram - 4 zones             16
     uint32_t   emg_counter[2];             // Counter for EMG activation - both channels 8
     uint32_t   rest_counter;               // Counter for rest position occurrences    4
     uint32_t   wire_disp;                  // Counter for total wire displacement measurement 4
     uint32_t   total_time_on;              // Total time of system power (in seconds)  4
     uint32_t   total_time_rest;            // Total time of system while rest position is maintained 4
     int32_t    k_p;                        // Position controller proportional constant 4
     int32_t    k_i;                        // Position controller integrative constant  4
     int32_t    k_d;                        // Position controller derivative constant   4
     int32_t    k_p_c;                      // Current controller proportional constant  4     18
     //End of row one. K_p_c is half on row one and half on row two.
     int32_t    k_i_c;                      // Current controller integrative constant   4
     int32_t    k_d_c;                      // Current controller derivative constant    4
     int32_t    k_p_dl;                     // Double loop position controller prop. constant 4
     int32_t    k_i_dl;                     // Double loop position controller integr. constant 4     16
     //End of row two. K_i_dl is half on row two and half on row three.
     int32_t    k_d_dl;                     // Double loop position controller deriv. constant 4
     int32_t    k_p_c_dl;                   // Double loop current controller prop. constant 4
     int32_t    k_i_c_dl;                   // Double loop current controller integr. constant 4
     int32_t    k_d_c_dl;                   // Double loop current controller deriv. constant 4     16
     //End of row three. K_d_c_dl is half on row three and half on row four.
     uint8_t    activ;                      // Startup activation                        1
     uint8_t    input_mode;                 // Motor Input mode                          1
     uint8_t    activate_pwm_rescaling;     // Activation of PWM rescaling for 12V motor 1     4
     float      m_mult[NUM_OF_SENSORS];     // Measurement multiplier                   12    12
     int32_t    m_off[NUM_OF_SENSORS];      // Measurement offset                       12    18
     int32_t    max_step_neg;               // Maximum number of steps per cycle for negative positions 4
     int32_t    max_step_pos;               // Maximum number of steps per cycle for positive positions 4     17
     uint16_t   emg_threshold[NUM_OF_EMGS]; // Minimum value for activation of EMG control 4
     uint8_t    emg_calibration_flag;       // Enable emg calibration on startup           1
     uint32_t   emg_max_value[NUM_OF_EMGS]; // Maximum value for EMG                     8     15
     uint8_t    emg_speed;                  // Maximum closure speed when using EMG      1
     uint8_t    double_encoder_on_off;      // Double encoder ON/OFF                     1
     int8_t     motor_handle_ratio;         // Discrete multiplier for handle device     1
     float      curr_lookup[LOOKUP_DIM];    // Table of values to get estimated curr    24    24
     int32_t    rest_pos;                   // Hand rest position while in EMG mode      4
     int32_t    rest_delay;                 // Hand rest position delay while in EMG mode 4
     int32_t    rest_vel;                   // Hand velocity closure for rest position reaching 4
     uint8_t    rest_position_flag;         // Enable rest position feature              1
     uint8_t    switch_emg;                 // EMG opening/closure switch                1
     int8_t     right_left;                 // Right/Left hand                           1
     uint8_t    unused_bytes[17];

};

//=============================================================     measurements
/** \brief Measurements structure
 *
**/
struct st_meas {
    int32_t pos[NUM_OF_SENSORS];      /*!< Encoder sensor position.*/
    int32_t curr;                     /*!< Motor current.*/
    int32_t estim_curr;               /*!< Current estimation.*/
    int8_t rot[NUM_OF_SENSORS];       /*!< Encoder sensor rotations.*/

    int32_t emg[NUM_OF_EMGS];         /*!< EMG sensors values.*/
    int32_t vel[NUM_OF_SENSORS];      /*!< Encoder rotational velocity.*/
    int32_t acc[NUM_OF_SENSORS];      /*!< Encoder rotational acceleration.*/
};

//=========================================================     motor references
/** \brief Motor Reference structure
 *
**/
struct st_ref {
    int32_t pos;                      /*!< Motor position reference.*/
    int32_t curr;                     /*!< Motor current reference.*/
    int32_t pwm;                      /*!< Motor direct pwm control.*/
    uint8_t onoff;                    /*!< Motor drivers enable.*/
    int16_t pos_encoder_ref; //[GS]
};


struct st_imu {
    uint8_t flags;        // Flags to know what we are reading (0/1) from each imu [ accel | gyro | magn ]
    int16_t accel_value[3];
    int16_t gyro_value[3];
    int16_t mag_value[3];
    float quat_value[4];
    int16_t temp_value;
};

//====================================      external global variables definition

extern struct st_data   g_rx;                       // Incoming Data
extern struct st_mem    g_mem, c_mem;               // Memory
extern struct st_meas   g_meas, g_measOld;          // Measurements
extern struct st_ref    g_ref, g_refNew, g_refOld;  // Reference variables //

extern uint32_t timer_value;
extern uint32_t timer_value0;
extern float execution_time_ms;
extern float execution_time_us;

extern ADC_ChannelConfTypeDef sConfig_global;
//extern uint16_t adc_aux_var;
extern uint16_t ADCValues[5];
extern uint8_t ChipTemperature;

// Bit Flag
extern uint8_t interrupt_flag;                        // interrupt flag enabler
extern uint8_t Rx_data;
extern uint8_t Rx_counter;
extern uint8_t Rx_Buffer_size;
extern uint8_t Rx_buffer[20];

// IMU variables
extern uint8_t N_IMU_Connected;
extern uint8_t IMU_connected[N_IMU_MAX];
extern int imus_data_size;
extern int single_imu_size[N_IMU_MAX];
extern struct st_imu g_imu[N_IMU_MAX], g_imuNew[N_IMU_MAX];

extern uint8_t Accel[N_IMU_MAX][6];
extern uint8_t Gyro[N_IMU_MAX][6];
extern uint8_t Mag[N_IMU_MAX][6];
extern uint8_t MagCal[N_IMU_MAX][3];
extern uint8_t Temp[N_IMU_MAX][2];
extern float Quat[N_IMU_MAX][4];

//extern uint8_t packet_data_param[PARAM_BYTE_SLOT + 6*PARAM_BYTE_SLOT + 9*PARAM_BYTE_SLOT + PARAM_BYTE_SLOT + PARAM_BYTE_SLOT*N_IMU_MAX + PARAM_BYTE_SLOT + PARAM_MENU_SLOT + PARAM_BYTE_SLOT + 1];

// Encoder variables
extern uint8_t Encoder_Map[N_ENCODER_MAX]; // Used to map how many encoders are connected to each CS pin, there are 18 CS on the board and each of them can contain 8 encoders
//extern uint8_t N_Encoder;
extern uint8_t N_Encoder_Connected;
extern uint8_t Encoder_connected[N_ENCODER_MAX];
extern short Encoder_Value[N_ENCODER_MAX][8];
extern short Encoder_Check[N_ENCODER_MAX][8];
// -----------------------------------------------------------------------------

extern uint16_t encoder_aux_1;
extern uint16_t encoder_aux_2;
extern uint16_t encoder_aux_3;
extern uint16_t encoder_aux_4;
extern uint16_t encoder_aux_5;
extern uint16_t encoder_aux_6;
extern uint16_t encoder_aux_7;
extern uint16_t encoder_aux_8;

extern uint16_t check_aux_1;
extern uint16_t check_aux_2;
extern uint16_t check_aux_3;
extern uint16_t check_aux_4;
extern uint16_t check_aux_5;
extern uint16_t check_aux_6;
extern uint16_t check_aux_7;
extern uint16_t check_aux_8;

extern uint8_t change_ext_ref_flag;                   // This flag is set when an external reference command is received


#endif

//[] END OF FILE
