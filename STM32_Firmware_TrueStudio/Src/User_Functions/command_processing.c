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
* \file         command_processing.c
*
* \brief        Command processing functions.
* \date         July 6th, 2021
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2021 Centro "E.Piaggio". All rights reserved.
*/

//=================================================================     includes
#include "command_processing.h"
#include "interruptions.h"
#include "utils.h"
#include "globals.h"
#include "commands.h"
#include "IMU_functions.h"
//#include <STDIO.H>

//================================================================     variables
// [GS] QUESTA LA METTO DOPO
//reg8 * EEPROM_ADDR = (reg8 *) CYDEV_EE_BASE;

//==============================================================================
//                                                            RX DATA PROCESSING
//==============================================================================
//  This function checks for the availability of a data packet and process it:
//      - Verify checksum;
//      - Process commands;
//==============================================================================
void commProcess(){

    uint8_t rx_cmd;
    rx_cmd = g_rx.buffer[0];
	
//==========================================================     verify checksum

    if (!(LCRChecksum(g_rx.buffer, g_rx.length - 1) == g_rx.buffer[g_rx.length - 1])){
        // Wrong checksum
        g_rx.ready = 0;
        return;
    }


    switch(rx_cmd) {
       
//=========================================================     CMD_SET_BAUDRATE
            
//        case CMD_SET_BAUDRATE:
//            cmd_set_baudrate();
//            break;  
            
//=============================================================     CMD_GET_INFO

        case CMD_GET_INFO:
            infoGet( *((uint16_t *) &g_rx.buffer[1]));
            break;

//=========================================================     CMD_GET_EMG

        case CMD_GET_EMG:
             cmd_get_emg();
             break;

//============================================================     CMD_GET_PARAM
            
        case CMD_GET_IMU_PARAM :
        	get_IMU_param_list(g_rx.buffer[1] << 8 | g_rx.buffer[2]);
			break;

//=================================================================     CMD_PING
            
        case CMD_PING:
            cmd_ping();
            break;

//=========================================================     CMD_STORE_PARAMS
            
        case CMD_STORE_PARAMS:
            cmd_store_params();
            break;

//=================================================     CMD_STORE_DEFAULT_PARAMS

        case CMD_STORE_DEFAULT_PARAMS:
            if(memStore(DEFAULT_EEPROM_DISPLACEMENT))
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//=======================================================     CMD_RESTORE_PARAMS

        //case CMD_RESTORE_PARAMS:
        //    if(memRestore())
        //        sendAcknowledgment(ACK_OK);
        //    else
        //        sendAcknowledgment(ACK_ERROR);
        //    break;

//=============================================================     CMD_INIT_MEM

        case CMD_INIT_MEM:
            if(memInit())
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//===========================================================     CMD_BOOTLOADER

        case CMD_BOOTLOADER:
            sendAcknowledgment(ACK_OK);
            HAL_Delay(100);
            //HAL_GPIO_WritePin(FTDI_EN_GPIO_Port, FTDI_EN_Pin, 0);
            //HAL_Delay(100);
            BootLoaderInit();
            break;

//=====================================================     CMD_GET_IMU_READINGS

        case CMD_GET_IMU_READINGS:
            cmd_get_imu_readings();
            break;

//=====================================================     CMD_GET_IMU_READINGS

        case CMD_GET_ADC_RAW:
            cmd_get_adc_raw();
            break;

//===========================================================     CMD_SET_INPUTS

         case CMD_SET_INPUTS:
            cmd_set_inputs();
         break;

//=========================================================     CMD_GET_ACTIVATE

         case CMD_GET_ACTIVATE:
            cmd_get_activate();
         break;

//=============================================================     CMD_ACTIVATE
         case CMD_ACTIVATE:
            cmd_activate();
         break;
            
//============================================================     CMD_GET_INPUT

         case CMD_GET_INPUTS:
            cmd_get_inputs();
         break;

//=========================================================     CMD_GET_CURRENTS

         case CMD_GET_CURRENTS:
            cmd_get_currents();
         break;

//=========================================================     CMD_GET_CURR_DIFF

         case CMD_GET_VELOCITIES:
            cmd_get_velocities();
         break;

//=========================================================         CMD_GET_ACCEL

         case CMD_GET_ACCEL:
            cmd_get_accelerations();
         break;

//=========================================================== ALL OTHER COMMANDS
        default:
            break;
            
    }
}


//==============================================================================
//                                                                     INFO SEND
//==============================================================================
void infoSend(){
    unsigned char packet_string[2500];
    infoPrepare(packet_string);
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 1);
	HAL_UART_Transmit(&huart1, (uint8_t*)packet_string, strlen(packet_string) , 300);
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 0);
}


//==============================================================================
//                                                              COMMAND GET INFO
//==============================================================================
void infoGet(uint16_t info_type) {
    static unsigned char packet_string[2500] = " ";

    //==================================     choose info type and prepare string
    // [info_reading] 
    switch (info_type) {
        case INFO_ALL:
            infoPrepare(packet_string);
            HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 1);
		    HAL_UART_Transmit(&huart1, (uint8_t*)packet_string, strlen(packet_string), 300);
            HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 0);
            break;
        case INFO_READING:
        	infoReading(packet_string);
            HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 1);
			HAL_UART_Transmit(&huart1, (uint8_t*)packet_string, strlen(packet_string), 300);
            HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 0);
            break;
        //case CYCLES_INFO:
        //    prepare_counter_info(packet_string);
        //    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 1);
		//	HAL_UART_Transmit(&huart1, (uint8_t*)packet_string, strlen(packet_string), 300);
        //    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 0);
        default:
            break;
    }

}

//==============================================================================
//                                                           PREPARE DEVICE INFO
//==============================================================================

void infoPrepare(unsigned char *info_string)
{
    int i,j;
    unsigned char str[2500]= "";
    if(c_mem.id != 250){                //To avoid dummy board ping
        strcpy(info_string, "");
        strcat(info_string, "\r\n");
        strcat(info_string, "Firmware version: ");
        strcat(info_string, VERSION);
        strcat(info_string, "\r\n\r\n");

        strcat(info_string, "DEVICE INFO\r\n");
        sprintf(str, "ID: %d\r\n", (uint16_t)(c_mem.id));
        strcat(info_string, str);
        
        if (g_mem.read_imu) {

        	sprintf(str, "IMU Connected: %d\r\n", (uint16_t) N_IMU_Connected);
        	strcat(info_string, str);
        	strcat(info_string, "\r\n");
        
        	strcat(info_string, "IMUs CONFIGURATION\r\n");
        	for (i=0; i<N_IMU_Connected; i++){
        		sprintf(str, "Imu %d \r\n\tID: %d\r\n", i, (uint16_t) IMU_connected[i]);
        		strcat(info_string, str);
            
        		sprintf(str, "\tAccelerometers: ");
        		if ((c_mem.IMU_conf[IMU_connected[i]][0]))
        			strcat(str, "YES\r\n");
        		else
        			strcat(str, "NO\r\n");
        		strcat(str, "\tGyroscopes: ");
        		if ((c_mem.IMU_conf[IMU_connected[i]][1]))
        			strcat(str, "YES\r\n");
        		else
        			strcat(str, "NO\r\n");
        		strcat(str, "\tMagnetometers: ");
        		if ((c_mem.IMU_conf[IMU_connected[i]][2]))
                strcat(str, "YES\r\n");
        		else
        			strcat(str, "NO\r\n");
        		strcat(str, "\tQuaternion: ");
        		if ((c_mem.IMU_conf[IMU_connected[i]][3]))
        			strcat(str, "YES\r\n");
        		else
        			strcat(str, "NO\r\n");
        		strcat(str, "\tTemperature: ");
        		if ((c_mem.IMU_conf[IMU_connected[i]][4]))
        			strcat(str, "YES\r\n");
        		else
        			strcat(str, "NO\r\n");
                 
        		strcat(info_string, str);
        	}

        	strcat(info_string, "\r\n");
        }
        
        if (g_mem.read_encoders) {

        	sprintf(str, "Number of Connected Encoder: %d", N_Encoder_Connected);
        	strcat(info_string, str);
        	strcat(info_string, "\r\n");

        	sprintf(str, "Encoder Map: [");
        	strcat(info_string, str);
        	for (i=0; i<N_ENCODER_MAX-1; i++) {
        		sprintf(str, "%d, ", Encoder_Map[i]);
        		strcat(info_string, str);
        	}
        	sprintf(str, "%d", Encoder_Map[N_ENCODER_MAX-1]);
        	strcat(info_string, str);
    		sprintf(str, "]");
    		strcat(info_string, str);
        	strcat(info_string, "\r\n");

        	for (i=0; i<N_ENCODER_MAX; i++) {
        		if (Encoder_Map[i] != 0) {
                	sprintf(str, "ENCODERS LINE %d: ",i);
                	strcat(info_string, str);
                	strcat(info_string, "\r\n");
                	for (j=0; j<Encoder_Map[i]; j++){
                    	sprintf(str, "Encoder %d Measurement: %u", j+1, Encoder_Value[i][j]);
                    	strcat(info_string, str);
                    	sprintf(str, ", Check Value Encoder %d: %u", j+1, Encoder_Check[i][j]);
                    	strcat(info_string, str);
                    	strcat(info_string, "\r\n");
                	}
        		}
        	}

        }
        
        if (g_mem.read_motor) {


//                switch(c_mem.right_left){
//                    case RIGHT_HAND:
//                        strcat(info_string, "Hand side: RIGHT\r\n");
//                        break;
//                    case LEFT_HAND:
//                        strcat(info_string, "Hand side: LEFT\r\n");
//                        break;
//                }

//                strcat(info_string, "PWM rescaling activation: ");
//                if(c_mem.activate_pwm_rescaling == MAXON_12V)
//                    strcat(info_string, "YES\n");
//                else
//                    strcat(info_string, "NO\n");

//                sprintf(str, "PWM Limit: %d\r\n", (int) dev_pwm_limit);
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");
//
                strcat(info_string, "MOTOR INFO\r\n");
                strcat(info_string, "Motor reference");

                if(g_mem.control_mode == CONTROL_CURRENT)
                    strcat(info_string," - Currents: ");
                else {
                    if (g_mem.control_mode == CONTROL_PWM)
                        strcat(info_string," - Pwm: ");
                    else
                        strcat(info_string," - Position: ");
                }

                if(g_mem.control_mode == CONTROL_CURRENT) {
                    sprintf(str, "%d ", (int)(g_refOld.curr));
                    strcat(info_string,str);
                }
                else {
                    if(g_mem.control_mode == CONTROL_PWM) {
                        sprintf(str, "%d ", (int)(g_refOld.pwm));
                        strcat(info_string,str);
                    }
                    else {
                        sprintf(str, "%d ", (int)(g_refOld.pos >> c_mem.res[0]));
                        strcat(info_string,str);
                    }
                }
                strcat(info_string,"\r\n");

                sprintf(str, "Motor enabled: ");
                if (g_ref.onoff & 0x01) {
                    strcat(str, "YES\r\n");
                } else {
                    strcat(str, "NO\r\n");
                }
                strcat(info_string, str);

//                strcat(info_string, "\r\nMEASUREMENTS INFO\r\n");
//                strcat(info_string, "Sensor value:\r\n");
//                for (i = 0; i < NUM_OF_SENSORS; i++) {
//                    sprintf(str, "%d -> %d", i+1,
//                    (int)(g_meas.pos[i] >> c_mem.res[i]));
//                    strcat(info_string, str);
//                    strcat(info_string, "\r\n");
//                }

//                sprintf(str, "Battery Voltage (mV): %ld", (int32) dev_tension );
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                sprintf(str, "Full charge power tension (mV): %ld", (int32) pow_tension );
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                sprintf(str, "Current (mA): %ld", (int32) g_meas.curr );
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                strcat(info_string, "\r\nDEVICE PARAMETERS\r\n");

//                strcat(info_string, "PID Controller:\r\n");
//                if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
//                    sprintf(str, "P -> %f  ", ((double) c_mem.k_p / 65536));
//                    strcat(info_string, str);
//                    sprintf(str, "I -> %f  ", ((double) c_mem.k_i / 65536));
//                    strcat(info_string, str);
//                    sprintf(str, "D -> %f\r\n", ((double) c_mem.k_d / 65536));
//                    strcat(info_string, str);
//                }
//                else {
//                    sprintf(str, "P -> %f  ", ((double) c_mem.k_p_dl / 65536));
//                    strcat(info_string, str);
//                    sprintf(str, "I -> %f  ", ((double) c_mem.k_i_dl / 65536));
//                    strcat(info_string, str);
//                    sprintf(str, "D -> %f\r\n", ((double) c_mem.k_d_dl / 65536));
//                    strcat(info_string, str);
//                }

//                strcat(info_string, "Current PID Controller:\r\n");
//                if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
//                    sprintf(str, "P -> %f  ", ((double) c_mem.k_p_c / 65536));
//                    strcat(info_string, str);
//                    sprintf(str, "I -> %f  ", ((double) c_mem.k_i_c / 65536));
//                    strcat(info_string, str);
//                    sprintf(str, "D -> %f\r\n", ((double) c_mem.k_d_c / 65536));
//                    strcat(info_string, str);
//                }
//                else {
//                    sprintf(str, "P -> %f  ", ((double) c_mem.k_p_c_dl / 65536));
//                    strcat(info_string, str);
//                    sprintf(str, "I -> %f  ", ((double) c_mem.k_i_c_dl / 65536));
//                    strcat(info_string, str);
//                    sprintf(str, "D -> %f\r\n", ((double) c_mem.k_d_c_dl / 65536));
//                    strcat(info_string, str);
//                }

//                strcat(info_string, "\r\n");

//                if (c_mem.activ == 0x01)
//                    strcat(info_string, "Startup activation: YES\r\n");
//                else
//                    strcat(info_string, "Startup activation: NO\r\n");

//                switch(c_mem.input_mode) {
//                    case INPUT_MODE_EXTERNAL:
//                        strcat(info_string, "Input mode: USB\r\n");
//                        break;
//                    case INPUT_MODE_ENCODER3:
//                        strcat(info_string, "Input mode: Handle\r\n");
//                        break;
//                    case INPUT_MODE_EMG_PROPORTIONAL:
//                        strcat(info_string, "Input mode: EMG proportional\r\n");
//                        break;
//                    case INPUT_MODE_EMG_INTEGRAL:
//                        strcat(info_string, "Input mode: EMG integral\r\n");
//                        break;
//                    case INPUT_MODE_EMG_FCFS:
//                        strcat(info_string, "Input mode: EMG FCFS\r\n");
//                        break;
//                    case INPUT_MODE_EMG_FCFS_ADV:
//                        strcat(info_string, "Input mode: EMG FCFS ADV\r\n");
//                        break;
//                }

                switch(c_mem.control_mode) {
                    case CONTROL_ANGLE:
                        strcat(info_string, "Control mode: Position\r\n");
                        break;
                    case CONTROL_PWM:
                        strcat(info_string, "Control mode: PWM\r\n");
                        break;
                    case CONTROL_CURRENT:
                        strcat(info_string, "Control mode: Current\r\n");
                        break;
                    case CURR_AND_POS_CONTROL:
                        strcat(info_string, "Control mode: Position and Current\r\n");
                        break;
                    default:
                        break;
                }

//                if (c_mem.double_encoder_on_off)
//                    strcat(info_string, "Absolute encoder position: YES\r\n");
//                else
//                    strcat(info_string, "Absolute encoder position: NO\r\n");

//                sprintf(str, "Motor-Handle Ratio: %d\r\n", (int)c_mem.motor_handle_ratio);
//                strcat(info_string, str);

//                strcat(info_string, "Sensor resolution:\r\n");
//                for (i = 0; i < NUM_OF_SENSORS; ++i) {
//                    sprintf(str, "%d -> %d", (int) (i + 1), (int) c_mem.res[i]);
//                    strcat(info_string, str);
//                    strcat(info_string, "\r\n");
//                }

//                strcat(info_string, "Measurement Offset:\r\n");
//                for (i = 0; i < NUM_OF_SENSORS; ++i) {
//                    sprintf(str, "%d -> %ld", (int) (i + 1), (int32) c_mem.m_off[i] >> c_mem.res[i]);
//                    strcat(info_string, str);
//                    strcat(info_string, "\r\n");
//                }

//                strcat(info_string, "Measurement Multiplier:\r\n");
//                for (i = 0; i < NUM_OF_SENSORS; ++i) {
//                    sprintf(str,"%d -> %f", (int)(i + 1), (double) c_mem.m_mult[i]);
//                    strcat(info_string, str);
//                    strcat(info_string,"\r\n");
//                }

//                strcat(info_string, "Current lookup table:\r\n");
//                sprintf(str, "p[0] - p[2]: %f, %f, %f\n", c_mem.curr_lookup[0], c_mem.curr_lookup[1], c_mem.curr_lookup[2]);
//                strcat(info_string, str);
//                sprintf(str, "p[3] - p[5]: %f, %f, %f\n", c_mem.curr_lookup[3], c_mem.curr_lookup[4], c_mem.curr_lookup[5]);
//                strcat(info_string, str);
//                strcat(info_string,"\r\n");

//                sprintf(str, "Position limit active: %d", (int)g_mem.pos_lim_flag);
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                sprintf(str, "Position limit motor: inf -> %ld  ", (int32)g_mem.pos_lim_inf >> g_mem.res[0]);
//                strcat(info_string, str);

//                sprintf(str, "sup -> %ld\r\n", (int32)g_mem.pos_lim_sup >> g_mem.res[0]);
//                strcat(info_string, str);

//                sprintf(str, "Max step pos and neg: %d %d", (int)g_mem.max_step_pos, (int)g_mem.max_step_neg);
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                sprintf(str, "Current limit: %d", (int)g_mem.current_limit);
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                sprintf(str, "EMG thresholds [0 - 1024]: %u, %u", g_mem.emg_threshold[0], g_mem.emg_threshold[1]);
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                sprintf(str, "EMG max values [0 - 4096]: %lu, %lu", g_mem.emg_max_value[0], g_mem.emg_max_value[1]);
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                if (g_mem.emg_calibration_flag)
//                    strcat(info_string, "Calibration enabled: YES\r\n");
//                else
//                    strcat(info_string, "Calibration enabled: NO\r\n");

//                sprintf(str, "EMG max speed: %d", (int)g_mem.emg_speed);
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                sprintf(str, "Rest time delay (ms): %d", (int)g_mem.rest_delay);
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                sprintf(str, "Rest velocity closure (ticks/sec): %d", (int)g_mem.rest_vel);
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                sprintf(str, "Rest position: %d", (int)(g_mem.rest_pos >> c_mem.res[0]));
//                strcat(info_string, str);
//                strcat(info_string, "\r\n");

//                prepare_counter_info(info_string);

        }

		sprintf(str, "Expansion Port: %d", (uint32_t)(g_mem.read_exp_port));
        strcat(info_string, str);
        strcat(info_string, "\r\n");

		sprintf(str, "Chip Temperature: %d Celsius", (uint32_t)(ChipTemperature));
        strcat(info_string, str);
        strcat(info_string, "\r\n");

		sprintf(str, "Time between two reading cycles: %ld us", (uint32_t)(execution_time_us));
        strcat(info_string, str);
        strcat(info_string, "\r\n");



    }
}

void infoReading(unsigned char* info_string)
{
    int i;
    unsigned char str[100];
    
    strcpy(info_string, "");
    strcat(info_string, "\r\n");           
    strcat(info_string, "SENSORS INFO\r\n");
    for (i=0; i<N_IMU_Connected; i++){
        sprintf(str, "Imu %d \r\n\tID: %d\r\n", i, (int) IMU_connected[i]);
        strcat(info_string, str);
        
			sprintf(str, "\tMagCal Parameters %d\t%d\t%d\r\n", (uint8_t) MagCal[IMU_connected[i]][0], (uint8_t) MagCal[IMU_connected[i]][1], (uint8_t) MagCal[IMU_connected[i]][2]);
        strcat(info_string, str);
			
        if ((c_mem.IMU_conf[IMU_connected[i]][0])){
            sprintf(str, "\tAcc (Norm on last column): %d\t%d\t%d\t%d\r\n", (int16_t) g_imu[i].accel_value[0], (int16_t) g_imu[i].accel_value[1],(int16_t) g_imu[i].accel_value[2], (int16_t) sqrt(g_imu[i].accel_value[0]*g_imu[i].accel_value[0] + g_imu[i].accel_value[1]*g_imu[i].accel_value[1] + g_imu[i].accel_value[2]*g_imu[i].accel_value[2]));
            strcat(info_string, str);
        }

        if ((c_mem.IMU_conf[IMU_connected[i]][1])){
            sprintf(str, "\tGyro: %d\t%d\t%d\r\n", (int16_t) g_imu[i].gyro_value[0], (int16_t) g_imu[i].gyro_value[1],(int16_t) g_imu[i].gyro_value[2]);
            strcat(info_string, str);
        }

        if ((c_mem.IMU_conf[IMU_connected[i]][2])){
            sprintf(str, "\tMag: %d\t%d\t%d\r\n", (int16_t) g_imu[i].mag_value[0], (int16_t) g_imu[i].mag_value[1],(int16_t) g_imu[i].mag_value[2]);
            strcat(info_string, str);
        }
        
        if ((c_mem.IMU_conf[IMU_connected[i]][3])){
            sprintf(str, "\tQuat (Norm on last column): %.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n", (float) g_imu[i].quat_value[0], (float) g_imu[i].quat_value[1],(float) g_imu[i].quat_value[2], (float) g_imu[i].quat_value[3] , (float)sqrt(g_imu[i].quat_value[0]*g_imu[i].quat_value[0]+g_imu[i].quat_value[1]*g_imu[i].quat_value[1]+g_imu[i].quat_value[2]*g_imu[i].quat_value[2]+g_imu[i].quat_value[3]*g_imu[i].quat_value[3]));
            strcat(info_string, str);
        }
        
        if ((c_mem.IMU_conf[IMU_connected[i]][4])){
            sprintf(str, "\tTemperature: %d\r\n", (int16_t) g_imu[i].temp_value);
            strcat(info_string, str);
        }
    }
		
	strcat(info_string, "\r\n");
        
    sprintf(str, "Time between two reading cycles: %.3f ms", execution_time_ms);
    strcat(info_string, str);
    strcat(info_string, "\r\n");
        
	sprintf(str, "Time between two reading cycles: %ld us", (uint32_t)(execution_time_us));
    strcat(info_string, str);
    strcat(info_string, "\r\n");
    strcat(info_string, "\r\n");

}

//==============================================================================
//                                                   PREPARE GENERIC DEVICE INFO
//==============================================================================

void prepare_counter_info(char *info_string) // [GS] Non Testata
{
    char str[100];
    int i;
    int step;

    strcat(info_string, "\r\nUSAGE STATISTICS\r\n");
    strcat(info_string, "\r\n");

    sprintf(str, "Date of maintenance: %d/%d/20%d\r\n", (int16_t)g_mem.maint_day, (int16_t)g_mem.maint_month, (int16_t)g_mem.maint_year);
    strcat(info_string, str);

    sprintf(str, "Positions histogram (ticks):\r\n");
    strcat(info_string, str);
    step = ( (int)(g_mem.pos_lim_sup >> g_mem.res[0]) / 10);
    for (i=1; i<=10;i++){
        sprintf(str, "Bin %d [%d-%d]: %lu\r\n", i, step*(i-1)+1, step*(i), g_mem.position_hist[i-1]);
        strcat(info_string, str);
    }
    strcat(info_string, "\r\n");

    sprintf(str, "Current histogram (mA):\r\n");
    strcat(info_string, str);
    step = ( (int)(g_mem.current_limit) / 4);
    for (i=1; i<=4;i++){
        sprintf(str, "Threshold %d [%d-%d]: %lu\r\n", i, step*(i-1), step*(i), g_mem.current_hist[i-1]);
        strcat(info_string, str);
    }
    strcat(info_string, "\r\n");

    sprintf(str, "EMG activations counter: %lu, %lu", g_mem.emg_counter[0], g_mem.emg_counter[1]);
    strcat(info_string, str);
    strcat(info_string, "\r\n");

    sprintf(str, "Rest position occurrences: %lu", g_mem.rest_counter);
    strcat(info_string, str);
    strcat(info_string, "\r\n");

    sprintf(str, "Angle total displacement (ticks): %lu", g_mem.wire_disp);
    strcat(info_string, str);
    strcat(info_string, "\r\n");

    sprintf(str, "Total power on time (sec): %lu", g_mem.total_time_on);
    strcat(info_string, str);
    strcat(info_string, "\r\n");

    sprintf(str, "Total rest position time (sec): %lu", g_mem.total_time_rest);
    strcat(info_string, str);
    strcat(info_string, "\r\n");

}

//==============================================================================
//                                                      WRITE FUNCTION FOR RS485
//==============================================================================

void commWrite(uint8_t *packet_data, const uint16_t packet_lenght)
{
	
		uint16_t aux_var = packet_lenght; 
		uint8_t packet_lenght_high = aux_var >> 8;
		aux_var = aux_var << 8;
		uint8_t packet_lenght_low = aux_var >> 8;		
		// frame - start
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 1);
		HAL_UART_Transmit(&huart1, (uint8_t*)":", 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t*)":", 1, HAL_MAX_DELAY);
    
		// frame - ID
		HAL_UART_Transmit(&huart1, (uint8_t*)&g_mem.id, 1, HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart1, &packet_lenght_high, 1, 5);
		HAL_UART_Transmit(&huart1, &packet_lenght_low, 1, HAL_MAX_DELAY);			 
		// frame - packet data
		HAL_UART_Transmit(&huart1, packet_data, packet_lenght, HAL_MAX_DELAY);
	
		HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 0);
		HAL_GPIO_WritePin(RS485_CTS_GPIO_Port, RS485_CTS_Pin, 1);
		HAL_GPIO_WritePin(RS485_CTS_GPIO_Port, RS485_CTS_Pin, 0);

}

//==============================================================================
//                                                             CHECKSUM FUNCTION
//==============================================================================
// Performs a XOR byte by byte on the entire vector
uint8_t LCRChecksum(uint8_t *data_array, uint8_t data_length) {
    
    uint8_t i;
    uint8_t checksum = 0x00;
    
    for(i = 0; i < data_length; ++i)
       checksum ^= data_array[i];
  
    return checksum;
}

//==============================================================================
//                                                       ACKNOWLEDGMENT FUNCTION
//==============================================================================
void sendAcknowledgment(const uint8_t value) {
    int packet_lenght = 2;
    uint8_t packet_data[2];

    packet_data[0] = value;
    packet_data[1] = value;

    commWrite(packet_data, packet_lenght);
}

//==============================================================================
//                                                                  STORE MEMORY
//==============================================================================

/**
* This function stores current memory settings on the eeprom with the specified
* displacement
**/

uint8_t memStore(int displacement)
{
		// displacement is not used
    int i;  // iterator
    int blocks;
  	int flash_error = 0;
		HAL_StatusTypeDef error;
		uint16_t aux_var[blocks];
		uint32_t Flash_Sector_Error=0;
	
    memcpy( &c_mem, &g_mem, sizeof(g_mem) );
	  // How much blocks are written (each block is composed by 2 byte)
	  // Il sizeof counts byte e.g sizeof(int32) = 4
	  blocks = sizeof(g_mem) / 2 + (sizeof(g_mem) % 2 > 0);
	  HAL_FLASH_Unlock(); // Unlock the flash memory
		HAL_Delay(1);
												
		while (flash_error == 0){
			if (HAL_FLASHEx_Erase(&mem_settings, &Flash_Sector_Error) == HAL_OK) {
				flash_error = 1;
			}	
		} 
		
		HAL_Delay(1);	
		// 1 WORD = 4 byte											 
		for(i=0; i < blocks; i++) {
			  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (FLASH_STARTING_ADDRESS + 2*i), *(&g_mem.flag + i));
			  HAL_Delay(1);
		}	
		HAL_Delay(1);
		HAL_FLASH_Lock();
		
    memcpy( &g_mem, &c_mem, sizeof(g_mem) );
    HAL_Delay(100);
	return 0;
}
//==============================================================================
//                                                                 RECALL MEMORY
//==============================================================================

/**
* This function loads user settings from the eeprom.
**/

void memRecall() {
	uint16_t i;
	int blocks;
	uint16_t aux_var[blocks];
	blocks = sizeof(g_mem) / 2 + (sizeof(g_mem) % 2 > 0);
		//g_mem.id = *(uint8_t*)FLASH_STARTING_ADDRESS;
   for (i = 0; i < blocks; i++) {
			*(&g_mem.flag + i)  = *(uint16_t*)(FLASH_STARTING_ADDRESS + 2*i);	 
	 }

    //check for initialization
    if (g_mem.flag == TRUE) {
        memcpy( &c_mem, &g_mem, sizeof(g_mem) );
    }
}

//==============================================================================
//                                                                RESTORE MEMORY
//==============================================================================

/**
* This function loads default settings from the eeprom.
**/
// [GS] TUTTE QUESTE FUNZIONI LE REINSERISCO MAN MANO CHE MI SERVONO

//uint8_t memRestore() {
//    uint16_t i;
//		int blocks;
//  	int flash_error = 0;
//		uint32_t Flash_Sector_Error=0;
//   	blocks = sizeof(g_mem) / 2 + (sizeof(g_mem) % 2 > 0);

//    for (i = 0; i < blocks; i++) {
//			*(&g_mem.flag + i)  = *(uint16_t*)(FLASH_STARTING_ADDRESS + 2*i + DEFAULT_EEPROM_DISPLACEMENT);	 
//	  }	

    //check for initialization
//    if (g_mem.flag == FALSE) {
//        return memInit();
//    } else {
//        return memStore(0);
//    }
//}	

//==============================================================================
//                                                                   MEMORY INIT
//==============================================================================
/**
* This function initialize memory when eeprom is compromised.
**/

uint8_t memInit()
{
    uint8_t i;
    
    //initialize memory settings
    g_mem.id            = 1;

    // set the initialized flag to show EEPROM has been populated
    g_mem.flag = TRUE;
    
    // Default value
    for (i = 0; i< N_IMU_MAX; i++){
        g_mem.IMU_conf[i][0] = 1;
        g_mem.IMU_conf[i][1] = 1;
        g_mem.IMU_conf[i][2] = 0;
        g_mem.IMU_conf[i][3] = 0;
        g_mem.IMU_conf[i][4] = 0;
    }
    
    g_mem.SPI_read_delay = 0;
	g_mem.read_imu = 1;
	g_mem.read_encoders = 0;
	g_mem.read_motor = 0;
	g_mem.read_exp_port = 0;

	g_mem.k_p           =0.0165 * 65536;
	g_mem.k_i           =     0 * 65536;
	g_mem.k_d           = 0.007 * 65536;  // changed in order to avoid metallic clatter, previous value 0.2
	g_mem.k_p_c         =     1 * 65536;
	g_mem.k_i_c         = 0.001 * 65536;
	g_mem.k_d_c         =     0 * 65536;

	g_mem.k_p_dl        =   0.1 * 65536;
	g_mem.k_i_dl        =     0 * 65536;
	g_mem.k_d_dl        =     0 * 65536;
	g_mem.k_p_c_dl      =   0.3 * 65536;
	g_mem.k_i_c_dl      =0.0002 * 65536;
	g_mem.k_d_c_dl      =     0 * 65536;

    g_mem.activ         = 1;
    g_mem.input_mode    = INPUT_MODE_EMG_FCFS;
    g_mem.control_mode  = CONTROL_ANGLE;

    g_mem.pos_lim_flag = 1;

    g_mem.activate_pwm_rescaling = MAXON_24V;           //rescaling active for 12V motor

    g_mem.pos_lim_inf = 0;
    g_mem.pos_lim_sup = (int32_t)20000 << g_mem.res[0];

    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        g_mem.res[i] = 3;
        g_mem.m_mult[i] = 1;
        g_mem.m_off[i] = (int32_t)0 << g_mem.res[i];
    }

    g_mem.max_step_pos = 0;
    g_mem.max_step_neg = 0;

    g_mem.current_limit = DEFAULT_CURRENT_LIMIT;

    // EMG calibration enabled by default
    g_mem.emg_calibration_flag = 0;

    g_mem.emg_max_value[0] = 1024;
    g_mem.emg_max_value[1] = 1024;

    g_mem.emg_threshold[0] = 200;
    g_mem.emg_threshold[1] = 200;

    g_mem.emg_speed = 100;

    g_mem.double_encoder_on_off = 1;
    g_mem.motor_handle_ratio = 22;

    for(i = 0; i < LOOKUP_DIM; i++) {
        g_mem.curr_lookup[i] = 0;
    }

    //Initialize rest position parameters
    g_mem.rest_position_flag = 0;
    g_mem.rest_pos = (int32_t)7000 << g_mem.res[0]; // 56000
    g_mem.rest_delay = 10;
    g_mem.rest_vel = 10000;
    g_mem.switch_emg = 0;

    g_mem.right_left = RIGHT_HAND;

    for (i=0; i<17; i++){
        g_mem.unused_bytes[i] = 0;
    }

    //Initialize counters
    //reset_counters();
		
    //write that configuration to EEPROM
    return ( memStore(0) );
}

//==============================================================================
//                                                    ROUTINE INTERRUPT FUNCTION
//==============================================================================
/**
* Bunch of functions used on request from UART communication
**/
// [GS] TUTTE QUESTE FUNZIONI LE REINSERISCO MAN MANO CHE MI SERVONO
/*
void cmd_set_baudrate(){
    
    // Set BaudRate
    c_mem.baud_rate = g_rx.buffer[1];
    
    switch(g_rx.buffer[1]){
        case 13:
            CLOCK_UART_SetDividerValue(13);
            break;
        default:
            CLOCK_UART_SetDividerValue(3);
    }
}
*/

void cmd_ping(){

    uint8_t packet_data[2];

    // Header
    packet_data[0] = CMD_PING;
    
    // Load Payload
    packet_data[1] = CMD_PING;

    // Send Package to uart
    commWrite(packet_data, 2);
}

void cmd_store_params(){
   
    if(memStore(0))
        sendAcknowledgment(ACK_OK);
    else
        sendAcknowledgment(ACK_ERROR);
}

void cmd_get_imu_readings(){
    //Retrieve accelerometers, gyroscopes and magnetometers readings
    
    uint8_t k_imu;
    static uint16_t c = 1;
    uint8_t k = 0;
    uint16_t gl_c = 1;
	float q0,q1,q2,q3;
    uint16_t quat_int_scale = 20000;
    // Packet: header + imu id(uint8) + imu flags(uint8) + crc  
    uint8_t packet_data[1600];
    uint8_t single_packet[70];
    
    //Header package 
    packet_data[0] = CMD_GET_IMU_READINGS;
 					  
    for (k_imu = 0; k_imu < N_IMU_Connected; k_imu++) 
    {	
			
		single_packet[0] = (uint8_t) 0x3A; //':';
        if (c_mem.IMU_conf[IMU_connected[k_imu]][0]){ // Acceleromters 
            *((int8_t *) &single_packet[c+0]) = Accel[IMU_connected[k_imu]][0];
            *((int8_t *) &single_packet[c+1]) = Accel[IMU_connected[k_imu]][1];
            *((int8_t *) &single_packet[c+2]) = Accel[IMU_connected[k_imu]][2];
            *((int8_t *) &single_packet[c+3]) = Accel[IMU_connected[k_imu]][3];
            *((int8_t *) &single_packet[c+4]) = Accel[IMU_connected[k_imu]][4];
			*((int8_t *) &single_packet[c+5]) = Accel[IMU_connected[k_imu]][5];
            c = c + 6;
        }
        if (c_mem.IMU_conf[IMU_connected[k_imu]][1]){ // Gyroscopes
			*((int8_t *) &single_packet[c+0]) = Gyro[IMU_connected[k_imu]][0];
			*((int8_t *) &single_packet[c+1]) = Gyro[IMU_connected[k_imu]][1];
            *((int8_t *) &single_packet[c+2]) = Gyro[IMU_connected[k_imu]][2];
			*((int8_t *) &single_packet[c+3]) = Gyro[IMU_connected[k_imu]][3];
			*((int8_t *) &single_packet[c+4]) = Gyro[IMU_connected[k_imu]][4];
			*((int8_t *) &single_packet[c+5]) = Gyro[IMU_connected[k_imu]][5];
            c = c + 6;
        }
        if (c_mem.IMU_conf[IMU_connected[k_imu]][2]){ // Magnetometers
			*((int8_t *) &single_packet[c+0]) = Mag[IMU_connected[k_imu]][0];
			*((int8_t *) &single_packet[c+1]) = Mag[IMU_connected[k_imu]][1];
            *((int8_t *) &single_packet[c+2]) = Mag[IMU_connected[k_imu]][2];
			*((int8_t *) &single_packet[c+3]) = Mag[IMU_connected[k_imu]][3];
			*((int8_t *) &single_packet[c+4]) = Mag[IMU_connected[k_imu]][4];
			*((int8_t *) &single_packet[c+5]) = Mag[IMU_connected[k_imu]][5];
            c = c + 6;
        }
        if (c_mem.IMU_conf[IMU_connected[k_imu]][3]){ // Quaternions

        	q0 = (Quat[IMU_connected[k_imu]][0]);
        	q1 = (Quat[IMU_connected[k_imu]][1]);
        	q2 = (Quat[IMU_connected[k_imu]][2]);
        	q3 = (Quat[IMU_connected[k_imu]][3]);

        	for (k=0; k<4; k++){
            	*((int8_t *) &single_packet[c+k]) = ((char*)(&q0))[3-k]; // (uint8_t*)(q0[3]);
        	}
        	c=c+4;
        	for (k=0; k<4; k++){
            	*((int8_t *) &single_packet[c+k]) = ((char*)(&q1))[3-k]; // (uint8_t*)(q0[3]);
        	}
        	c=c+4;
        	for (k=0; k<4; k++){
            	*((int8_t *) &single_packet[c+k]) = ((char*)(&q2))[3-k]; // (uint8_t*)(q0[3]);
        	}
        	c=c+4;
        	for (k=0; k<4; k++){
            	*((int8_t *) &single_packet[c+k]) = ((char*)(&q3))[3-k]; // (uint8_t*)(q0[3]);
        	}
        	c=c+4;

        }
        if (c_mem.IMU_conf[IMU_connected[k_imu]][4]){ // Temperatures
            *((int8_t *) &single_packet[c+0]) = (int8_t) Temp[IMU_connected[k_imu]][0]; 
			*((int8_t *) &single_packet[c+1]) = (int8_t) Temp[IMU_connected[k_imu]][1];
            c = c + 2;
        }
        single_packet[single_imu_size[IMU_connected[k_imu]] - 1] = (uint8_t) 0x3A; //':';
        c = 1;
               
        for(k=0; k < single_imu_size[IMU_connected[k_imu]]; k++) {
            packet_data[gl_c + k] = (uint8_t) single_packet[k]; 
        }
        gl_c = gl_c + single_imu_size[IMU_connected[k_imu]];  
        
        memset(&single_packet, 0, sizeof(single_packet));     
        
    }

    // Calculate Checksum and send message to UART 
    packet_data[imus_data_size-1] = LCRChecksum (packet_data, imus_data_size-1);
    commWrite(packet_data, imus_data_size);
}

void cmd_get_emg(){

    uint8_t packet_data[6];

    // Header
    packet_data[0] = CMD_GET_EMG;

    *((int8_t *) &packet_data[1]) = (uint8_t)(g_measOld.emg[0] >> 8);
    *((int8_t *) &packet_data[2]) = (uint8_t)((g_measOld.emg[0] << 8) >> 8);
	*((int8_t *) &packet_data[3]) = (uint8_t)(g_measOld.emg[1] >> 8);
    *((int8_t *) &packet_data[4]) = (uint8_t)((g_measOld.emg[1] << 8) >> 8);

    packet_data[5] = LCRChecksum (packet_data, 5);

    commWrite(packet_data, 6);

}

uint16_t value_swap(uint16_t input) {

  uint8_t high, low;
	uint16_t aux_var;
	
  high = input >> 8;
	aux_var = (input << 8);
	low = aux_var >> 8;
	return ((low << 8) | high);
	
}

void cmd_get_adc_raw(){

	    uint8_t packet_data[6];
	    uint16_t adc_raw1, adc_raw2;

	    // Header
	    packet_data[0] = CMD_GET_ADC_RAW;

	    adc_raw1 = ADCValues[3];
	    adc_raw2 = ADCValues[4];

	    *((int8_t *) &packet_data[1]) = (uint8_t)(adc_raw1 >> 8);
	    *((int8_t *) &packet_data[2]) = (uint8_t)((adc_raw1 << 8) >> 8);
		*((int8_t *) &packet_data[3]) = (uint8_t)(adc_raw2 >> 8);
	    *((int8_t *) &packet_data[4]) = (uint8_t)((adc_raw2 << 8) >> 8);

	    packet_data[5] = LCRChecksum (packet_data, 5);

	    commWrite(packet_data, 6);

}

//==============================================================================
//                                                            GET PARAMETER LIST
//==============================================================================

void get_IMU_param_list(uint16_t index){

	//Auxiliary variables
    uint16_t i, j, k, h;
    uint16_t start_byte = 0;

    //Package to be sent variables
    uint16_t num_imus_id_params = 7;
    uint16_t num_mag_cal_params = 0;
    uint16_t first_imu_parameter = 2;

    uint8_t packet_data_param[PARAM_BYTE_SLOT + num_imus_id_params*PARAM_BYTE_SLOT + num_mag_cal_params*PARAM_BYTE_SLOT + PARAM_BYTE_SLOT + (uint16_t)(PARAM_BYTE_SLOT*N_IMU_Connected) + 5*PARAM_BYTE_SLOT + 3*PARAM_MENU_SLOT + PARAM_BYTE_SLOT + 1];
    int aux_lenght = PARAM_BYTE_SLOT + num_imus_id_params*PARAM_BYTE_SLOT + num_mag_cal_params*PARAM_BYTE_SLOT + PARAM_BYTE_SLOT + (uint16_t)(PARAM_BYTE_SLOT*N_IMU_Connected) + 5*PARAM_BYTE_SLOT + 3*PARAM_MENU_SLOT + PARAM_BYTE_SLOT + 1;

    for (i=0; i<aux_lenght; i++){
    	packet_data_param[i] = 0;
    }

    uint16_t packet_length = PARAM_BYTE_SLOT + num_imus_id_params*PARAM_BYTE_SLOT + num_mag_cal_params*PARAM_BYTE_SLOT + PARAM_BYTE_SLOT + (uint16_t)(PARAM_BYTE_SLOT*N_IMU_Connected) + 5*PARAM_BYTE_SLOT + 3*PARAM_MENU_SLOT + PARAM_BYTE_SLOT + 1;

    //Parameters menu string definitions
    char n_imu_str[]          = "Number of connected IMUs:";
    char ids_str[11]            = " ";
    char id_str[16]             = " ";
    char mag_param_str[20]      = "Mag cal parameters:";
    char imu_table_str[42]      = " ";
    char spi_read_delay_str[26] = " ";
    char imu_read_str[24]		= " ";
    char encoder_read_str[24]	= " ";
    char motor_read_str[21]		= " ";
    char exp_port_read_str[39]	= " ";

    //Strings lenghts

    uint8_t id_str_len = strlen(id_str);
    uint8_t n_imu_str_len = strlen(n_imu_str);
    uint8_t ids_str_len = strlen(ids_str);
    uint8_t mag_param_str_len = strlen(mag_param_str);
    uint8_t imu_table_str_len = strlen(imu_table_str);
    uint8_t spi_read_delay_str_len = strlen(spi_read_delay_str);
    uint8_t imu_read_str_len = strlen(imu_read_str);
    uint8_t encoder_read_str_len = strlen(encoder_read_str);
    uint8_t motor_read_str_len = strlen(motor_read_str);
    uint8_t exp_port_read_str_len = strlen(exp_port_read_str);
    char spi_delay_menu[118]    = " ";
    uint8_t spi_delay_menu_len;

    sprintf(spi_delay_menu, "0 -> None\n1 -> Low (%u us delay for each 8-bit register read)\n2 -> High (%u us delay for each 8-bit register read)\n", (int)SPI_DELAY_LOW, (int)SPI_DELAY_HIGH);
    spi_delay_menu_len = strlen(spi_delay_menu);

    char on_off_menu[51] = "0 -> OFF\n1 -> ON\nYou must restart the board now\n";
    char exp_port_menu[89] = "0 -> None\n1 -> SD/RTC board\n2 -> WiFi board\n3 -> General\nYou must restart the board now\n";
    uint8_t on_off_menu_len = strlen(on_off_menu);
    uint8_t exp_port_menu_len = strlen(exp_port_menu);


    // Compute number of read parameters depending on N_IMU_Connected and
    // update packet_length
    num_mag_cal_params = (uint16_t)(N_IMU_Connected / 2);
    if ( (N_IMU_Connected - num_mag_cal_params*2) > 0 ) {num_mag_cal_params++;}

    packet_length = PARAM_BYTE_SLOT + num_imus_id_params*PARAM_BYTE_SLOT + num_mag_cal_params*PARAM_BYTE_SLOT + PARAM_BYTE_SLOT + (uint16_t)(PARAM_BYTE_SLOT*N_IMU_Connected) + 5*PARAM_BYTE_SLOT + 3*PARAM_MENU_SLOT + PARAM_BYTE_SLOT + 1;


    first_imu_parameter = 1 + num_imus_id_params + num_mag_cal_params + 2;
    packet_data_param[0] = CMD_GET_IMU_PARAM ;
    packet_data_param[1] = 1 + num_imus_id_params + num_mag_cal_params + 1 + (uint8_t)N_IMU_Connected + 5;        // NUM_PARAMS


    switch(index) {
        case 0:
			//List of all parameters with relative types
			/*-------------N IMU--------------*/
            start_byte = 0;
            packet_data_param[2] = TYPE_UINT8;
            packet_data_param[3] = 1;
            packet_data_param[4] = (uint8_t)N_IMU_Connected;
            for(i = n_imu_str_len; i != 0; i--)
                {packet_data_param[5 + n_imu_str_len - i] = n_imu_str[n_imu_str_len - i];}

            /*-------------IMUS ID--------------*/
			start_byte = start_byte + PARAM_BYTE_SLOT;
            i = 0;
            for (k = 0; k < num_imus_id_params; k++){
                sprintf(ids_str, "Port %u ID:", k);
                h = 4;
                ids_str_len = strlen(ids_str);
                packet_data_param[2+start_byte + PARAM_BYTE_SLOT*k] = TYPE_UINT8;
                packet_data_param[3+start_byte + PARAM_BYTE_SLOT*k] = 3;

                for (j = 3*k; j <= 3*k+2; j++) {  // for each possible imu on port k
                    if (IMU_connected[i] == j) {
                        packet_data_param[h+start_byte + PARAM_BYTE_SLOT*k] = (uint8_t)IMU_connected[i];
                        i++;
                    } else {
                        packet_data_param[h+start_byte + PARAM_BYTE_SLOT*k] = 255;
                    }
                    h++;
                }

                for(j = ids_str_len; j != 0; j--)
                    packet_data_param[7+start_byte + PARAM_BYTE_SLOT*k + ids_str_len - j] = ids_str[ids_str_len - j];
			}

            /*-------------GET MAG PARAM--------------*/

			start_byte = start_byte + PARAM_BYTE_SLOT*num_imus_id_params; // num_imus_id_params = 6;
            for (k = 0; k < num_mag_cal_params; k++){ // num_mag_cal_params = 1
                packet_data_param[2+start_byte + PARAM_BYTE_SLOT*k] = TYPE_UINT8;
                packet_data_param[3+start_byte + PARAM_BYTE_SLOT*k] = 3;
                packet_data_param[4+start_byte + PARAM_BYTE_SLOT*k] = (uint8_t) MagCal[IMU_connected[2*k]][0];
                packet_data_param[5+start_byte + PARAM_BYTE_SLOT*k] = (uint8_t) MagCal[IMU_connected[2*k]][1];
                packet_data_param[6+start_byte + PARAM_BYTE_SLOT*k] = (uint8_t) MagCal[IMU_connected[2*k]][2];

                // check if there is a second value
                if ( N_IMU_Connected < 2*(k+1) ) {
                    // there is only one value
                    for(j = mag_param_str_len; j != 0; j--)
                        {packet_data_param[7+start_byte + PARAM_BYTE_SLOT*k + mag_param_str_len - j] = mag_param_str[mag_param_str_len - j];}
                } else {
                    // fill the second value
                    packet_data_param[3+start_byte + PARAM_BYTE_SLOT*k] = 6;
                    packet_data_param[7+start_byte + PARAM_BYTE_SLOT*k] = MagCal[IMU_connected[2*k+1]][0];
                    packet_data_param[8+start_byte + PARAM_BYTE_SLOT*k] = MagCal[IMU_connected[2*k+1]][1];
                    packet_data_param[9+start_byte + PARAM_BYTE_SLOT*k] = MagCal[IMU_connected[2*k+1]][2];
                    for(j = mag_param_str_len; j != 0; j--)
                        {packet_data_param[10+start_byte + PARAM_BYTE_SLOT*k + mag_param_str_len - j] = mag_param_str[mag_param_str_len - j];}
                }
            }

            /*-----------------ID-----------------*/
  			start_byte = start_byte + PARAM_BYTE_SLOT*num_mag_cal_params;
            sprintf(id_str, "%u - Device ID:", first_imu_parameter-1);
            id_str_len = strlen(id_str);
            packet_data_param[2+start_byte] = TYPE_UINT8;
            packet_data_param[3+start_byte] = 1;
            packet_data_param[4+start_byte] = c_mem.id;
            for(i = id_str_len; i != 0; i--)
                {packet_data_param[5+start_byte + id_str_len - i] = id_str[id_str_len - i];}

            /*-------------GET IMUS MODE-------------*/
			start_byte = start_byte + PARAM_BYTE_SLOT;
            for (i = 0; i < (uint8_t)N_IMU_Connected; i++){
                sprintf(imu_table_str, "%u - IMU %d configuration:", first_imu_parameter + i, (int) IMU_connected[i]);
                imu_table_str_len = strlen(imu_table_str);

                packet_data_param[(uint16_t)(2 + start_byte + PARAM_BYTE_SLOT*i)] = TYPE_UINT8;
                packet_data_param[(uint16_t)(3 + start_byte + PARAM_BYTE_SLOT*i)] = 5;

                packet_data_param[(uint16_t)(4 + start_byte + PARAM_BYTE_SLOT*i)] = (uint8_t)(c_mem.IMU_conf[IMU_connected[i]][0]);
                packet_data_param[(uint16_t)(5 + start_byte + PARAM_BYTE_SLOT*i)] = (uint8_t)(c_mem.IMU_conf[IMU_connected[i]][1]);
                packet_data_param[(uint16_t)(6 + start_byte + PARAM_BYTE_SLOT*i)] = (uint8_t)(c_mem.IMU_conf[IMU_connected[i]][2]);
                packet_data_param[(uint16_t)(7 + start_byte + PARAM_BYTE_SLOT*i)] = (uint8_t)(c_mem.IMU_conf[IMU_connected[i]][3]);
                packet_data_param[(uint16_t)(8 + start_byte + PARAM_BYTE_SLOT*i)] = (uint8_t)(c_mem.IMU_conf[IMU_connected[i]][4]);

                for(j = imu_table_str_len; j != 0; j--)
                    {packet_data_param[(uint16_t)(9 + start_byte + PARAM_BYTE_SLOT*i + imu_table_str_len - j)] = imu_table_str[imu_table_str_len - j];}
            }

            /*-----------------SPI DELAY-----------------*/

			start_byte = start_byte + (uint16_t)(PARAM_BYTE_SLOT*N_IMU_Connected) ;
            sprintf(spi_read_delay_str, "%u - SPI read delay:", first_imu_parameter+N_IMU_Connected);
            packet_data_param[2+start_byte] = TYPE_FLAG;
            packet_data_param[3+start_byte] = 1;
            packet_data_param[4+start_byte] = c_mem.SPI_read_delay;
            switch(c_mem.SPI_read_delay) {
                case 0:
                    strcat(spi_read_delay_str, " None");
                    spi_read_delay_str_len = 26;
                    break;
                case 1:
                    strcat(spi_read_delay_str, " Low");
                    spi_read_delay_str_len = 25;
                    break;
                case 2:
                    strcat(spi_read_delay_str, " High");
                    spi_read_delay_str_len = 26;
                    break;
                default:
					strcat(spi_read_delay_str, " Undefined");
                    spi_read_delay_str_len = 32;

                    break;
            }
            for(i = spi_read_delay_str_len; i != 0; i--)
                {packet_data_param[5+start_byte + spi_read_delay_str_len - i] = spi_read_delay_str[spi_read_delay_str_len - i];}
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data_param[5+start_byte + spi_read_delay_str_len] = 1;

            /*-----------------IMU READ-----------------*/

			start_byte = start_byte + (uint16_t)(PARAM_BYTE_SLOT) ;
            sprintf(imu_read_str, "%u - Read all IMUs:", first_imu_parameter+N_IMU_Connected+1);
            packet_data_param[2+start_byte] = TYPE_FLAG;
            packet_data_param[3+start_byte] = 1;
            packet_data_param[4+start_byte] = c_mem.read_imu;
            switch(c_mem.read_imu) {
                case 0:
                    strcat(imu_read_str, " OFF");
                    imu_read_str_len = 24;
                    break;
                case 1:
                    strcat(imu_read_str, " ON");
                    imu_read_str_len = 23;
                    break;
            }
            for(i = imu_read_str_len; i != 0; i--)
                {packet_data_param[5+start_byte + imu_read_str_len - i] = imu_read_str[imu_read_str_len - i];}
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data_param[5+start_byte + imu_read_str_len] = 2;

            /*-----------------ENCODER READ-----------------*/

			start_byte = start_byte + (uint16_t)(PARAM_BYTE_SLOT) ;
            sprintf(encoder_read_str, "%u - Read Encoders:", first_imu_parameter+N_IMU_Connected+2);
            packet_data_param[2+start_byte] = TYPE_FLAG;
            packet_data_param[3+start_byte] = 1;
            packet_data_param[4+start_byte] = c_mem.read_encoders;
            switch(c_mem.read_encoders) {
                case 0:
                    strcat(encoder_read_str, " OFF");
                    encoder_read_str_len = 24;
                    break;
                case 1:
                    strcat(encoder_read_str, " ON");
                    encoder_read_str_len = 23;
                    break;
            }
            for(i = encoder_read_str_len; i != 0; i--)
                {packet_data_param[5+start_byte + encoder_read_str_len - i] = encoder_read_str[encoder_read_str_len - i];}
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data_param[5+start_byte + encoder_read_str_len] = 2;

            /*-----------------MOTOR READ-----------------*/

			start_byte = start_byte + (uint16_t)(PARAM_BYTE_SLOT) ;
            sprintf(motor_read_str, "%u - Read Motor:", first_imu_parameter+N_IMU_Connected+3);
            packet_data_param[2+start_byte] = TYPE_FLAG;
            packet_data_param[3+start_byte] = 1;
            packet_data_param[4+start_byte] = c_mem.read_motor;
            switch(c_mem.read_motor) {
                case 0:
                    strcat(motor_read_str, " OFF");
                    motor_read_str_len = 21;
                    break;
                case 1:
                    strcat(motor_read_str, " ON");
                    motor_read_str_len = 20;
                    break;
            }
            for(i = motor_read_str_len; i != 0; i--)
                {packet_data_param[5+start_byte + motor_read_str_len - i] = motor_read_str[motor_read_str_len - i];}
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data_param[5+start_byte + motor_read_str_len] = 2;

            /*-----------------EXPANSION PORT READ-----------------*/

			start_byte = start_byte + (uint16_t)(PARAM_BYTE_SLOT) ;
            sprintf(exp_port_read_str, "%u - Read Expansion port:", first_imu_parameter+N_IMU_Connected+4);
            packet_data_param[2+start_byte] = TYPE_FLAG;
            packet_data_param[3+start_byte] = 1;
            packet_data_param[4+start_byte] = c_mem.read_exp_port;
            switch(c_mem.read_exp_port) {
                case 0:
                    strcat(exp_port_read_str, " None");
                    exp_port_read_str_len = 31;
                    break;
                case 1:
                    strcat(exp_port_read_str, " SD/RTC board");
                    exp_port_read_str_len = 39;
                    break;
                case 2:
					strcat(exp_port_read_str, " WiFi board");
					exp_port_read_str_len = 37;
					break;
                case 3:
					strcat(exp_port_read_str, " Other");
					exp_port_read_str_len = 32;
					break;
            }
            for(i = exp_port_read_str_len; i != 0; i--)
                {packet_data_param[5+start_byte + exp_port_read_str_len - i] = exp_port_read_str[exp_port_read_str_len - i];}
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data_param[5+start_byte + exp_port_read_str_len] = 3;

            /*------------PARAMETERS MENU-----------*/
            start_byte = start_byte + PARAM_BYTE_SLOT;
            for(i = spi_delay_menu_len; i!= 0; i--)
                packet_data_param[(uint16_t)(2 + start_byte) + spi_delay_menu_len - i] = spi_delay_menu[spi_delay_menu_len - i];

            start_byte = start_byte + PARAM_MENU_SLOT;
			for(i = on_off_menu_len; i!= 0; i--)
				packet_data_param[(uint16_t)(2 + start_byte) + on_off_menu_len - i] = on_off_menu[on_off_menu_len - i];

			start_byte = start_byte + PARAM_MENU_SLOT;
			for(i = exp_port_menu_len; i!= 0; i--)
				packet_data_param[(uint16_t)(2 + start_byte) + exp_port_menu_len - i] = exp_port_menu[exp_port_menu_len - i];

            packet_data_param[packet_length - 1] = LCRChecksum(packet_data_param,packet_length - 1);
			commWrite(packet_data_param, packet_length);

        break;

        //=========================================================     other_params
        default:

            if (index < first_imu_parameter-1)
                break;

            if (index >= first_imu_parameter+N_IMU_Connected) {
            	uint8_t aux = index - (first_imu_parameter+N_IMU_Connected);
            	switch(aux) {
					case 0:
						g_mem.SPI_read_delay = g_rx.buffer[3];  //SPI read delay - uint8
						break;
					case 1:
						g_mem.read_imu = g_rx.buffer[3];  //IMU read- uint8
						break;
					case 2:
						g_mem.read_encoders = g_rx.buffer[3];  //Encoders read - uint8
						break;
					case 3:
						g_mem.read_motor = g_rx.buffer[3];  //Motor read - uint8
						break;
					case 4:
						g_mem.read_exp_port = g_rx.buffer[3];  //Expansion port read- uint8
						break;
            	}
                break;
            }

            if (index == first_imu_parameter-1) {
                g_mem.id = g_rx.buffer[3];          //ID - uint8
			} else {
                //Set Imu table (index > = first_imu_parameter)
                g_mem.IMU_conf[IMU_connected[index-first_imu_parameter]][0] = g_rx.buffer[3];
                g_mem.IMU_conf[IMU_connected[index-first_imu_parameter]][1] = g_rx.buffer[4];
                g_mem.IMU_conf[IMU_connected[index-first_imu_parameter]][2] = g_rx.buffer[5];
                g_mem.IMU_conf[IMU_connected[index-first_imu_parameter]][3] = g_rx.buffer[6];
                g_mem.IMU_conf[IMU_connected[index-first_imu_parameter]][4] = g_rx.buffer[7];

                // Recompute IMU packets dimension
                imus_data_size = 1; //header
                for (i = 0; i < N_IMU_Connected; i++)    {
                    single_imu_size[IMU_connected[i]] = 1 + 6*g_mem.IMU_conf[IMU_connected[i]][0] + 6*g_mem.IMU_conf[IMU_connected[i]][1] + 6*g_mem.IMU_conf[IMU_connected[i]][2] + 16*g_mem.IMU_conf[IMU_connected[i]][3] + 2*g_mem.IMU_conf[IMU_connected[i]][4]+ 1;
                    imus_data_size = imus_data_size + single_imu_size[IMU_connected[i]];
                }
                imus_data_size = imus_data_size + 1;    //checksum
            }
        break;
    }

}

//==============================================================================
//                                                                    SET INPUTS
//==============================================================================

void cmd_set_inputs(){ // [GS] Inserita ma non testata, ho dei dubbi sull'assegnamento di g_refNew

    // Store position setted in right variables

    if(g_mem.control_mode == CONTROL_CURRENT) {
        g_refNew.curr = *((int16_t *) &g_rx.buffer[1]);
    }
    else {
        if(g_mem.control_mode == CONTROL_PWM) {
            g_refNew.pwm = *((int16_t *) &g_rx.buffer[1]);
        }
        else {
            g_refNew.pos = *((int16_t *) &g_rx.buffer[1]);   // motor 1
            g_refNew.pos = g_refNew.pos << g_mem.res[0];
        }
    }

    // Check if the reference is nor higher or lower than the position limits
    if (c_mem.pos_lim_flag && (g_mem.control_mode == CURR_AND_POS_CONTROL || g_mem.control_mode == CONTROL_ANGLE)) {

        if (g_refNew.pos < c_mem.pos_lim_inf)
            g_refNew.pos = c_mem.pos_lim_inf;

        if (g_refNew.pos > c_mem.pos_lim_sup)
            g_refNew.pos = c_mem.pos_lim_sup;
    }

    change_ext_ref_flag = TRUE;
}

void cmd_activate(){

    // Store new value reads
    g_refNew.onoff = g_rx.buffer[1];

    // Check type of control mode enabled
    if ((g_mem.control_mode == CONTROL_ANGLE) || (g_mem.control_mode == CURR_AND_POS_CONTROL)) {
        g_refNew.pos = g_meas.pos[0];
    }

    // Activate/Disactivate motor
    HAL_GPIO_WritePin(Driver_EN_GPIO_Port, Driver_EN_Pin, g_refNew.onoff);

}

void cmd_get_activate(){ // [GS] Non Testata

    uint8_t packet_data[3];

    // Header
    packet_data[0] = CMD_GET_ACTIVATE;

    // Fill payload
    packet_data[1] = g_ref.onoff;

    // Calculate checksum
    packet_data[2] = LCRChecksum(packet_data, 2);

    // Send package to UART
    commWrite(packet_data, 3);

}

void cmd_get_inputs(){ // [GS] Non Testata

    // Packet: header + motor_measure(int16) + crc

    uint8_t packet_data[6];

    //Header package

    packet_data[0] = CMD_GET_INPUTS;

    *((int16_t *) &packet_data[1]) = (int16_t) (g_refOld.pos  >> g_mem.res[0]);

    // Calculate Checksum and send message to UART

    packet_data[5] = LCRChecksum(packet_data, 5);

    commWrite(packet_data, 6);
}

void cmd_get_currents(){ // [GS] Non Testata

    // Packet: header + motor_measure(int16) + crc

    uint8_t packet_data[6];

    //Header package

    packet_data[0] = CMD_GET_CURRENTS;

    *((int16_t *) &packet_data[1]) = (int16_t) g_measOld.curr; //Real Current
    *((int16_t *) &packet_data[3]) = (int16_t) g_measOld.estim_curr; //Estimated Current

    // Calculate Checksum and send message to UART

    packet_data[5] = LCRChecksum (packet_data, 5);

    commWrite(packet_data, 6);
}

void cmd_get_velocities(){ // [GS] Non Testata


    uint8_t index;

    // Packet: header + measure(int16) + crc
    uint8_t packet_data[8];

    //Header package
    packet_data[0] = CMD_GET_VELOCITIES;

    for (index = NUM_OF_SENSORS; index--;)
        *((int16_t *) &packet_data[(index << 1) + 1]) = (int16_t) (g_measOld.vel[index] >> g_mem.res[index]);

    // Calculate Checksum and send message to UART

    packet_data[7] = LCRChecksum (packet_data, 7);

    commWrite(packet_data, 8);

}

void cmd_get_accelerations(){ // [GS] Non Testata

    uint8_t index;

    // Packet: header + measure(int16) + crc

    uint8_t packet_data[8];

    //Header package
    packet_data[0] = CMD_GET_ACCEL;

    for (index = NUM_OF_SENSORS; index--;)
        *((int16_t *) &packet_data[(index << 1) + 1]) = (int16_t)(g_measOld.acc[index] >> g_mem.res[index]);

    // Calculate Checksum and send message to UART

    packet_data[7] = LCRChecksum (packet_data, 7);

    commWrite(packet_data, 8);

}





























/*

void commProcess(void){

    uint8 CYDATA rx_cmd;

    rx_cmd = g_rx.buffer[0];

//==========================================================     verify checksum

    if (!(LCRChecksum(g_rx.buffer, g_rx.length - 1) == g_rx.buffer[g_rx.length - 1])){
        // Wrong checksum
        g_rx.ready = 0;
        return;
    }

    switch(rx_cmd) {

//=====================================================     CMD_GET_MEASUREMENTS

        case CMD_GET_MEASUREMENTS:
            cmd_get_measurements();
            break;

//=====================================================     CMD_GET_CURR_AND_MEAS

        case CMD_GET_CURR_AND_MEAS:
            cmd_get_curr_and_meas();
            break;

//=========================================================     CMD_GET_CURR_DIFF

        case CMD_GET_CURR_DIFF:
            cmd_get_currents_for_cuff();
            break;

//============================================================     CMD_SET_PARAM

        case CMD_SET_ZEROS:
            setZeros();
            break;

//=================================================     CMD_STORE_DEFAULT_PARAMS

        case CMD_STORE_DEFAULT_PARAMS:
            if(memStore(DEFAULT_EEPROM_DISPLACEMENT))
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//=======================================================     CMD_RESTORE_PARAMS

        case CMD_RESTORE_PARAMS:
            if(memRestore())
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//=============================================================     CMD_INIT_MEM

        case CMD_INIT_MEM:
            if(memInit())
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//============================================================     CMD_HAND_CALIBRATE

        case CMD_HAND_CALIBRATE:
            calib.speed = *((int16 *) &g_rx.buffer[1]);
            calib.repetitions = *((int16 *) &g_rx.buffer[3]);

            if(calib.speed == -1 && calib.repetitions == -1) {
                calib.enabled = FALSE;
                calib.speed = 0;
                calib.repetitions = 0;
                break;
            }
            // Speed & repetitions saturations
            if (calib.speed < 0) {
                calib.speed = 0;
            } else if (calib.speed > 200) {
                calib.speed = 200;
            }
            if (calib.repetitions < 0) {
                calib.repetitions = 0;
            } else if (calib.repetitions > 32767) {
                calib.repetitions = 32767;
            }

            g_refNew.pos = 0;
            calib.enabled = TRUE;

            sendAcknowledgment(ACK_OK);
            break;

//=========================================================== ALL OTHER COMMANDS
        default:
            break;

    }
}


//==============================================================================
//                                                                GET PARAM LIST
//==============================================================================

void get_param_list() {

    //Package to be sent variables
    uint8 packet_data[PARAM_BYTE_SLOT*NUM_OF_PARAMS + PARAM_MENU_SLOT*4 + PARAM_BYTE_SLOT] = "";      //50*NUM_OF_PARAMS + 150*NUM_DIFFERENT_MENUS
    uint16 packet_lenght = PARAM_BYTE_SLOT*NUM_OF_PARAMS + PARAM_MENU_SLOT*4 + PARAM_BYTE_SLOT;

    //Auxiliary variables
    uint8 CYDATA i;
    uint8 CYDATA idx = 0;       //Parameter number
    uint8 CYDATA idx_menu = 0;
    uint8 CYDATA sod = 0;       //sizeof data
    uint8 CYDATA string_lenght;
    char CYDATA aux_str[50] = "";
    uint8* m_addr = (uint8*)&(c_mem.id);
    uint8* m_tmp = m_addr;

    // Arrays
    const uint8 TYPES[NUM_OF_PARAMS] = {
        TYPE_UINT8, TYPE_FLOAT, TYPE_FLOAT, TYPE_FLAG, TYPE_FLAG, TYPE_FLAG, TYPE_UINT8, TYPE_INT16,
        TYPE_FLOAT, TYPE_FLAG, TYPE_INT32, TYPE_INT32, TYPE_INT16, TYPE_UINT16, TYPE_FLAG, TYPE_UINT32,
        TYPE_UINT8, TYPE_FLAG, TYPE_INT8, TYPE_FLAG, TYPE_FLOAT, TYPE_UINT8, TYPE_INT32, TYPE_INT32,
        TYPE_INT32, TYPE_FLAG, TYPE_FLAG, TYPE_FLAG, TYPE_FLAG
    };

    const uint8 NUM_ITEMS[NUM_OF_PARAMS] = {
        1, 3, 3, 1, 1, 1, 3, 3,
        3, 1, 2, 2, 1, 2, 1, 2,
        1, 1, 1, 1, 6, 3, 1, 1,
        1, 1, 1, 1, 1
    };

    const uint8 NUM_MENU[11] = {3, 1, 2, 3, 3, 3, 3, 3, 3, 4, 3};

    const char* PARAMS_STR[NUM_OF_PARAMS] = {
        "1 - Device ID:", "2 - Position PID [P, I, D]:", "3 - Current PID [P, I, D]:", "4 - Startup Activation:",
        "5 - Input mode:", "6 - Control mode:", "7 - Resolutions:", "8 - Measurement Offsets:",
        "9 - Multipliers:", "10 - Pos. limit active:", "11 - Pos. limits [inf, sup]:", "12 - Max steps [neg, pos]:",
        "13 - Current limit:", "14 - EMG thresholds:", "15 - EMG calibration on startup:", "16 - EMG max values:",
        "17 - EMG max speed:", "18 - Absolute encoder position:", "19 - Motor handle ratio:", "20 - PWM rescaling:",
        "21 - Current lookup:", "22 - Date of maintenance [DD/MM/YY]:", "23 - Rest position:", "24 - Rest position time delay (ms):",
        "25 - Rest vel closure (ticks/sec):", "26 - Rest position enabled:", "27 - EMG inversion:", "28 - Hand side:", "29 - Reset counters:"
    };

    //Parameters menu
    const char input_mode_menu[99] = "0 -> Usb\n1 -> Handle\n2 -> EMG proportional\n3 -> EMG Integral\n4 -> EMG FCFS\n5 -> EMG FCFS Advanced\n";
    const char control_mode_menu[64] = "0 -> Position\n1 -> PWM\n2 -> Current\n3 -> Position and Current\n";
    const char yes_no_menu[42] = "0 -> Deactivate [NO]\n1 -> Activate [YES]\n";
    const char right_left_menu[22] = "0 -> Right\n1 -> Left\n";

    packet_data[0] = CMD_GET_PARAM_LIST;
    packet_data[1] = NUM_OF_PARAMS;

    for (idx = 0; idx < NUM_OF_PARAMS; idx++) {

        // Add parameter type and size to packet
        packet_data[2 + PARAM_BYTE_SLOT*idx] = TYPES[idx];
        packet_data[3 + PARAM_BYTE_SLOT*idx] = NUM_ITEMS[idx];

        // Find size of data
        switch (TYPES[idx]) {
            case TYPE_FLAG: case TYPE_INT8: case TYPE_UINT8:
                sod = 1; break;
            case TYPE_INT16: case TYPE_UINT16:
                sod = 2; break;
            case TYPE_INT32: case TYPE_UINT32: case TYPE_FLOAT:
                sod = 4; break;
        }

        // Add parameter data to packet
        switch (TYPES[idx]) {
            case TYPE_FLAG: case TYPE_UINT8:
                for (i=0; i<NUM_ITEMS[idx]; i++){
                    m_tmp = m_addr + i*sod;
                    *((uint8*)(packet_data + (4 + PARAM_BYTE_SLOT*idx) + i*sod)) = *((uint8*)m_tmp);
                }
                break;
            case TYPE_INT8:
                for (i=0; i<NUM_ITEMS[idx]; i++){
                    m_tmp = m_addr + i*sod;
                    *((int8*)(packet_data + (4 + PARAM_BYTE_SLOT*idx) + i*sod)) = *((int8*)m_tmp);
                }
                break;
            case TYPE_INT16:
                for (i=0; i<NUM_ITEMS[idx]; i++){
                    if (idx == 7) {     //Offset measurements
                        *((int16*)(packet_data + (4 + PARAM_BYTE_SLOT*idx) + i*sod)) = (int16) (c_mem.m_off[i] >> c_mem.res[i]);
                    }
                    else {
                        m_tmp = m_addr + i*sod;
                        *((int16*)(packet_data + (4 + PARAM_BYTE_SLOT*idx) + i*sod)) = *((int16*)m_tmp);
                    }
                }
                break;
            case TYPE_UINT16:
                for (i=0; i<NUM_ITEMS[idx]; i++){
                    m_tmp = m_addr + i*sod;
                    *((uint16*)(packet_data + (4 + PARAM_BYTE_SLOT*idx) + i*sod)) = *((uint16*)m_tmp);
                }
                break;
            case TYPE_INT32:
                for (i=0; i<NUM_ITEMS[idx]; i++){
                    switch (idx){
                        case 10:    // POSITION LIMITS (param 11)
                            *((int32 *)( packet_data + (4 + PARAM_BYTE_SLOT*idx))) = (c_mem.pos_lim_inf >> c_mem.res[0]);
                            *((int32 *)( packet_data + (4 + PARAM_BYTE_SLOT*idx) + sod)) = (c_mem.pos_lim_sup >> c_mem.res[0]);
                            break;
                        case 22:    // REST POSITION (param 23)
                            *((int32 *)( packet_data + (4 + PARAM_BYTE_SLOT*idx))) = (c_mem.rest_pos >> c_mem.res[0]);
                            break;
                        default:
                            m_tmp = m_addr + i*sod;
                            *((int32*)(packet_data + (4 + PARAM_BYTE_SLOT*idx) + i*sod)) = *((int32*)m_tmp);
                            break;
                    }
                }
                break;
            case TYPE_UINT32:
                for (i=0; i<NUM_ITEMS[idx]; i++){
                    m_tmp = m_addr + i*sod;
                    *((uint32*)(packet_data + (4 + PARAM_BYTE_SLOT*idx) + i*sod)) = *((uint32*)m_tmp);
                }
                break;

            case TYPE_FLOAT:
                switch (idx) {

                case 1:     // POSITION PID (param 2)
                    if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx))) = (float) c_mem.k_p / 65536;
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx) + sod)) = (float) c_mem.k_i / 65536;
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx) + 2*sod)) = (float) c_mem.k_d / 65536;
                    }
                    else {
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx))) = (float) c_mem.k_p_dl / 65536;
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx) + sod)) = (float) c_mem.k_i_dl / 65536;
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx) + 2*sod)) = (float) c_mem.k_d_dl / 65536;
                    }
                    break;
                case 2:     // POSITION PID (param 3)
                    if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx))) = (float) c_mem.k_p_c / 65536;
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx) + sod)) = (float) c_mem.k_i_c / 65536;
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx) + 2*sod)) = (float) c_mem.k_d_c / 65536;
                    }
                    else {
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx))) = (float) c_mem.k_p_c_dl / 65536;
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx) + sod)) = (float) c_mem.k_i_c_dl / 65536;
                        *((float *) (packet_data + (4 + PARAM_BYTE_SLOT*idx) + 2*sod)) = (float) c_mem.k_d_c_dl / 65536;
                    }
                    break;
                default:
                    for (i=0; i<NUM_ITEMS[idx]; i++){
                        m_tmp = m_addr + i*sod;
                        *((float*)(packet_data + (4 + PARAM_BYTE_SLOT*idx) + i*sod)) = *((float*)m_tmp);
                    }
                    break;
                }
                break;
        }

        sprintf(aux_str, (char*)PARAMS_STR[idx]);
        string_lenght = strlen(aux_str);

        // Parameters with a menu
        if (TYPES[idx] == TYPE_FLAG){
            switch(NUM_MENU[idx_menu]){
                case 1:     // input mode menu
                    switch(*m_addr) {
                        case INPUT_MODE_EXTERNAL:
                            strcat(aux_str, " Usb");
                        break;
                        case INPUT_MODE_ENCODER3:
                            strcat(aux_str, " Handle");
                        break;
                        case INPUT_MODE_EMG_PROPORTIONAL:
                            strcat(aux_str, " EMG proportional");
                        break;
                        case INPUT_MODE_EMG_INTEGRAL:
                            strcat(aux_str, " EMG integral");
                        break;
                        case INPUT_MODE_EMG_FCFS:
                            strcat(aux_str, " EMG FCFS");
                        break;
                        case INPUT_MODE_EMG_FCFS_ADV:
                            strcat(aux_str, " EMG FCFS Advanced");
                        break;
                    }
                    break;
                case 2:     // control mode menu
                    switch(*m_addr){
                        case CONTROL_ANGLE:
                            strcat(aux_str, " Position");
                        break;
                        case CONTROL_PWM:
                            strcat(aux_str, " PWM");
                        break;
                        case CONTROL_CURRENT:
                            strcat(aux_str, " Current");
                        break;
                        case CURR_AND_POS_CONTROL:
                            strcat(aux_str, " Position and Current");
                        break;
                    }
                    break;
                case 3:     // yes/no menu
                    if(*m_addr){
                        strcat(aux_str, " YES\0");
                    }
                    else {
                        strcat(aux_str, " NO\0");
                    }
                    break;
                case 4:     // right/lef menu
                    switch(*m_addr){
                        case RIGHT_HAND:
                            strcat(aux_str, " Right\0");
                        break;
                        case LEFT_HAND:
                            strcat(aux_str, " Left\0");
                        break;
                    }
                    break;
            }
            //Recomputes string lenght
            string_lenght = strlen(aux_str)+1;
        }

        // Add parameter string to packet
        for(i = string_lenght; i != 0; i--)
            packet_data[(4 + PARAM_BYTE_SLOT*idx) + (sod*NUM_ITEMS[idx]) + string_lenght - i] = aux_str[string_lenght - i];
        //The following byte indicates the number of menus at the end of the packet to send
        if (TYPES[idx] == TYPE_FLAG){
            packet_data[(4 + PARAM_BYTE_SLOT*idx) + (sod*NUM_ITEMS[idx]) + string_lenght] = NUM_MENU[idx_menu];
            idx_menu = idx_menu + 1;
        }


        // Adjust m_addr increment, according to c_mem structure
        switch (idx) {
            case 2:
                // double loop PID controller parameters
                //(increment after CURRENT PID parameters)
                m_addr = m_addr + NUM_ITEMS[idx]*sod + 6*sod;
                break;
            case 7:
                //(increment after OFFSET parameters)
                m_addr = m_addr + NUM_ITEMS[idx]*4;     //int16->int32 conversion
                break;
            case 20:
                //(increment after LOOKUP TABLE parameter)
                m_addr = m_addr + NUM_ITEMS[idx]*sod + 1;   //1 = baudrate
                break;
            default:
                m_addr = m_addr + NUM_ITEMS[idx]*sod;
                break;

            // reset counters reads g_mem.unused_bytes[0]
        }
    }

    string_lenght = strlen((char*)input_mode_menu);
    for(i = string_lenght; i != 0; i--)
        packet_data[PARAM_BYTE_SLOT*NUM_OF_PARAMS + 2 + string_lenght - i] = input_mode_menu[string_lenght - i];

    string_lenght = strlen((char*)control_mode_menu);
    for(i = string_lenght; i != 0; i--)
        packet_data[PARAM_BYTE_SLOT*NUM_OF_PARAMS + 2 + PARAM_MENU_SLOT + string_lenght - i] = control_mode_menu[string_lenght - i];

    string_lenght = strlen((char*)yes_no_menu);
    for(i = string_lenght; i!= 0; i--)
        packet_data[PARAM_BYTE_SLOT*NUM_OF_PARAMS + 2 + 2*PARAM_MENU_SLOT + string_lenght - i] = yes_no_menu[string_lenght - i];

    string_lenght = strlen((char*)right_left_menu);
    for(i = string_lenght; i!= 0; i--)
        packet_data[PARAM_BYTE_SLOT*NUM_OF_PARAMS + 2 + 3*PARAM_MENU_SLOT + string_lenght - i] = right_left_menu[string_lenght - i];

    packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
    commWrite(packet_data, packet_lenght);
}

//==============================================================================
//                                                             MANAGE PARAM LIST
//==============================================================================

void manage_param_list(uint16 index) {
    uint8 CYDATA i;
    int32 aux_int;
    uint8 aux_uchar;

    switch(index) {
        case 0:         //List of all parameters with relative types
            get_param_list();
        break;

//===================================================================     set_id
        case 1:         //ID - uint8
            g_mem.id = g_rx.buffer[3];
        break;

//=======================================================     set_pid_parameters
        case 2:         //Position PID - float[3]
            if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
                g_mem.k_p = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
            else {
                g_mem.k_p_dl = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i_dl = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d_dl = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
        break;

//==================================================     set_curr_pid_parameters
        case 3:         //Current PID - float[3]
            if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
                g_mem.k_p_c = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i_c = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d_c = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
            else {
                g_mem.k_p_c_dl = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i_c_dl = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d_c_dl = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }

        break;

//===================================================     set_startup_activation
        case 4:         //Startup flag - uint8
            if(g_rx.buffer[3])
                g_mem.activ = 0x01;
            else
                g_mem.activ = 0x00;
        break;

//===========================================================     set_input_mode
        case 5:         //Input mode - uint8
            g_mem.input_mode = g_rx.buffer[3];

            // Hold the actual position
            g_refNew.pos = g_meas.pos[0];
        break;

//=========================================================     set_control_mode
        case 6:         //Control mode - uint8
            g_mem.control_mode = g_rx.buffer[3];
        break;

//===========================================================     set_resolution
        case 7:         //Resolution - uint8[3]
            for (i =0; i < NUM_OF_SENSORS; i++) {
                g_mem.res[i] = g_rx.buffer[i+3];
            }
        break;

//===============================================================     set_offset
        case 8:         //Measurement Offset - int32[3]
            for(i = 0; i < NUM_OF_SENSORS; ++i) {
                g_mem.m_off[i] = *((int16 *) &g_rx.buffer[3 + i * 2]);
                g_mem.m_off[i] = g_mem.m_off[i] << g_mem.res[i];

                g_meas.rot[i] = 0;
            }
            reset_last_value_flag = 1;
        break;

//===========================================================     set_multiplier
        case 9:         //Multipliers - float[3]
            for(i = 0; i < NUM_OF_SENSORS; ++i)
                g_mem.m_mult[i] = *((float *) &g_rx.buffer[3 + i * 4]);
        break;

//=====================================================     set_pos_limit_enable
        case 10:        //Position limit flag - uint8
            g_mem.pos_lim_flag = *((uint8 *) &g_rx.buffer[3]);
        break;

//============================================================     set_pos_limit
        case 11:        //Position limits - int32[4]
            g_mem.pos_lim_inf = *((int32 *) &g_rx.buffer[3]);
            g_mem.pos_lim_sup = *((int32 *) &g_rx.buffer[7]);

            g_mem.pos_lim_inf = g_mem.pos_lim_inf << g_mem.res[0];
            g_mem.pos_lim_sup = g_mem.pos_lim_sup << g_mem.res[0];
        break;

//==================================================     set_max_steps_per_cycle
        case 12:        //Max steps - int32[2]
            aux_int = *((int32 *) &g_rx.buffer[3]);
            if (aux_int <= 0)
                g_mem.max_step_neg = aux_int;

            aux_int = *((int32 *) &g_rx.buffer[3 + 4]);
            if (aux_int >= 0)
                g_mem.max_step_pos = aux_int;

        break;

//========================================================     set_current_limit
        case 13:        //Current limit - int16
            g_mem.current_limit = *((int16*) &g_rx.buffer[3]);
        break;

//========================================================     set_emg_threshold
        case 14:        //Emg threshold - uint16[2]
            g_mem.emg_threshold[0] = *((uint16*) &g_rx.buffer[3]);
            g_mem.emg_threshold[1] = *((uint16*) &g_rx.buffer[5]);
        break;

//=======================================================     set_emg_calib_flag
        case 15:        //Emg calibration flag - uint8
            g_mem.emg_calibration_flag = *((uint8*) &g_rx.buffer[3]);
        break;

//========================================================     set_emg_max_value
        case 16:        //Emg max value - uint32[2]
            g_mem.emg_max_value[0] = *((uint32*) &g_rx.buffer[3]);
            g_mem.emg_max_value[1] = *((uint32*) &g_rx.buffer[7]);
        break;

//============================================================     set_emg_speed
        case 17:        //Emg max speed - uint8
            g_mem.emg_speed = *((uint8*) &g_rx.buffer[3]);
        break;

//================================================     set_double_encoder_on_off
        case 18:        //Absolute encoder flag - uint8
            aux_uchar = *((uint8*) &g_rx.buffer[3]);
            if (aux_uchar) {
                g_mem.double_encoder_on_off = 1;
            } else {
                g_mem.double_encoder_on_off = 0;
            }
        break;

//===================================================     set_motor_handle_ratio
        case 19:        //Motor handle ratio - int8
            g_mem.motor_handle_ratio = *((int8*) &g_rx.buffer[3]);
        break;

//===================================================     set_motor_supply_type
        case 20:        //Motor type - uint8
            g_mem.activate_pwm_rescaling = g_rx.buffer[3];
        break;

//===================================================     set_curr_lookup_table
        case 21:        //Current lookup table - float
            for(i = 0; i < LOOKUP_DIM; i++)
                g_mem.curr_lookup[i] = *((float *) &g_rx.buffer[3 + i*4]);
        break;

//========================================================    set_maintenace_date
        case 22:         //Maintenance date - uint8[3]
            g_mem.maint_day     = g_rx.buffer[3];
            g_mem.maint_month   = g_rx.buffer[4];
            g_mem.maint_year    = g_rx.buffer[5];
        break;

//============================================================     set_rest_pos
        case 23:        //Rest Position - int32
            g_mem.rest_pos = *((int32 *) &g_rx.buffer[3]);
            g_mem.rest_pos = g_mem.rest_pos << g_mem.res[0];
        break;

//============================================================     set_rest_delay_pos
        case 24:        //Rest Position Time Delay - int32
            g_mem.rest_delay = *((int32 *) &g_rx.buffer[3]);
            if (g_mem.rest_delay < 10) g_mem.rest_delay = 10;
        break;

//============================================================     set_rest_vel
        case 25:        //Rest Position Velocity - int32
            g_mem.rest_vel = *((int32 *) &g_rx.buffer[3]);
        break;

//================================================     set_rest_position_flag
        case 26:        //Rest position flag - uint8
            aux_uchar = *((uint8*) &g_rx.buffer[3]);
            if (aux_uchar) {
                g_mem.rest_position_flag = TRUE;
            } else {
                g_mem.rest_position_flag = FALSE;
            }
        break;

//===================================================     set_switch_emg
        case 27:        //EMG inversion - uint8
            g_mem.switch_emg = g_rx.buffer[3];
        break;

//================================================     set_right_left_flag
        case 28:        //Right/Left hand flag - uint8
            aux_uchar = *((uint8*) &g_rx.buffer[3]);
            if (aux_uchar) {    // 1
                g_mem.right_left = LEFT_HAND;
            } else {            // 0
                g_mem.right_left = RIGHT_HAND;
            }
			reset_last_value_flag = 1;
        break;

//===================================================     reset_counters
        case 29:        //Reset counters - uint8
            aux_uchar = *((uint8*) &g_rx.buffer[3]);
            if (aux_uchar) {
                reset_counters();
            }
        break;
    }
}

//==============================================================================
//                                                            COMMAND SET ZEROS
//==============================================================================

void setZeros()
{
    uint8 CYDATA i;        // iterator

    for(i = 0; i < NUM_OF_SENSORS; ++i) {
        g_mem.m_off[i] = data_encoder_raw[i];
        g_meas.rot[i] = 0;
    }
    reset_last_value_flag = 1;

    sendAcknowledgment(ACK_OK);
}

//==============================================================================
//                                                     WRITE FUNCTIONS FOR RS485
//==============================================================================

void commWrite_old_id(uint8 *packet_data, uint16 packet_lenght, uint8 old_id)
{
    uint16 CYDATA index;    // iterator

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    // frame - ID
    //if(old_id)
        UART_RS485_PutChar(old_id);
    //else
        //UART_RS485_PutChar(g_mem.id);

    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index) {
        UART_RS485_PutChar(packet_data[index]);
    }

    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}

    RS485_CTS_Write(1);
    RS485_CTS_Write(0);
}

void commWrite_to_cuff(uint8 *packet_data, uint16 packet_lenght)
{
    uint16 CYDATA index;    // iterator

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    // frame - ID
    UART_RS485_PutChar(g_mem.id - 1);
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index) {
        UART_RS485_PutChar(packet_data[index]);
    }

    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}
}


//==============================================================================
//                                                    ROUTINE INTERRUPT FUNCTION
//==============================================================================

void cmd_get_measurements(){

    uint8 CYDATA index;
    // Packet: header + measure(int16) + crc

    uint8 packet_data[8];

    //Header package
    packet_data[0] = CMD_GET_MEASUREMENTS;

    for (index = NUM_OF_SENSORS; index--;)
        *((int16 *) &packet_data[(index << 1) + 1]) = (int16)(g_measOld.pos[index] >> g_mem.res[index]);

    // Calculate Checksum and send message to UART
    packet_data[7] = LCRChecksum (packet_data, 7);

    commWrite(packet_data, 8);

}

void cmd_get_curr_and_meas(){

    uint8 CYDATA index;

    //Packet: header + curr_meas(int16) + pos_meas(int16) + CRC

    uint8 packet_data[12];

    //Header package
    packet_data[0] = CMD_GET_CURR_AND_MEAS;

    // Currents
    *((int16 *) &packet_data[1]) = (int16) g_measOld.curr; //Real current
    *((int16 *) &packet_data[3]) = (int16) g_measOld.estim_curr; //Estimated current

    // Positions
    for (index = NUM_OF_SENSORS; index--;)
        *((int16 *) &packet_data[(index << 1) + 5]) = (int16) (g_measOld.pos[index] >> g_mem.res[index]);

    // Calculate Checksum and send message to UART

    packet_data[11] = LCRChecksum (packet_data, 11);
    commWrite(packet_data, 12);

}

void cmd_get_currents_for_cuff(){

    // Packet: header + motor_measure(int16) + crc

    uint8 packet_data[4];

    //Header package

    packet_data[0] = CMD_SET_CUFF_INPUTS;

    *((int16 *) &packet_data[1]) = (int16) g_measOld.estim_curr; //Estimated Current

    // Calculate Checksum and send message to UART

    packet_data[3] = LCRChecksum (packet_data, 3);

    commWrite_to_cuff(packet_data, 4);
}

void cmd_store_params(){

    // Check input mode enabled
    uint32 off_1;
    float mult_1;
    uint8 CYDATA packet_lenght = 2;
    uint8 CYDATA packet_data[2];
    uint8 CYDATA old_id;

    if( c_mem.input_mode == INPUT_MODE_EXTERNAL ) {
        off_1 = c_mem.m_off[0];
        mult_1 = c_mem.m_mult[0];

        g_refNew.pos = (int32)((float)g_refNew.pos / mult_1);

        g_refNew.pos = (int32)((float)g_refNew.pos * g_mem.m_mult[0]);

        g_refNew.pos += (g_mem.m_off[0] - off_1);

        // Check position Limits

        if (c_mem.pos_lim_flag) {                   // position limiting
            if (g_refNew.pos < c_mem.pos_lim_inf)
                g_refNew.pos = c_mem.pos_lim_inf;

            if (g_refNew.pos > c_mem.pos_lim_sup)
                g_refNew.pos = c_mem.pos_lim_sup;
        }
    }
    // Store params

    if (c_mem.id != g_mem.id) {     //If a new id is going to be set we will lose communication
        old_id = c_mem.id;          //after the memstore(0) and the ACK won't be recognised
        if(memStore(0)) {
            packet_data[0] = ACK_OK;
            packet_data[1] = ACK_OK;
            commWrite_old_id(packet_data, packet_lenght, old_id);
        }
        else{
            packet_data[0] = ACK_ERROR;
            packet_data[1] = ACK_ERROR;
            commWrite_old_id(packet_data, packet_lenght, old_id);
        }
    }
    else {
        if(memStore(0))
            sendAcknowledgment(ACK_OK);
        else
            sendAcknowledgment(ACK_ERROR);
    }
}

*/
