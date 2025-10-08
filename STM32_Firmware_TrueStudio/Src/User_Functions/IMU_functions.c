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
* \file         IMU_functions.c
*
* \brief        Implementation fo IMU module functions.
* \date         July 6th, 2021
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2021 Centro "E.Piaggio". All rights reserved.
*/


#include "IMU_functions.h"

/*******************************************************************************
* Function Name: IMU Reset
*********************************************************************************/
void ImuReset() {
	
	InitIMUgeneral();
	
	for (int k_quat = 0; k_quat < N_IMU_MAX; k_quat++) {
		Quat[k_quat][0] = 0.99998;
		Quat[k_quat][1] = 0.001;
		Quat[k_quat][2] = 0.001;
		Quat[k_quat][3] = 0.001;
	}
}

/********************************************************************************
* Function Name: InitIMUGeneral                                                 *
*********************************************************************************/
void InitIMUgeneral()
{
    uint8_t k_imu=0;
    uint8_t count = 0;
    uint8_t IMU_ack;
    int tmp[N_IMU_MAX];

    // Initialize Memory Structure 
    memset(&g_imu, 0, sizeof(struct st_imu));
    memset(&IMU_connected, 0, sizeof(IMU_connected));

    // Initialize IMU to Read MagCal Parameters
    for (k_imu=0; k_imu < N_IMU_MAX; k_imu++) 
    {	
	    InitIMUMagCal(k_imu);
    }
    
    // Reading of MagCal Parameters
    HAL_Delay(20);
    for (k_imu = 0; k_imu < N_IMU_MAX; k_imu++){
    	ReadMagCal(k_imu);
    }

    // First ping to be sure to wakeup IMUs
//    for (k_imu=0; k_imu < N_IMU_MAX; k_imu++)
//    {
//    	ChipSelector(k_imu);
//	    HAL_Delay(5);
//	    ReadControlRegister(MPU9250_WHO_AM_I);
//		ChipSelector(dummy_IMU);
//		HAL_Delay(5);
 //   }
    
    // Identify IMU connected
	IMU_ack = 0;
    N_IMU_Connected = 0;
    for (k_imu = 0; k_imu < N_IMU_MAX; k_imu++) {      
    	ChipSelector(k_imu);
    	HAL_Delay(5);
    	IMU_ack = ReadControlRegister(MPU9250_WHO_AM_I);
		ChipSelector(dummy_IMU);
		if (IMU_ack == 0x71) {
			N_IMU_Connected++;
			IMU_ack = 0;
			tmp[k_imu] = 1;
		} else {
			ChipSelector(k_imu);
			HAL_Delay(5);
			IMU_ack = ReadControlRegister(MPU9250_WHO_AM_I);
			ChipSelector(dummy_IMU);
			if (IMU_ack == 0x71) {
				N_IMU_Connected++;
				IMU_ack = 0;
				tmp[k_imu] = 1;
			} else {
				tmp[k_imu] = 0;
			}
		}
    }

	k_imu = 0;
	count = 0;
    for(k_imu = 0; k_imu < N_IMU_MAX; k_imu++)
    {
    	if(tmp[k_imu] == 1) {
			IMU_connected[count] = k_imu;
			count++;
		}
    }
     
    for (k_imu=0; k_imu < N_IMU_MAX; k_imu++) {	
	    ChipSelector(k_imu);
	    HAL_Delay(5);
	    InitIMU(k_imu);
	    HAL_Delay(5); 
    }
	HAL_Delay(50);
     
    // Set Standard Read for the IMU
    memset(&single_imu_size, 0, sizeof(single_imu_size));
    imus_data_size = 1; //header    
    for (k_imu = 0; k_imu < N_IMU_Connected; k_imu++) {
        //single_imu_size[IMU_connected[k_imu]] = 1 + 6*c_mem.IMU_conf[IMU_connected[k_imu]][0] + 6*c_mem.IMU_conf[IMU_connected[k_imu]][1] + 6*c_mem.IMU_conf[IMU_connected[k_imu]][2]+ 8*c_mem.IMU_conf[IMU_connected[k_imu]][3] + 2*c_mem.IMU_conf[IMU_connected[k_imu]][4]+ 1;
        single_imu_size[IMU_connected[k_imu]] = 1 + 6*c_mem.IMU_conf[IMU_connected[k_imu]][0] + 6*c_mem.IMU_conf[IMU_connected[k_imu]][1] + 6*c_mem.IMU_conf[IMU_connected[k_imu]][2]+ 16*c_mem.IMU_conf[IMU_connected[k_imu]][3] + 2*c_mem.IMU_conf[IMU_connected[k_imu]][4]+ 1;
        imus_data_size = imus_data_size + single_imu_size[IMU_connected[k_imu]];
    }
    imus_data_size = imus_data_size + 1;    //checksum

}
	
/*******************************************************************************
* Function Name: IMU Initialization
*********************************************************************************/
void InitIMU(uint8_t n){	
  
	ChipSelector(n);
	WriteControlRegister(MPU9250_PWR_MGMT_1, 0x10); 
	ChipSelector(dummy_IMU);
	HAL_Delay(5);	
	ChipSelector(n); 
	WriteControlRegister(MPU9250_USER_CTRL, 0x20);  //I2C master enable - disable I2C (prima 0x30)
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n); 
	WriteControlRegister(MPU9250_CONFIG, 0x06); //Gyro & Temp Low Pass Filter 0x01 = 184Hz, 0x02 = 92Hz, 0x04 = 20Hz, 0x05 = 10Hz
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n);
	WriteControlRegister(MPU9250_GYRO_CONFIG , GYRO_SF_2000); //Gyro full scale select 0x00=250°/s 0x80=500°/s 0x18=2000°/s 
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n);
	WriteControlRegister(MPU9250_ACCEL_CONFIG, ACC_SF_2G); // Acc full scale select 0x00 = 2g 0x08 = 4g 0x10 = 8g 0x18 = 16g
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n);
	WriteControlRegister(MPU9250_ACCEL_CONFIG2, LP_ACC_FREQ_10);
	//WriteControlRegister(MPU9250_ACCEL_CONFIG2, NO_ACC_FIL);
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	//mag register
	ChipSelector(n); 
	WriteControlRegister(MPU9250_I2C_MST_CTRL, 0x0D); //set slave I2C speed
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n); 
	//SLV0 (use to write)
	WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x0C); //set compass address
	ChipSelector(dummy_IMU);
	HAL_Delay(5);		
	ChipSelector(n); 
	WriteControlRegister(MPU9250_I2C_SLV0_REG, AK8936_CNTL); //compass mode register
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n); 
	// Istruction used to read Compass
	WriteControlRegister(MPU9250_I2C_SLV0_D0, 0x16); //0x12 continuous mode1  0x16 continuous mode2
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n); 
	WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x81); //enable data from register + 1 bit to write
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n); 
	//SLV0 (use to read)
	WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x8C); // RCR  | AK8963_address (0x0C) 
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n); 
	// Istruction used to read Compass
	WriteControlRegister(MPU9250_I2C_SLV0_REG, 0x03); // 0x03:start from Xout Low in case of calibration 0x10:start from ASAX
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n); 
	// Istruction used to read Compass
	WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x8D); //How many bits read  SEMPRE DISPARI 0x8D era quella che funzionava
	ChipSelector(dummy_IMU);
	HAL_Delay(5);
	ChipSelector(n); 
	WriteControlRegister(MPU9250_PWR_MGMT_1, 0x00); 
	ChipSelector(dummy_IMU);
	HAL_Delay(25);
	
}	

/*******************************************************************************
* Function Name: IMU Mag Cal Initialization
*********************************************************************************/
void InitIMUMagCal(uint8_t n){	

	ChipSelector(n);
	WriteControlRegister(MPU9250_PWR_MGMT_1, 0x10); 
	ChipSelector(dummy_IMU);
	HAL_Delay(10);	
	ChipSelector(n);
	WriteControlRegister(MPU9250_USER_CTRL, 0x20);  //I2C master enable - disable I2C (prima 0x30)
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	ChipSelector(n);
	WriteControlRegister(MPU9250_CONFIG, 0x06); //Gyro & Temp Low Pass Filter 0x01 = 184Hz, 0x02 = 92Hz, 0x04 = 20Hz, 0x05 = 10Hz, 0x06 = 5Hz
	ChipSelector(dummy_IMU);
	HAL_Delay(10);	
	ChipSelector(n);
	WriteControlRegister(MPU9250_GYRO_CONFIG , GYRO_SF_2000); //Gyro full scale select 0x00=250°/s 0x80=500°/s 0x18=2000°/s 
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	ChipSelector(n);
	WriteControlRegister(MPU9250_ACCEL_CONFIG, ACC_SF_2G); // Acc full scale select 0x00 = 2g 0x08 = 4g 0x10 = 8g 0x18 = 16g
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	ChipSelector(n);	
	WriteControlRegister(MPU9250_ACCEL_CONFIG2, 0x05);
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	//mag register
	ChipSelector(n);	
	WriteControlRegister(MPU9250_I2C_MST_CTRL, 0x0D); //set slave I2C speed
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	//SLV0 (use to write)
	ChipSelector(n);	
	WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x0C); //set compass address
	ChipSelector(dummy_IMU);
	HAL_Delay(10);		
	ChipSelector(n);	
	WriteControlRegister(MPU9250_I2C_SLV0_REG, AK8936_CNTL); //compass mode register
	ChipSelector(dummy_IMU);
	HAL_Delay(10);	
	ChipSelector(n);	
	WriteControlRegister(MPU9250_I2C_SLV0_D0, 0x1F); //0x1F ROM access
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	ChipSelector(n);	
	WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x81); //enable data from register + 1 bit to write
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	//SLV0 (use to read)
	ChipSelector(n);	
	WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x8C); // RCR  | AK8963_address (0x0C) 
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	ChipSelector(n);	
	WriteControlRegister(MPU9250_I2C_SLV0_REG, 0x10); // 0x10:start from ASAX
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	ChipSelector(n);	
	WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x83);
	ChipSelector(dummy_IMU);
	HAL_Delay(10);
	ChipSelector(n);	
	WriteControlRegister(MPU9250_PWR_MGMT_1, 0x00); 
	ChipSelector(dummy_IMU);
	HAL_Delay(20);	

}

/********************************************************************************
* Function Name: ChipSelector                                                   *
*********************************************************************************/
void ChipSelector(uint8_t n)
{
	switch(n) {
			
		case 0:
			HAL_GPIO_WritePin(CS00_GPIO_Port, CS00_Pin, 0);
			break;
		case 1:
			HAL_GPIO_WritePin(CS01_GPIO_Port, CS01_Pin, 0);
			break;
		case 2:
			HAL_GPIO_WritePin(CS02_GPIO_Port, CS02_Pin, 0);
			break;
		case 3:
			HAL_GPIO_WritePin(CS03_GPIO_Port, CS03_Pin, 0);
			break;
		case 4:
			HAL_GPIO_WritePin(CS04_GPIO_Port, CS04_Pin, 0);
			break;
		case 5:
			HAL_GPIO_WritePin(CS05_GPIO_Port, CS05_Pin, 0);
			break;
		case 6:
			HAL_GPIO_WritePin(CS06_GPIO_Port, CS06_Pin, 0);
			break;
		case 7:
			HAL_GPIO_WritePin(CS07_GPIO_Port, CS07_Pin, 0);
			break;
		case 8:
			HAL_GPIO_WritePin(CS08_GPIO_Port, CS08_Pin, 0);
			break;
		case 9:
			HAL_GPIO_WritePin(CS09_GPIO_Port, CS09_Pin, 0);
			break;
		case 10:
			HAL_GPIO_WritePin(CS10_GPIO_Port, CS10_Pin, 0);
			break;
		case 11:
			HAL_GPIO_WritePin(CS11_GPIO_Port, CS11_Pin, 0);
			break;
		case 12:
			HAL_GPIO_WritePin(CS12_GPIO_Port, CS12_Pin, 0);
			break;
		case 13:
			HAL_GPIO_WritePin(CS13_GPIO_Port, CS13_Pin, 0);
			break;
		case 14:
			HAL_GPIO_WritePin(CS14_GPIO_Port, CS14_Pin, 0);
			break;
		case 15:
			HAL_GPIO_WritePin(CS15_GPIO_Port, CS15_Pin, 0);
			break;
		case 16:
			HAL_GPIO_WritePin(CS16_GPIO_Port, CS16_Pin, 0);
			break;
		case 17:
			HAL_GPIO_WritePin(CS17_GPIO_Port, CS17_Pin, 0);
			break;
		case 18:
			HAL_GPIO_WritePin(CSBB_GPIO_Port, CSBB_Pin, 0); // IMU already included on the board
			break;
		default:
			HAL_GPIO_WritePin(CSBB_GPIO_Port, CSBB_Pin, 1);
			HAL_GPIO_WritePin(CS00_GPIO_Port, CS00_Pin, 1);
			HAL_GPIO_WritePin(CS01_GPIO_Port, CS01_Pin, 1);
			HAL_GPIO_WritePin(CS02_GPIO_Port, CS02_Pin, 1);
			HAL_GPIO_WritePin(CS03_GPIO_Port, CS03_Pin, 1);
			HAL_GPIO_WritePin(CS04_GPIO_Port, CS04_Pin, 1);
			HAL_GPIO_WritePin(CS05_GPIO_Port, CS05_Pin, 1);
			HAL_GPIO_WritePin(CS06_GPIO_Port, CS06_Pin, 1);
			HAL_GPIO_WritePin(CS07_GPIO_Port, CS07_Pin, 1);
			HAL_GPIO_WritePin(CS08_GPIO_Port, CS08_Pin, 1);
			HAL_GPIO_WritePin(CS09_GPIO_Port, CS09_Pin, 1);
			HAL_GPIO_WritePin(CS10_GPIO_Port, CS10_Pin, 1);
			HAL_GPIO_WritePin(CS11_GPIO_Port, CS11_Pin, 1);
			HAL_GPIO_WritePin(CS12_GPIO_Port, CS12_Pin, 1);
			HAL_GPIO_WritePin(CS13_GPIO_Port, CS13_Pin, 1);
			HAL_GPIO_WritePin(CS14_GPIO_Port, CS14_Pin, 1);
			HAL_GPIO_WritePin(CS15_GPIO_Port, CS15_Pin, 1);
			HAL_GPIO_WritePin(CS16_GPIO_Port, CS16_Pin, 1);
			HAL_GPIO_WritePin(CS17_GPIO_Port, CS17_Pin, 1);
			break;
		}
}

/********************************************************************************
* Function Name: ReadAllIMUs
*********************************************************************************/
void ReadAllIMUs(){
    static uint8_t k_imu = 0;
    uint16_t tmp = 0, i = 0, j = 0;
	  int16_t var_h;
    int16_t value = 0;
     
    for (k_imu = 0; k_imu < N_IMU_Connected; k_imu++){ 
        // Read k_imu IMU
        ReadIMU(IMU_connected[k_imu]);
			
        for (j = 0; j < 3; j++) {
            var_h = Accel[IMU_connected[k_imu]][2*j];
					  var_h = var_h << 8;
            g_imuNew[k_imu].accel_value[j] = (int16_t)(var_h | Accel[IMU_connected[k_imu]][2*j + 1]);
					  var_h = 0;
        }
				for (j = 0; j < 3; j++) {
            var_h = Gyro[IMU_connected[k_imu]][2*j];
					  var_h = var_h << 8;
            g_imuNew[k_imu].gyro_value[j] = (int16_t)(var_h | Gyro[IMU_connected[k_imu]][2*j + 1]);
					  var_h = 0;
        }
                
        for (j = 0; j < 3; j++) {
            var_h = Mag[IMU_connected[k_imu]][2*j];
					  var_h = var_h << 8;
            g_imuNew[k_imu].mag_value[j] = (int16_t)(var_h << 8 | Mag[IMU_connected[k_imu]][2*j + 1]);
					  var_h = 0;
        }  
        
        for (j = 0; j < 4; j++) {
            g_imuNew[k_imu].quat_value[j] = (float)Quat[IMU_connected[k_imu]][j];
        }
        
        var_h = Temp[IMU_connected[k_imu]][0];
				var_h = var_h << 8;
        g_imuNew[k_imu].temp_value = (int16_t)(var_h << 8 | Temp[IMU_connected[k_imu]][1]);
				var_h = 0;
    }
		
}

/*******************************************************************************
* Function Name: IMU Read
*********************************************************************************/
void ReadIMU(int n)
{
    if (c_mem.IMU_conf[n][0]) ReadAcc(n);
    if (c_mem.IMU_conf[n][1]) ReadGyro(n);
    if (c_mem.IMU_conf[n][2]) ReadMag(n);
    if (c_mem.IMU_conf[n][3]) ReadQuat(n);
    if (c_mem.IMU_conf[n][4]) ReadTemp(n);
}

/*******************************************************************************
* Function Name: Read Acc's Data of IMU n
*********************************************************************************/
void ReadAcc(int row)
{
	uint8_t low=0, high=0;
	//read X
	ChipSelector(row);
    low=ReadControlRegister(MPU9250_ACCEL_XOUT_L);
	ChipSelector(dummy_IMU);
    SPI_delay();
    ChipSelector(row);
	high=ReadControlRegister(MPU9250_ACCEL_XOUT_H);
    ChipSelector(dummy_IMU);
	SPI_delay();
	Accel[row][0] = high;
	Accel[row][1] = low;
	low=0, high=0;
			
	//read Y
	ChipSelector(row);
    low=ReadControlRegister(MPU9250_ACCEL_YOUT_L);
    ChipSelector(dummy_IMU);
	SPI_delay();
	ChipSelector(row);
	high=ReadControlRegister(MPU9250_ACCEL_YOUT_H);
    ChipSelector(dummy_IMU);
	SPI_delay();
	Accel[row][2] = high;
	Accel[row][3] = low;
	low=0, high=0;
		
	//read Z
	ChipSelector(row);
    low=ReadControlRegister(MPU9250_ACCEL_ZOUT_L);
    ChipSelector(dummy_IMU);
	SPI_delay();
	ChipSelector(row);
    high=ReadControlRegister(MPU9250_ACCEL_ZOUT_H);
    ChipSelector(dummy_IMU);
	SPI_delay();
	Accel[row][4] = high;
	Accel[row][5] = low;
	low=0, high=0;
	
}

/*******************************************************************************
* Function Name: Read Gyro's Data of IMU n
*********************************************************************************/
void ReadGyro(int row){
	uint8_t low=0, high=0;
    	
	//read X
	ChipSelector(row);
	low=ReadControlRegister(MPU9250_GYRO_XOUT_L);
	ChipSelector(dummy_IMU);
	SPI_delay();
	ChipSelector(row);
	high=ReadControlRegister(MPU9250_GYRO_XOUT_H);
	ChipSelector(dummy_IMU);
	SPI_delay();  
	Gyro[row][0] = high; 
	Gyro[row][1] = low;
	low=0, high=0;
    
	//read Y
	ChipSelector(row);
	low=ReadControlRegister(MPU9250_GYRO_YOUT_L);
	ChipSelector(dummy_IMU);
	SPI_delay();
	ChipSelector(row);
	high=ReadControlRegister(MPU9250_GYRO_YOUT_H);
	ChipSelector(dummy_IMU);
	SPI_delay();  
	Gyro[row][2] = high; 
	Gyro[row][3] = low;
	low=0, high=0;

	//read Z
	ChipSelector(row);
	low=ReadControlRegister(MPU9250_GYRO_ZOUT_L);
	ChipSelector(dummy_IMU);
	SPI_delay();
	ChipSelector(row);
	high=ReadControlRegister(MPU9250_GYRO_ZOUT_H);
	ChipSelector(dummy_IMU);
	SPI_delay();  
	Gyro[row][4] = high; 
	Gyro[row][5] = low;        
	low=0, high=0;

}

/*******************************************************************************
* Function Name: Read Compass' Data of IMU n
*********************************************************************************/
void ReadMag(int row){
	uint8_t low, high;
	
	//read X
	ChipSelector(row);
	low = ReadControlRegister(MPU9250_EXT_SENS_DATA_00);
	ChipSelector(dummy_IMU);
	SPI_delay();
	ChipSelector(row);
	high = ReadControlRegister(MPU9250_EXT_SENS_DATA_01);		
	ChipSelector(dummy_IMU);
	SPI_delay();  
	Mag[row][0] = high; 
	Mag[row][1] = low;
	low=0, high=0;
    
	//read Y
	ChipSelector(row);
	low = ReadControlRegister(MPU9250_EXT_SENS_DATA_02);
	ChipSelector(dummy_IMU);
	SPI_delay();
	ChipSelector(row);
	high = ReadControlRegister(MPU9250_EXT_SENS_DATA_03);		
	ChipSelector(dummy_IMU);
	SPI_delay();  
	Mag[row][2] = high; 
	Mag[row][3] = low;
	low=0, high=0;
    
	//read Z
	ChipSelector(row);
	low = ReadControlRegister(MPU9250_EXT_SENS_DATA_04);
	ChipSelector(dummy_IMU);
	SPI_delay();
	ChipSelector(row);
	high = ReadControlRegister(MPU9250_EXT_SENS_DATA_05);
	ChipSelector(dummy_IMU);
	SPI_delay();	
	Mag[row][4] = high; 
	Mag[row][5] = low;
	low=0, high=0;

}

/********************************************************************************
* Function Name: ReadMagCal
*********************************************************************************/
void ReadMagCal(uint8_t row){
	uint8_t mag_x=0, mag_y=0, mag_z=0; 	
	//read X
	ChipSelector(row);
	mag_x = ReadControlRegister(MPU9250_EXT_SENS_DATA_00);
	ChipSelector(dummy_IMU);
	SPI_delay();
	//read Y		
	ChipSelector(row);
	mag_y = ReadControlRegister(MPU9250_EXT_SENS_DATA_01);     
	ChipSelector(dummy_IMU);
	SPI_delay();
	//read Z			
	ChipSelector(row);
	mag_z = ReadControlRegister(MPU9250_EXT_SENS_DATA_02); 
	ChipSelector(dummy_IMU);
	SPI_delay();
	
	MagCal[row][0] = mag_x;
	MagCal[row][1] = mag_y;
	MagCal[row][2] = mag_z;
}

/*******************************************************************************
* Function Name: Read Quaternion of IMU n
*********************************************************************************/
void ReadQuat(int n)
{	
      int16_t var_h;
      float qd0,qd1,qd2,qd3;        
      float aP[3]; //, fa[3];
      float Napla[4],g[4],qL[4];
        
      if (!c_mem.IMU_conf[n][0]) ReadAcc(n);
      if (!c_mem.IMU_conf[n][1]) ReadGyro(n);
                
      // how N sees P; N is 'Earth' equivalent, P is for sensor
      // e.g. N = imu1, P = imu0, how imu1 sees imu0
      var_h = Accel[n][0];
      aP[0] = (int16_t)(var_h << 8 | Accel[n][1])*TICK2ACC; 
      var_h = Accel[n][2];
      aP[1] = (int16_t)(var_h << 8 | Accel[n][3])*TICK2ACC; 
      var_h = Accel[n][4];
      aP[2] = (int16_t)(var_h << 8 | Accel[n][5])*TICK2ACC;
      
      g[0] = 0;
      var_h = Gyro[n][0];
      g[1] = ((int16_t)(var_h << 8 | Gyro[n][1]))*TICK2GYRO;
      if ( fabs(g[1]) < GYRO_THR ) g[1] = 0;
			var_h = Gyro[n][2];
      g[2] = ((int16_t)(var_h << 8 | Gyro[n][3]))*TICK2GYRO; 
			if ( fabs(g[2]) < GYRO_THR ) g[2] = 0;
      var_h = Gyro[n][4];
      g[3] = ((int16_t)(var_h << 8 | Gyro[n][5]))*TICK2GYRO;
      if ( fabs(g[3]) < GYRO_THR ) g[3] = 0;

      // Normalize to 1
      v3_normalize(aP);
        				
      // Current quaternion assignment
      qL[0] = Quat[n][0];
      qL[1] = Quat[n][1];
      qL[2] = Quat[n][2];
      qL[3] = Quat[n][3];   

      // Napla computation
      Napla[0] = (-qL[2]*((2*(qL[1]*qL[3]-qL[0]*qL[2]) - aP[0])) + qL[1]*((2*(qL[0]*qL[1] + qL[2]*qL[3]) - aP[1])))*BETA;
      Napla[1] = (qL[3]*((2*(qL[1]*qL[3]-qL[0]*qL[2]) - aP[0])) + qL[0]*((2*(qL[0]*qL[1] + qL[2]*qL[3]) - aP[1])) -2*qL[1]*((2*(0.5 - qL[1]*qL[1] - qL[2]*qL[2]) - aP[2])))*BETA;
      Napla[2] = (-qL[0]*((2*(qL[1]*qL[3]-qL[0]*qL[2]) - aP[0])) +qL[3]*((2*(qL[0]*qL[1] + qL[2]*qL[3]) - aP[1])) -2*qL[2]*((2*(0.5 - qL[1]*qL[1] - qL[2]*qL[2]) - aP[2])))*BETA;
      Napla[3] = (qL[1]*((2*(qL[1]*qL[3]-qL[0]*qL[2]) - aP[0])) + qL[2]*((2*(qL[0]*qL[1] + qL[2]*qL[3]) - aP[1])))*BETA;	
					
      qd0 = 0.5*(-(qL[1]*g[1] + qL[2]*g[2] + qL[3]*g[3])) - Napla[0];
      qd1 = 0.5*(qL[0]*g[1] + (qL[2]*g[3] - qL[3]*g[2])) - Napla[1];
      qd2 = 0.5*(qL[0]*g[2] + (qL[3]*g[1] - qL[1]*g[3])) - Napla[2];
      qd3 = 0.5*(qL[0]*g[3] + (qL[1]*g[2] - qL[2]*g[1])) - Napla[3];
                
      qL[0] = qL[0] + qd0*execution_time_ms*1000;
      qL[1] = qL[1] + qd1*execution_time_ms*1000;
      qL[2] = qL[2] + qd2*execution_time_ms*1000;
      qL[3] = qL[3] + qd3*execution_time_ms*1000;
			
      if (sqrt(qL[0]*qL[0] + qL[1]*qL[1] + qL[2]*qL[2] + qL[3]*qL[3]) == 0) {
		qL[0] = Quat[n][0];
		qL[1] = Quat[n][1];
		qL[2] = Quat[n][2];
		qL[3] = Quat[n][3];
      }

      v4_normalize(qL);
        			
	  // Quaternion update
      Quat[n][0] = qL[0];
      Quat[n][1] = qL[1];
      Quat[n][2] = qL[2];
      Quat[n][3] = qL[3];

}

/*******************************************************************************
* Function Name: Read Temperature Data of IMU n
*********************************************************************************/
void ReadTemp(int n)
{
	uint8_t low=0, high=0;	
	
	//read X
	ChipSelector(n);
	low=ReadControlRegister(MPU9250_TEMP_OUT_L);
	ChipSelector(dummy_IMU);
	SPI_delay();
	ChipSelector(n);
	high=ReadControlRegister(MPU9250_TEMP_OUT_H);
	ChipSelector(dummy_IMU);
	SPI_delay();

	Temp[n][0] = high; 
	Temp[n][1] = low; 
	low=0, high=0;

}

/********************************** *********************************************
* Function Name: Write Control Register
*********************************************************************************/
void WriteControlRegister(uint8_t address, uint8_t data){
	
	uint8_t outputBuffer[2] = {MPU9250_WCR | address, data};	
	while(HAL_SPI_Transmit(&hspi1, outputBuffer, 2, HAL_MAX_DELAY) != HAL_OK);

}

/*******************************************************************************
* Function Name: Read Control Register
*********************************************************************************/
uint8_t ReadControlRegister(uint8_t address){	

	uint8_t data_read;
	uint8_t address_f;
    address_f	= (address | MPU9250_RCR);

   	while (HAL_SPI_Transmit(&hspi1, (uint8_t*)&address_f, 1, HAL_MAX_DELAY) != HAL_OK) {}
	while (HAL_SPI_Receive(&hspi1, &data_read, 1, HAL_MAX_DELAY) != HAL_OK);
	return data_read;

}

/*******************************************************************************
* Function Name: SPI delay
*********************************************************************************/
void SPI_delay(){ // Sto usando la libreria DWT ma al momento non l'ho testata
    switch( c_mem.SPI_read_delay ) {
        case 1:     // Low
        	DWT_Delay((uint16_t)SPI_DELAY_LOW);
        	break;
        case 2:     // High
        	DWT_Delay((uint16_t)SPI_DELAY_HIGH);
            break;
        default:    // None
            break;
    }
}
