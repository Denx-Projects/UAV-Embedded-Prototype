/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU.c
  * @brief          : IMU program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
**/
#include "IMU.h"
#include "cmsis_os.h"
#include "main.h"


#define IMU_Address (0x68 << 1)
#define WHO_AM_I_REG 0x00
#define REG_BAMK_SEL 0x7F
#define ACCEL_START_REG 0x2D
#define Timeout 100
#define Conversor_Axis 2048.0f
#define Conversor_Ms 9.87f
#define Conversor_Gyro 16.4f


extern I2C_HandleTypeDef hi2c1;
extern osMutexId IMU_Mutex;


HAL_StatusTypeDef
IMU_Init(void)
{
	uint8_t Info;
	uint8_t who_am_i;
	if ((HAL_I2C_Mem_Read( &hi2c1, IMU_Address, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100) != HAL_OK) || (who_am_i != 0xE1))
	{
     return HAL_ERROR;
	}
	// Select Bank 0
	Info = 0x00;
	if (HAL_I2C_Mem_Write(&hi2c1, IMU_Address, 0x7F, I2C_MEMADD_SIZE_8BIT, &Info, 1, 100) != HAL_OK)
	{
	 return HAL_ERROR;
	}
	// Wake up IKU
	Info = 0x01;
	if (HAL_I2C_Mem_Write(&hi2c1, IMU_Address, 0x06, I2C_MEMADD_SIZE_8BIT, &Info, 1, 100) != HAL_OK)
	{
	 return HAL_ERROR;
	}
	// Activate Accel and Gyro
	Info = 0x00;
	if (HAL_I2C_Mem_Write(&hi2c1, IMU_Address, 0x07, I2C_MEMADD_SIZE_8BIT, &Info, 1, 100) != HAL_OK)
    {
	 return HAL_ERROR;
    }
	// Select Bank 2
	Info = 0x20;
	if (HAL_I2C_Mem_Write(&hi2c1, IMU_Address, 0x7F, I2C_MEMADD_SIZE_8BIT, &Info, 1, 100) != HAL_OK)
    {
     return HAL_ERROR;
	}
	// Gyroscope ±2000 dps + DLPF
	Info = 0x2D;
	if (HAL_I2C_Mem_Write(&hi2c1, IMU_Address, 0x01, I2C_MEMADD_SIZE_8BIT, &Info, 1, 100) != HAL_OK)
	{
	 return HAL_ERROR;
	}
	// Accelerometer ±16 g + DLPF
	Info = 0x2D;
	if (HAL_I2C_Mem_Write(&hi2c1, IMU_Address, 0x14, I2C_MEMADD_SIZE_8BIT, &Info, 1, 100) != HAL_OK )
	{
	 return HAL_ERROR;
	}
	// Volver a Bank 0
	Info = 0x00;
	if (HAL_I2C_Mem_Write(&hi2c1, IMU_Address, 0x7F, I2C_MEMADD_SIZE_8BIT, &Info, 1, 100) != HAL_OK )
	{
	 return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef IMU_Read(IMU_Data_t *data)
{
	uint8_t Value_IMU[12];
	int16_t X_Raw;
	int16_t Y_Raw;
	int16_t Z_Raw;
	int16_t Gyro_X_Raw;
	int16_t Gyro_Y_Raw;
	int16_t Gyro_Z_Raw;

    if (HAL_I2C_Mem_Read(&hi2c1, IMU_Address, ACCEL_START_REG, I2C_MEMADD_SIZE_8BIT, Value_IMU, 12, Timeout) != HAL_OK)
    {
        return HAL_ERROR;
    }
    else
    {
    X_Raw = (int16_t)((Value_IMU[0] << 8) | Value_IMU[1]);
    Y_Raw = (int16_t)((Value_IMU[2] << 8) | Value_IMU[3]);
    Z_Raw = (int16_t)((Value_IMU[4] << 8) | Value_IMU[5]);
    Gyro_X_Raw = (int16_t)((Value_IMU[6] << 8)  | Value_IMU[7]);
    Gyro_Y_Raw = (int16_t)((Value_IMU[8] << 8)  | Value_IMU[9]);
    Gyro_Z_Raw = (int16_t)((Value_IMU[10] << 8) | Value_IMU[11]);

    data->accel_x = X_Raw/Conversor_Axis*Conversor_Ms;
    data->accel_y = Y_Raw/Conversor_Axis*Conversor_Ms;
    data->accel_z = Z_Raw/Conversor_Axis*Conversor_Ms;

    data->gyro_x = Gyro_X_Raw/Conversor_Gyro;
    data->gyro_y = Gyro_Y_Raw/Conversor_Gyro;
    data->gyro_z = Gyro_Z_Raw/Conversor_Gyro;


    return HAL_OK;
    }
}
