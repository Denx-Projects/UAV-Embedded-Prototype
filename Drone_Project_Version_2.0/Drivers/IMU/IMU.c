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
#include "cmsis_os.h"
#include "main.h"

#include "IMU.h"


#define IMU_Address (0x68 << 1)
#define WHO_AM_I_REG 0x00
#define REG_BAMK_SEL 0x7F
#define ACCEL_START_REG 0x2D
#define Timeout 100
#define Conversor_Axis 2048.0f
#define Conversor_Gyro 16.4f


extern I2C_HandleTypeDef hi2c1;
extern osSemaphoreId Samples_TookHandle;
struct IMU_RAW_t;
struct IMU_Error_t;





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
	// Accelerometer ±16 g +DLPF
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

HAL_StatusTypeDef IMU_Read(IMU_Data_t *data, IMU_Error_t *error, IMU_Data_t *copy, int16_t Samples_Required)
{
	uint8_t Value_IMU[12];
	int16_t X_Raw;
	int16_t Y_Raw;
	int16_t Z_Raw;
	int16_t Gyro_X_Raw;
	int16_t Gyro_Y_Raw;
	int16_t Gyro_Z_Raw;
	IMU_RAW_t Raw;
	int16_t Copia = 0;

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

    Raw.accel_x = X_Raw /Conversor_Axis;
    Raw.accel_y = Y_Raw/Conversor_Axis;
    Raw.accel_z = Z_Raw/Conversor_Axis;

    Raw.gyro_x = Gyro_X_Raw/Conversor_Gyro;
    Raw.gyro_y = Gyro_Y_Raw/Conversor_Gyro;
    Raw.gyro_z = Gyro_Z_Raw/Conversor_Gyro;

    if (uxSemaphoreGetCount(Samples_TookHandle) > 0)
    {
    	error->accel_x= error->accel_x + Raw.accel_x;
    	error->accel_y = error->accel_y + Raw.accel_y;
    	error->accel_z = error->accel_z + Raw.accel_z;
    	error->gyro_x = error->gyro_x + Raw.gyro_x;
    	error->gyro_y = error->gyro_y + Raw.gyro_y;
    	error->gyro_z = error->gyro_z + Raw.gyro_z;
    	xSemaphoreTake(Samples_TookHandle, 0);
    	if (uxSemaphoreGetCount(Samples_TookHandle) == 0)
    	{
    		error->accel_x = error->accel_x/Samples_Required;
    		error->accel_y = error->accel_y/Samples_Required;
    		error->accel_z = error->accel_x/Samples_Required - 1.0f;
    		error->gyro_x = error->gyro_x/Samples_Required;
    		error->gyro_y = error->gyro_y/Samples_Required;
    		error->gyro_z = error->gyro_z/Samples_Required;
    	}
    }
    else
    {
    	data->accel_x = Raw.accel_x - error->accel_x;
    	data->accel_y = Raw.accel_y - error->accel_y;
    	data->accel_z = Raw.accel_z - error->accel_z;

    	data->gyro_x = Raw.gyro_x - error->gyro_x;
    	data->gyro_y = Raw.gyro_y - error->gyro_y;
    	data->gyro_z = Raw.gyro_z - error->gyro_z;
    }


    return HAL_OK;
    }
}
