/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Attitude.c
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

#include "Attitude.h"








HAL_StatusTypeDef
Attitude_Init(Attitude_Data_t *data_a)
{
	data_a->Pitch = 0;
	data_a->Roll = 0;
	data_a->Yaw = 0;
	return HAL_OK;
}

HAL_StatusTypeDef Attitude_Change(IMU_Data_t *data_i, Attitude_Data_t *data_a,int16_t Increment)
{
	float Pitch_Raw_Accel;
	float Roll_Raw_Accel;
    float Pitch_Raw_Gyro;
	float Roll_Raw_Gyro;
	float Yaw_Raw_Gyro;
	float Pitch_Copy = (float) data_a->Pitch;
	float Roll_Copy = (float) data_a->Roll;
	float Yaw_Copy = (float) data_a->Yaw;
	float Pitch_Value;
	float Roll_Value;

	//uint16_t Yaw_Raw_Gyro;

	Pitch_Raw_Accel = 180*atan (data_i->accel_x/sqrt(data_i->accel_y*data_i->accel_y + data_i->accel_z*data_i->accel_z))/M_PI;
	Roll_Raw_Accel = 180*atan (data_i->accel_y/sqrt(data_i->accel_x*data_i->accel_x + data_i->accel_z*data_i->accel_z))/M_PI;
	Pitch_Raw_Gyro = Pitch_Copy + (data_i->gyro_y/1000)*Increment;
	Roll_Raw_Gyro = Roll_Copy + (data_i->gyro_x/1000)*Increment;
	Yaw_Raw_Gyro = Yaw_Copy + (data_i->gyro_z/1000)*Increment;

	Pitch_Value = 0.98f*Pitch_Raw_Gyro + 0.02f*Pitch_Raw_Accel;
	Roll_Value = 0.98f*Roll_Raw_Gyro + 0.02f*Roll_Raw_Accel;

	if (Pitch_Value >= 360)
	{
		Pitch_Value = Pitch_Value - 360;
	}
	else if (Pitch_Value <= -360)
	{
		Pitch_Value = Pitch_Value +360;
	}

	if (Roll_Value >= 360)
	{
		Roll_Value = Roll_Value - 360;
	}
	else if (Roll_Value <= -360)
	{
		Roll_Value = Roll_Value +360;
	}

	if (Yaw_Raw_Gyro >= 360)
	{
		Yaw_Raw_Gyro = Yaw_Raw_Gyro - 360;
	}
	else if (Yaw_Raw_Gyro <= -360)
	{
		Yaw_Raw_Gyro = Yaw_Raw_Gyro +360;
	}

	if (fabs (Yaw_Copy - Yaw_Raw_Gyro) > 0.05f)
	{
		Yaw_Copy = Yaw_Raw_Gyro;
	}
	else
	{
		Yaw_Raw_Gyro = Yaw_Copy;
	}




 	data_a->Pitch = (int16_t) Pitch_Value;
 	data_a->Roll = (int16_t) Roll_Value;
 	data_a->Yaw = (int16_t) Yaw_Raw_Gyro;

	return HAL_OK;
}
