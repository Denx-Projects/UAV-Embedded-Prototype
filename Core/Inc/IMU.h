/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU.h
  * @brief          : Header for IMU.c file.
  *                   This file contains the common defines of the application.
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
  */

#ifndef IMU_H
#define IMU_H



#include "main.h"

 typedef struct
 {
	 float accel_x;
	 float accel_y;
	 float accel_z;

	 float gyro_x;
	 float gyro_y;
	 float gyro_z;
 } IMU_Data_t;

 HAL_StatusTypeDef
 IMU_Init(void);
 HAL_StatusTypeDef
 IMU_Read(IMU_Data_t *data);

#endif

