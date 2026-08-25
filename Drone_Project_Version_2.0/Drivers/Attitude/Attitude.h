/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Attitude.h
  * @brief          : Header for Attitude.c file.
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

#ifndef Attitude_H
#define Attitude_H

 typedef struct
 {
	 int16_t Pitch;
	 int16_t Roll;
	 int16_t Yaw;
 } Attitude_Data_t;

#include "main.h"
#include "C:\Users\migue\OneDrive\Escritorio\Escritorio\Drone_Project\Drone_Project\Drivers\IMU\IMU.h"
#include <math.h>

 HAL_StatusTypeDef Attitude_Init(Attitude_Data_t *data_a);
 HAL_StatusTypeDef Attitude_Change(IMU_Data_t *data_i, Attitude_Data_t *data_a, int16_t Increment);

#endif

