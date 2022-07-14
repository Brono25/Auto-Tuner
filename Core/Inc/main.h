/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define HLF_BUFFER_LEN 2048
#define FULL_BUFFER_LEN (2 * HLF_BUFFER_LEN)
#define BLOCK_SIZE_FLOAT HLF_BUFFER_LEN


#define THRSHLD 35
#define DC_BIAS 2200

#define BLOCK_SIZE 2048
#define FS 40000


void print_arr(float *arr, uint16_t length);
void mpm_sum_f32(float32_t *pSrc, uint16_t scrLen, float32_t *pRes);
void mpm_find_peak_f32(float32_t *pSrc, uint16_t *tau);
void mpm_NSDF_f32(float32_t *pSrc, float32_t **pDst);
void mpm_parabolic_interpolation_f32(uint16_t x_pos, float32_t a, float32_t b, float32_t c, float32_t *delta_tau);
void mpm_mcleod_pitch_method_f32(float32_t *pSrc, float32_t *pitch_estimate);





/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
