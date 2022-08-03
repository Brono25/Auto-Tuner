/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <stdio.h>
#include <stdlib.h>
#include "oled_print.h"
#include "mpm.h"
#include "TJ_MPU6050.h"
#include "ssd1306.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




// ARM IIR Filter Settings
#define NUM_IIR_STAGES 1
// ADC Defines
#define DC_BIAS 2280
#define BLOCK_SIZE 1024
#define FS 40000
#define THRESHOLD 600

#define NUM_STRINGS 6

#define E2 82.41
#define A2 110.0
#define D3 146.83
#define G3 196.0
#define B3 246.94
#define E4 329.63

#define E2_STRING_NUM 0
#define A2_STRING_NUM 1
#define D3_STRING_NUM 2
#define G3_STRING_NUM 3
#define B3_STRING_NUM 4
#define E4_STRING_NUM 5

#define U_FREQ_ERROR 20
#define L_FREQ_ERROR 20


#define TABLE_SIZE   5  //must be odd
#define TABLE_CENTRE TABLE_SIZE / 2
#define MIN_CORRECT 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

OPAMP_HandleTypeDef hopamp1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

int motor_wait = 0;
int callback_state = 0;
int Screenmode = 0;
int string_tracking = E2_STRING_NUM;
float32_t curr_target_string[NUM_STRINGS];

int mode = 0;

int correct_pitch_counter = 0;


//Current state
void (*state)();


// Pitch
float32_t pitch_table[TABLE_SIZE] = {0};
float32_t *table_pos_ptr = &pitch_table[0];





// ADC Variables
float32_t guitar_signal[BLOCK_SIZE];
uint16_t  adc_buff[2 * BLOCK_SIZE];
uint16_t *in_ptr;


// RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;

int screenflip = 0;

// ARM IIR Filter Variables
arm_biquad_casd_df1_inst_f32 iir_settings;
float iir_state[4];
static float iir_taps[5] = {
								0.997987115675119,
								-1.995974231350238,
								 0.997987115675119,
								 1.995970179642828,
								-0.995978283057647
							};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void state_string_pitch(int UPP_LIM, int LOW_LIM, float32_t target_freq);
void state_get_pitch_error();
void state_tune_up_fine();
void state_tune_down_fine();
void state_tune_down_fast();
void state_tune_up_fast();
void state_get_pitch();
void state_tune();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim)
{


	if(htim->Instance == TIM16)
	{
		MPU6050_Get_Accel_Scale(&myAccelScaled);
		MPU6050_Get_Gyro_Scale(&myGyroScaled);
		if (myAccelScaled.x > 0 && screenflip == 0)
		{
			ssd1306_Init1();
			screenflip = 1;
		} else if (myAccelScaled.x < 0 && screenflip == 1)
		{
			ssd1306_Init2();
			screenflip = 0;

		}

	} else if (htim->Instance == TIM2)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	}
}


/*  HAL_TIM_SET_PRESCALER(&htim2,newValue); */

void init_tunings()
{
	curr_target_string[0] = E2;
	curr_target_string[1] = A2;
	curr_target_string[2] = D3;
	curr_target_string[3] = G3;
	curr_target_string[4] = B3;
	curr_target_string[5] = E4;
}


void adc_to_guitar_signal(uint16_t *src, float32_t *guitar_signal)
{
	for(int i = 0; i < BLOCK_SIZE; i++)
	{
		guitar_signal[i] = (float32_t) src[i] - DC_BIAS;
	}
	arm_biquad_cascade_df1_f32(&iir_settings, guitar_signal, guitar_signal, BLOCK_SIZE);
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1)
{
	in_ptr = &adc_buff[0];
	adc_to_guitar_signal(in_ptr, &guitar_signal[0]);

	float32_t *p  = &guitar_signal[0];
	for(int i = 0; i < BLOCK_SIZE; i++)
	{
		if(*p > THRESHOLD)
		{
			callback_state = 1;
			return;
		}
		p++;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	in_ptr = &adc_buff[BLOCK_SIZE];
	adc_to_guitar_signal(in_ptr, &guitar_signal[0]);

	float32_t *p  = &guitar_signal[0];
	for(int i = 0; i < BLOCK_SIZE; i++)
	{
		if(*p > THRESHOLD)
		{
			callback_state = 1;
			return;
		}
		p++;
	}
}


int cmpfunc(const void * a, const void * b) {
  float32_t fa = *(const float32_t*) a;
  float32_t fb = *(const float32_t*) b;
  return (fa > fb) - (fa < fb);
}

void get_frequency(float32_t *signal, float32_t target_freq, float32_t *out_freq)
{
	if (callback_state == 1)
	{
		float32_t curr_freq = 0;
		mpm_mcleod_pitch_method_f32(&signal[0], &curr_freq);
		callback_state = 0;
		*out_freq = ceill(4 * curr_freq) / 4;
	}
}

float32_t get_error_in_cents(float32_t curr_frequency, float32_t target_frequency)
{
	float32_t error = 1200 * log2(curr_frequency / target_frequency);
	return round(error);
}


void toggle_motor_wait()
{
	if (motor_wait == 2)
	{
		motor_wait = 0;
	} else
	{
		motor_wait++;
	}
}

void state_tune_up_fine()
{
	int pulse_width = 46;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
	HAL_Delay(100);
	pulse_width = 50;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
	state = state_get_pitch;
}

void state_tune_up_fast()
{
	int pulse_width = 40;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
	HAL_Delay(200);
	pulse_width = 50;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
	HAL_Delay(100);
	motor_wait = 0;
	state = state_get_pitch;
}

void state_tune_down_fine()
{
	int pulse_width = 54;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
	HAL_Delay(100);
	pulse_width = 50;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
	state = state_get_pitch;
}

void state_tune_down_fast()
{
	int pulse_width = 64;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
	HAL_Delay(200);
	pulse_width = 50;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
	HAL_Delay(100);
	motor_wait = 0;
	state = state_get_pitch;
}


void iterate_table_pos()
{
	if(table_pos_ptr == &pitch_table[TABLE_SIZE - 1])
	{
		table_pos_ptr = &pitch_table[0];
	}else
	{
		table_pos_ptr++;
	}

}

float32_t  get_min_table()
{

	float32_t min = pitch_table[0];
	for (int i = 0; i < TABLE_SIZE; i++)
	{
		if(pitch_table[i] < min)
		{
			min = pitch_table[i];
		}
	}
	//qsort(pitch_table, TABLE_SIZE, sizeof(float32_t), cmpfunc);
	//float32_t median_error = tmp[TABLE_CENTRE];
	return min;
}


int init_table(int PITCH_U, int PITCH_L, float32_t target_freq)
{
	#define END TABLE_SIZE - 1

	if( pitch_table[END] < PITCH_L || pitch_table[END] > PITCH_U)
	{
		float32_t freq = 0;
		get_frequency(&guitar_signal[0], target_freq, &freq);
		if(PITCH_L < freq && freq < PITCH_U)
		{
			*table_pos_ptr = freq;
			iterate_table_pos();
			return 1;
		}
	}
	return 0;
}



void state_string_pitch(int UPP_LIM, int LOW_LIM, float32_t target_freq)
{

	if (correct_pitch_counter ==  MIN_CORRECT)
	{
		char *str = "Correct";
		oled_print_string(str);
		string_tracking++;
		correct_pitch_counter = 0;
		HAL_Delay(2000);
		return;
	}

	while(init_table(UPP_LIM, LOW_LIM, target_freq));

	float32_t freq = 0;
	get_frequency(&guitar_signal[0], target_freq, &freq);

	if(LOW_LIM < freq && freq < UPP_LIM)
	{
		*table_pos_ptr = freq;
		iterate_table_pos();
		float32_t m_freq = get_min_table();
		float32_t error = get_error_in_cents(m_freq, target_freq);
		oled_print_f32(&error);

		if (motor_wait == 0)
		{
			if(error > 40) {
				state = state_tune_down_fast;
				correct_pitch_counter = 0;
			} else if (error > 7)
			{
				state = state_tune_down_fine;
				correct_pitch_counter = 0;
			}else if(error < -40.0)
			{

				state = state_tune_up_fast;
				correct_pitch_counter = 0;
			}else if(error < -7)
			{
				state = state_tune_up_fine;
				correct_pitch_counter = 0;
			} else
			{
				correct_pitch_counter++;
			}
		}
	}
	else
	{
		oled_clear_screen();
	}
	toggle_motor_wait();
}


void state_get_pitch()
{
	#define E2_U 100
	#define E2_L 55
	#define A2_U 140
	#define A2_L 85
	#define D3_U 190
	#define D3_L 115
	#define G3_U 230
	#define G3_L 155
	#define B3_U 320
	#define B3_L 205
	#define E4_U 350
	#define E4_L 305

	if (string_tracking == E2_STRING_NUM)
	{
		state_string_pitch(E2_U, E2_L, E2);

	} else if (string_tracking == A2_STRING_NUM)
	{
		state_string_pitch(A2_U, A2_L, A2);

	}else if (string_tracking == D3_STRING_NUM)
	{
		state_string_pitch(D3_U, D3_L, D3);

	}else if (string_tracking == G3_STRING_NUM)
	{
		state_string_pitch(G3_U, G3_L, G3);

	}else if (string_tracking == B3_STRING_NUM)
	{
		state_string_pitch(B3_U, B3_L, B3);

	}else if (string_tracking == E4_STRING_NUM)
	{
		state_string_pitch(E4_U, E4_L, E4);
	} else
	{
		string_tracking =  E2_STRING_NUM;
	}










}





void metronome(void)
{

	HAL_TIM_Base_Start_IT(&htim2);



	int bpm = 100;

	oled_timing_screen(bpm);
	ssd1306_UpdateScreen();




	while (1)
	{
		int exit = HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin);

		if (exit == 1)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			return;
		}

		if (HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin))
		{
			bpm++;
			oled_timing_screen(bpm);
			ssd1306_UpdateScreen();

		}
		if (HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin))
		{
			bpm--;
			oled_timing_screen(bpm);
			ssd1306_UpdateScreen();

		}


		__HAL_TIM_SET_PRESCALER(&htim2, bpm * 100);


	}




}








/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	MPU_ConfigTypeDef myMpuConfig;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_OPAMP1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_OC_Start(&htim6, TIM_CHANNEL_6);
	HAL_OPAMP_SelfCalibrate (&hopamp1);
	HAL_OPAMP_Start(&hopamp1);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff, 2 * BLOCK_SIZE);
	arm_biquad_cascade_df1_init_f32(&iir_settings, NUM_IIR_STAGES, &iir_taps[0], &iir_state[0]);


	oled_init();
	init_tunings();

	MPU6050_Init(&hi2c1);
	myMpuConfig.Accel_Full_Scale = AFS_SEL_2g;
    myMpuConfig.CONFIG_DLPF = Internal_8MHz;
    myMpuConfig.ClockSource = DLPF_184A_188G_Hz;
    myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
    myMpuConfig.Sleep_Mode_Bit = 0;
    MPU6050_Config(&myMpuConfig);

	//HAL_TIM_Base_Start_IT(&htim16);



	state = state_get_pitch;



	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);



	int counter = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	while (1)
	{
		if (mode == 0)
		{




			int Button1_val = HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin);
			int Button2_val = HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin);
			int Button3_val = HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin);

			if (Button2_val == 1 || Button3_val == 1)
			{

				if (counter == 0 )
				{

					oled_selection_screen();
					ssd1306_DrawRectangle(0, 36, 128,  58, 0);
					ssd1306_UpdateScreen();
					counter = 1;
					HAL_Delay(100);

				} else
				{
					oled_selection_screen();
					ssd1306_DrawRectangle(0, 8, 128,  30, 0);
					ssd1306_UpdateScreen();

					counter = 0;
					HAL_Delay(100);
				}
			}

			if (Button1_val == 1)
			{
				if (counter == 0)
				{
					oled_tone_screen(100);
					ssd1306_UpdateScreen();
					HAL_Delay(200);
			} else {
					metronome();
					oled_selection_screen();
					HAL_Delay(200);


				}
			}


		}else if (mode == 1)
		{
			//oled_timing_screen(100);
			state();

		}







    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00300F33;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
  hopamp1.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALPOWER;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2352;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 20000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 4000 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|Button2_Vcc_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Switch_High_GPIO_Port, Switch_High_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA1 LED_BLUE_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button2_Pin Button3_Pin */
  GPIO_InitStruct.Pin = Button2_Pin|Button3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 Switch_High_Pin Button2_Vcc_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|Switch_High_Pin|Button2_Vcc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Switch_Int_Pin */
  GPIO_InitStruct.Pin = Switch_Int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Switch_Int_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button1_Pin */
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	Screenmode = HAL_GPIO_ReadPin(GPIOA, Switch_Int_Pin);



	if (Screenmode == 0)
	{
		oled_selection_screen();
		mode = 0;

	} else
	{
		oled_clear_screen();
		mode = 1;
		char *s = "Tuning";
		oled_print_string(s);
		HAL_Delay(2000);
		oled_clear_screen();

	}


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

