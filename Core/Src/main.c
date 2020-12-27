/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define white 1
#define black 0
#define OUT_LINE  100.0
//-----------------------
#define ON 33456254
#define OFF 33441974
#define ONE 33444014
#define TWO 33478694
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum {START,DATA,DATA_FULL}State_ir;
typedef enum {CENTER,RIGHT,LEFT} Out_state;
typedef enum {HOME,STARTING,RUN} State;

struct Control_NEC
{
	uint32_t input_capture;
	uint32_t input_diference;
	uint32_t input_last;
	uint32_t data;
	uint8_t sample;
	State_ir  state;
}NEC_IR;

struct Control_pid
{
	int U;
	float KP;
	float KD;
	float KI;
	float error_now;
	float error_last[6];
	float sum_errors;
};

struct Line_Follower
{
	int track_background_color;
 	int sensors_qtr[16];
	int speed_left;
	int speed_right;
	int MAX_SPEED;
	int MED_SPEED;
	State state;
	Out_state out_state;
	struct Control_pid PID;
}LF;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Init_Control_NEC(struct Control_NEC* nec);
void Init_Line_Follower(struct Line_Follower* lf);
void Turn_on_off(struct Control_NEC* nec,struct Line_Follower* lf);
int Read_adc(void);
void Read_sensors_qtr(struct Line_Follower* lf);
void Get_error(struct Line_Follower* lf);
void Motor_left (struct Line_Follower* lf);
void Motor_right(struct Line_Follower* lf);
void Turbine(int speed);
void Line_follower_state_machine(struct Line_Follower* lf);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Init_Control_NEC(&NEC_IR);
  Init_Line_Follower(&LF);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOC, DIS_A_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DIS_B_Pin,GPIO_PIN_RESET);
  Turbine(12);
  HAL_Delay(8000);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Turn_on_off(&NEC_IR,&LF);
	  Line_follower_state_machine(&LF);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Init_Line_Follower(struct Line_Follower* lf)
{
	lf->track_background_color = white;
	for(int i=0;i<16;i++)
		lf->sensors_qtr[i] = 0;
	lf->speed_left = 0;
	lf->speed_right = 0;
	lf->MAX_SPEED = 499;
	lf->MED_SPEED = 345;
	lf->state = HOME;
	lf->out_state = CENTER;
	lf->PID.U = 0;
	lf->PID.KP = 61.2444;
	lf->PID.KD = 168.4224;
	lf->PID.KI = 1.0;
	lf->PID.error_now = 0.0;
	for(int i=0;i<6;i++)
		lf->PID.error_last[i] = 0.0;
	lf->PID.sum_errors = 0.0;
}
//-----------------------------------------------------------------------
void Init_Control_NEC(struct Control_NEC* nec)
{
	nec->input_capture = 0;
	nec->input_diference = 0;
	nec->input_last = 0;
	nec->data = 0;
	nec->sample = 0;
	nec->state = START;
}
//---------------------------------------------------------------------
int Read_adc(void)
{
	int sensor_value = 4095;
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,5) == HAL_OK)
		sensor_value  = (int)(HAL_ADC_GetValue(&hadc1));
	HAL_ADC_Stop(&hadc1);
	return sensor_value;
}
//---------------------------------------------------------------------
void Read_sensors_qtr(struct Line_Follower* lf)
{
	for(int i = 0;i<16;i++)
	{
		HAL_GPIO_WritePin(GPIOE, S0_Pin, i & 0x01);
		HAL_GPIO_WritePin(GPIOE, S1_Pin, i & 0x02);
		HAL_GPIO_WritePin(GPIOE, S2_Pin, i & 0x04);
		HAL_GPIO_WritePin(GPIOE, S3_Pin, i & 0x08);
		lf->sensors_qtr[i] = Read_adc();
	}
}
//---------------------------------------------------------------------
void Get_error(struct Line_Follower* lf)
{
	int max;
	int min;
	int threshold;
	int range;
	int bit_sensor[16];
	int sum = 0;
	int weigth[8] = {8,7,6,5,4,3,2,1};
	int errorLeft = 0;
	int errorRight = 0;
	//Read samples from each sensor
	Read_sensors_qtr(lf);
	max = min = lf->sensors_qtr[0];
	for(int i=1;i<16;i++)
	{
		if(lf->sensors_qtr[i]> max)
			max = lf->sensors_qtr[i];
		if(lf->sensors_qtr[i] < min)
			min = lf->sensors_qtr[i];
	}
	range = max-min;
	if(range > 400)
	{
		threshold = (range/2)+min;
		for(int i=0;i<16;i++)
		{
			if(lf->track_background_color)
				bit_sensor[i] = (lf->sensors_qtr[i] < threshold) ? 1 : 0;
			else
				bit_sensor[i] = (lf->sensors_qtr[i] > threshold) ? 1 : 0;
		}
		for(int i=0;i<8;i++)
		{
			errorLeft += bit_sensor[i]*weigth[i];
			errorRight += bit_sensor[15-i]*weigth[i];
		}
		for(int i=0;i<16;i++)
			sum += bit_sensor[i];
		lf->PID.error_now = (float)(errorRight-errorLeft)/(float)(sum);
		lf->out_state = ((lf->PID.error_now <= 2.5)&&(lf->PID.error_now >=(0-2.5)))  ? CENTER : lf->out_state;
		lf->out_state = ((lf->PID.error_now > 2.5)&&(lf->PID.error_now <=8.0))        ?  RIGHT: lf->out_state;
        lf->out_state = ((lf->PID.error_now <(0-2.5))&&(lf->PID.error_now >=(0-8.0))) ? LEFT: lf->out_state;
	}
	else
		lf->PID.error_now =  OUT_LINE;
}

//---------------------------------------------------------------------
void Motor_left(struct Line_Follower* lf)
{
	if(!lf->speed_left)
	{
		HAL_GPIO_WritePin(GPIOC, DIR_A_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);

	}
	else
	{ 
		lf->speed_left = (lf->speed_left  >= lf->MAX_SPEED) ? lf->MAX_SPEED : lf->speed_left;
		if (lf->speed_left >=1)
		{
			HAL_GPIO_WritePin(GPIOC, DIR_A_Pin,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,lf->speed_left);
		}
		else
		{
			lf->speed_left *= (0-1);
			lf->speed_left = (lf->speed_left >= lf->MAX_SPEED) ? lf->MAX_SPEED : lf->speed_left;
			HAL_GPIO_WritePin(GPIOC, DIR_A_Pin,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,lf->speed_left );
      }
   }
   return;
}
//---------------------------------------------------------------------
void Motor_right(struct Line_Follower* lf)
{
	if(!lf->speed_right)
	{
		HAL_GPIO_WritePin(GPIOB, DIR_B_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);

	}
	else
	{
		lf->speed_right = (lf->speed_right >= lf->MAX_SPEED) ? lf->MAX_SPEED : lf->speed_right;
		if (lf->speed_right >=1)
		{
			HAL_GPIO_WritePin(GPIOB, DIR_B_Pin,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,lf->speed_right);
		}
		else
		{
			lf->speed_right *= (0-1);
			lf->speed_right = (lf->speed_right >= lf->MAX_SPEED) ? lf->MAX_SPEED : lf->speed_right;
			HAL_GPIO_WritePin(GPIOB, DIR_B_Pin,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,lf->speed_right);
      }
   }
   return;
}
//---------------------------------------------------------------------
void Turbine(int speed)
{
	if(speed < 0)
		speed = 0;
	else if(speed > 100)
		speed = 100;
	speed += 100;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,speed);
	return;
}
//-------------------------------------------------------------------
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM2)
	{
		if(NEC_IR.state == START)
		{
			//Restart counter
			__HAL_TIM_SetCounter(&htim2,0);
			//Clear variables
			NEC_IR.input_capture = 0;
			NEC_IR.input_last = 0;
			NEC_IR.input_diference = 0;
			NEC_IR.sample = 0;
			//start protocol
			NEC_IR.state = DATA;
	  }
	  else if(NEC_IR.state == DATA)
	  {
		  //Read Timer
		  NEC_IR.input_capture = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value
		  //get difference between first pulse to second pulse
		  NEC_IR.input_diference =  NEC_IR.input_capture - NEC_IR.input_last;
		  //if the logic bit 1 occurs
		  if(NEC_IR.input_diference > 215 && NEC_IR.input_diference < 235)//225
		  {
			 NEC_IR.data |= (1UL << (31 - NEC_IR.sample));   // write 1
			 NEC_IR.sample++;//increase sample
		  }
		  //if the logic bit 0 occurs
		  else if(NEC_IR.input_diference > 102 && NEC_IR.input_diference < 122)//112
		  {
			  NEC_IR.data &= ~(1UL << (31 - NEC_IR.sample));//write 0
			  NEC_IR.sample++;//increase sample
		  }
		  if(NEC_IR.sample==31)//if sample is 31 data is full
			  NEC_IR.state = DATA_FULL;
		  NEC_IR.input_last = NEC_IR.input_capture;
     }
   }
   return;
}
//--------------------------------------------------------------------------------------
void Turn_on_off(struct Control_NEC* nec,struct Line_Follower* lf)
{
	if(nec->state == DATA_FULL)
	{
		switch (nec->data)
		{
			case ON:
				lf->state = STARTING;
		      	break;
		    case OFF:
				lf->state = HOME;
		        break;
		    case ONE:
		        lf->PID.KP = 51.4257;//98.5219//142.8568;//51.4257;
		        lf->PID.KI = 1.0;//1.52//2.318;//1.0
		        lf->PID.KD = 141.4208;//270.9322//406.3984//;141.4208
		        lf->MAX_SPEED = 399;//499//419
		        lf->MED_SPEED  = 300;//499//289
		        lf->state = STARTING;
		        break;
		    case TWO:
		    	lf->PID.KP = 61.2444;//98.5219//142.8568;//51.4257;
		    	lf->PID.KD = 168.4224;//270.9322//406.3984//;141.4208
		    	lf->PID.KI = 1.0;//1.52//2.318;//1.0
		    	lf->MAX_SPEED = 499;//499//419
		    	lf->MED_SPEED  = 345;//499//289
		    	lf->state = STARTING;
		    	break;
		}
		HAL_Delay(150);
		nec->data = 0;
		nec->state = START;
	}
}
//-----------------------------------------------------------------------------------------
void Line_follower_state_machine(struct Line_Follower* lf)
{
	switch (lf->state)
	{
		case HOME:
			Turbine(12);
			lf->speed_left = 0;
			lf->speed_right = 0;
			break;
		case STARTING:
			Turbine(38);
			lf->state = RUN;
			HAL_Delay(1000);
			break;
		case RUN:
			//Read real error -10 to 10
			Get_error(lf);
			//Out line
			if(lf->PID.error_now == OUT_LINE)
			{
				//Out line state machine//
				switch (lf->out_state)
				{
					case CENTER:
		            	lf->speed_left = lf->MED_SPEED;
		            	lf->speed_right = lf->MED_SPEED;
		                break;
		            case RIGHT:
		            	lf->speed_left = lf->MAX_SPEED;//left
		            	lf->speed_right = (0-lf->MAX_SPEED);//Right
		            	break;
		            case LEFT:
		            	lf->speed_left = (0-lf->MAX_SPEED);
		            	lf->speed_right = lf->MAX_SPEED;
		                break;
				}
			}
			//On line
			else
			{
				lf->PID.sum_errors = 0;
				for(int i =0;i<6;i++)
					lf->PID.sum_errors += lf->PID.error_last[i];
				lf->PID.U = (int)(lf->PID.KP * lf->PID.error_now + lf->PID.KD * (lf->PID.error_now - lf->PID.error_last[0])+lf->PID.KI*lf->PID.sum_errors);
				lf->speed_left = lf->MED_SPEED + lf->PID.U;
				lf->speed_right = lf->MED_SPEED - lf->PID.U;
				lf->PID.error_last[5] = lf->PID.error_last[4];
				lf->PID.error_last[4] = lf->PID.error_last[3];
				lf->PID.error_last[3] = lf->PID.error_last[2];
				lf->PID.error_last[2] = lf->PID.error_last[1];
				lf->PID.error_last[1] = lf->PID.error_last[0];
				lf->PID.error_last[0] = lf->PID.error_now;
			}
			break;
		}
		Motor_left(lf);
		Motor_right(lf);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
