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
#include "stdlib.h"
#include "MCP3553.h"
#include "string.h"
#include "stdio.h"
#include "PID.h"
#include "pressureReceive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float pressure = PRESSURE_NORMAL;
float pressureCalc;
char Txdata[50];
char rx_buffer[30];
//char pressureRx[30];
uint8_t rx_index=0;
uint8_t rx_data;
int16_t voltage,current;
int16_t pwm;
volatile uint8_t flag_value = 0,flag_auto = 0, flag_speed = 0,flag_back = 0;
uint32_t Difference = 0,Difference_last=0;
float frequency = 0,frequencyCalc = 0;
uint8_t change_speed=0;
PidParameter PID_PRESSURE = {20,10,0.5};
PidParameter PID_SPEED = {0.12,1,0};
uint8_t count =0;
uint8_t countError = 0;
volatile uint32_t timeStart;
volatile PressureParameter pressureParam;
extern float last_error_pres,integrated_error_pres;
extern uint32_t timerPID_pres;
extern float last_error_pres_speed,integrated_error_pres_speed;
extern uint32_t timerPID_pres_speed;
extern float delta;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if (htim -> Instance == htim1.Instance)
	{
		uint8_t state_system = !HAL_GPIO_ReadPin(BLM_E_GPIO_Port,BLM_E_Pin);//1:run; 0:stop
		uint8_t state_dir = HAL_GPIO_ReadPin(BLM_SI_GPIO_Port, BLM_SI_Pin);//0: back; 1: forward
		uint8_t state_valse = HAL_GPIO_ReadPin(VLV1_GPIO_Port,VLV1_Pin);//0: close; 1: open
		uint8_t state_extreme_back= !HAL_GPIO_ReadPin(SW0_GPIO_Port,SW0_Pin);//0: not; 1: yes
		uint8_t state_extreme_front= !HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin);//0:not; 1: yes
		uint8_t state_motor= HAL_GPIO_ReadPin(BLM_ER_GPIO_Port,BLM_ER_Pin);//0: not error; 1:error
		sprintf(Txdata,"%.1f %.1f %d %d %d %d %d %d %d\n",pressure-PRESSURE_NORMAL,frequency,state_system,state_dir,
				state_valse,state_extreme_back,state_extreme_front,state_motor,flag_auto);
		HAL_UART_Transmit(&huart1, (uint8_t*)Txdata, strlen(Txdata), HAL_MAX_DELAY);
		if (change_speed)
			{
				count = 0;
				change_speed = 0;
			}
		else
			{
				if(frequency>200 || frequency<-200)
					frequency = 0;
				else if(++count==9)
					{
						frequency = 0;
						count = 0;
					}
			}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SW0_Pin)
	{
		//close the solenoid valve
		HAL_GPIO_WritePin(VLV1_GPIO_Port, VLV1_Pin, GPIO_PIN_RESET);
		//Stop motor
		HAL_GPIO_WritePin(BLM_E_GPIO_Port, BLM_E_Pin, GPIO_PIN_SET);
		//motor go forward
		HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
		flag_auto = 0;
		flag_back = 0;
		flag_speed = 0;
	}
	else
	if(GPIO_Pin == SW1_Pin)
	{
		//open the solenoid valve
		HAL_GPIO_WritePin(VLV1_GPIO_Port, VLV1_Pin, GPIO_PIN_SET);
		//motor go back
		HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_RESET);
		
		if (flag_auto == 0)
		{
			//stop motor
			HAL_GPIO_WritePin(BLM_E_GPIO_Port, BLM_E_Pin, GPIO_PIN_SET);
			//speed = 0
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
			flag_speed = 0;			
		}
		else
		{
			flag_back = 1;
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,150);
		}
	}
	else
	if(GPIO_Pin == BLM_ER_Pin) //error motor
	{
			flag_auto = 0;
			frequencyCalc = 0;
			flag_speed = 0;
			flag_back = 0;
			HAL_GPIO_WritePin(BLM_E_GPIO_Port, BLM_E_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance)
	{
		if (!rx_index)
		{
			for(int i=0;i<20;i++)
			{
				rx_buffer[i]='\0';
			}
		}
		if(rx_data!='\n' && rx_data!='\r' && rx_data != '/')
		{
			rx_buffer[rx_index++]=rx_data;
		}
		else
		{
			flag_value = 1;
			rx_index=0;
		}
		HAL_UART_Receive_IT(&huart1,&rx_data, 1);
	}
}
void set_value_from_UART(char* rxBuffer)
{
	char num[30];
	uint8_t num_index=0;
	while(rxBuffer[num_index]!='\0')
	{
	  num[num_index]=rxBuffer[num_index+1];
	  num_index++;
	}
	switch (rxBuffer[0])
	{
		case 'a': //pressure
		{
			timeStart = HAL_GetTick();
			HAL_GPIO_WritePin(BLM_E_GPIO_Port, BLM_E_Pin, GPIO_PIN_RESET);
			pressureParam = pressureCalcParam(num,pressure);
			last_error_pres = 0;
			integrated_error_pres = 0;
			timerPID_pres = HAL_GetTick();
			last_error_pres_speed = 0;
			integrated_error_pres_speed = 0;
			timerPID_pres_speed = HAL_GetTick();
			flag_auto = 1;
			HAL_GPIO_WritePin(VLV1_GPIO_Port, VLV1_Pin, GPIO_PIN_RESET);
			break;
		}
		case 'f': //speed (frequency) of motor in pwm
		{
			flag_auto = 0;
			frequencyCalc=atof(num);
			if(HAL_GPIO_ReadPin(BLM_SI_GPIO_Port, BLM_SI_Pin)==GPIO_PIN_RESET)
				frequencyCalc= - frequencyCalc;
			if (frequencyCalc > FREQUENCY_MAX) frequencyCalc = FREQUENCY_MAX;
			else if (frequencyCalc < -FREQUENCY_MAX) frequencyCalc = -FREQUENCY_MAX;
			flag_speed = 1;
			HAL_GPIO_WritePin(BLM_E_GPIO_Port, BLM_E_Pin, GPIO_PIN_RESET);
			last_error_pres_speed = 0;
			integrated_error_pres_speed = 0;
			timerPID_pres_speed = HAL_GetTick();
			break;
		}
		case 'e': //enable motor
			HAL_GPIO_WritePin(BLM_E_GPIO_Port, BLM_E_Pin, GPIO_PIN_RESET);
			break;
		case 's': //stop motor
			flag_auto = 0;
			frequencyCalc = 0;
			flag_speed = 0;
			flag_back = 0;
			HAL_GPIO_WritePin(BLM_E_GPIO_Port, BLM_E_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);			
			break;
		case 'b'://motor go back
			flag_auto = 0;
			HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_RESET);
			if(frequencyCalc>0) frequencyCalc = -frequencyCalc;
			break;
		case 'g': //motor go forward
			flag_auto = 0;
			HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_SET);
			if(frequencyCalc<0) frequencyCalc = -frequencyCalc;
			break;
		case 'o': //open valve
			flag_auto = 0;
			HAL_GPIO_WritePin(VLV1_GPIO_Port, VLV1_Pin, GPIO_PIN_SET);
			break;
		case 'c'://close valve
			flag_auto = 0;
			HAL_GPIO_WritePin(VLV1_GPIO_Port, VLV1_Pin, GPIO_PIN_RESET);
			break;
//		case 'p':
//		{
//			PID_PRESSURE.Kp = atof(num);
//			break;
//		}
//		case 'i': //
//		{
//			PID_PRESSURE.Ki = atof(num);
//			break;
//		}
//		case 'd': //
//		{
//			PID_PRESSURE.Kd = atof(num);
//			break;
//		}
//		case 'q':
//			delta = atof(num);
//		break;
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		Difference = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
		// Freq/Prescale = 24000
		frequency = (float)24000*60/3/Difference; //rpm
		if(!HAL_GPIO_ReadPin(BLM_SI_GPIO_Port,BLM_SI_Pin))
			frequency = -frequency;
		change_speed = 1;
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
  for(uint8_t i=0;i<5;i++)
	  pressure = readMCP3553()/0.00064;	
	
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart1,&rx_data, 1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);	

  HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BLM_E_GPIO_Port, BLM_E_Pin, GPIO_PIN_SET);
	
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,(uint16_t)(PWM_MAX*0.4));

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		float vol = readMCP3553();
		if(vol<3.3 && vol>0.7) pressure = vol/0.00064;
		if(pressure < PRESSURE_MIN||pressure>PRESSURE_MAX)
		{
			HAL_GPIO_WritePin(VLV1_GPIO_Port,VLV1_Pin,GPIO_PIN_SET);
			flag_auto = 0;
		}
		if (flag_value)
		{
			set_value_from_UART(rx_buffer);
			flag_value = 0;
		}
		if (!HAL_GPIO_ReadPin(BLM_E_GPIO_Port, BLM_E_Pin))
		{
			if(flag_auto & !flag_back)
			{
				pressureCalc = pressureReceive(pressure, timeStart,pressureParam);
				frequencyCalc = PID_Pressure_Calc(PID_PRESSURE, pressure, pressureCalc);
				pwm = PID_Speed_Calc(PID_SPEED,frequency,frequencyCalc);
				if(pwm>=0)
				{
					if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW1_Pin))
					{
						HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_SET);
						__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pwm);
					}
				}
				else
				{
					if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin))
					{
						HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_RESET);
						__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-pwm);
					}
				}
			}
			else if(flag_speed)
			{
				pwm = PID_Speed_Calc(PID_SPEED,frequency,frequencyCalc);
				if(pwm>=0)
				{
					if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW1_Pin))
					{
						HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_SET);
						__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pwm);
					}
				}
				else
				{
					if (HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin))
					{
						HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_RESET);
						__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-pwm);
					}
				}
			}
		}
		if (!HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin) && !HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin))
		{
			HAL_GPIO_WritePin(BLM_E_GPIO_Port, BLM_E_Pin, GPIO_PIN_SET);
		}
		else if (!HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin))
		{
			if(!HAL_GPIO_ReadPin(BLM_SI_GPIO_Port, BLM_SI_Pin))
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
				HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_SET);
			}
		}
		else if (!HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin))
		{
			if(!flag_back && !flag_auto)
			{
				if(HAL_GPIO_ReadPin(BLM_SI_GPIO_Port, BLM_SI_Pin))
				{
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
					HAL_GPIO_WritePin(BLM_SI_GPIO_Port, BLM_SI_Pin, GPIO_PIN_RESET);
				}					
				HAL_GPIO_WritePin(VLV1_GPIO_Port, VLV1_Pin, GPIO_PIN_SET);				
			}
			else if(flag_auto)
			{
				flag_back = 1;
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,150);
			}
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 360-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICFilter = 2;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_ConfigTI1Input(&htim3, TIM_TI1SELECTION_XORCOMBINATION) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, VLV1_Pin|SPI_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_SS_Pin|BLM_E_Pin|BLM_SI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : VLV1_Pin */
  GPIO_InitStruct.Pin = VLV1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VLV1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW0_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW0_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_MISO_Pin */
  GPIO_InitStruct.Pin = SPI_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SCK_Pin */
  GPIO_InitStruct.Pin = SPI_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLM_E_Pin BLM_SI_Pin */
  GPIO_InitStruct.Pin = BLM_E_Pin|BLM_SI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BLM_ER_Pin */
  GPIO_InitStruct.Pin = BLM_ER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLM_ER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
