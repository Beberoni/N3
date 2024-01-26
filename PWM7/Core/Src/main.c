/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Octave 4,5  ex) 48MHz / 240(prescale) / 440(4 octave A) = 455
enum notes{
	c = 7645,	c_s = 7215,	d = 6810,	d_s = 6428,    e = 6067,    f = 5727,    f_s = 5405,	g = 5102,	g_s = 4816,	a = 4545, 	a_s = 4290,	b = 4050, //4 octave
	C = 3822,	C_s = 3608,	D = 3405,	D_s = 3214,    E = 3034,    F = 2863,    F_s = 2703,	G = 2551,	G_s = 2408,	A = 2273, 	A_s = 2145,	B = 2025  //5 octave
};

//uint16_t LG_Bell[] =  { D, G, F_s, E, D, b, C, D, E, a, b, C, b, D,
//						D, G, F_s, E, D, G, G, A, G, F_s, E, F_s, G };
uint16_t LG_Bell[] =  { D/2, G/2, F_s/2, E/2, D/2, b/2, C/2, D/2, E/2, a/2, b/2, C/2, b/2, D/2,
						D/2, G/2, F_s/2, E/2, D/2, G/2, G/2, A/2, G/2, F_s/2, E/2, F_s/2, G/2 };
uint16_t LG_interval[] = {	375, 166, 166, 166, 375, 375, 166, 166, 166, 166, 166, 166, 375, 375, 375, 166, 166, 166, 375, 375, 166, 166, 166, 166, 166, 166, 500};
uint16_t Bell[] = { G, A, B, G, C/2 };
uint16_t Bell2[] = { f*2, C*2, g*2, D*2, C*2 };
uint16_t Bell3[] = { e*2, a*2, d*2, g*2};
uint16_t interval[] = { 166, 166, 166, 250, 250};
uint16_t interval2[] = { 250, 250, 250, 250, 500};
uint16_t interval3[] = { 250, 250, 250, 250};
uint16_t PWM_Width = 1;

uint8_t up_pulse = 0;
uint8_t down_pulse = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
_Bool status = 0; // 1?��?�� ?��?��, 0?��?�� 종료
int timeCount = 0;
uint8_t rx2_data;
uint8_t opcode = '0';
int Charging_Status;

void startBR(){
	if(status){
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		timeCount=0;
		status = 0;
	}else{
		status = 1;
		PWM_Width = (TIM3->ARR+1)/10;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		while(status){
			if(timeCount <= 1800000){
				Charging_Status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
				if(Charging_Status==1){
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
				}
				else{

				  for(int j=0; j< 10; j++)
				  {
					  TIM3->CCR1 = PWM_Width*j;
//					  delay_us(1);
				  }
				  for(int k=10; k>0; k--)
				  {
					  TIM3->CCR1 = PWM_Width*k;
//					  delay_us(1);
				  }

//				delay_us(50000);
//				delay_us(50000);
//				HAL_Delay(99);
				timeCount += 1;
				}
			}else{
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				HAL_TIM_Base_Stop(&htim1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
				timeCount=0;
				status = 0;
				break;
			}
		}

	}
}
void Sound_Open(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	for(int i=0; i < (sizeof(LG_Bell)/sizeof(LG_Bell[0])); i++)
	{

	  TIM3->ARR = LG_Bell[i]-1;
//	  TIM3->CCR1 = TIM3->ARR/2-1;
	  PWM_Width = (TIM3->ARR+1)/500;

		  for(int j=100; j< 200; j++)
		  {
			  TIM3->CCR1 = (TIM3->ARR/4-1) + PWM_Width*j;
			  delay_us(LG_interval[i]*5);
		  }
		  for(int k=200; k>100; k--)
		  {
			  TIM3->CCR1 = (TIM3->ARR/4-1) + PWM_Width*k;
			  delay_us(LG_interval[i]*5);
		  }

//	  HAL_Delay(interval[i]);

	}
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);


	 		//40Hz = 500, 12Hz = 1666, 10Hz = 2000, 8Hz = 2500
	  TIM3->ARR = 50000-1;
//	    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)IV, 10);

//	  PWM_Width = TIM3->ARR/1000;
//	  up_pulse = 1;

}
/*
void Sound_Open(){
	  int freq = 380;
	  TIM3->ARR = freq-1;
	  TIM3->CCR1 = freq/2-1;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(250);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  freq = 300;
	  TIM3->ARR = freq-1;
	  TIM3->C1 = freq/2-1;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(250);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  freq = 250;
	  TIM3->ARR = freq-1;
	  TIM3->CCR1 = freq/2-1;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(250);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  freq = 190;
	  TIM3->ARR = freq-1;
	  TIM3->CCR1 = freq/2-1;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(250);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  freq = 5000;		//40Hz = 500, 12Hz = 1666, 10Hz = 2000, 8Hz = 2500
	  TIM3->ARR = freq-1;
	  TIM3->CCR1 = freq/2-1;
}*/

void Sound_Close(){
	  int freq = 1900;
	  TIM3->ARR = freq-1;
	  TIM3->CCR1 = freq/2-1;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(550);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  freq = 2500;
	  TIM3->ARR = freq-1;
	  TIM3->CCR1 = freq/2-1;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(250);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  freq = 3000;
	  TIM3->ARR = freq-1;
	  TIM3->CCR1 = freq/2-1;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(250);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  freq = 3800;
	  TIM3->ARR = freq-1;
	  TIM3->CCR1 = freq/2-1;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(250);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  freq = 50000;		//40Hz = 500, 12Hz = 1666, 10Hz = 2000, 8Hz = 2500
	  TIM3->ARR = freq-1;
	  TIM3->CCR1 = freq/2-1;
}
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
  HAL_Delay(300);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  int Charging;
  Charging = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
  if(Charging==1){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
  }
  else{
  HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
  HAL_Delay(100);
  Sound_Open();
  HAL_Delay(1000);
  timeCount = 0;
	status = 0;
	startBR();
  Sound_Close();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	T_count++;
//	if(T_count == 1000) {T_ms++;}
//	if(T_count == 65535) { T_count = 0; }
//
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); T_count=0; T_ms=0;
//	//	if((up_pulse == 1) && (TIM3->CCR1 < TIM3->ARR-1))
//	{
//		TIM3->CCR1 = PWM_Width * T_count;
//		T_count++;
//		if(T_count == 65535) { T_count = 0;}
//		if(TIM3->CCR1 == TIM3->ARR-1) {up_pulse = 0; down_pulse = 1;}
//	}
//	else if(down_pulse == 1)
//	{
//		TIM3->CCR1 = PWM_Width * T_count;
//		T_count--;
//		if(T_count == 0) { T_count = 0;}
//		if(TIM3->CCR1 == 0) {up_pulse = 1; down_pulse = 0;}
//	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
		int freq = 5000;
		if(rx2_data=='1')
		{
			freq = 5000;		//Duty ON:OFF = 50:50
			TIM3->ARR = freq-1;
			TIM3->CCR1 = freq/2-1;
			HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
//			HAL_UART_Transmit(&huart2, &opcode, 1, 10);
		}
		else if(rx2_data=='2')
		{
			freq = 5000;		//Duty ON:OFF = 25:75
			TIM3->ARR = freq-1;
			TIM3->CCR1 = freq/4-1;
			HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
//			HAL_UART_Transmit(&huart2, &opcode, 1, 10);
		}
		else if(rx2_data=='3')
		{
			freq = 5000;		//Duty ON:OFF = 20:80
			TIM3->ARR = freq-1;
			TIM3->CCR1 = freq/5-1;
			HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
//			HAL_UART_Transmit(&huart2, &opcode, 1, 10);
		}
		else if(rx2_data=='4')
		{
			int freq = 5000;		//Duty ON:OFF = 10:90
			TIM3->ARR = freq-1;
			TIM3->CCR1 = freq/10-1;
			HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
//			HAL_UART_Transmit(&huart2, &opcode, 1, 10);
		}
		else if(rx2_data=='5')
		{
			freq = 16666;		//40Hz = 5000, 12Hz = 16666, 10Hz = 20000, 8Hz = 25000
			TIM3->ARR = freq-1;
			TIM3->CCR1 = freq/2-1;
			HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
//			HAL_UART_Transmit(&huart2, &opcode, 1, 10);
		}
		else if(rx2_data=='6')
		{
			freq = 20000;		//40Hz = 5000, 12Hz = 16666, 10Hz = 20000, 8Hz = 25000
			TIM3->ARR = freq-1;
			TIM3->CCR1 = freq/2-1;
			HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
//			HAL_UART_Transmit(&huart2, &opcode, 1, 10);
		}
		else if(rx2_data=='7')
		{
			freq = 25000;		//40Hz = 5000, 12Hz = 16666, 10Hz = 20000, 8Hz = 25000
			TIM3->ARR = freq-1;
			TIM3->CCR1 = freq/2-1;
			HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
//			HAL_UART_Transmit(&huart2, &opcode, 1, 10);
		}
		else
		{
			int freq = 5000;		//40Hz = 5000, 12Hz = 16666, 10Hz = 20000, 8Hz = 25000
			TIM3->ARR = freq-1;
			TIM3->CCR1 = freq/2-1;
			//HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
//			HAL_UART_Transmit(&huart2, &opcode, 1, 10);
		}
	}
}
void delay_us(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);              // ???��머�?? 0?���? 초기?��
	while((__HAL_TIM_GET_COUNTER(&htim1))<time);   // ?��?��?�� ?��간까�? ??�?
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	  {
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
