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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define N3_VER  "V1.001"

//40Hz = 5000, 12Hz = 16666, 10Hz = 20000, 8Hz = 25000
#define freq_40Hz  5000-1;
#define freq_12Hz  16666-1;
#define freq_10Hz  20000-1;
#define freq_8Hz   25000-1;

#define MAX_RX_BUFFER 20
#define MAX_TX_BUFFER 50
#define MainBuf_SIZE 20
//Octave 4,5  ex) 48MHz / 240(prescale) / 440(4 octave A) = 455
enum notes{
	c = 7645,	c_s = 7215,	d = 6810,	d_s = 6428,    e = 6067,    f = 5727,    f_s = 5405,	g = 5102,	g_s = 4816,	a = 4545, 	a_s = 4290,	b = 4050, //4 octave
	C = 3822,	C_s = 3608,	D = 3405,	D_s = 3214,    E = 3034,    F = 2863,    F_s = 2703,	G = 2551,	G_s = 2408,	A = 2273, 	A_s = 2145,	B = 2025  //5 octave
};

uint16_t ms_count = 0;
uint16_t s_count = 0;
uint16_t PWM_Width = 1;
uint8_t RxBuffer[MAX_RX_BUFFER];
uint8_t TxBuffer[MAX_TX_BUFFER];
uint8_t MainBuffer[MainBuf_SIZE];
uint8_t TxBuffer_end = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Tx_Ver[] = N3_VER;    //UART TX Buffer init
//Define Uart Command
char *search_COMMAND1 = "1";
char *search_COMMAND2 = "2";
char *search_COMMAND3 = "3";
char *search_COMMAND4 = "4";
char *search_COMMAND5 = "5";
char *search_COMMAND6 = "6";
char *search_COMMAND7 = "7";
char *search_STATUS   = "STATUS";
char *search_CONNECTED = "CONNECTED";
char *search_DISCONNECTED = "DISCONNECTED";
uint8_t Current_mode = 1;

uint16_t Close_Bell[] = { 0, C/2, A, F, C };
uint16_t Close_interval[] = { 250, 250, 250, 250, 250};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
_Bool status = 0; // 1?��?�� ?��?��, 0?��?�� 종료

uint8_t rx2_data;
uint8_t opcode = '0';
uint8_t Charging_Status;

void startBR(){
	if(status)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		status = 0;
	}
	else
	{
		status = 1;
		TIM3->PSC = 240-1;
		TIM3->ARR = freq_40Hz;
		TIM3->CCR1 = TIM3->ARR/2-1; //Duty 50:50
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//		TIM1->CCR1 = 0;

	}
}
void Sound_Open(){
	//	uint16_t LG_Bell[] =  { D/2, G/2, F_s/2, E/2, D/2, b/2, C/2, D/2, E/2, a/2, b/2, C/2, b/2, D/2,
	//							D/2, G/2, F_s/2, E/2, D/2, G/2, G/2, A/2, G/2, F_s/2, E/2, F_s/2, G/2 };
	//	uint16_t LG_interval[] = {	375, 166, 166, 166, 375, 375, 166, 166, 166, 166, 166, 166, 375, 375, 375, 166, 166, 166, 375, 375, 166, 166, 166, 166, 166, 166, 500};

	uint16_t Bell[] =  { D, G, B, D/2 };
	uint16_t interval[] = {	250, 250, 250, 250 };
	if (TIM3->PSC != 24-1) { TIM3->PSC = 24-1 ; } //prescaler change
	for(int i=0; i < (sizeof(Bell)/sizeof(Bell[0])); i++)
		{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		  TIM3->ARR = Bell[i]-1;
		  TIM3->CCR1 = TIM3->ARR/2-1;
//		  PWM_Width = (TIM3->ARR+1)/500;

		  for(int j=502; j>2; j--)
		  {
			  TIM3->CCR1 = (TIM3->ARR/2) - (TIM3->ARR/j);
			  delay_us(interval[i]*2);
		  }
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		}

}
void Sound_Close(){


	if (TIM3->PSC != 24-1) { TIM3->PSC = 24-1 ; } //prescaler change
	for(int i=0; i < (sizeof(Close_Bell)/sizeof(Close_Bell[0])); i++)
	{
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  TIM3->ARR = Close_Bell[i]-1;
	  if(TIM3->ARR != Close_Bell[i]-1) { TIM3->ARR = Close_Bell[i]-1;   }//frequency
//		  PWM_Width = (TIM3->ARR+1)/500;

	  for(int j=502; j>2; j--)
	  {
		  TIM3->CCR1 = (TIM3->ARR/2) - (TIM3->ARR/j); //duty ratio
		  delay_us(Close_interval[i]*2);
	  }
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1); //for delay_us
  HAL_TIM_Base_Start_IT(&htim14); //for timer interrupt
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==1) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);  } //if N3 Status is Charing, Shut down System.
  else{
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1); // Turn on Status LED
	  HAL_Delay(100);
	//  Sound_Close();
	//  HAL_Delay(1000);
	  Sound_Open();
	  HAL_Delay(1000);
	  /* UART Rx_DMA */
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuffer, MAX_RX_BUFFER); // Receive an amount of data in DMA mode till either the expected number of data is received or an IDLE event occurs.
	  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	  /* UART Rx_DMA */
	  status = 0;
	  startBR();
	  HAL_Delay(100);
	//  Sound_Close();
	//  HAL_TIM_Base_Stop(&htim1);
	//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //LDO Off

	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
  }
  while (1)
  {
	  while(Charging_Status == 0)
	  {
		  if(s_count < 1800) //1800 s = 30min
		  {

			  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuffer, MAX_RX_BUFFER);
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);


		  }
		  else if(s_count >= 1800)
		  {
			  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			  Sound_Close();
			  HAL_TIM_Base_Stop(&htim1);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //LDO Off
		  }
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
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14)
	{
		Charging_Status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
		if(Charging_Status == 1) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); HAL_TIM_Base_Stop(&htim1); HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);} //if N3 Status is Charing, Shut down System.
//		if(status == 0)
//		{
			ms_count++;
			if(ms_count >= 10) { s_count++; ms_count = 0; } //TIM14 100ms interrupt 100ms* 10= 1s
			if(s_count >= 65535) { s_count = 0; }
//	}
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART2)
	{
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		memcpy(MainBuffer, RxBuffer, sizeof(RxBuffer));
		memset(RxBuffer, 0, sizeof(RxBuffer));


		if(strstr((const char *)MainBuffer, search_CONNECTED) != NULL)// DIS"CONNECTED , CONNECTED
		{

			if(strstr((const char *)MainBuffer, search_DISCONNECTED) != NULL)
			{ 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); }//if BT disconnected, turn on power LED.
			else
			{	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); }//if BT connected, turn off power LED.}
			TxBuffer_end = strlen((const char *)MainBuffer);
				print("%s",MainBuffer);
		}
		else if(strstr((const char *)MainBuffer, search_COMMAND1) != NULL)
		{
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			TIM3->PSC = 240-1; //prescaler change		//Duty ON:OFF = 50:50
			TIM3->ARR = freq_40Hz; //
			TIM3->CCR1 = (TIM3->ARR+1)/2-1;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			Current_mode = 1;

			TxBuffer_end = strlen((const char *)MainBuffer);
			print("%s",MainBuffer);
		}
		else if(strstr((const char *)MainBuffer, search_COMMAND2) != NULL)
		{
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			TIM3->PSC = 240-1; //prescaler change		//Duty ON:OFF = 25:75
			TIM3->ARR = freq_40Hz; //40Hz
			TIM3->CCR1 = (TIM3->ARR+1)/4-1;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			Current_mode = 2;
			TxBuffer_end = strlen((const char *)MainBuffer);
			print("%s",MainBuffer);
		}
		else if(strstr((const char *)MainBuffer, search_COMMAND3) != NULL)
		{
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			TIM3->PSC = 240-1; //prescaler change		//Duty ON:OFF = 20:80
			TIM3->ARR = freq_40Hz; //40Hz
			TIM3->CCR1 = (TIM3->ARR+1)/5-1;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			Current_mode = 3;
			TxBuffer_end = strlen((const char *)MainBuffer);
			print("%s",MainBuffer);
		}
		else if(strstr((const char *)MainBuffer, search_COMMAND4) != NULL)
		{
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			TIM3->PSC = 240-1; //prescaler change		//Duty ON:OFF = 10:90
			TIM3->ARR = freq_40Hz; //40Hz
			TIM3->CCR1 = (TIM3->ARR+1)/10-1;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			Current_mode = 4;
			TxBuffer_end = strlen((const char *)MainBuffer);
			print("%s",MainBuffer);
		}
		else if(strstr((const char *)MainBuffer, search_COMMAND5) != NULL)
		{
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			TIM3->PSC = 240-1; //prescaler change		//Duty ON:OFF = 50:50
			TIM3->ARR = freq_12Hz; //12Hz
			TIM3->CCR1 = (TIM3->ARR+1)/2-1;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			Current_mode = 5;
			TxBuffer_end = strlen((const char *)MainBuffer);
			print("%s",MainBuffer);
		}
		else if(strstr((const char *)MainBuffer, search_COMMAND6) != NULL)
		{
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			TIM3->PSC = 240-1; //prescaler change		//Duty ON:OFF = 50:50
			TIM3->ARR = freq_10Hz; //10Hz
			TIM3->CCR1 = (TIM3->ARR+1)/2-1;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			Current_mode = 6;
			TxBuffer_end = strlen((const char *)MainBuffer);
			print("%s",MainBuffer);
		}
		else if(strstr((const char *)MainBuffer, search_COMMAND7) != NULL)
		{
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			TIM3->PSC = 240-1; //prescaler change		//Duty ON:OFF = 50:50
			TIM3->ARR = freq_8Hz; //8Hz
			TIM3->CCR1 = (TIM3->ARR+1)/2-1;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			Current_mode = 7;
			TxBuffer_end = strlen((const char *)MainBuffer);
			print("%s",MainBuffer);
		}
		else if(strstr((const char *)MainBuffer, search_STATUS) != NULL)
		{
			sprintf((char*) &TxBuffer, "%s\r\n",N3_VER);
//			TxBuffer_end = strlen((const char *)TxBuffer)+2;
			TxBuffer_end = sizeof(TxBuffer);
			print("%s", TxBuffer);
			sprintf((char*) &TxBuffer, "%d sec\r\n", s_count);
//			TxBuffer_end = strlen((const char *)TxBuffer)+2;
			TxBuffer_end = sizeof(TxBuffer);
			print("%s", TxBuffer);
			sprintf((char*) &TxBuffer, "mode: %d\r\n", Current_mode);
//			TxBuffer_end = strlen((const char *)TxBuffer)+2;
			TxBuffer_end = sizeof(TxBuffer);
			print("%s", TxBuffer);
		}
		else
		{
			sprintf((char*) &TxBuffer, "Unknown Command\r\n" );
//			TxBuffer_end = strlen((const char *)TxBuffer+4);
			TxBuffer_end = sizeof(TxBuffer);
			print("%s",TxBuffer);
		}

	memset(MainBuffer, 0, sizeof(MainBuffer));	//MainBuffer clear
	memset(RxBuffer, 0, sizeof(MAX_RX_BUFFER)); //RxBuffer clear

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuffer, MAX_RX_BUFFER);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
//	RxBuffer[RxIndex++] = rx2_data;
//	HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
	/*
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
			HAL_UART_Transmit(&huart2, &rx2_data, 1, 10);
//			HAL_UART_Transmit(&huart2, &opcode, 1, 10);
		}
	}
	*/
//}
void delay_us(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);              // TIM1 for delay
	while((__HAL_TIM_GET_COUNTER(&htim1))<time);   // 1us count
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//POWER SWITCH PUSHED
	if(GPIO_Pin == GPIO_PIN_0)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		Sound_Close();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //LDO off
	}

}

//for UART_TX_DMA
void print(char *fmt, ...){

	while(huart2.gState != HAL_UART_STATE_READY);
	va_list args;

	va_start(args, fmt);
	vsprintf((char *)TxBuffer, fmt, args);
	va_end(args);

	if(huart2.gState == HAL_UART_STATE_READY)
	{
		HAL_UART_Transmit(&huart2, TxBuffer, TxBuffer_end, 10);
		memset(TxBuffer, 0, sizeof(TxBuffer));	//TxBuffer clear}
		TxBuffer_end = 0;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
