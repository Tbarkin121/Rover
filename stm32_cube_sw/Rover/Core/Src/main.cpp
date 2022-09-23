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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "motor.h"
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

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint32_t              TxMailbox;
uint8_t               TxData[8];
uint8_t				  RxData[8];
uint8_t 			  datacheck;
uint32_t count = 0;

Motor motor1;
Motor motor2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  datacheck = 0;
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 10;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.SlaveStartFilterBank = 0;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  TxHeader.IDE = CAN_ID_STD;			//Type of Identifier (Standard)
  TxHeader.StdId = 0x200;				//Standard ID
  TxHeader.RTR = CAN_RTR_DATA;			//
  TxHeader.DLC = 8; 					//Data Length
  TxHeader.TransmitGlobalTime = DISABLE;

  float current1 = 1.0;
  float current2 = 1.0;
  float target_vel1 = 0.0;
  float target_vel2 = 0.0;
  int16_t current_word1 = (int16_t)(current1/20.0*16384);
  int16_t current_word2 = (int16_t)(current2/20.0*16384);

  TxData[0] = current_word1 >> 8;
  TxData[1] = (int8_t)(current_word1 & 0x00ff);
  TxData[2] = current_word2 >> 8;
  TxData[3] = (int8_t)(current_word2 & 0x00ff);
  TxData[4] = current_word1 >> 8;
  TxData[5] = (int8_t)(current_word1 & 0x00ff);
  TxData[6] = current_word2 >> 8;
  TxData[7] = (int8_t)(current_word2 & 0x00ff);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t loop_count = 0;
  float dt = 0.001;

  motor1.PID_Init(0.001, 0.000001, 0.0001, 1.0);
  motor2.PID_Init(0.001, 0.000001, 0.0001, 1.0);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(datacheck)
//	  {
//		  HAL_Delay(2);
//	  }

	  if(dt*loop_count < 10)
	  {
		  float freq = 0.25;
	      target_vel1 = sinf(2.0*M_PI*loop_count*dt*freq)*4000;
		  target_vel2 = -target_vel1;
//		  current = sinf(2.0*M_PI*loop_count*dt)*1.0;
		  current1 = motor1.PID_Controller(target_vel1 - motor1.state.Vel);
		  current2 = motor2.PID_Controller(target_vel2 - motor2.state.Vel);
	  }
	  else if (dt*loop_count < 15)
	  {
		  target_vel1 = 2000;
		  target_vel2 = -target_vel1;
		  current1 = motor1.PID_Controller(target_vel1 - motor1.state.Vel);
		  current2 = motor2.PID_Controller(target_vel2 - motor2.state.Vel);
	  }
	  else
	  {
		  current1 = 0.0;
		  current2 = 0.0;
	  }
	  current_word1 = (int16_t)(current1/20.0*16384);
	  current_word2 = (int16_t)(current2/20.0*16384);
	  TxData[0] = current_word1 >> 8;
	  TxData[1] = (int8_t)(current_word1 & 0x00ff);
	  TxData[2] = current_word2 >> 8;
	  TxData[3] = (int8_t)(current_word2 & 0x00ff);
//	  TxData[4] = current_word >> 8;
//	  TxData[5] = (int8_t)(current_word & 0x00ff);
//	  TxData[6] = current_word >> 8;
//	  TxData[7] = (int8_t)(current_word & 0x00ff);

	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);


	  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  {
		 Error_Handler ();
	  }



	  /* Wait 1 second */
	  HAL_Delay(1);
	  loop_count++;
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	count++;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }
//
  if ((RxHeader.StdId == 0x201))
  {
	  motor1.state.Ang = RxData[0]<<8 | RxData[1];
	  motor1.state.Vel = RxData[2]<<8 | RxData[3];
	  motor1.state.Torq = RxData[4]<<8 | RxData[5];
	  motor1.state.Temp = RxData[6];
  }
  else if((RxHeader.StdId == 0x202))
  {
	  motor2.state.Ang = RxData[0]<<8 | RxData[1];
	  motor2.state.Vel = RxData[2]<<8 | RxData[3];
	  motor2.state.Torq = RxData[4]<<8 | RxData[5];
	  motor2.state.Temp = RxData[6];
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
