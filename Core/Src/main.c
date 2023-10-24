/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
const float slope1 = 0.0476;
const float slope2 = 0.0472;
const float slope3 = 1;
const float slope4 = 1;
const float slope5 = 1;
const float slope6 = 1;
const float slope7 = 1;
const float slope8 = 1;


const float offset1 = -126.46;
const float offset2 = -127.09;
const float offset3 = 0;
const float offset4 = 0;
const float offset5 = 0;
const float offset6 = 0;
const float offset7 = 0;
const float offset8 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Control Flags
uint8_t CanMsgInFlag = RESET;
uint8_t CanDataFlag1 = RESET;
uint8_t CanDataFlag2 = RESET;

uint8_t CanDataRqFlag = RESET;

uint8_t usartDataInFlag = RESET;

//CAN MSG Buffers
CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxHeaderData;

CAN_RxHeaderTypeDef RxHeader;

uint32_t TxMailbox;

uint8_t TxData[8]= {0};
uint8_t RxData[8] = {0};


uint8_t count = 0;

//USART Buffers
//Transmission Buffer
uint8_t usartBuff[64]= {'\0'};
uint8_t usartRxBuff[64]= {'\0'};
//Processing Buffers
uint16_t dataBuff1[4];
uint16_t dataBuff2[4];
uint8_t dataPrep[8];
uint8_t emptyFrame[8];


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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //start timer request interrupt
  HAL_TIM_Base_Start_IT(&htim2);

  //Start the CAN peripheral
  HAL_CAN_Start(&hcan);

  //Start notification and set to interrupt on pending message
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  //Set Arbitration and COntrol Fields of the TxField
  TxHeader.DLC = 8; 			//Specify length of data being sent (here 8 bytes)
  TxHeader.ExtId = 0; 			//Not using Extended ID
  TxHeader.IDE = CAN_ID_STD;	//Specify identifier type: std or extended (Here std)
  TxHeader.RTR = CAN_RTR_DATA; 	//Specify if sending data or remote frame (here data)
  TxHeader.StdId = 0x103; 		//ID for the CAN peripheral used for filtering (up to 11bit wide)
  TxHeader.TransmitGlobalTime = DISABLE; //Not used

  //Set Arbitration and COntrol Fields of the TxField
  TxHeaderData.DLC = 0; 			//Specify length of data being sent (here 8 bytes)
  TxHeaderData.ExtId = 0; 			//Not using Extended ID
  TxHeaderData.IDE = CAN_ID_STD;	//Specify identifier type: std or extended (Here std)
  TxHeaderData.RTR = CAN_RTR_DATA; 	//Specify if sending data or remote frame (here data)
  TxHeaderData.StdId = 0x070; 		//ID for the CAN peripheral used for filtering (up to 11bit wide)
  TxHeaderData.TransmitGlobalTime = DISABLE; //Not used

  //Data to send via CAN
  /*
  dataBuff[0] = 1;
  dataBuff[1] = 22;
  dataBuff[2] = 333;
  dataBuff[3] = 4095;

  memcpy(TxData, dataBuff, 8);
	*/
  //Start UART Interrupt
  HAL_UART_Receive_IT(&huart1, usartRxBuff, 9);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
  while (1)
  {
	//send the message (Should be received in the RxFifo0)
	if(usartDataInFlag){
		for(uint8_t i = 0; i<8; i++){
			if(usartRxBuff[i]>='1' && usartRxBuff[i] <= '9') {
				TxData[i] = (uint8_t) (usartRxBuff[i]-'0');
			}
			else {TxData[i] = 0;}
		}


		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
		usartDataInFlag = RESET;
		//clean up
		memset(usartRxBuff,'\0', 16);
		HAL_UART_Receive_IT(&huart1, usartRxBuff, 9);
	}
	if(CanMsgInFlag){
		//sprintf((char *)&usartBuff, "Toggle PIN%d %d\n", (int)dataPrep[0], (int)dataPrep[1]);
		sprintf((char * ) &usartBuff, "PS%i%i%i%i%i%i%i%i\n",
			  (int) dataPrep[0],
			  (int) dataPrep[1],
			  (int) dataPrep[2],
			  (int) dataPrep[3],
			  (int) dataPrep[4],
			  (int) dataPrep[5],
			  (int) dataPrep[6],
			  (int) dataPrep[7]
		);
		HAL_UART_Transmit(&huart1, (unsigned char *)usartBuff, strlen((char *)usartBuff), HAL_MAX_DELAY);
		memset((char *)usartBuff, '\0', strlen((char *)usartBuff));
		CanMsgInFlag = RESET;
	}
	/*sprintf((char *)&usartBuff, "%d %d %d %d %d\n", CanMsgInFlag, CanDataFlag1, CanDataFlag2, CanDataRqFlag, usartDataInFlag);
	HAL_UART_Transmit(&huart1, (unsigned char *)usartBuff, strlen((char *)usartBuff), HAL_MAX_DELAY);

	memset((char *)usartBuff, '\0', strlen((char *)usartBuff));
	*/
	if(CanDataFlag1 && CanDataFlag2){
			sprintf((char *)&usartBuff, "%4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d\n", (int)(slope1 * (float)dataBuff1[0] + offset1),(int)(slope2 * (float)dataBuff1[1]+offset2),(int)(slope3 * (float)dataBuff1[2]+offset3),(int)(slope4* (float)dataBuff1[3] + offset4), (int)(slope5 * dataBuff2[0]+ offset5),(int)(slope6 *dataBuff2[1] + offset6),(int)(slope7 * dataBuff2[2] +offset7),(int)(slope8 * dataBuff2[3]+offset8));
			HAL_UART_Transmit(&huart1, (unsigned char *)usartBuff, strlen((char *)usartBuff), HAL_MAX_DELAY);
			memset((char *)usartBuff, '\0', strlen((char *)usartBuff));
			CanDataFlag1 = RESET;
			CanDataFlag2 = RESET;

			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	if(CanDataRqFlag) {

		HAL_CAN_AddTxMessage(&hcan, &TxHeaderData, emptyFrame, &TxMailbox);

		CanDataRqFlag = RESET;


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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 20;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  //create a CAN filter config typdef struct
  CAN_FilterTypeDef canfilterconfig;

  //Set the CAN filter typedef elements
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;	//Enable or Disable Filters
  canfilterconfig.FilterBank = 10;						//Specify which Filter bank to use
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x100<<5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x100<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;				//Decides which bank belongs to slave CAN (useless as there is no CAN)

  //Call CAN Filter Configuration function
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);


  /* USER CODE END CAN_Init 2 */

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
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3600-1;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if(RxHeader.StdId == 0x132) {
		memcpy(dataPrep, RxData, 8);
		CanMsgInFlag =SET;
	}else if(RxHeader.StdId == 0x171){
		memcpy(dataBuff1, RxData, 8);
		CanDataFlag1 =SET;

	}else if(RxHeader.StdId == 0x172){
		memcpy(dataBuff2, RxData, 8);
		CanDataFlag2 =SET;

	}




}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	usartDataInFlag = SET;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	CanDataRqFlag = SET;

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