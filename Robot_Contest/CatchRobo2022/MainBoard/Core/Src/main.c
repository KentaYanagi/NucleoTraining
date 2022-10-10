/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern void initialise_monitor_handles(void);

void Send_Data();

uint8_t UART5_Data;
int8_t lx,ly;
int sirial_step=0;
uint8_t hand_UD='N';
uint8_t hand_OC='N';
uint8_t hand_shrink='N';
uint8_t hand_interval='N';
uint8_t hand_rotate='N';
uint8_t hand_root_Rotate='N';
uint8_t shooter_start='F';


uint8_t speed_control='N';
uint8_t speed_button_off=1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART5)
	{

		switch(sirial_step){
		case 0:

			 if(HAL_UART_Receive_IT(&huart5, (uint8_t*) &UART5_Data, 1)==HAL_OK){
			 //sirial_step++;

				 if(UART5_Data=='i'){
					 //sirial_step=2;
				 }
				 else if(UART5_Data=='A'){
					 hand_rotate='L';
					 //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
				 }
				 else if(UART5_Data=='V'){
					 hand_rotate='N';
				 }
				 else if(UART5_Data=='C'){
					 hand_rotate='R';
					 //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
				 }
				 else if(UART5_Data=='e'){
					 hand_UD='U';
				 }
				 else if(UART5_Data=='F'){
				 	 hand_UD='N';
				 }
				 else if(UART5_Data=='g'){
					 hand_UD='D';
				 }
				 else if(UART5_Data=='k'){
					 hand_shrink='S';
				 }
				 else if(UART5_Data=='L'){
					 hand_shrink='N';
				 }
				 else if(UART5_Data=='m'){
					 hand_shrink='E';
				 }
				 else if(UART5_Data=='o'){
					 hand_interval='L';
				 }
				 else if(UART5_Data=='P'){
					 hand_interval='N';
				 }
				 else if(UART5_Data=='q'){
					 hand_interval='S';
				 }
				 else if(UART5_Data=='s'){
					 hand_OC='O';
				 }
				 else if(UART5_Data=='T'){
					 hand_OC='N';
				 }
				 else if(UART5_Data=='u'){
					 hand_OC='C';
				 }
				 else if(UART5_Data=='y'){
					 speed_button_off=1;
				 }
				 else if(UART5_Data=='w'){
					 if(speed_button_off==1){
						 speed_button_off=0;
						 if(speed_control=='S'){
							 speed_control='N';
						 }
						 else{
							 speed_control='S';
						 }
					 }
				 }
				 else if(UART5_Data=='a'){
					 //speed_control=1.0;
					 hand_root_Rotate='L';
				 }
				 else if(UART5_Data=='X'){
					 hand_root_Rotate='N';
				 }
				 else if(UART5_Data=='c'){
					 //speed_control=0.1;
					 hand_root_Rotate='R';
				 }
				 else if(UART5_Data=='N'){
					 shooter_start='T';
				 }


			 }

			break;

/*
		case 1:
			if(HAL_UART_Receive_IT(&huart5, (uint8_t*) &lx, 1)==HAL_OK){
				sirial_step=2;
			}
			break;

		case 2:
			lx=(int8_t)UART5_Data;
			if(HAL_UART_Receive_IT(&huart5, (uint8_t*) &ly, 1)==HAL_OK){

				Send_Data_To_Omni();
				sirial_step=0;
			}

			break;
*/

		default:
			break;

		}



	}
}

/*
float m_x,m_y,m_length,m_direction;
float pd4=atan2(-1.0,1.0);// ??��?��?/4
int8_t send_FA_data=0;
int8_t send_FB_data=0;
int8_t send_BA_data=0;
int8_t send_BB_data=0;
*/

uint32_t id;
uint32_t dlc;
int8_t data[8];

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[8];

CAN_FilterTypeDef filter;
void Filter_Init(){
	filter.FilterIdHigh         = 0;
	filter.FilterIdLow          = 0;
	filter.FilterMaskIdHigh     = 0;
	filter.FilterMaskIdLow      = 0;
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterBank           = 0;
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;
	filter.SlaveStartFilterBank = 14;
	filter.FilterActivation     = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &filter);
}


void Send_Data(){

	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)){

		  TxHeader.StdId = 0x552;
		  TxHeader.RTR = CAN_RTR_DATA;
		  TxHeader.IDE = CAN_ID_STD;
		  TxHeader.DLC = 8;
		  TxHeader.TransmitGlobalTime = DISABLE;
		  TxData[0] = hand_UD;
		  TxData[1] = hand_OC;
		  TxData[2] = hand_shrink;
		  TxData[3] = hand_interval;
		  TxData[4] = hand_rotate;
		  TxData[5] = hand_root_Rotate;
		  TxData[6] = shooter_start;
		  TxData[7] = speed_control;
		  if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)==HAL_OK){
			  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		  }
		  shooter_start='F';
	 }
}

uint8_t InputData[2];

void Send_Data2(){

	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)){

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET)
		{
			InputData[0]='F';
		}
		else{
			InputData[0]='T';
		}

		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == GPIO_PIN_RESET)
		{
			InputData[1]='F';
		}
		else{
			InputData[1]='T';
		}


		  TxHeader.StdId = 0x334;
		  TxHeader.RTR = CAN_RTR_DATA;
		  TxHeader.IDE = CAN_ID_STD;
		  TxHeader.DLC = 8;
		  TxHeader.TransmitGlobalTime = DISABLE;
		  TxData[0] = InputData[0];
		  TxData[1] = InputData[1];
		  TxData[2] = 0x33;
		  TxData[3] = 0x44;
		  TxData[4] = 0x55;
		  TxData[5] = 0x66;
		  TxData[6] = 0x77;
		  TxData[7] = 0x88;
		  if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)==HAL_OK){
			  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		  }
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart5, (uint8_t*) &UART5_Data, 1);
  HAL_CAN_Start(&hcan1);
  Filter_Init();
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  initialise_monitor_handles();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Send_Data();
	  Send_Data2();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
