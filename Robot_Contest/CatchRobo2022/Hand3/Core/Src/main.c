/* USER CODE BEGIN Header */
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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	HAL_CAN_ConfigFilter(&hcan, &filter);
}


uint32_t id;
uint32_t dlc;
uint8_t data[8];
uint8_t switch_data[2]={'T','T'};

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[8];

int wait_time=0;
int roop_time=0;
int step=0;
int close_step_left=0;
int devide_open_count=-1;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
    	if(RxHeader.StdId==0x552){
			id = (RxHeader.IDE == CAN_ID_STD)? RxHeader.StdId : RxHeader.ExtId;
			dlc = RxHeader.DLC;

/*
			if(RxData[1]=='C'){
				close_step_left=100;
			}
			else if(RxData[1]=='O'){
				close_step_left=0;
			}

			if(RxData[6]=='T' && switch_data[0]=='T' && switch_data[1]=='T' && step==0){
				wait_time=3500;
				step=1;
			}

			if(RxData[6]=='T' && switch_data[0]=='T' && switch_data[1]=='T' && step==3){
				wait_time=3500;
				step=4;
			}
*/

			if(RxData[6]=='T'  && step==0){
				wait_time=6000;
				step=1;
			}

			data[0] = RxData[0];
			data[1] = RxData[1];
			data[2] = RxData[2];
			data[3] = RxData[3];
			data[4] = RxData[4];
			data[5] = RxData[5];
			data[6] = RxData[6];
			data[7] = RxData[7];
    	}
    }


    /*
	if(RxHeader.StdId==0x334){
		id = (RxHeader.IDE == CAN_ID_STD)? RxHeader.StdId : RxHeader.ExtId;     // ID
		dlc = RxHeader.DLC;                                                     // DLC
		if(RxData[0]=='T'){
			switch_data[0]=RxData[0];
		}
		if(RxData[1]=='T'){
			switch_data[1]=RxData[1];
		}

	}
	*/


}

#define DEVIDE_OPEN_SPEED 190
#define INIT_OPEN_COUNT 2200

int stepping_output[4][2]={{0,0},{0,1},{1,1},{1,0}};
int stepping_step=0;
int open_count=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2){


		if(devide_open_count>0){
			devide_open_count--;
			if(devide_open_count == 0){
				if(step==2){
					step=0;
				}
			}
		}

    	if(1400 < devide_open_count && devide_open_count <=2200){
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,DEVIDE_OPEN_SPEED);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    	}
    	else if( 0 <= devide_open_count && devide_open_count <800){
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,DEVIDE_OPEN_SPEED);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    	}
    	else if(step ==1){
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,110);
    	}
    	else{
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,0);
    	}

    	if(wait_time>0){
    		wait_time--;
    		if(wait_time==0){
    			if(step==1){
					step=2;
					devide_open_count=INIT_OPEN_COUNT;
    			}

    		}
    	}


    	/*
    	if(3800 < devide_open_count && devide_open_count <=4300){
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,DEVIDE_OPEN_SPEED);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    	}
    	else if( (4300 <= devide_open_count && devide_open_count <5300) || (open_count>0) ){
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,DEVIDE_OPEN_SPEED);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			if(open_count >0){
				open_count--;
			}
    	}
    	else if(step ==1 || step==2 || step==4 || step==5){
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,110);

    	}
    	else{
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,0);
    	}

    	if(wait_time>0){
    		wait_time--;
    		if(wait_time==0){
    			if(step==1){
					roop_time=2;
					step=2;
					devide_open_count=INIT_OPEN_COUNT;
    			}
    			else if(step==4){
					step=5;
					devide_open_count=INIT_OPEN_COUNT;
    			}

    		}
    	}


    	if(data[1]=='O'){
    		stepping_step++;
    		stepping_step%=4;
    	}
    	else if(data[1]=='C'){
    		stepping_step--;
    		if(stepping_step<0){
    			stepping_step+=4;
    		}
    	}

    	if(stepping_output[stepping_step%4][0]==0){
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_SET);
    	}
    	else {
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,GPIO_PIN_SET);
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_RESET);
    	}
    	if(stepping_output[stepping_step%4][1]==0){
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
    	}
    	else {
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_SET);
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
    	}

		if(devide_open_count>0){
			devide_open_count--;
			if(devide_open_count == 0){
				if(roop_time>0){
					roop_time--;
					devide_open_count=INIT_OPEN_COUNT;
				}
				else{
					if(step==2){
						step++;
					}
					else if(step==5){
						step++;
						open_count=300;
					}
				}
			}
		}

    	if(3800 < devide_open_count && devide_open_count <=4300){
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,DEVIDE_OPEN_SPEED);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    	}
    	else if( (4300 <= devide_open_count && devide_open_count <5300) || (open_count>0) ){
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,DEVIDE_OPEN_SPEED);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			if(open_count >0){
				open_count--;
			}
    	}
    	else if(step ==1 || step==2 || step==4 || step==5){
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,110);

    	}
    	else{
			__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,0);
    	}

    	if(wait_time>0){
    		wait_time--;
    		if(wait_time==0){
    			if(step==1){
					roop_time=2;
					step=2;
					devide_open_count=INIT_OPEN_COUNT;
    			}
    			else if(step==4){
					step=5;
					devide_open_count=INIT_OPEN_COUNT;
    			}

    		}
    	}

    	*/

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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	if(HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1)!=HAL_OK){
	  Error_Handler();
	}

HAL_CAN_Start(&hcan);
Filter_Init();
HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_TIM_Base_Start_IT(&htim2);

	HAL_CAN_Start(&hcan);
	Filter_Init();
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
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
  htim2.Init.Prescaler = 6400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20;
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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 31;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
