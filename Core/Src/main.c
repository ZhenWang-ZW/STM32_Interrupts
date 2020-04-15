
#include "main.h"

typedef struct{
	GPIO_TypeDef* GPIO_x;
	uint16_t GPIO_Pin;
}GPIO_TYPE;

#define PA10 		(GPIO_TYPE){.GPIO_x = GPIOA, .GPIO_Pin=GPIO_PIN_10}
#define PB3 		(GPIO_TYPE){.GPIO_x = GPIOB, .GPIO_Pin=GPIO_PIN_3}
#define PB5 		(GPIO_TYPE){.GPIO_x = GPIOB, .GPIO_Pin=GPIO_PIN_5}
#define PB4 		(GPIO_TYPE){.GPIO_x = GPIOB, .GPIO_Pin=GPIO_PIN_4}
#define PB10 		(GPIO_TYPE){.GPIO_x = GPIOB, .GPIO_Pin=GPIO_PIN_10}
#define PA8 		(GPIO_TYPE){.GPIO_x = GPIOA, .GPIO_Pin=GPIO_PIN_8}
#define PC1 		(GPIO_TYPE){.GPIO_x = GPIOC, .GPIO_Pin=GPIO_PIN_1}
#define PC0 		(GPIO_TYPE){.GPIO_x = GPIOC, .GPIO_Pin=GPIO_PIN_0}

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

/* Global variables. START ------------------------------------------------- */
// LED array
const GPIO_TYPE GPIO_ARRAY[8] = {PA10, PB3, PB5, PB4, PB10, PA8, PC1, PC0};

// USART2
uint8_t msg[20]="";
char Rx_Char;

// TIM6
int reverse = 0;
int gpio_pin_index = 0;
int tim6_period = 10000; //default period of 10000 (1 sec)

// ADC1
uint32_t ADC_Status;
uint32_t ADC_Result;
float ADC_Voltage;
int previous = -1, current = 1;

// TIM1 IC
static uint32_t captures[2];
static uint32_t capture_difference, time_period;
static int edge_count;
static capture_completed = 0;
/* Global variables. END ---------------------------------------------------- */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_NVIC_Init(void);
void USART2_write(int c);
void USART_PRINT(char *msg);


void Interrupts_start(void){
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_ADC_Start_IT(&hadc1);

	__disable_irq();
	USART2->CR1 |= 0x00000020;
	NVIC_EnableIRQ(USART2_IRQn);
	__enable_irq();
}

/* TIM1 CH2 Interrupts */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if(edge_count==0){
		captures[edge_count] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		edge_count++;
	}
	else{
		captures[edge_count] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		edge_count=0;
		capture_completed=1;
	}

	if(capture_completed){
		if(captures[1]>captures[0]){
			capture_difference = captures[1] - captures[0];
		}else{
			capture_difference = ((0xffff - captures[0]) + captures[1] + 1);
		}
		capture_completed = 0;
		time_period = capture_difference/1;
	}
}

/* TIM6 Interrupts Cylon led display*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(reverse == 0){
		if(gpio_pin_index==0){
			HAL_GPIO_TogglePin(GPIO_ARRAY[gpio_pin_index].GPIO_x, GPIO_ARRAY[gpio_pin_index].GPIO_Pin);
		}else{
			HAL_GPIO_TogglePin(GPIO_ARRAY[gpio_pin_index-1].GPIO_x, GPIO_ARRAY[gpio_pin_index-1].GPIO_Pin);
			HAL_GPIO_TogglePin(GPIO_ARRAY[gpio_pin_index].GPIO_x, GPIO_ARRAY[gpio_pin_index].GPIO_Pin);
		}
		gpio_pin_index++;
		if(gpio_pin_index>7){
			reverse=1;
			gpio_pin_index--;
		}
	}else{
		HAL_GPIO_TogglePin(GPIO_ARRAY[gpio_pin_index].GPIO_x, GPIO_ARRAY[gpio_pin_index].GPIO_Pin);
		HAL_GPIO_TogglePin(GPIO_ARRAY[gpio_pin_index-1].GPIO_x, GPIO_ARRAY[gpio_pin_index-1].GPIO_Pin);
		gpio_pin_index--;
		if(gpio_pin_index==0){
			reverse=0;
			gpio_pin_index++;
		}
	}
}

/* USART2 input Interrupts */
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
	/* USER CODE BEGIN USART2_IRQn 1 */
	if (USART2->ISR & 0x0020) {
		Rx_Char = USART2->RDR;
	}
}

/* ADC interrupts. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start(&hadc1);
	ADC_Status = HAL_ADC_PollForConversion(&hadc1, 100);
	if (ADC_Status == HAL_OK)
	{
	  ADC_Result = HAL_ADC_GetValue(&hadc1);
	}
	ADC_Voltage = (float)ADC_Result  * 2.0 / 4095.0;
	if(ADC_Voltage<1){
		tim6_period = 2500;
		current = 0;
	}else{
		tim6_period = 10000;
		current = 1;
	}
	if(previous !=  current){
		MX_TIM6_Init();
		previous = current;
	}
//	HAL_Delay(50);
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* Initialize interrupts */
  MX_NVIC_Init();


  /* Interrupts start. */
  Interrupts_start();


  /* Infinite loop */
  while (1)
  {
	  switch(Rx_Char){
		case 'A':
		case 'a':
			sprintf(msg, "ADC Result = %d", ADC_Result);
			USART_PRINT(msg);
			Rx_Char='\0';
			break;
		case 'V':
		case 'v':
			sprintf(msg, "ADC Voltage = %.2f", ADC_Voltage);
			USART_PRINT(msg);
			Rx_Char='\0';
			break;
		case 'T':
		case 't':
			sprintf(msg, "ADC Voltage = %c\r\n", Rx_Char);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
			Rx_Char='\0';
			break;
		case 'F':
		case 'f':
			sprintf(msg, "ADC Voltage = %c\r\n", Rx_Char);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
			Rx_Char='\0';
			break;
		case 'C':
		case 'c':
			sprintf(msg, "ADC Voltage = %c\r\n", Rx_Char);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
			Rx_Char='\0';
			break;
		case 'E':
		case 'e':
			sprintf(msg, "ADC Voltage = %c\r\n", Rx_Char);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
			Rx_Char='\0';
			break;
		case 'W':
		case 'w':
			sprintf(msg, "ADC Voltage = %c\r\n", Rx_Char);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
			Rx_Char='\0';
			break;
		case 'Q':
		case 'q':
			sprintf(msg, "ADC Voltage = %c\r\n", Rx_Char);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
			Rx_Char='\0';
			break;
		default:
			break;
		}
//		HAL_Delay(1000);
  }

}

/* Write a character to USART2 */
void USART2_write (int ch) {
    while (!(USART2->ISR & 0x0080)) {}   // wait until Tx buffer empty
    USART2->TDR = (ch & 0xFF);
}

void USART_PRINT(char *buffer){
	for(int i=0; i<strlen(buffer);i++){
		USART2_write(buffer[i]);
	}
	USART2_write('\r');
	USART2_write('\n');
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_9;
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
//  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END ADC1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim6.Init.Prescaler = 8000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = tim6_period-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
//  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
