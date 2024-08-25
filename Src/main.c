 #include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo_control.h"
#include "servo_flash.h"
#include "servo_tools.h"
#include "servo_amplifier.h"
#include "op320.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DELAY_HELP 8
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for OP320TaskFunc */
const osThreadAttr_t OP320TaskFunc_attributes = {
  .name = "",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ReadServoResponse */
const osThreadAttr_t ReadServoResponse_attributes = {
  .name = "",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for Servo */
const osThreadAttr_t Servo_attributes = {
  .name = "",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void OP320Task(void *argument);
void ReadServoResponseTask(void *argument);
void ServoTask(void *argument);

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  readFlash( ( uint8_t *)&servoParams, sizeof( servoParams ) );

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of OP320TaskFunc */
  if( osThreadNew(OP320Task, NULL, &OP320TaskFunc_attributes) == NULL
		  || osThreadNew(ReadServoResponseTask, NULL, &ReadServoResponse_attributes) == NULL
		  ||  osThreadNew(ServoTask, NULL, &Servo_attributes) == NULL )
	  Error_Handler();
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

	initServo();
	initOP320();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_9B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_EVEN;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, out_cut_act_Pin|out_fix_act_Pin|out_help_ON_Pin
                          |out_help_reverse_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = debug_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(debug_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = in_pomp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(in_pomp_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : in_emerg_Pin in_start_Pin in_alarm_Pin in_help_sw_Pin */
  GPIO_InitStruct.Pin = in_emerg_Pin|in_start_Pin|in_help_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : out_cut_act_Pin out_fix_act_Pin out_help_ON_Pin out_help_reverse_Pin
                           out_cut_act_Pin in_pomp_Pin out_fix_up_Pin */
  GPIO_InitStruct.Pin = out_cut_act_Pin|out_fix_act_Pin|out_help_ON_Pin|out_help_reverse_Pin
                          |out_pneumatic_act_Pin|out_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_OP320Task */
/**
  * @brief  Function implementing the OP320TaskFunc thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_OP320Task */
void OP320Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	osDelay( DELAY_START_TASK );
	for(;;){
		 osDelay( DELAY_READ_BUT );
		 readButton();
		 updateData();
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ReadServoResponseTask */
/**
* @brief Function implementing the ReadServoResponseTaskFunc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadServoResponseTask */
void ReadServoResponseTask(void *argument)
{
  /* USER CODE BEGIN ReadServoResponseTask */
  /* Infinite loop */
	ServoRXMessage rxMes;
	for(;;)
	{
		osMessageQueueGet( rxMesQueue, &rxMes, NULL, osWaitForever );
		if( isResponseOk( rxMes.resp ) ){
			if( rxMes.requestType == GET_DATA ){
				switch( rxMes.dataType ){
					case ALARM_DATA:
					{
						updateAlarmStatus( rxMes.data );
						break;
					}
					case PARAM_DATA:
					{
						reg_param_val = (uint16_t) rxMes.data;
						break;
					}
					case INFO_DATA:
					{
						if( isRun() ){
							continueServo( rxMes.data );
						}
						break;
					}
					default:
					{
						  break;
					}
				}
			}
		} else if( isModeJog() ){
			updateAlarmStatus( rxMes.resp );
		}
	}

  /* USER CODE END ReadServoResponseTask */
}

/* USER CODE BEGIN Header_ServoTask */
/**
* @brief Function implementing the ServoTaskFunc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoTask */
void ServoTask(void *argument)
{
  /* USER CODE BEGIN ServoTask */
  /* Infinite loop */
	bool press_but, lock = false, activate = false;
	int c = 0, helpc = 0, cp = 0;
	osDelay( DELAY_START_TASK );
	setParam( 0, 1 );
	blockIO( BLOCK_DI_AI );
	for(;;)
	{
	osDelay( 350 );

	// ---------------- emergency
	if( !isEmergOk() ){
		updateAlarmStatus( AL_E6_EMERG );
	// ---------------- alarm
	} else if( reg_mode == MODE_ALARM ){
		if( !alarm ){
			updateAlarmStatus( AL_OK );
		} else if( isTxQueueEmpty() ){
			getAlarm();
			osDelay( 500 );
		}
	// -------------------normal mode
	} else {

		// button trigger
		press_but = isPressBut();
		if( press_but ){
			if( !lock ){
				activate = true;
				lock = true;
			}
		} else if( lock ) {
			lock = false;
		}

		// select operation
		switch( reg_mode ){
			case MODE_CUTTING:
			{
				if( activate ){
					operation();
				}
				break;
			}
			case MODE_WORK:
			{
				if( activate ){
					setHelp( false );
					MRJ2S_pause();
					osDelay( 200 );
				} else {
					setHelp( isHelpSwitch() );
					if( isTxQueueEmpty() ){
						getFeedbackImp();
						osDelay( 100 );
					}
				}
				break;
			}
			case MODE_HELP_REVERSE:
			case MODE_HELP_DIRECT:
			{
				setHelp( press_but );
				break;
			}
			case MODE_JOG_DIRECT:
			{
				if( press_but ){
					cp = 0;
					if( isTxQueueEmpty() ){
						getFeedbackImp();
						osDelay( 200 );
					}
					setHelp( isHelpSwitch() );
				} else if(cp < 1) {
					cp++;
				} else {
					MRJ2S_pause();
					osDelay( 100 );
					cp = 0;
				}
				break;
			}
			case MODE_JOG_REVERSE:
			{
				if( press_but ){
					cp = 0;
					if( isTxQueueEmpty() ){
						getFeedbackImp();
						osDelay( 100 );
						if( helpc < DELAY_HELP ){
							helpc += 1;
						} else {
							helpc = 0;
							toggleHelp();
						}
					}
				} else if( cp < 1 ) {
					cp++;
				} else {
					MRJ2S_pause();
					cp = 0;
				}
				break;
			}
			case MODE_PAUSA:
			{
				if( activate ){
					MRJ2S_continue();
				}
				break;
			}
			default: break;
		}


		//  blink
		if( reg_mode == MODE_PAUSA
				|| reg_mode == MODE_CUTTING
				|| reg_mode == MODE_HELP_DIRECT
				|| reg_mode == MODE_HELP_REVERSE ){
			 if( c < 3 ){
				c++;
			} else {
				c = 0;
				setLed( !isLed() );
			}
		}

		if( activate ){
			activate = false;
		}

	}
	}
  /* USER CODE END ServoTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	__NVIC_SystemReset();
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
