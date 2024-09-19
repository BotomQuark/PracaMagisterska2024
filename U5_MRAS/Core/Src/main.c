/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "icache.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "time.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
	uint8_t StartByte;
	uint8_t Data[4u];
	uint8_t StopByte;
} UART_DATA_S_T;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* UART defines */
#define UART_START_BYTE		0x02u
#define	UART_STOP_BYTE		0x03u

/* Timer defines */
#define	SAMPLE_TIMER		TIM3
#define PWM1_TIMER			TIM2
#define PWM2_TIMER			TIM4
#define PWM_MAX_FILL		159999


/* Encoder defines */
#define SAMPLE_TIME_SEC				0.002
#define IMPULSES_PER_ROTATION		150
#define DELTA_POSITION				(double)((double)360/(double)IMPULSES_PER_ROTATION)



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t s_SampleNow = 0x00u;
static int s_PosDiffImpulses = 0;
static long int s_CurrentRpm = 0;
static long int s_HighestRpm = 0;
static long int s_LowestRpm = 0;
static int s_LastControlValue = 0;
static int s_ControlValue = 0;

static long int s_HighestControlValue = 0;
static long int s_LowestControlValue = 0;

static int s_RotDir;
static int s_ZeroCounter = 0x00u;
static int s_LastSentImpulseTime = 0x00u;
static uint32_t s_LastImpTime;
static uint32_t s_CurrentImpTime;
static uint32_t s_ImpTimeDiff = 0u;

static uint32_t s_HighestImpTimeDiff;
static uint32_t s_LowestImpTimeDiff;

#ifdef DEBUG
static uint32_t s_LongestSampleTime = 0x00000000u;
static uint32_t s_ShortestSampleTime = 0xFFFFFFFFu;
static uint32_t s_LastSampleTime = 0x00u;
static uint32_t s_TimeSinceLastSample = 0x00u;
volatile static uint32_t s_TooHighRpmCnt = 0x00u;
volatile static uint32_t s_TooLowRpmCnt = 0x00u;
#endif

volatile static uint8_t s_StartMotor = 0x00u;

volatile uint32_t DebugVar = 0u;
volatile uint8_t DebugIdx = 0u;

UART_DATA_S_T UartRxData;
UART_DATA_S_T UartTxData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */

uint32_t getTimeMicrosec();

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
	uint32_t TmpReg;
	uint8_t IsDataValid = 0u;
	uint32_t TmpImpTimeDiff;
	uint32_t SampleCurrentTime;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ICACHE_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  PWM2_TIMER->CCR1 = 0;
  PWM1_TIMER->CCR1 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  s_LastSampleTime = HAL_GetTick();
  UartTxData.StartByte = 0x02u;
  UartTxData.StopByte = 0x03u;
  s_LastImpTime = getTimeMicrosec();
  s_CurrentImpTime = getTimeMicrosec();
  while (1)
  {
	//HAL_UART_Receive_IT(/*huart=*/&huart1, /*pData=*/, /*Size=*/);
	//HAL_StatusTypeDef HAL_UART_Transmit_IT(/*huart=*/, /*pData*/, /*Size=*/);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (0x00u != s_SampleNow)
	{
		s_SampleNow = 0x00u;

#ifdef DEBUG
		s_TimeSinceLastSample = HAL_GetTick() - s_LastSampleTime;
		s_LastSampleTime = HAL_GetTick();
		if (s_TimeSinceLastSample > s_LongestSampleTime)
		{
			s_LongestSampleTime = s_TimeSinceLastSample;
		}
		if (s_TimeSinceLastSample < s_ShortestSampleTime)
		{
			s_ShortestSampleTime = s_TimeSinceLastSample;
		}
#endif

		if (s_LastSentImpulseTime == s_CurrentImpTime)
		{
			/* No impulse occured since last sample time */
			s_ZeroCounter++;
		}
		else
		{
			s_LastSentImpulseTime = s_CurrentImpTime;
			s_ZeroCounter = 0u;
		}



		if (100u < s_ZeroCounter)
		{
			/* 100 times the freq time passed without impulse */
			s_ZeroCounter = 0u;
			s_RotDir = 0u;
		}


		/* Calculate momentary RPM */
		if (0 != s_RotDir)
		{
			SampleCurrentTime = getTimeMicrosec();
			if (((SampleCurrentTime - s_CurrentImpTime) > (s_CurrentImpTime - s_LastImpTime))  && ((2u < s_ZeroCounter)))
			{
				/* Speed it close to 0rpm but might not be quite 0 */
				s_ImpTimeDiff = SampleCurrentTime - s_LastImpTime;
			}

			if (0u != s_ImpTimeDiff)
			{
				s_CurrentRpm = (long int)((double)100*(double)1000000/(double)s_ImpTimeDiff); // Impulses/second
				s_CurrentRpm = (long int)((double)((double)60*(double)s_CurrentRpm)/(double)180);
			}
			else
			{
				s_CurrentRpm = 0;
			}
		}
		else
		{
			s_CurrentRpm = 0;
		}

		if(-1 == s_RotDir)
		{
			s_CurrentRpm = -1*s_CurrentRpm;
		}

#ifdef DEBUG
		if (s_CurrentRpm > s_HighestRpm)
		{
			s_HighestRpm = s_CurrentRpm;
			s_HighestImpTimeDiff = s_ImpTimeDiff;
		}
		if (s_CurrentRpm < s_LowestRpm)
		{
			s_LowestRpm = s_CurrentRpm;
			s_LowestImpTimeDiff = s_ImpTimeDiff;
		}


		if (s_CurrentRpm > 18354838)
		{
			s_TooHighRpmCnt++;
		}
		if (s_CurrentRpm < -18354838)
		{
			s_TooLowRpmCnt++;
		}

#endif

		memcpy(UartTxData.Data, &s_CurrentRpm, sizeof(s_CurrentRpm));

		/* Send Current position through UART */
		HAL_UART_Transmit(/*huart=*/&huart5,
							/*pData*/(uint8_t*)&UartTxData,
							/*Size=*/sizeof(UART_DATA_S_T),//<- 6 bytes
							/*Timeout=*/100);
		s_PosDiffImpulses = 0;

		/* Wait for response about U */
		HAL_UART_Receive_IT(/*huart=*/&huart5,
							/*pData*/(uint8_t*)&UartRxData,
							/*Size=*/sizeof(UART_DATA_S_T)); //<- 6 bytes




		/* Detect if data is valid (first 2 bits are always 0)
		 * and ignore values outside range (need software limiting at matlab)
		 */
		IsDataValid = 0u;
		if ((0x02u == UartRxData.StartByte) &&
				(0x03u == UartRxData.StopByte))
		{
			memcpy(&s_ControlValue, UartRxData.Data, sizeof(s_ControlValue));

#ifdef DEBUG
			if (s_ControlValue > s_HighestControlValue)
			{
				s_HighestControlValue = s_ControlValue;
			}
			if (s_ControlValue < s_LowestControlValue)
			{
				s_LowestControlValue = s_ControlValue;
			}
#endif

			if ((s_ControlValue <= 10000) &&
					(s_ControlValue >= -10000))
			{
				IsDataValid = 1u;
			}
		}

		if (1u == IsDataValid)
		{
			if (s_LastControlValue != s_ControlValue)
			{
				//s_ControlValue = 0.3*s_ControlValue + 0.7*s_LastControlValue;
				/* Set control */
				if (s_ControlValue > 0)
				{
					TmpReg = (uint32_t)s_ControlValue;
					TmpReg = (TmpReg * PWM_MAX_FILL) / 10000;
					PWM1_TIMER->CCR1 = TmpReg; //(PWM_MAX_FILL/4);
					PWM2_TIMER->CCR1 = 0;
				}
				else if (s_ControlValue < 0)
				{
					TmpReg = (uint32_t)(-s_ControlValue);
					TmpReg = (TmpReg * PWM_MAX_FILL) / 10000;
					PWM1_TIMER->CCR1 = 0;
					PWM2_TIMER->CCR1 = TmpReg;
				}
				else
				{
					PWM1_TIMER->CCR1 = 0;
					PWM2_TIMER->CCR1 = 0;
				}

				s_LastControlValue = s_ControlValue;
			}

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* 10ms -> sample time */
	if (SAMPLE_TIMER == htim->Instance)
	{
		s_SampleNow = 1u;
	}
	else
	{
		/* Do nothing */
	}
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState EncAState;
	GPIO_PinState EncBState;

	if (ENC_A_Pin == GPIO_Pin)
	{
		EncAState = GPIO_PIN_SET;
		EncBState = HAL_GPIO_ReadPin(/*GPIOx=*/ENC_B_GPIO_Port,
										/*GPIO_Pin=*/ENC_B_Pin);

		if (EncAState == EncBState)
		{
			s_RotDir = 1;
		}
		else
		{
			s_RotDir = -1;
		}


		s_LastImpTime = s_CurrentImpTime;
		s_CurrentImpTime = getTimeMicrosec();
		s_ImpTimeDiff = s_CurrentImpTime - s_LastImpTime;

	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState EncAState;
	GPIO_PinState EncBState;
	uint32_t TmpCurrentTime;

	if (ENC_A_Pin == GPIO_Pin)
	{
		EncAState = GPIO_PIN_RESET;
		EncBState =  HAL_GPIO_ReadPin(/*GPIOx=*/ENC_B_GPIO_Port,
										/*GPIO_Pin=*/ENC_B_Pin);

		if (EncAState == EncBState)
		{
			s_RotDir = 1;
		}
		else
		{
			s_RotDir = -1;
		}


		s_LastImpTime = s_CurrentImpTime;
		s_CurrentImpTime = getTimeMicrosec();
		s_ImpTimeDiff = s_CurrentImpTime - s_LastImpTime;
	}
	else if (USER_BUTTON_Pin == GPIO_Pin)
	{
		/* Stop motor */
		s_ControlValue = 0;
		PWM1_TIMER->CCR1 =  (5000 * PWM_MAX_FILL) / 10000;
		PWM2_TIMER->CCR1 = 0;
	}
}


uint32_t getTimeMicrosec()
{
	uint32_t ms = HAL_GetTick();
	uint32_t st = SysTick->VAL;

	// Did UptimeMillis rollover while reading SysTick->VAL?
	if (ms != HAL_GetTick())
	{
		// Rollover occurred so read both again.
		// Must read both because we don't know whether the
		// rollover occurred before or after reading SysTick->VAL.
		// No need to check for another rollover because there is
		// no chance of another rollover occurring so quickly.
		ms = HAL_GetTick();
		st = SysTick->VAL;
	}

	return ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);
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
