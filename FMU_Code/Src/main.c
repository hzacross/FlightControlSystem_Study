/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "md_include.h"
#include "mavlink.h"
#include "example.h"
#include "sensor.h"
#include "attitude_estimator_mahony.h"
#include "position_estimator_inav.h"
#include "scheduler.h"
#include "Mag_calibration.h"
#include "rc_process.h"
#include "ANO.h"
#include "filter.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

__IO ITStatus Timer6UpdateReady = RESET;
uint16_t Main_1000HZ_Timer_Counter = 0;

/*修改处：因为编译选项中加入--gnu，而编译环境是CC_ARM，所以必须保证优先是CC_ARM选型编译*/
#if defined ( __CC_ARM )
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined (__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

uint8_t flag = 1;
uint32_t testcnt = 0;

P_LowPassFilter1p Baro_Lp;
float barolp = 0.0f;

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
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(2000);
	GPS_Init();
	MPU6000_Init();
	Baro_Init();
	HMC5883_Init();
	HAL_TIM_Base_Start_IT(&htim6);
	Mavlink_Init();
	Com_TwoBoard_Init();
	Get_Gyro_Offset();
	Filter_init();
	HAL_Delay(100);	
	
	Get_calibration_offset();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Timer6UpdateReady == SET)
		{
			Timer6UpdateReady = RESET;
			Main_1000HZ_Timer_Counter++;
			Main_1000HZ_Timer_Counter = Main_1000HZ_Timer_Counter >= 1000 ? 0 : Main_1000HZ_Timer_Counter;
			
			//1000hz任务
			
			
			if(Main_1000HZ_Timer_Counter % 2 == 0)   //500hz 任务
			{
				
				Loop_Read_MPU6000();
				IMU_Data_Combine();	
//				mavlink_test();
				
				RCDataProcess();
				SystemStateUpdate();
				Control_Scheduler();
				
//				testcnt++;
			}
			
			if(Main_1000HZ_Timer_Counter % 5 == 0)   //200hz 任务
			{
				imuCalculateEstimatedAttitude();
				Loop_Read_Bar();
				Loop_Read_Mag();
			}
			
			if(Main_1000HZ_Timer_Counter % 20 == 0)   //50hz 任务
			{
				BARO_Alt = Get_Baro_Alt(m_Ms56xx.pressure);
				barolp = LowPassFilter1p(&Baro_Lp,BARO_Alt,5,0.02);
				position_estimator_inav_update();
				MagCail_Trig();
				Mag_Calibration_update();
				FCS2Ground_Transmit();
			}
			
			if(Main_1000HZ_Timer_Counter % 50 == 0)  //20hz任务
			{
//				set_mRed_State(Red_ON_Lighting);
				LED_Handler();
				
//				if(flag == 1)
//				{
//					Save_Cali_parameter();
//					flag = 0;
//				}
				
				
			}
			
			
		}
		
		Loop_GPS_Parse();
//		Loop_Mavlink_Parse();
//		Example_Exchage_COM();
		Com_TwoBoard_RB_Update();
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
//  /* Place your implementation of fputc here ??*/
//  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&huart7, (uint8_t *)&ch, 1, 0xFFFF);
//  return ch;
	return 0;
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	Timer6UpdateReady = SET;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
if(UartHandle->Instance == USART6)
	{
	  Com_TwoBoard_TX_InterruptHandler();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  if(UartHandle->Instance == UART4)
	{
	  GPS_RX_InterruptHandler();
	}
	
	if(UartHandle->Instance == UART8)
	{
		//Mavlin_RX_InterruptHandler();
	}
	
	if(UartHandle->Instance == USART6)
	{
	  Com_TwoBoard_RX_InterruptHandler();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
