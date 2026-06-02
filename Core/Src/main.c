/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scara.h"
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  SCARA_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    SCARA_ProcessSerial();

    /* 由中断设置的 RDY 待发送标志 → 主循环安全发送 */
    if (scara.rdy_pending)
    {
        scara.rdy_pending = 0;
        SCARA_SendResponse("RDY\r\n");
    }

    /* 回零状态机：2段式光电寻零 */
    if (scara.state == ROBOT_HOMING)
    {
        uint32_t home_m1_stopped = !scara.motor1.busy;
        uint32_t home_m2_stopped = !scara.motor2.busy;

        /* 阶段0: 负方向搜索传感器触发 */
        if (scara.home_approach_phase == 0)
        {
            if (HAL_GPIO_ReadPin(AIN1_GPIO_Port, AIN1_Pin) == GPIO_PIN_SET)
            {
                motor_stop(&scara.motor1);
                scara.home_m1_done = 1;
            }
            if (HAL_GPIO_ReadPin(BIN1_GPIO_Port, BIN1_Pin) == GPIO_PIN_SET)
            {
                motor_stop(&scara.motor2);
                scara.home_m2_done = 1;
            }
            if (scara.home_m1_done && scara.home_m2_done)
            {
                scara.home_approach_phase = 1;
                SCARA_SetPosition(DEG_TO_STEPS(90), DEG_TO_STEPS(90));
                /* 后退一段距离 */
                motor_start(&scara.motor1, HOME_BACKOFF_STEPS, HOME_SPEED);
                motor_start(&scara.motor2, HOME_BACKOFF_STEPS, HOME_SPEED);
                scara.home_m1_done = 0;
                scara.home_m2_done = 0;
            }
        }
        /* 阶段1: 后退完成，慢速逼近传感器 */
        else if (scara.home_approach_phase == 1)
        {
            if (home_m1_stopped && home_m2_stopped)
            {
                scara.home_approach_phase = 2;
                motor_start(&scara.motor1, -HOME_BACKOFF_STEPS, HOME_APPROACH_SPEED);
                motor_start(&scara.motor2, -HOME_BACKOFF_STEPS, HOME_APPROACH_SPEED);
            }
        }
        /* 阶段2: 慢速触发传感器 → 回零完成 */
        else if (scara.home_approach_phase == 2)
        {
            if (HAL_GPIO_ReadPin(AIN1_GPIO_Port, AIN1_Pin) == GPIO_PIN_SET)
            {
                motor_stop(&scara.motor1);
                scara.home_m1_done = 1;
            }
            if (HAL_GPIO_ReadPin(BIN1_GPIO_Port, BIN1_Pin) == GPIO_PIN_SET)
            {
                motor_stop(&scara.motor2);
                scara.home_m2_done = 1;
            }
            if (scara.home_m1_done && scara.home_m2_done)
            {
                SCARA_SetPosition(DEG_TO_STEPS(90), DEG_TO_STEPS(90));
                scara.state = ROBOT_IDLE;
                SCARA_SendResponse("RDY HOME DONE\r\n");
            }
        }

        /* 回零超时保护 */
        if (HAL_GetTick() - scara.home_start_ms > HOME_TIMEOUT_MS)
        {
            motor_stop(&scara.motor1);
            motor_stop(&scara.motor2);
            scara.state = ROBOT_ERROR;
            SCARA_SendResponse("ER HOME TIMEOUT\r\n");
        }
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
#ifdef USE_FULL_ASSERT
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
