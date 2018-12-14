
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "bq25703.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t DevID = 0;
uint8_t ManID[2] = {0};
struct _bq_cfg
{
  uint16_t low_power[2];
  uint16_t charge_current[2];
  uint16_t max_charge_voltage[2];
  uint16_t min_sys_voltage[2];
}bq_cfg;
uint8_t addr_tmp = 0;
uint8_t val_tmp = 0;
uint8_t low_power_h8 = 0x02;
uint8_t low_power_l8 = 0x0e;
uint8_t low_power_read[2] = {0};

uint8_t charge_option3_h8 = 0x40;
uint8_t charge_option3_l8 = 0x00;
uint8_t charge_option3_read[2] = {0};

uint8_t charge_cur_h8 = 0x04;
uint8_t charge_cur_l8 = 0x00;
uint8_t charge_cur_read[2] = {0};

uint8_t charge_vol_max_h8 = 0x30;
uint8_t charge_vol_max_l8 = 0x00;
uint8_t charge_vol_max_read[2] = {0};

uint8_t sys_vol_min_h8 = 0x2c;
uint8_t sys_vol_min_l8 = 0x00;
uint8_t sys_vol_min_read[2] = {0};

uint8_t adc_opt_h8 = 0xe0;
uint8_t adc_opt_l8 = 0x6f; // 0x3a
uint8_t adc_opt_read[2] = {0};

uint8_t charger_status_read[2] = {0}; //0x20

uint8_t adc[8] = {0};
float adc_real[8] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  led_on();
  HAL_Delay(600);
  bq_read(&DevID, 1, BQ25703A_DevID);
  bq_read(ManID, 1, BQ25703A_ManID);
  led_off();
  // exit low-power mode
//  struct _bq_cfg *p = &bq_cfg;
//  p->low_power[0] = ((uint8_t)0x01 << 8) | 0x00; /* address */
//  p->low_power[1] = ((uint8_t)0x02 << 8) | 0x0e;
//  p->charge_current[0] = ((uint8_t)0x03 << 8) | 0x02; /* address */
//  p->charge_current[1] = ((uint8_t)0x02 << 8) | 0x00;
//  p->max_charge_voltage[0] = ((uint8_t)0x05 << 8) | 0x04;
//  p->max_charge_voltage[1] = ((uint8_t)0x30 << 8) | 0x00;
//  p->min_sys_voltage[0] = ((uint8_t)0x0d << 8) | 0x0c;
//  p->min_sys_voltage[1] = ((uint8_t)0x32 << 8) | 0x00;
  
  // start config 
  
  bq_write(&charge_option3_l8, 1, 0x34);
  bq_write(&charge_option3_h8, 1, 0x35);
  bq_read(charge_option3_read, 2, 0x34);
  HAL_Delay(50);
  bq_write(&low_power_l8, 1, 0x00);
  bq_write(&low_power_h8, 1, 0x01);
  bq_read(low_power_read, 2, 0x00);
  
  bq_write(&charge_cur_l8, 1, 0x02);
  bq_write(&charge_cur_h8, 1, 0x03);
  bq_read(charge_cur_read, 2, 0x02);
  
  bq_write(&charge_vol_max_l8, 1, 0x04);
  bq_write(&charge_vol_max_h8, 1, 0x05);
  bq_read(charge_vol_max_read, 2, 0x04);
  
  bq_write(&sys_vol_min_l8, 1, 0x0c);
  bq_write(&sys_vol_min_h8, 1, 0x0d);
  bq_read(sys_vol_min_read, 2, 0x0c);
  
  bq_write(&adc_opt_l8, 1, 0x3a);
  bq_write(&adc_opt_h8, 1, 0x3b);
  bq_read(adc_opt_read, 2, 0x3a);
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    for(uint8_t i = 0; i < 8; i++)
      {adc[i]=0;adc_real[i]=0;}
      
    bq_read(adc, 2, 0x26);
    bq_read(adc+2, 2, 0x28);
    bq_read(adc+6, 2, 0x2c);
    
    adc_real[0] = adc[0] * 12.0 / 1000.0;             //PSYS, V
    adc_real[1] = (adc[1] * 64.0 + 3200.0) / 1000.0;  //VBUS, V
    adc_real[2] = adc[2] * 256.0 / 1000.0;            //IDCHG, A
    adc_real[3] = adc[3] * 64.0 / 1000.0;             //ICHG, A
    adc_real[6] = (adc[6] * 64.0 + 2800.0) / 1000.0;  //VBAT, V
    adc_real[7] = (adc[7] * 64.0 + 2800.0) / 1000.0;  //VSYS, V
    bq_read(charger_status_read, 2, 0x20);
    HAL_Delay(100);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
