#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "spl06_001_hal.h"
void SystemClock_Config(void);

float temperature; 
float pressure;   
uint32_t baro_height; 
int _write(int fd, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_TIM6_Init();
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_Delay(2000);

    
    if(spl06_init() != 0)
       {
           printf("SPL06 init failed\r\n");
           while(1);
       }
    printf("SPL06 init ok！\r\n");

       while(1)
       {
    	   temperature=user_spl06_get_temperature();
    	   pressure=user_spl06_get_pressure();
           printf("tem：%.1f℃\r\n", temperature); 
           printf("pre：%.1fmpar\r\n", pressure); 

           printf("\r\n\r\n\r\n");
           HAL_Delay(1000) ;
           
       }
   }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef  *htim) 
{
    if (htim->Instance==TIM6) 
    {
        static uint8_t cnt_12ms = 0;

        cnt_12ms++;
        if(cnt_12ms > 4) 
        {
            cnt_12ms = 0;
            temperature = user_spl06_get_temperature();
            pressure = user_spl06_get_pressure();
            baro_height = (uint32_t)((102000.0f - pressure) * 78.740f); 
        }
    }

}
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        printf("Error Handler Triggered\n");
        HAL_Delay(1000);
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert Failed: file %s on line %d\n", file, line);
}
#endif /* USE_FULL_ASSERT */
