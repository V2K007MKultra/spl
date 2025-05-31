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

float temperature; // 读取的温度值 单位℃摄氏度
float pressure;   // 温度补偿后的气压值 单位mpar 毫帕
uint32_t baro_height; // 解算后的气压高度值，单位mm毫米
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

    // 初始化SPL06-001
    if(spl06_init() != 0)
       {
           printf("SPL06初始化失败！\r\n");
           while(1);
       }
    printf("SPL06初始化！\r\n");

       while(1)
       {
    	   temperature=user_spl06_get_temperature();
    	   pressure=user_spl06_get_pressure();
           printf("气压计内部温度为：%.1f℃\r\n", temperature); // 发送海拔高度数据到串口助手
           printf("温度补偿后的气压值为：%.1fmpar\r\n", pressure); // 发送海拔高度数据到串口助手

           printf("\r\n\r\n\r\n");
           HAL_Delay(1000) ;
           // 高度读取程序都在3ms定时器中断里面运行
       }
   }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef  *htim) // TIM6中断
{
    if (htim->Instance==TIM6) // 检查指定的TIM中断发生与否: TIM 中断源
    {
        static uint8_t cnt_12ms = 0;

        cnt_12ms++;
        if(cnt_12ms > 4) // 3ms中断*4 = 12ms每次读取
        {
            cnt_12ms = 0;
            temperature = user_spl06_get_temperature();
            pressure = user_spl06_get_pressure();
            baro_height = (uint32_t)((102000.0f - pressure) * 78.740f); // 每1mpar平均海拔高度为78.740mm
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
