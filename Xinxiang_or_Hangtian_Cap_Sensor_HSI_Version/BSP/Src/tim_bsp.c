/**@file        tim_bsp.c
* @brief        TIM驱动
* @details      TIM配置, 如定时器, PWM输出, 输入捕获等配置
* @author       杨春林
* @date         2020-08-13
* @version      V1.0.0
* @copyright    2020-2030,深圳市信为科技发展有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2020/08/13  <td>1.0.0    <td>杨春林    <td>创建初始版本
* </table>
*
**********************************************************************************
*/
#include "tim_bsp.h"


void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

Tim_DevDef tim_dev[TIM_INDEX_TOTAL] = {0};


#ifdef BSP_USING_TIM2

static TIM_HandleTypeDef htim2;
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void TIM2_PWM_Init(uint32_t prescaler, uint32_t period, uint32_t pulse)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = prescaler;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = period;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**@brief       TIM3 PWM 启动
* @param[in]    channel : 指定 TIM 通道
* @return       函数执行结果
* - HAL_OK : 成功
*/
static uint32_t TIM2_PWM_Start(uint32_t channel)
{
    return HAL_TIM_PWM_Start(&htim2, channel);
}

/**@brief       TIM2 PWM 周期和脉宽设置
* @param[in]    channel : 指定 TIM 通道
* @param[in]    period : PWM频率
* @param[in]    pulse : 占空比，以PWM频率为基准进行计算
* @return       函数执行结果
* - None
* @note         占空比计算：若 period = 5000，pulse = 4500，
* 占空比 = 4500 / 5000 * 100% = 90%
*/
static void TIM2_PWM_Set(uint32_t channel, uint32_t period, uint32_t pulse)
{
    __HAL_TIM_SET_AUTORELOAD(&htim2, (HAL_RCC_GetPCLK1Freq() / period) - 1);
    __HAL_TIM_SET_COMPARE(&htim2, channel, (HAL_RCC_GetPCLK1Freq() / period) * pulse / period);
}

/**@brief       读取 TIM2 PWM 周期和脉宽
* @param[in]    channel : 指定 TIM 通道
* @param[in]    period : PWM频率缓存
* @param[in]    pulse : 占空比缓存
* @return       函数执行结果
* - None
*/
static void TIM2_PWM_Get(uint32_t channel, uint32_t *period, uint32_t *pulse)
{
    if(period != NULL)
    {
        *period = HAL_RCC_GetPCLK1Freq() / (__HAL_TIM_GET_AUTORELOAD(&htim2) + 1);
    }
    if(period != NULL && pulse != NULL)
    {
        *pulse = (__HAL_TIM_GET_COMPARE(&htim2, channel) * (*period * *period)) / HAL_RCC_GetPCLK1Freq();
    }
}
#endif // BSP_USING_TIM2

#ifdef BSP_USING_TIM3

static TIM_HandleTypeDef htim3;

/**@brief       TIM3初始化
* @param[in]    prescaler : 预分频值; 
* @param[in]    period : 周期;
* @param[in]    pulse : 脉宽;
* @return       函数执行结果
* - None
*/
static void TIM3_PWM_Init(uint32_t prescaler, uint32_t period, uint32_t pulse)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = prescaler;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = period;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**@brief       TIM3 PWM 启动
* @param[in]    channel : 指定 TIM 通道
* @return       函数执行结果
* - HAL_OK : 成功
*/
static uint32_t TIM3_PWM_Start(uint32_t channel)
{
    return HAL_TIM_PWM_Start(&htim3, channel);
}

/**@brief       TIM3 PWM 周期和脉宽设置
* @param[in]    channel : 指定 TIM 通道
* @param[in]    period : PWM频率
* @param[in]    pulse : 占空比，以PWM频率为基准进行计算
* @return       函数执行结果
* - None
* @note         占空比计算：若 period = 5000，pulse = 4500，
* 占空比 = 4500 / 5000 * 100% = 90%
*/
static void TIM3_PWM_Set(uint32_t channel, uint32_t period, uint32_t pulse)
{
    uint64_t tim_clock, psc;

#if defined(STM32F4) || defined(STM32F7)
    if (htim3.Instance == TIM9 || htim3.Instance == TIM10 || htim3.Instance == TIM11)
#elif defined(STM32L4)
    if (htim3.Instance == TIM15 || htim3.Instance == TIM16 || htim3.Instance == TIM17)
#elif defined(STM32F1) || defined(STM32F0) || defined(STM32G0)
    if (0)
#endif
    {
#if !defined(STM32F0) && !defined(STM32G0)
        tim_clock = HAL_RCC_GetPCLK2Freq() * 2;
#endif
    }
    else
    {
#if defined(STM32L4) || defined(STM32F0) || defined(STM32G0)
        tim_clock = HAL_RCC_GetPCLK1Freq();
#else
        tim_clock = HAL_RCC_GetPCLK1Freq() * 2;
#endif
    }

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    tim_clock /= 1000000UL;
    period = (unsigned long long)period * tim_clock / 1000ULL ;
    psc = period / MAX_PERIOD + 1;
    period = period / psc;
    __HAL_TIM_SET_PRESCALER(&htim3, psc - 1);

    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1);

    pulse = (unsigned long long)pulse * tim_clock / psc / 1000ULL;
    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }
    __HAL_TIM_SET_COMPARE(&htim3, channel, pulse - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    /* Update frequency value */
    HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
}

#endif // BSP_USING_TIM3


#ifdef BSP_USING_TIM7

static TIM_HandleTypeDef htim7;

/**@brief       TIM7初始化
* @param[in]    prescaler : 预分频值; 
* @param[in]    period : 周期;
* @return       函数执行结果
* - None
*/
static void TIM7_Init(uint32_t prescaler, uint32_t period)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = prescaler;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = period;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**@brief       TIM7 启动
* @return       函数执行结果
* - HAL_OK : 成功
*/
static uint32_t TIM7_Base_Start(void)
{
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
    return HAL_TIM_Base_Start_IT(&htim7);
}

#endif // BSP_USING_TIM7
/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance==TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
    if(htim_base->Instance==TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
    if(htim_base->Instance==TIM7)
    {
        __HAL_RCC_TIM7_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration    
    PA5     ------> TIM2_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }
  else if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration    
    PA7     ------> TIM3_CH2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}
/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance==TIM2)
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
    if(htim_base->Instance==TIM3)
    {
        __HAL_RCC_TIM3_CLK_DISABLE();
    }
    else if(htim_base->Instance==TIM7)
    {
        __HAL_RCC_TIM7_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM7_IRQn);
    }
}

#ifdef BSP_USING_TIM7
/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS
    rt_interrupt_enter();
#endif // USING_RT_THREAD_OS

    HAL_TIM_IRQHandler(&htim7);

/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS
    rt_interrupt_leave();
#endif // USING_RT_THREAD_OS
}
#endif // BSP_USING_TIM7

/**@brief       TIM3设备对象初始化，将初始化、TIM操作的各个函数指针赋给tim_dev
* @return       函数执行结果
* - None
*/
void TIM_DevObj_Init(void)
{
#ifdef BSP_USING_TIM1
    tim_dev[TIM1_INDEX].tim_init        = TIM1_Init;
#endif // BSP_USING_TIM1

#ifdef BSP_USING_TIM2
    tim_dev[TIM2_INDEX].tim_pwm_init    = TIM2_PWM_Init;
    tim_dev[TIM2_INDEX].tim_pwm_start   = TIM2_PWM_Start;
    tim_dev[TIM2_INDEX].tim_set_pwm     = TIM2_PWM_Set;
    tim_dev[TIM2_INDEX].tim_get_pwm     = TIM2_PWM_Get;
#endif // BSP_USING_TIM2
    
#ifdef BSP_USING_TIM3
    tim_dev[TIM3_INDEX].tim_pwm_init    = TIM3_PWM_Init;
    tim_dev[TIM3_INDEX].tim_pwm_start   = TIM3_PWM_Start;
    tim_dev[TIM3_INDEX].tim_set_pwm     = TIM3_PWM_Set;
#endif // BSP_USING_TIM3
    
    
#ifdef BSP_USING_TIM4
    tim_dev[TIM4_INDEX].tim_init        = TIM4_Init;
#endif // BSP_USING_TIM4
        
#ifdef BSP_USING_TIM5
    tim_dev[TIM5_INDEX].tim_init        = TIM5_Init;
#endif // BSP_USING_TIM5
    
#ifdef BSP_USING_TIM6
    tim_dev[TIM6_INDEX].tim_init        = TIM6_Init;
#endif // BSP_USING_TIM6
    
#ifdef BSP_USING_TIM7
    tim_dev[TIM7_INDEX].tim_init        = TIM7_Init;
    tim_dev[TIM7_INDEX].tim_base_start  = TIM7_Base_Start;
#endif // BSP_USING_TIM7
        
#ifdef BSP_USING_TIM8
    tim_dev[TIM8_INDEX].tim_init        = TIM8_Init;
#endif // BSP_USING_TIM8

#ifdef BSP_USING_TIM9
    tim_dev[TIM9_INDEX].tim_init        = TIM9_Init;
#endif // BSP_USING_TIM9

#ifdef BSP_USING_TIM10
    tim_dev[TIM10_INDEX].tim_init        = TIM10_Init;
#endif // BSP_USING_TIM10
    
#ifdef BSP_USING_TIM11
    tim_dev[TIM11_INDEX].tim_init        = TIM11_Init;
#endif // BSP_USING_TIM11
    
    
#ifdef BSP_USING_TIM12
    tim_dev[TIM12_INDEX].tim_init        = TIM12_Init;
#endif // BSP_USING_TIM12
        
#ifdef BSP_USING_TIM13
    tim_dev[TIM13_INDEX].tim_init        = TIM13_Init;
#endif // BSP_USING_TIM13
    
#ifdef BSP_USING_TIM14
    tim_dev[TIM14_INDEX].tim_init        = TIM14_Init;
#endif // BSP_USING_TIM14
    
#ifdef BSP_USING_TIM15
    tim_dev[TIM15_INDEX].tim_init        = TIM15_Init;
#endif // BSP_USING_TIM15
        
#ifdef BSP_USING_TIM16
    tim_dev[TIM16_INDEX].tim_init        = TIM16_Init;
#endif // BSP_USING_TIM16

#ifdef BSP_USING_TIM17
    tim_dev[TIM17_INDEX].tim_init        = TIM17_Init;
#endif // BSP_USING_TIM17
}
