#include "main.h"
#include "Picocap_app.h"
#include "adc_app.h"
#include "dac_bsp.h"
#include "iwdg_bsp.h"
#include "ModBus.h"
#include "In_Memory_app.h"
#include "tim_bsp.h"

#define APPLICATION_ADDRESS     (uint32_t)0x08003000

#define VECTOR_TABLE_SIZE       48 * 4

#if   (defined ( __CC_ARM ))
__IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
__no_init __IO uint32_t VectorTable[48];
#elif defined   (  __GNUC__  )
__IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#endif // (defined ( __CC_ARM ))



/* 系统时钟配置 */
void SystemClock_Config(void);


int main(void)
{
    /* F0系列没有中断向量表重映射寄存器，所以，这里使用另一种方法：拷贝中断向量表到
    0x20000000开始的内存空间中，再将这个内存地址到0x00000000，完成中断向量表的重映射 */
    memcpy((void*)VectorTable, (void*)APPLICATION_ADDRESS, VECTOR_TABLE_SIZE); 
    /* 将系统内存映射到0x00000000 */
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
    
    uint32_t                            FilterResult;               //滤波后的结果
    uint32_t                            PCap_PWM_Period_Old;        //上一次PWM周期的值
    static DataFilterParam              FilterParam;                //数据滤波需要的参数结构体
    static PCap_DataConvert_Param       DataConvert_Param;          //PCap做数据转换需要的参数结构体
    static PCap_DataConvert_Out_Param   DataConvert_Out;            //PCap转换后输出数据的结构体
#ifdef USING_ADC_TEMPER_SENSOR    
    static ADC_TemperParam_TypeDef      ADC_TemperParam;            //ADC温度处理需要的参数结构体
    static ADC_TemperOut_TypeDef        ADC_TemperOut;              //ADC温度处理输出数据的结构体
#endif // USING_ADC_TEMPER_SENSOR 
    static ModBusBaseParam_TypeDef      ModBusBaseParam;            //ModBus处理的基本参数结构
    static ModBus_Device_Param          ModBus_Device;              //ModBus管理设备的结构体
    uint32_t                    PCap_Tick_Start  = HAL_GetTick();   //计算超时的起始值，用于Pcap定时采集
    __IO uint32_t               Auto_Up_Tick_Start = HAL_GetTick(); //计算超时的起始值，用于ModBus定时自动上传
    uint32_t                            temp_buf;
    float                               temp_res_value;
    
    /* 为 ModBus 添加 ADC 和 Pcap 设备对象 */
#ifdef USING_ADC_TEMPER_SENSOR 
    ModBus_Device.ADC_TemperParam         = &ADC_TemperParam;
    ModBus_Device.ADC_TemperOut           = &ADC_TemperOut;
#endif // USING_ADC_TEMPER_SENSOR 
    ModBus_Device.DataFilter              = &FilterParam;
    ModBus_Device.PCap_DataConvert        = &DataConvert_Param; 
    ModBus_Device.PCap_DataConvert_Out    = &DataConvert_Out;            
    
/**************************************** 初始化HAL库和系统时钟 ****************************************/
    HAL_Init();                                         //HAL库初始化
    SystemClock_Config();                               //系统时钟配置
/*******************************************************************************************************/


/******************************************* 初始化外设模块 ********************************************/
    InMemory_SystemParam_Check();                       //检查内部设备数据    
    DataFilterParam_Init(&FilterParam, DATA_BUF_MAX);   //滤波参数初始化
    DataConvertParam_Init(&DataConvert_Param);          //PCap数据转换参数初始化
#ifdef USING_ADC_TEMPER_SENSOR 
    Sensor_ADC_TemperParam_Init(&ADC_TemperParam);      //ADC温度处理参数初始化
#endif // USING_ADC_TEMPER_SENSOR 
    PCap_Init();                                        //PCap初始化    
#ifdef USING_ADC_TEMPER_SENSOR 
    Sensor_ADC_Init();                                  //ADC初始化
#endif // USING_ADC_TEMPER_SENSOR
#ifdef USING_DAC
    BSP_DAC_Init();                                     //DAC初始化
#endif // USING_DAC
#ifdef USING_PWM
    TIM_DevObj_Init();                                  //定时器设备对象初始化
    tim_dev[TIM3_INDEX].tim_pwm_init(0, 8121, 4061);    //定时器3初始化
    tim_dev[TIM3_INDEX].tim_pwm_start(TIM_CHANNEL_2);   //定时器3 PWM 启动
    tim_dev[TIM3_INDEX].tim_set_pwm(TIM_CHANNEL_2, 
                                    DataConvert_Out.PCap_PWM_Value, 
                                    DataConvert_Out.PCap_PWM_Value / 2);
#endif // USING_PWM
    BSP_IWDG_Init();                                    //独立看门狗初始化
    ModBus_Init(&ModBusBaseParam);                      //ModBus初始化(包括串口初始化)
/*******************************************************************************************************/
    
    while (1)
    {
/********************************************* 看门狗喂狗 **********************************************/
        BSP_IWDG_Refresh();
/*******************************************************************************************************/
        
        
/********************************************* ModBus处理 **********************************************/
        //串口数据更新了
        if(Sensor_USART_Get_RX_Updata_Flag())       
        {
            //ModBus消息帧处理
            ModbusHandle(&ModBusBaseParam, &ModBus_Device);
            //清除串口数据更新标志
            Sensor_USART_Clear_RX_Updata_Flag();    
            //处理了 ModBus 消息后，自动上传定时从当前时间开始重新计时
            Auto_Up_Tick_Start = HAL_GetTick();       
        }                
        //如果自动上传时间不为 0 且自动上传定时时间到
        else if((ModBusBaseParam.AutoUpload != 0)   
            && ((ModBusBaseParam.AutoUpload * AUTOUPLOAD_CYCLE) <= (HAL_GetTick() - Auto_Up_Tick_Start)))
        {
            //Modbus帧自动上传
            ModbusAutoUpload(&ModBusBaseParam, &ModBus_Device);
            //记录当前时间为下一次自动上传重新计时
            Auto_Up_Tick_Start = HAL_GetTick();       
        }
/*******************************************************************************************************/
        
        
/*********************************** Pcap采集、滤波、数据转换、输出 ************************************/
        //定时时间到采集电容值
        if(HAL_GetTick() - PCap_Tick_Start > PCAP_COLLECT_CYCLE)    
        {
            //读取PCap电容数据并判断返回状态,成功状态则进行滤波和数据转换
            if(Sensor_PCap_GetResult(RESULT_REG1_ADDR, 
                                    &ModBus_Device.PCap_DataConvert_Out->PCap_ResultValue, 
                                    1) == OP_SUCCESS)
            {
                //数据滤波并判断是否成功
                if(Sensor_DataFilter(&FilterParam, 
                                    ModBus_Device.PCap_DataConvert_Out->PCap_ResultValue, 
                                    &FilterResult) == OP_SUCCESS)
                {
                    //数值转换
                    Sensor_PCap_DataConvert(&DataConvert_Param, 
                                            FilterResult, 
                                            &DataConvert_Out);
                    
/*********************************************** DAC处理 ***********************************************/
#ifdef USING_DAC                                       
                    if(DataConvert_Param.CapDA_ClibEn != CLIB_ENABLE)
                    {
                        DataConvert_Out.PCap_DA_Value = Sensor_PCap_DA_Convert(&DataConvert_Param, 
                                                                            DataConvert_Out.LiquidHeightAD);  
                    }                        
                    //如果未使能DA标定 且 DA值更新了才做DA转换    
                    if(DataConvert_Out.PCap_DA_Value != BSP_Get_DAC(DA_CHANNEL_2))
                    {                                                                                          
                        //DA转换,使用通道2
                        BSP_DAC_Convert(DataConvert_Out.PCap_DA_Value, DA_CHANNEL_2); 
                    }                  
#endif // USING_DAC
/*******************************************************************************************************/
                    
/*********************************************** PWM处理 ***********************************************/
#ifdef USING_PWM
                    if(DataConvert_Param.CapPWM_ClibEn != CLIB_ENABLE)
                    {
                        DataConvert_Out.PCap_PWM_Value = Sensor_PCap_PWM_Convert(&DataConvert_Param, 
                                                                                DataConvert_Out.LiquidHeight);
                    }                    
                    //如果未使能PWM标定 且 PWM值更新了才做PWM转换    
                    if(DataConvert_Out.PCap_PWM_Value != PCap_PWM_Period_Old)
                    { 
                        tim_dev[TIM3_INDEX].tim_set_pwm(TIM_CHANNEL_2, 
                                                        1000000000 / DataConvert_Out.PCap_PWM_Value, 
                                                        1000000000 / DataConvert_Out.PCap_PWM_Value / 2);
                        PCap_PWM_Period_Old = DataConvert_Out.PCap_PWM_Value;
                    }
#endif // USING_PWM
/*******************************************************************************************************/                    
                }
            }
/******************************************* PCap PT1000处理 *******************************************/
            //读取PCap PT1000数据并判断返回状态,成功状态则进行数值转换
            if(Sensor_PCap_GetResult(RESULT_REG10_ADDR, 
                                    &temp_buf, 
                                    1) == OP_SUCCESS)
            {            
                temp_res_value = PCap_GetTemp((float)temp_buf / 2097152.0);
                DataConvert_Out.PCap_Temper_Value = (temp_res_value + 273.1 + 0.05) * 10;
            }
            //记录当前时间为下一次采集电容值重新计时
            PCap_Tick_Start = HAL_GetTick();      
        }
/*******************************************************************************************************/    
        
/*******************************************************************************************************/
        
        
/*********************************************** ADC处理 ***********************************************/
#ifdef USING_ADC_TEMPER_SENSOR
        //判断ADC是否被更新
        if(Sensor_ADC_Get_Updata_Flag() == UPDATA_OK)
        {
            //获取AD转换出的温度值
            ADC_TemperOut.TemperInAir = (Sensor_ADC_Get_TemperData() + 273.1 + 0.5) * 10;
            //清除ADC更新标志,并打开ADC转换
            Sensor_ADC_Clean_Updata_Flag();
        }          
#endif // USING_ADC_TEMPER_SENSOR
/*******************************************************************************************************/        
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

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

