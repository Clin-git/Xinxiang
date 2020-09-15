/**@file        In_Memory_app.c
* @brief        读写内部存储器的应用
* @details      内部存储器中读取或写入系统内部参数
* @author       杨春林
* @date         2020-08-31
* @version      V1.0.0
* @copyright    2020-2030,深圳市信为科技发展有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author  <th>Description
* <tr><td>2020/08/31  <td>1.0.0    <td>杨春林  <td>创建初始版本
* </table>
*
**********************************************************************************
*/

#include "In_Memory_app.h"


/** 系统参数按照 [ ModBus参数 + 滤波系数 + PCap数据转换参数 + ADC温度处理参数 + 初始值标志 ]
来排列 */

const ModBusBaseParam_TypeDef   Default_ModBusBaseParam = {         //ModBus处理的基本参数结构
    65,                         ///< 设备地址
    USART_BAUDRATE_9600_CODE,   ///< 串口波特率代码
    USART_PARITY_NONE_CODE,     ///< 串口奇偶校验代码
    0,                          ///< 自动上传周期
    0,                          ///< 输出模式
    FREEZE_DISABLE,             ///< 设备冻结
    IN_MEMORY_WR_DISABLE,       ///< 内部存储器写使能
    NULL,                       ///< ModBus发送/接收处理结构体
#ifdef USING_RT_THREAD_OS
    NULL,                       ///< 串口发送锁
#else
    0,
#endif // USING_RT_THREAD_OS
    
/* 使用soway上位机升级程序(Boot程序), BOOT_PROGRAM在main.h中定义 */
#ifdef BOOT_PROGRAM
    0,                          ///< 程序擦除标志
    0,                          ///< 升级等待时间
#endif // BOOT_PROGRAM
    /** ModBus回调函数 */
    NULL
};
const uint32_t Default_FilterFactor = 4;                             ///< 滤波系数
const PCap_DataConvert_Param    Default_DataConvert_Param = {       //PCap做数据转换需要的参数结构体
    COMPENSATE_DISABLE,         ///< 补偿不使能
    1135,                       ///< 高度量程 1135 mm
    0,                          ///< 电容AD值零点
    20900,                      ///< 电容AD值下刻度
    47000,                      ///< 电容AD值上刻度
    65535,                      ///< 电容AD值满点
#ifdef USING_DAC
    428,                        ///< 电容DA值零点
    0,                          ///< 电容DA值下刻度
    0,                          ///< 电容DA值上刻度
    3824,                       ///< 电容DA值满量程
    CLIB_DISABLE,               ///< 电容DA标定不使能
#endif // USING_DAC
#ifdef USING_PWM
    5910,                       ///< 电容PWM 0 点
    5000,                       ///< 电容PWM 1 点
    4380,                       ///< 电容PWM 2 点
    3840,                       ///< 电容PWM 3 点
    3420,                       ///< 电容PWM 4 点
    2810,                       ///< 电容PWM 5 点
    0,                          ///< 电容PWM 0 点高度值
    335,                        ///< 电容PWM 1 点高度值
    535,                        ///< 电容PWM 2 点高度值
    735,                        ///< 电容PWM 3 点高度值
    935,                        ///< 电容PWM 4 点高度值
    1135,                       ///< 电容PWM 5 点高度值
    CLIB_DISABLE,               ///< 电容PWM标定使能
#endif // USING_PWM
    1655630,                    ///< 电容零点
    2426863,                    ///< 电容满量程
    1.0,                        ///< 电容修正系数K
    0.0,                        ///< 电容修正系数B
};
#ifdef USING_ADC_TEMPER_SENSOR
const ADC_TemperParam_TypeDef   Default_ADC_TemperParam = {         //ADC温度处理需要的参数结构体
    1.0,                        ///< 温度1修正系数K1
    0.0,                        ///< 温度1修正系数B1
    1.0,                        ///< 温度2修正系数K2
    0.0,                        ///< 温度2修正系数B2
    0,                          ///< 温度DA值零点
    4095,                       ///< 温度DA值满量程
    4095,                       ///< 温度DA值量程
};
#endif // USING_ADC_TEMPER_SENSOR
const uint8_t   System_Param_Flag = SYSTEMPARAM_IS_PROGRAMED;       ///< 写入初始值标志


/**@brief       内部Flash系统参数检查,若出现不一致的参数,重新将 缺省值 写入内部Flash
* @return       函数执行结果
* - None
*/
void InMemory_SystemParam_Check(void)
{
    uint16_t Cnt;
    uint8_t Check_Sta = 0;
    uint8_t Cur_Param_Bak1;

    for(Cnt = 0; Cnt < DEVICE_PARAM_LEN; Cnt++)
    {
        //备份1
        Cur_Param_Bak1 = InMemory_Read_OneByte((SYSTEM_PARAM_BAK1_BASE_ADDRESS + Cnt));
        //设备参数与备份1不同
        if(InMemory_Read_OneByte((SYSTEM_PARAM_BASE_ADDRESS + Cnt)) != Cur_Param_Bak1)
        {
            Check_Sta = 1;
        }
        //备份1有误
        if(Check_Sta)
        {
            break;
        }
    }

    if(InMemory_Read_OneByte(SYSPARAM_FLAG_BASE_ADDRESS) != SYSTEMPARAM_IS_PROGRAMED)
    {
        Check_Sta = 1;
    }
    if(Check_Sta == 0 && InMemory_Read_OneByte(SYSPARAM_FLAG_BAK1_BASE_ADDRESS) != SYSTEMPARAM_IS_PROGRAMED)
    {
        Check_Sta = 1;
    }
    //如果数据有误就写入缺省系统参数
    if(Check_Sta)
    {
        //写两份系统参数缺省值
        //写ModBus参数
        InMemory_Write2T_MultiBytes(MODBUS_PARAM_BASE_ADDRESS, 
                                    (uint8_t *)&Default_ModBusBaseParam, 
                                    MODBUS_PARAM_LEN);
        //写滤波系数
        InMemory_Write2T_MultiBytes(FILTERFACTOR_BASE_ADDRESS, 
                                    (uint8_t *)&Default_FilterFactor, 
                                    FILTERFACTOR_PARAM_LEN);
        //写PCap数据转换参数
        InMemory_Write2T_MultiBytes(PCAP_PARAM_BASE_ADDRESS, 
                                    (uint8_t *)&Default_DataConvert_Param, 
                                    PCAP_PARAM_LEN);
#ifdef USING_ADC_TEMPER_SENSOR
        //写ADC温度处理参数
        InMemory_Write2T_MultiBytes(ADC_TEMPERPARAM_BASE_ADDRESS, 
                                    (uint8_t *)&Default_ADC_TemperParam, 
                                    ADC_TEMPER_PARAM_LEN);
#endif // USING_ADC_TEMPER_SENSOR
        //写初始值标志
        InMemory_Write2T_MultiBytes(SYSPARAM_FLAG_BASE_ADDRESS, 
                                    (uint8_t *)&System_Param_Flag, 
                                    sizeof(System_Param_Flag));
    }
}


/**@brief       向内部存储器指定位置写1个字节
* @param[in]    RWAddr : 写起始地址
* @param[in]    WrData : 要写入的数据;
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         本函数通过调用内部Flash或EEPROM驱动API实现的
*/
uint8_t InMemory_Write_OneByte(uint16_t RWAddr, uint8_t WrData)
{
#if defined(__IN_EEPROM_H)
    return InEEPROM_Write_OneByte(RWAddr, WrData);
#elif defined(__IN_FLASH_H)
    return InFlash_Write_OneByte(RWAddr, WrData);
#endif // defined(__IN_EEPROM_H) or defined(__IN_FLASH_H)
}

/**@brief       向内部存储器指定位置读1个字节
* @param[in]    RWAddr : 读起始地址
* @return       函数执行结果
* - 1个字节数据
* @note         本函数通过调用内部Flash或EEPROM驱动API实现的
*/
uint8_t InMemory_Read_OneByte(uint16_t RWAddr)
{
#if defined(__IN_EEPROM_H)
    return InEEPROM_Read_OneByte(RWAddr);
#elif defined(__IN_FLASH_H)
    return InFlash_Read_OneByte(RWAddr);
#endif // defined(__IN_EEPROM_H) or defined(__IN_FLASH_H)
}
/**@brief       向内部存储器指定位置写多个字节
* @param[in]    RWAddr : 写起始地址
* @param[in]    pWrbuf : 要写入的数据缓存指针;
* @param[in]    Wrlen : 数据长度
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         本函数通过调用内部Flash或EEPROM驱动API实现的
*/
uint8_t InMemory_Write_MultiBytes(uint16_t RWAddr, uint8_t const *pWrbuf, uint16_t Wrlen)
{
#if defined(__IN_EEPROM_H)
    return InEEPROM_Write_MultiBytes(RWAddr, pWrbuf, Wrlen);
#elif defined(__IN_FLASH_H)
    return InFlash_Write_MultiBytes(RWAddr, pWrbuf, Wrlen);
#endif // defined(__IN_EEPROM_H) or defined(__IN_FLASH_H)
}

/**@brief       向内部存储器指定位置读多个字节
* @param[in]    RWAddr : 读起始地址
* @param[in]    pWrbuf : 要读取的数据缓存指针;
* @param[in]    Wrlen : 数据长度
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         本函数通过调用内部Flash或EEPROM驱动API实现的
*/
void InMemory_Read_MultiBytes(uint16_t RWAddr, uint8_t *pRdbuf, uint16_t Rdlen)
{
#if defined(__IN_EEPROM_H)
    InEEPROM_Read_MultiBytes(RWAddr, pRdbuf, Rdlen);
#elif defined(__IN_FLASH_H)
    InFlash_Read_MultiBytes(RWAddr, pRdbuf, Rdlen);
#endif // defined(__IN_EEPROM_H) or defined(__IN_FLASH_H)
}

/**@brief       向STM32F072xx内部Flash指定位置写多个字节且备份1份
* @param[in]    FlashAddr : 写起始地址
* @param[in]    pWrbuf : 要写入的数据缓存指针;
* @param[in]    Wrlen : 数据长度
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
*/
uint8_t InMemory_Write2T_MultiBytes(uint16_t FlashAddr, const uint8_t *pWrbuf, uint16_t Wrlen)
{
    //错误状态
    uint8_t Err;
    //写状态
    uint8_t Wrsta;

    Err = OP_SUCCESS;
    Wrsta = OP_SUCCESS;

    //系统参数存储区
    Wrsta = InMemory_Write_MultiBytes(FlashAddr, pWrbuf, Wrlen);
    if(OP_SUCCESS != Wrsta)
    {
        Err = OP_FAILED;
    }
    //系统参数备份区1
    Wrsta = InMemory_Write_MultiBytes(FlashAddr + SYSTEM_PARAM_BAK1_BASE_ADDRESS, pWrbuf, Wrlen);
    if(OP_SUCCESS != Wrsta)
    {
        Err = OP_FAILED;
    }

    return Err;
}

/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS
int check_device_param(void)
{
    InMemory_SystemParam_Check();           // 检查内部系统参数
    
    return RT_EOK;
}
INIT_BOARD_EXPORT(check_device_param);
#endif // USING_RT_THREAD_OS

