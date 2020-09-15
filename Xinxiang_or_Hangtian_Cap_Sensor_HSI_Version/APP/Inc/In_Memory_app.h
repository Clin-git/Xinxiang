/**@file        In_Memory_app.h
* @details      In_Memory_app.c的头文件,声明了内部存储器应用的API函数
* @author       杨春林
* @date         2020-08-31
* @version      V1.0.0
* @copyright    2020-2030,深圳市信为科技发展有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Maintainer  <th>Description
* <tr><td>2020/08/31  <td>1.0.0    <td>杨春林      <td>创建初始版本
* </table>
*
**********************************************************************************
*/
#ifndef __IN_MEMORY_APP_H
#define __IN_MEMORY_APP_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

#if defined(STM32F0)
#include "In_Flash.h"
#elif defined(STM32L0)
#include "In_EEPROM.h"
#endif // defined(STM32F0) or defined(STM32L0)
#include "TypeConvert.h"
#include "adc_app.h"
#include "Picocap_app.h"
#include "ModBus.h"


/*************************** 系统参数在内部存储器中的所在地址 ***************************/
/** ModBus参数 */
#define MODBUS_PARAM_BASE_ADDRESS       ((uint32_t)0)                                              
/** 滤波系数 */
#define FILTERFACTOR_BASE_ADDRESS       ((uint32_t)(MODBUS_PARAM_BASE_ADDRESS + sizeof(ModBusBaseParam_TypeDef)))      
/** PCap参数，这里为了4字节对齐，所以 + sizeof(void*) */
#define PCAP_PARAM_BASE_ADDRESS         ((uint32_t)(FILTERFACTOR_BASE_ADDRESS + sizeof(void*)))      
/** ADC温度转换参数 */
#define ADC_TEMPERPARAM_BASE_ADDRESS    ((uint32_t)(PCAP_PARAM_BASE_ADDRESS + sizeof(PCap_DataConvert_Param)))      
/** 系统写入初始值标志 */
#define SYSPARAM_FLAG_BASE_ADDRESS      ((uint32_t)(ADC_TEMPERPARAM_BASE_ADDRESS + sizeof(ADC_TemperParam_TypeDef)))   

#define MODBUS_PARAM_BAK1_BASE_ADDRESS      ((uint32_t)(MODBUS_PARAM_BASE_ADDRESS + 0x800))
#define FILTERFACTOR_BAK1_BASE_ADDRESS      ((uint32_t)(FILTERFACTOR_BASE_ADDRESS + 0x800))
#define PCAP_PARAM_BAK1_BASE_ADDRESS        ((uint32_t)(PCAP_PARAM_BASE_ADDRESS + 0x800))
#define ADC_TEMPERPARAM_BAK1_BASE_ADDRESS   ((uint32_t)(ADC_TEMPERPARAM_BASE_ADDRESS + 0x800))
#define SYSPARAM_FLAG_BAK1_BASE_ADDRESS     ((uint32_t)(SYSPARAM_FLAG_BASE_ADDRESS + 0x800))

#define MODBUS_PARAM_BAK2_BASE_ADDRESS      ((uint32_t)(MODBUS_PARAM_BAK1_BASE_ADDRESS + 0x800))
#define FILTERFACTOR_BAK2_BASE_ADDRESS      ((uint32_t)(FILTERFACTOR_BAK1_BASE_ADDRESS + 0x800))
#define PCAP_PARAM_BAK2_BASE_ADDRESS        ((uint32_t)(PCAP_PARAM_BAK1_BASE_ADDRESS + 0x800))
#define ADC_TEMPERPARAM_BAK2_BASE_ADDRESS   ((uint32_t)(ADC_TEMPERPARAM_BAK1_BASE_ADDRESS + 0x800))
#define SYSPARAM_FLAG_BAK2_BASE_ADDRESS     ((uint32_t)(SYSPARAM_FLAG_BAK1_BASE_ADDRESS + 0x800))

/** 系统参数基地址（起始地址） */
#define SYSTEM_PARAM_BASE_ADDRESS           MODBUS_PARAM_BASE_ADDRESS
#define SYSTEM_PARAM_BAK1_BASE_ADDRESS      MODBUS_PARAM_BAK1_BASE_ADDRESS
#define SYSTEM_PARAM_BAK2_BASE_ADDRESS      MODBUS_PARAM_BAK2_BASE_ADDRESS

/***************************** 设备参数占用存储空间的长度 ****************************/                                                                      
#define MODBUS_PARAM_LEN                sizeof(ModBusBaseParam_TypeDef)         ///< 基本参数总长度   
#define FILTERFACTOR_PARAM_LEN          sizeof(void*)
#define PCAP_PARAM_LEN                  sizeof(PCap_DataConvert_Param)
#ifdef USING_ADC_TEMPER_SENSOR
#define ADC_TEMPER_PARAM_LEN            sizeof(ADC_TemperParam_TypeDef)
#else
#define ADC_TEMPER_PARAM_LEN            0
#endif // #ifdef USING_ADC_TEMPER_SENSOR
/** 设备参数总长度：1字节滤波系数 + PCap数据转换参数 + ADC温度处理参数 + 1字节初始值标志 */
#define DEVICE_PARAM_LEN                (FILTERFACTOR_PARAM_LEN + \
                                        PCAP_PARAM_LEN + \
                                        ADC_TEMPER_PARAM_LEN + \
                                        1) 
#define SYSTEM_PARAM_LEN                (MODBUS_PARAM_LEN + DEVICE_PARAM_LEN)   ///< 系统参数总长度   

/********************************** 内部存储器写入标志值 *********************************/   
#define SYSTEMPARAM_IS_PROGRAMED        0xAA            ///< 写入初始值标志


/********************************** 内部存储器操作宏定义 *********************************/   
#if defined(__IN_FLASH_H)
#define IN_MEMORY_WR_ENABLE             IN_FLASH_WR_ENABLE
#define IN_MEMORY_WR_DISABLE            IN_FLASH_WR_DISABLE
#define IN_MEMORY_ERR_MAX               WRITE_FLASH_ERR_MAX
#elif defined(__IN_EEPROM_H)
#define IN_MEMORY_WR_ENABLE             IN_EEPROM_WR_ENABLE
#define IN_MEMORY_WR_DISABLE            IN_EEPROM_WR_DISABLE
#define IN_MEMORY_ERR_MAX               WRITE_EEPROM_ERR_MAX
#endif // defined(__IN_FLASH_H) or defined(__IN_EEPROM_H)



/**@brief       内部Flash系统参数检查,若出现不一致的参数,重新将 缺省值 写入内部Flash
* @return       函数执行结果
* - None
*/
void InMemory_SystemParam_Check(void);

/**@brief       向内部存储器指定位置写1个字节
* @param[in]    RWAddr : 写起始地址
* @param[in]    WrData : 要写入的数据;
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         本函数通过调用内部Flash或EEPROM驱动API实现的
*/
uint8_t InMemory_Write_OneByte(uint16_t RWAddr, uint8_t WrData);

/**@brief       向内部存储器指定位置读1个字节
* @param[in]    RWAddr : 读起始地址
* @return       函数执行结果
* - 1个字节数据
* @note         本函数通过调用内部Flash或EEPROM驱动API实现的
*/
uint8_t InMemory_Read_OneByte(uint16_t RWAddr);

/**@brief       向内部存储器指定位置写多个字节
* @param[in]    RWAddr : 写起始地址
* @param[in]    pWrbuf : 要写入的数据缓存指针;
* @param[in]    Wrlen : 数据长度
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         本函数通过调用内部Flash或EEPROM驱动API实现的
*/
uint8_t InMemory_Write_MultiBytes(uint16_t RWAddr, uint8_t const *pWrbuf, uint16_t Wrlen);

/**@brief       向内部存储器指定位置读多个字节
* @param[in]    RWAddr : 读起始地址
* @param[in]    pWrbuf : 要读取的数据缓存指针;
* @param[in]    Wrlen : 数据长度
* @return       函数执行结果
* - None
* @note         本函数通过调用内部Flash或EEPROM驱动API实现的
*/
void InMemory_Read_MultiBytes(uint16_t RWAddr, uint8_t *pRdbuf, uint16_t Rdlen);

/**@brief       向STM32F072xx内部Flash指定位置写多个字节且备份1份
* @param[in]    FlashAddr : 写起始地址
* @param[in]    pWrbuf : 要写入的数据缓存指针;
* @param[in]    Wrlen : 数据长度
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
*/
uint8_t InMemory_Write2T_MultiBytes(uint16_t FlashAddr, const uint8_t *pWrbuf, uint16_t Wrlen);

#ifdef __cplusplus
}
#endif
#endif // __IN_MEMORY_APP_H
