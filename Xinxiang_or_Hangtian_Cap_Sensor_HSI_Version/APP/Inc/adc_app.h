/**@file        adc_app.h
* @details      adc_app.c的头文件,定义了ADC应用的宏定义,声明了ADC应用的API函数
* @author       杨春林
* @date         2020-05-06
* @version      V1.0.0
* @copyright    2020-2030,深圳市信为科技发展有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2020/05/06  <td>1.0.0    <td>杨春林    <td>创建初始版本
* </table>
*
**********************************************************************************
*/

#ifndef __ADC_APP_H
#define __ADC_APP_H
#ifdef __cplusplus
 extern "C" {
#endif


#include "adc_bsp.h"
#include "VariaType.h"

#if defined(STM32F0)
#define VREF30ADDR          ((uint16_t*) ((uint32_t) 0x1FFFF7B8))   ///< 30摄氏度ADC参考值地址
#define VREF110ADDR         ((uint16_t*) ((uint32_t) 0x1FFFF7C2))   ///< 110摄氏度ADC参考值地址
#elif defined(STM32L0)
#define VREF30ADDR          ((uint16_t*) ((uint32_t) 0x1FF8007A))   ///< 130摄氏度ADC参考值地址
#define VREF130ADDR         ((uint16_t*) ((uint32_t) 0x1FF8007E))   ///< 130摄氏度ADC参考值地址
#define VDD_CALIB           ((uint16_t) (300))
#define VDD_APPLI           ((uint16_t) (330))
#endif // defined(STM32F0) or defined(STM32L0)

#define AD_GROUP_MAX        10                                  ///< 每个通道采集数量，用户可以自定义
#define AD_CHANNEL_MAX      AD_CHANNEL_TOTAL                    ///< 使能ADC通道数，用户可以自定义
#define AD_SEQBUFF_MAX      (AD_GROUP_MAX * AD_CHANNEL_MAX)     ///< ADC序列缓存大小

/** ADC 温度标定值 */
typedef struct {
    uint16_t TempDAMin;                                         ///< 温度DA值零点
    uint16_t TempDAMax;                                         ///< 温度DA值满量程
}ADC_Temper_CalibDef;

/** ADC温度处理需要的参数结构 */
typedef struct {
    float Temper_K1;                                            ///< 温度1修正系数K1
    float Temper_B1;                                            ///< 温度1修正系数B1
    float Temper_K2;                                            ///< 温度2修正系数K2
    float Temper_B2;                                            ///< 温度2修正系数B2
    ADC_Temper_CalibDef ADC_Temper_Calib;                       ///< ADC 温度标定值
    uint16_t TempDARange;                                       ///< 温度DA值量程
}ADC_TemperParam_TypeDef;

/** ADC温度处理输出数据的结构 */
typedef struct {
    uint16_t TemperInAirAD;                                     ///< 环境温度AD值
    uint16_t TemperInLiquidAD;                                  ///< 液体温度AD值
    float TemperInAir;                                          ///< 环境温度
    float TemperInLiquid;                                       ///< 液体温度
}ADC_TemperOut_TypeDef;

/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS
#include <rtthread.h>

#define ADC_DEVICE_NAME         "adc"

/** ADC设备对象 */
struct rt_adc_device_obj {
    struct rt_device        dev;
    ADC_TemperParam_TypeDef ADC_TemperParam;
    ADC_TemperOut_TypeDef   ADC_TemperOut;
};
#endif // USING_RT_THREAD_OS


/**@brief       初始化温度转换需要的参数结构
* @param[out]   ADC_TemperParam : 温度转换需要的参数结构指针; 
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void Sensor_ADC_TemperParam_Init(ADC_TemperParam_TypeDef *ADC_TemperParam);


/**@brief       初始化ADC，求出stm32芯片内部温度传感器的温度变化斜率，启动ADC的DMA传输
* @return       函数执行结果
* - None
*/
void Sensor_ADC_Init(void);

/**@brief       用户获取ADC通道上平均滤波后的数据
* @param[in]    Channel_Num : 通道号,指定获取的通道;
* @return       函数执行结果
* - 平均滤波后的ADC值
*/
uint16_t Sensor_ADC_GetChn_Value(AD_CHANNEL_NUM AD_Channel_Num);

/**@brief       获取ADC温度数据
* @return       函数执行结果
* - 温度值
* @note         温度计算可参考STM32F072数据手册
*/
float Sensor_ADC_Get_TemperData(void);

/**@brief       获取ADC数据更新标志
* @return       函数执行结果
* - ADC更新标志
*/
uint8_t Sensor_ADC_Get_Updata_Flag(void);
    
/**@brief       清除ADC更新标志
* @return       函数执行结果
* - None
*/
void Sensor_ADC_Clean_Updata_Flag(void);

#ifdef __cplusplus
}
#endif
#endif // __ADC_APP_H
