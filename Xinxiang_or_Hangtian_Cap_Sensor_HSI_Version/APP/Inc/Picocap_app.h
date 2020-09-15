/**@file        Picocap_app.h
* @details      Picocap_app.c的头文件,定义了PCap应用的宏定义,声明了PCap应用的API函数
* @author       杨春林
* @date         2020-04-30
* @version      V1.0.0
* @copyright    2020-2030,深圳市信为科技发展有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2020/04/30  <td>1.0.0    <td>杨春林    <td>创建初始版本
* </table>
*
**********************************************************************************
*/

#ifndef __PICOCAP_APP_H
#define __PICOCAP_APP_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "Picocap.h"


#define PCAP_COLLECT_CYCLE              100     ///< PCap采集周期，单位 ms

#define DATA_BUF_MAX                    10      ///< PCap接收原始数据值的缓存上限

#define DATA_2ND_FILTER_BUF_MAX         10      ///< PCap二阶滤波缓存上限

#define DATA_1ST_FILTER_BUF_MAX         96      ///< PCap一阶滤波缓存上限

#define PCAP_DAC_MIN_VALUE              0       ///< PCap DAC输出最小值

#define PCAP_DAC_MAX_VALUE              4095    ///< PCap DAC输出最大值

#define PCAP_ADC_MIN_VALUE              0       ///< PCap ADC输出最小值

#define PCAP_ADC_MAX_VALUE              65535   ///< PCap ADC输出最大值

#ifdef USING_PWM
#define PCAP_PWM_MIN_VALUE              2730    ///< PCap PWM输出最小值

#define PCAP_PWM_MAX_VALUE              5990    ///< PCap PWM输出最大值

#define PCAP_PWM_HIGH_MIN_VALUE         0       ///< PCap PWM高度最小值

#define PCAP_PWM_HIGH_MAX_VALUE         1135    ///< PCap PWM高度最大值
#endif // USING_PWM

#define TempA	(3.9083E-3)  //0.0039083
#define TempB	(-5.775E-7)  //-0.0000005775
#define R2Value	(1000)

/** 数据滤波需要的参数结构 */
typedef struct { 
    uint8_t FilterFactor;               ///< 滤波系数  
    uint8_t FilterStart;                ///< 滤波开始标志
    uint8_t InputCountMax;              ///< 累计输入的最大值，上限为DATA_BUF_MAX    
    uint8_t FilterBufMax;               ///< 滤波最大缓存
    uint16_t FilterCycle;               ///< 滤波周期
    uint32_t InputRangeMax;             ///< 输入范围最大值
    uint32_t InputRangeMin;             ///< 输入范围最小值 
}DataFilterParam;

/** PCap AD标定值 */
typedef struct {
    uint16_t CapADMin;                  ///< 电容AD值零点
    uint16_t CapADLow;                  ///< 电容AD值下刻度
    uint16_t CapADHigh;                 ///< 电容AD值上刻度
    uint16_t CapADMax;                  ///< 电容AD值满量程
}CapAD_CalibDef;

/** PCap DA标定值 */
typedef struct {
    uint16_t CapDAMin;                  ///< 电容DA值零点
    uint16_t CapDALow;                  ///< 电容DA值下刻度
    uint16_t CapDAHigh;                 ///< 电容DA值上刻度
    uint16_t CapDAMax;                  ///< 电容DA值满量程    
}CapDA_CalibDef;

/** PCap PWM标定值 */
typedef struct {
    uint16_t CapPWM0;                   ///< 电容PWM 0 点
    uint16_t CapPWM1;                   ///< 电容PWM 1 点
    uint16_t CapPWM2;                   ///< 电容PWM 2 点
    uint16_t CapPWM3;                   ///< 电容PWM 3 点
    uint16_t CapPWM4;                   ///< 电容PWM 4 点
    uint16_t CapPWM5;                   ///< 电容PWM 5 点
}CapPWM_CalibDef;

/** PCap PWM高度标定值 */
typedef struct {
    uint16_t CapPWM_High0;              ///< 电容PWM 0 点高度值
    uint16_t CapPWM_High1;              ///< 电容PWM 1 点高度值
    uint16_t CapPWM_High2;              ///< 电容PWM 2 点高度值
    uint16_t CapPWM_High3;              ///< 电容PWM 3 点高度值
    uint16_t CapPWM_High4;              ///< 电容PWM 4 点高度值
    uint16_t CapPWM_High5;              ///< 电容PWM 5 点高度值    
}CapPWM_High_CalibDef;

/** PCap 电容标定值 */
typedef struct {
    uint32_t CapMin;                    ///< 电容零点
    uint32_t CapMax;                    ///< 电容满量程
}Cap_CalibDef;
    
/** PCap做数据转换需要的参数结构 */
typedef struct {
    uint8_t CompenEn;                   ///< 补偿使能
    uint16_t HeightRange;               ///< 高度量程
    CapAD_CalibDef CapAD_Calib;         ///< PCap AD标定值
#ifdef USING_DAC
    CapDA_CalibDef CapDA_Calib;         ///< PCap DA标定值 以及 标定使能
    uint8_t CapDA_ClibEn;               ///< 电容DA标定使能
#endif // USING_DAC
#ifdef USING_PWM
    CapPWM_CalibDef CapPWM_Calib;       ///< PCap PWM标定值
    CapPWM_High_CalibDef CapPWM_High_Calib; ///< PCap PWM高度标定值
    uint8_t CapPWM_ClibEn;              ///< 电容PWM标定使能
#endif // USING_PWM
    Cap_CalibDef Cap_Calib;             ///< PCap 电容标定值
    float Correct_K;                    ///< 电容修正系数K
    float Correct_B;                    ///< 电容修正系数B
}PCap_DataConvert_Param;

/** PCap转换后输出数据的结构 */
typedef struct {
    uint16_t LiquidHeightAD;            ///< 液位高度AD值
    uint16_t LiquidHeight_Percentage;   ///< 液位高度百分比
    uint32_t LiquidHeight;              ///< 液位高度
    uint32_t PCap_ResultValue;          ///< PCap原始值
#ifdef USING_DAC
    uint16_t PCap_DA_Value;             ///< Pcap DA值
#endif // USING_DAC
#ifdef USING_PWM
    uint16_t PCap_PWM_Value;            ///< Pcap PWM值
#endif // USING_PWM
    uint16_t PCap_Temper_Value;         ///< Pcap 温度值
}PCap_DataConvert_Out_Param;

/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS
#include <rtthread.h>

#define PCAP_DEVICE_NAME        "pcap"

struct rt_pcap_device_obj {
    struct rt_device            dev;
    DataFilterParam             DataFilter;
    PCap_DataConvert_Param      PCap_DataConvert;
    PCap_DataConvert_Out_Param  PCap_DataConvert_Out;
};
#endif // USING_RT_THREAD_OS


/**@brief       切换滤波水平函数
* @param[in]    FiltFactor : 滤波系数;
* @param[in]    FilterParam : 指定滤波参数结构体;
* @return       函数执行结果
* - None
* @note         滤波水平分9级，数字从低到高对应滤波深度从低到高
*/
void SwitchCurFilter(uint8_t FiltFactor, DataFilterParam *FilterParam);


/**@brief       初始化数据滤波需要的参数结构
* @param[in]    Filterfactor_CountMax : 滤波系数和累计输入的最大值,滤波系数在高8位,
* 累计输入的最大值在低8位;
* @param[out]   FilterParam : 数据滤波需要的参数结构指针;
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void DataFilterParam_Init(DataFilterParam *FilterParam, uint16_t Filterfactor_CountMax);


/**@brief       初始化数据转换需要的参数结构
* @param[out]   DataConvert_Param : 数据转换需要的参数结构指针;
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void DataConvertParam_Init(PCap_DataConvert_Param *DataConvert_Param);


/**@brief       获取PCap原始采集值
* @param[in]    reg_addr : 结果寄存器的地址;
* @param[out]   PCap_Result : 保存PCap的输出结果;
* @param[in]    Read_Cnt : 读取的电容个数;
* @return       函数执行结果
* - OP_SUCCESS(操作成功)
* - OP_FAILED(操作失败)
*/
uint8_t Sensor_PCap_GetResult(uint8_t reg_addr, uint32_t *PCap_Result, uint8_t Read_Cnt);

/**@brief       数据按指定滤波参数进行滤波
* @param[in]    FilterParam : 指定滤波参数结构体;
* @param[in]    InputValue : 输入的数据;
* @param[out]   OutputValue : 输出数据的指针;
* @return       函数执行结果
* - OP_SUCCESS(操作成功)
* - OP_FAILED(操作失败)
* @note         使用本函数前,先将FilterParam初始化(即对它的每一个成员赋合适的值),否则不能正常使用
*/
uint8_t Sensor_DataFilter(DataFilterParam *FilterParam, uint32_t InputValue, uint32_t *OutputValue);

/**@brief       将电容值转换成液位高度和AD值
* @param[in]    DataConvert_Param : 指定控制参数结构体;
* @param[in]    Cap_Value : 输入的数据(电容值);
* @param[out]   DataConvert_Out : 输出数据的参数结构体指针
* @return       函数执行结果
* - None
* @note         使用本函数前,先将DataConvert_Param初始化(即对它的每一个成员赋合适的值),否则不能正常使用
*/
void Sensor_PCap_DataConvert(PCap_DataConvert_Param *DataConvert_Param, 
                                uint32_t Cap_Value, 
                                PCap_DataConvert_Out_Param *DataConvert_Out);

#ifdef USING_DAC
/**@brief       将输入的AD值转换成DA值
* @param[in]    DataConvert_Param : 指定控制参数结构体;
* @param[in]    Cap_AD_Value : 输入的数据(AD值);
* @return       函数执行结果
* - DA值
* @note         使用本函数前,先将DataConvert_Param初始化(即对它的每一个成员赋合适的值),否则不能正常使用
*/
uint16_t Sensor_PCap_DA_Convert(PCap_DataConvert_Param *DataConvert_Param, 
                                uint32_t Cap_AD_Value);
#endif // USING_DAC

#ifdef USING_PWM
/**@brief       数据按指定控制参数进行数据转换
* @param[in]    DataConvert_Param : 指定控制参数结构体;
* @param[in]    LiquidHeight : 输入的数据(液位高度值);
* @return       函数执行结果
* - PWM值
* @note         使用本函数前,先将DataConvert_Param初始化(即对它的每一个成员赋合适的值),否则不能正常使用
*/
uint16_t Sensor_PCap_PWM_Convert(PCap_DataConvert_Param *DataConvert_Param, 
                                uint32_t LiquidHeight);
#endif // USING_PWM

/**@brief       获取从Pcap输出的热敏电阻与参考电阻的比值转换后的温度值
* @param[in]    R_ratio : 热敏电阻与参考电阻的比值
* @return       函数执行结果
* - 温度值; 
*/
float PCap_GetTemp(float R_ratio);

#ifdef __cplusplus
}
#endif
#endif // __PICOCAP_APP_H
