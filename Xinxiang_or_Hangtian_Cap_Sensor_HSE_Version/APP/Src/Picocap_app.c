/**@file        Picocap_app.c
* @brief        PCap芯片应用及其数据处理
* @details      读取PCap芯片,对读出的数据进行滤波,数据转换
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

#include "Picocap_app.h"
#include "VariaType.h"
#include "TypeConvert.h"
#include "In_Memory_app.h"
#include "math.h"


/**@brief       切换滤波水平函数
* @param[in]    FiltFactor : 滤波系数;
* @param[in]    FilterParam : 指定滤波参数结构体;
* @return       函数执行结果
* - None
* @note         滤波水平分9级，数字从低到高对应滤波深度从低到高
*/
void SwitchCurFilter(uint8_t FiltFactor, DataFilterParam *FilterParam)
{  
    switch(FiltFactor)
    {
    case 0:
        FilterParam->FilterBufMax   = 1;
        FilterParam->FilterCycle    = 1;
    break;

    case 1:
        FilterParam->FilterBufMax   = 1;
        FilterParam->FilterCycle    = 12;
    break;

    case 2:
        FilterParam->FilterBufMax   = 2;
        FilterParam->FilterCycle    = 12;
    break;

    case 3:
        FilterParam->FilterBufMax   = 3;
        FilterParam->FilterCycle    = 12;
    break;

    case 4:
        FilterParam->FilterBufMax   = 4;
        FilterParam->FilterCycle    = 15;
    break;

    case 5:
        FilterParam->FilterBufMax   = 5;
        FilterParam->FilterCycle    = 24;
    break;

    case 6:
        FilterParam->FilterBufMax   = 6;
        FilterParam->FilterCycle    = 30;
    break;

    case 7:
        FilterParam->FilterBufMax   = 8;
        FilterParam->FilterCycle    = 30;
    break;

    case 8:
        FilterParam->FilterBufMax   = 8;
        FilterParam->FilterCycle    = 60;
    break;

    case 9:
        FilterParam->FilterBufMax   = 10;
        FilterParam->FilterCycle    = 96;
    break;

    default:
        FilterParam->FilterBufMax   = 1;
        FilterParam->FilterCycle    = 12;
    break;
    }

    FilterParam->FilterStart = 0;
}



/**@brief       初始化数据滤波需要的参数结构
* @param[in]    Filterfactor_CountMax : 滤波系数和累计输入的最大值,滤波系数在高8位,
* 累计输入的最大值在低8位;
* @param[out]   FilterParam : 数据滤波需要的参数结构指针;
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void DataFilterParam_Init(DataFilterParam *FilterParam, uint16_t Filterfactor_CountMax)
{        
    InMemory_Read_MultiBytes(FILTERFACTOR_BASE_ADDRESS, 
                            (uint8_t *)&FilterParam->FilterFactor, 
                            sizeof(FilterParam->FilterFactor));      //滤波系数
    SwitchCurFilter(FilterParam->FilterFactor, FilterParam);         //切换滤波水平
    FilterParam->InputCountMax = Filterfactor_CountMax; 
    InMemory_Read_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMax),   //CapMax成员相对地址
                            (uint8_t *)&FilterParam->InputRangeMax, 
                            sizeof(FilterParam->InputRangeMax));      //电容满点值
    InMemory_Read_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMin),   //CapMin成员相对地址
                            (uint8_t *)&FilterParam->InputRangeMin, 
                            sizeof(FilterParam->InputRangeMin));      //电容零点值
}


/**@brief       初始化数据转换需要的参数结构
* @param[out]   DataConvert_Param : 数据转换需要的参数结构指针;
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void DataConvertParam_Init(PCap_DataConvert_Param *DataConvert_Param)
{   
    InMemory_Read_MultiBytes(PCAP_PARAM_BASE_ADDRESS, 
                            (uint8_t *)DataConvert_Param, 
                            sizeof(PCap_DataConvert_Param));      //补偿使能标志        
}

/**@brief       获取PCap原始采集值
* @param[in]    reg_addr : 结果寄存器的地址;
* @param[out]   PCap_Result : 保存PCap的输出结果;
* @param[in]    Read_Cnt : 读取结果的个数;
* @return       函数执行结果
* - OP_SUCCESS(操作成功)
* - OP_FAILED(操作失败)
*/
uint8_t Sensor_PCap_GetResult(uint8_t reg_addr, uint32_t *PCap_Result, uint8_t Read_Cnt)
{
    uint32_t PCap_Status;
    uint8_t Result = OP_FAILED;    
    uint8_t i;
    
    //读取PCap状态
    PCap_Status = PCap_Res_Sta();
    if(PCap_Status & PCAP_RUNBIT)
    {
        for(i = 0; i < Read_Cnt; i++)
        {
            PCap_Result[i] = PCap_Res_Data(reg_addr + i);           
        }
        Result = OP_SUCCESS;
    }
    if(reg_addr <= RESULT_REG7_ADDR)
    {
        PCap_Measure();
    }
            
    return Result;
}

/**@brief       数据按指定滤波参数进行滤波
* @param[in]    FilterParam : 指定滤波参数结构体;
* @param[in]    InputValue : 输入的数据;
* @param[out]   OutputValue : 输出数据的指针;
* @return       函数执行结果
* - OP_SUCCESS(操作成功)
* - OP_FAILED(操作失败)
* @note         使用本函数前,先将FilterParam初始化(即对它的每一个成员赋合适的值),否则不能正常使用
*/
uint8_t Sensor_DataFilter(DataFilterParam *FilterParam, uint32_t InputValue, uint32_t *OutputValue)
{
    static uint32_t Data_1st_FilterBuf[DATA_1ST_FILTER_BUF_MAX];        //一阶滤波缓存
    static uint32_t Data_1st_FilterBufBak[DATA_1ST_FILTER_BUF_MAX];     //一阶滤波缓存备份
    static uint32_t Data_2nd_FilterBuf[DATA_2ND_FILTER_BUF_MAX];        //二阶滤波缓存
    static uint32_t Data_2nd_FilterBufBak[DATA_2ND_FILTER_BUF_MAX];     //二阶滤波缓存备份
    static uint32_t DataResBuf[DATA_BUF_MAX];                           //接收原始数据值的缓存
    static uint8_t DataResBuf_Count;
    uint8_t i;
    uint32_t Data_Avg1;
    uint32_t Data_Avg2;
    uint32_t Data_Avg3;
    uint64_t Data_FilterSum;                                            //滤波数据累计和
    
    if(InputValue > FilterParam->InputRangeMax)
    {
        InputValue = FilterParam->InputRangeMax;
    }
    if(InputValue < FilterParam->InputRangeMin)
    {
        InputValue = FilterParam->InputRangeMin;
    }
    
    if(FilterParam->FilterStart == 0)
    {
        DataResBuf_Count = 0;
        for(i = 0; i < DATA_1ST_FILTER_BUF_MAX; i++)
        {
            Data_1st_FilterBuf[i] = InputValue;
        }
        for(i = 0; i < DATA_2ND_FILTER_BUF_MAX; i++)
        {
            Data_2nd_FilterBuf[i] = InputValue;
        }
        FilterParam->FilterStart = 1;
        
        return OP_FAILED;
    }
    
    DataResBuf[DataResBuf_Count++] = InputValue;
    if(DataResBuf_Count < DATA_BUF_MAX)
    {
        return OP_FAILED;
    }
    else
    {
        DataResBuf_Count = 0;
    }
    
    Data_Avg1 = GetDelExtremeAndAverage(DataResBuf, DATA_BUF_MAX, 2, 5);
    
    if(FilterParam->FilterCycle < 2)
    {
        *OutputValue = Data_Avg1;
        return OP_SUCCESS;
    }
    else
    {
        for(i = 0; i < FilterParam->FilterCycle - 1; i++)
        {
            Data_1st_FilterBuf[i] = Data_1st_FilterBuf[i + 1];
        }
        Data_1st_FilterBuf[i] = Data_Avg1;
        
        if(FilterParam->FilterCycle >= 3)
        {
            for(i = 0; i < FilterParam->FilterCycle; i++)
            {
                Data_1st_FilterBufBak[i] = Data_1st_FilterBuf[i];
            }
            Data_Avg2 = GetDelExtremeAndAverage(Data_1st_FilterBufBak, 
                                            FilterParam->FilterCycle, 
                                            FilterParam->FilterCycle / 3, 
                                            FilterParam->FilterCycle / 3);
        }
        else
        {
            Data_FilterSum = 0;                                                               
            for(i = 0; i < FilterParam->FilterCycle; i++)
            {
                Data_FilterSum += Data_1st_FilterBuf[i];
            }           
            Data_Avg2 = Data_FilterSum / FilterParam->FilterCycle;        //计算滤波数组的平均值
        }
        
        if(FilterParam->FilterBufMax < 2)            
        {
            *OutputValue = Data_Avg2;
            return OP_SUCCESS;
        }
        else
        {
            for(i = 0; i < FilterParam->FilterBufMax - 1; i++)
            {
                Data_2nd_FilterBuf[i] = Data_2nd_FilterBuf[i + 1];
            }
            Data_2nd_FilterBuf[i] = Data_Avg2;
            
            if(FilterParam->FilterBufMax >= 3)
            {
                for(i = 0; i < FilterParam->FilterBufMax; i++)
                {
                    Data_2nd_FilterBufBak[i] = Data_2nd_FilterBuf[i];
                }
                Data_Avg3 = GetDelExtremeAndAverage(Data_2nd_FilterBufBak, 
                                                FilterParam->FilterBufMax, 
                                                FilterParam->FilterBufMax / 3, 
                                                FilterParam->FilterBufMax / 3);
            }
            else
            {
                Data_FilterSum = 0;                                                               
                for(i = 0; i < FilterParam->FilterBufMax; i++)
                {
                    Data_FilterSum += Data_2nd_FilterBuf[i];
                }           
                Data_Avg3 = Data_FilterSum / FilterParam->FilterBufMax;        //计算滤波数组的平均值
            }
            
            *OutputValue = Data_Avg3;
            return OP_SUCCESS;
        }       
    }    
}

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
                                PCap_DataConvert_Out_Param *DataConvert_Out)
{
    float PCap_Result;
    float LiquidHeightRate;

    //补偿使能,修正K,B值
    if(DataConvert_Param->CompenEn == COMPENSATE_ENABLE)        
    {
        PCap_Result = Cap_Value * DataConvert_Param->Correct_K 
                                + DataConvert_Param->Correct_B;
    }
    else if(DataConvert_Param->CompenEn == COMPENSATE_DISABLE)        
    {
        PCap_Result = Cap_Value;
    }
    //计算电容比率
    LiquidHeightRate = (float)(PCap_Result - DataConvert_Param->Cap_Calib.CapMin) 
                                / (float)(DataConvert_Param->Cap_Calib.CapMax - DataConvert_Param->Cap_Calib.CapMin);
    
    if(1.0 < LiquidHeightRate)
    {
        LiquidHeightRate = 1.0;
    }
    else if (0.0 > LiquidHeightRate)
    {
        LiquidHeightRate = 0.0;
    }
    //转换成0--65535范围内的AD值
    DataConvert_Out->LiquidHeightAD = (uint16_t)(LiquidHeightRate * PCAP_ADC_MAX_VALUE);
    //转换成液位高度
    DataConvert_Out->LiquidHeight = (uint32_t)(LiquidHeightRate 
                                            * DataConvert_Param->HeightRange);      
    DataConvert_Out->LiquidHeight_Percentage = (uint16_t)(LiquidHeightRate * 1000);
}

#ifdef USING_DAC
/**@brief       将输入的AD值转换成DA值
* @param[in]    DataConvert_Param : 指定控制参数结构体;
* @param[in]    Cap_AD_Value : 输入的数据(AD值);
* @return       函数执行结果
* - DA值
* @note         使用本函数前,先将DataConvert_Param初始化(即对它的每一个成员赋合适的值),否则不能正常使用
*/
uint16_t Sensor_PCap_DA_Convert(PCap_DataConvert_Param *DataConvert_Param, 
                                uint32_t Cap_AD_Value)
{
    uint16_t Output_DA_Value;
    float PCap_Result;
    float Rate;
    
    //补偿使能,修正K,B值
    if(DataConvert_Param->CompenEn == COMPENSATE_ENABLE)        
    {
        PCap_Result = Cap_AD_Value * DataConvert_Param->Correct_K 
                                + DataConvert_Param->Correct_B;
    }
    else if(DataConvert_Param->CompenEn == COMPENSATE_DISABLE)        
    {
        PCap_Result = Cap_AD_Value;
    }
    
    Rate = (float)(PCap_Result - DataConvert_Param->CapAD_Calib.CapADMin) 
                    / (float)(DataConvert_Param->CapAD_Calib.CapADMax - DataConvert_Param->CapAD_Calib.CapADMin);
    
    if(1.0 < Rate)
    {
        Rate = 1.0;
    }
    else if (0.0 > Rate)
    {
        Rate = 0.0;
    }
    
    //如果没有4点差值法则直接计算DA输出值
    if((0x0000 == DataConvert_Param->CapDA_Calib.CapDALow) || (0x0000 == DataConvert_Param->CapDA_Calib.CapDAHigh)
        || (0x0000 == DataConvert_Param->CapAD_Calib.CapADLow) || (0x0000 == DataConvert_Param->CapAD_Calib.CapADHigh))
    {
        Output_DA_Value = (uint16_t)(Rate * (DataConvert_Param->CapDA_Calib.CapDAMax 
                                            - DataConvert_Param->CapDA_Calib.CapDAMin)) 
                                            + DataConvert_Param->CapDA_Calib.CapDAMin;
    }
    else
    {                 
        //AD值在零点AD以下
        if(Cap_AD_Value < DataConvert_Param->CapAD_Calib.CapADMin)  
        {
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDAMin;
        }
        //AD值在零点AD和下刻度AD之间
        else if((Cap_AD_Value < DataConvert_Param->CapAD_Calib.CapADLow) 
            && (Cap_AD_Value) >= DataConvert_Param->CapAD_Calib.CapADMin)
        {
            Rate = (float)(Cap_AD_Value - DataConvert_Param->CapAD_Calib.CapADMin)
                / (float)(DataConvert_Param->CapAD_Calib.CapADLow - DataConvert_Param->CapAD_Calib.CapADMin);
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDALow - DataConvert_Param->CapDA_Calib.CapDAMin;
            Output_DA_Value = (uint16_t)(Rate * Output_DA_Value) 
                                            + DataConvert_Param->CapDA_Calib.CapDAMin;
        }
        //AD值在下刻度AD值和上刻度AD值之间
        else if((Cap_AD_Value < DataConvert_Param->CapAD_Calib.CapADHigh) 
            && (Cap_AD_Value >= DataConvert_Param->CapAD_Calib.CapADLow))
        {
            Rate = (float)(Cap_AD_Value - DataConvert_Param->CapAD_Calib.CapADLow) 
                / (float)(DataConvert_Param->CapAD_Calib.CapADHigh - DataConvert_Param->CapAD_Calib.CapADLow);
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDAHigh - DataConvert_Param->CapDA_Calib.CapDALow;
            Output_DA_Value = (uint16_t)(Rate * Output_DA_Value) 
                                            + DataConvert_Param->CapDA_Calib.CapDALow;
        }
        //AD值在上刻度AD值和满量程AD值之间
        else if((Cap_AD_Value < DataConvert_Param->CapAD_Calib.CapADMax) 
            && (Cap_AD_Value >= DataConvert_Param->CapAD_Calib.CapADHigh))
        {
            Rate = (float)(Cap_AD_Value - DataConvert_Param->CapAD_Calib.CapADHigh) 
                / (float)(DataConvert_Param->CapAD_Calib.CapADMax - DataConvert_Param->CapAD_Calib.CapADHigh);
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDAMax - DataConvert_Param->CapDA_Calib.CapDAHigh;
            Output_DA_Value = (uint16_t)(Rate * Output_DA_Value) 
                                                    + DataConvert_Param->CapDA_Calib.CapDAHigh;
        }                                            
        //AD值在满量程以上
        else                                                       
        {
            Output_DA_Value = DataConvert_Param->CapDA_Calib.CapDAMax;
        }
    } 
    
    return Output_DA_Value;
}
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
                                uint32_t LiquidHeight)
{
    uint16_t PCapPWM_OutValue;
    float Rate;
    
        //液位高度 < 0点高度
    if(LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)
    {
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM0;                                        
    }
    //液位高度在 0点 ~ 1点高度 之间
    if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High0 
        && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High1)
    {
        Rate = (float)((float)LiquidHeight / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High1 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM0 - DataConvert_Param->CapPWM_Calib.CapPWM1;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM0 - (uint16_t)(Rate * PCapPWM_OutValue);
                                        
    }
    //液位高度在 1点 ~ 2点高度 之间
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High1 
            && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High2)
    {
        Rate = (float)(((float)LiquidHeight - (DataConvert_Param->CapPWM_High_Calib.CapPWM_High1 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)) 
                / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High2 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High1));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM1 - DataConvert_Param->CapPWM_Calib.CapPWM2;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM1 - (uint16_t)(Rate * PCapPWM_OutValue);
    }
    //液位高度在 2点 ~ 3点高度 之间
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High2 
            && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High3)
    {
        Rate = (float)(((float)LiquidHeight - (DataConvert_Param->CapPWM_High_Calib.CapPWM_High2 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)) 
                / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High3 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High2));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM2 - DataConvert_Param->CapPWM_Calib.CapPWM3;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM2 - (uint16_t)(Rate * PCapPWM_OutValue);
    }   
    //液位高度在 3点 ~ 4点高度 之间
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High3 
            && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High4)
    {
        Rate = (float)(((float)LiquidHeight - (DataConvert_Param->CapPWM_High_Calib.CapPWM_High3 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)) 
                / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High4 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High3));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM3 - DataConvert_Param->CapPWM_Calib.CapPWM4;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM3 - (uint16_t)(Rate * PCapPWM_OutValue);
    } 
    //液位高度在 4点 ~ 5点高度 之间
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High4
            && LiquidHeight < DataConvert_Param->CapPWM_High_Calib.CapPWM_High5)
    {
        Rate = (float)(((float)LiquidHeight - (DataConvert_Param->CapPWM_High_Calib.CapPWM_High4 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High0)) 
                / (DataConvert_Param->CapPWM_High_Calib.CapPWM_High5 - DataConvert_Param->CapPWM_High_Calib.CapPWM_High4));
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM4 - DataConvert_Param->CapPWM_Calib.CapPWM5;
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM4 - (uint16_t)(Rate * PCapPWM_OutValue);
    } 
    //液位高度 >= 5点高度
    else if(LiquidHeight >= DataConvert_Param->CapPWM_High_Calib.CapPWM_High5)
    {
        PCapPWM_OutValue = DataConvert_Param->CapPWM_Calib.CapPWM5;
    } 
    
    return PCapPWM_OutValue;
}
#endif // USING_PWM

/**@brief       获取从Pcap输出的热敏电阻与参考电阻的比值转换后的温度值
* @param[in]    R_ratio : 热敏电阻与参考电阻的比值
* @return       函数执行结果
* - 温度值; 
*/
float PCap_GetTemp(float R_ratio)
{
	return (-1 * TempA + sqrt(TempA * TempA - 4 * TempB * (1 - R_ratio))) / (2 * TempB);
}

/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS
#include <board.h>

static struct rt_pcap_device_obj pcap_device_obj;       //Pcap设备对象

/**@brief       Pcap设备初始化,本函数被rt_device_register注册后,被
* rt_device_init或rt_device_open调用
* @param[in]    dev : 设备句柄
* @return       函数执行结果
* - RT_EOK : 设备初始化成功; 
*/
static rt_err_t pcap_init(rt_device_t dev)
{    
    rt_thread_mdelay(1000);
    PCap_Init();
    
    return RT_EOK;
}

/**@brief       Pcap设备打开,本函数被rt_device_register注册后,被
* rt_device_open调用
* @param[in]    dev : 设备句柄
* @param[in]    oflag : 设备访问模式标志
* @return       函数执行结果
* - RT_EOK : 设备初始化成功; 
* @note         这里不使用设备打开,故只返回一个RT_EOK
*/
static rt_err_t  pcap_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

/**@brief       Pcap设备关闭,本函数被rt_device_register注册后,被
* rt_device_close调用
* @param[in]    dev : 设备句柄
* @return       函数执行结果
* - RT_EOK:设备关闭成功; 
* @note         这里不使用设备关闭,故只返回一个RT_EOK
*/
static rt_err_t  pcap_close(rt_device_t dev)
{
    return RT_EOK;
}

/**@brief       Pcap设备读数据,本函数被rt_device_register注册后,被
* rt_device_read调用
* @param[in]    dev : 设备句柄
* @param[in]    pos : 读取数据偏移量;
* @param[out]   buffer : 内存缓冲区指针，读取的数据将会被保存在缓冲区中;
* @param[in]    size : 读取数据的大小
* @return       函数执行结果
* - size : 读到数据的实际大小，如果是字符设备，返回大小以字节为
* 单位，如果是块设备，返回的大小以块为单位;
* - 0 : 需要读取当前线程的 errno 来判断错误状态
*/
static rt_size_t pcap_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    if(Sensor_PCap_GetResult(pos, buffer, size) != OP_SUCCESS)
    {
        return 0;
    }    
    
    return size;
}

/**@brief       Pcap设备控制,本函数被rt_device_register注册后,被
* rt_device_control调用
* @param[in]    dev : 设备句柄
* @param[in]    cmd : 命令控制字，这个参数通常与设备驱动程序相关;
* @param[in]    arg : 控制的参数
* @return       函数执行结果
* - RT_EOK:函数执行成功;
* @note         这里不使用设备控制,故只返回一个RT_EOK
*/
static rt_err_t  pcap_control(rt_device_t dev, int cmd, void *args)
{
    return RT_EOK;
}


/**@brief       cap设备参数初始化,注册Pcap设备
* @return       函数执行结果
* - int整型值(RT_EOK)
* @note         本函数使用RT-Thread的自动初始化组件INIT_DEVICE_EXPORT
* 调用执行,系统复位后自动初始化
*/
static int pcap_device_init(void)
{
    pcap_device_obj.dev.type     = RT_Device_Class_Miscellaneous;
    pcap_device_obj.dev.init     = pcap_init;
    pcap_device_obj.dev.open     = pcap_open;
    pcap_device_obj.dev.close    = pcap_close;
    pcap_device_obj.dev.read     = pcap_read;
    pcap_device_obj.dev.write    = RT_NULL;
    pcap_device_obj.dev.control  = pcap_control;    
    
    pcap_device_obj.dev.user_data = &pcap_device_obj;
    
    DataFilterParam_Init(&pcap_device_obj.DataFilter, DATA_BUF_MAX);    // 滤波参数初始化
    DataConvertParam_Init(&pcap_device_obj.PCap_DataConvert);           // PCap数据转换参数初始化
    
    rt_device_register(&pcap_device_obj.dev, PCAP_DEVICE_NAME,          // 注册Pcap设备
                        RT_DEVICE_FLAG_RDWR
                        | RT_DEVICE_FLAG_STANDALONE);
    
    return RT_EOK;
}
INIT_DEVICE_EXPORT(pcap_device_init);

#endif // USING_RT_THREAD_OS

