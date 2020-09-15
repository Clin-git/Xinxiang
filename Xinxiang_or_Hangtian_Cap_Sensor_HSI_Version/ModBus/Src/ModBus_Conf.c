/**@file        Modbus_Conf.c
* @brief        Modbus 功能码处理配置
* @details      对上位机发送的指令进行解析并响应,所有功能码和自动上传的程序代码都编写在本文件
* @author       庄明群
* @date         2020-07-20
* @version      V2.0.0
* @copyright    2020-2030,深圳市信为科技发展有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author  <th>Maintainer  <th>Description
* <tr><td>2020/07/20  <td>2.0.0    <td>庄明群  <td>杨春林      <td>新添加的程序代码(指令执行部分)
* </table>
*
**********************************************************************************
*/

#include "ModBus_Conf.h"

#if defined(USING_MODBUS_RTU)
#include "ModBus_RTU.h"
#elif defined(USING_MODBUS_ASCII)
#include "ModBus_ASCII.h"
#endif // defined(USING_MODBUS_RTU) or defined(USING_MODBUS_ASCII)


#ifdef __PICOCAP_APP_H

/* 使用soway上位机升级程序(Boot程序), BOOT_PROGRAM在main.h中定义 */
#ifndef BOOT_PROGRAM
static uint32_t Calib_CapMin;                   ///< 标定电容零点值
static uint32_t Calib_CapMax;                   ///< 标定电容满点值
#ifdef USING_ADC_TEMPER_SENSOR
static uint16_t Calib_TempDAMin;                ///< 标定温度DA零点值
static uint16_t Calib_TempDAMax;                ///< 标定温度DA满量程
#endif
#ifdef USING_DAC
static uint16_t Calib_DAMin;                    ///< 标定电容DA零点值
static uint16_t Calib_DALow;                    ///< 标定电容DA下刻度
static uint16_t Calib_DAHigh;                   ///< 标定电容DA上刻度
static uint16_t Calib_DAMax;                    ///< 标定电容DA满量程
static uint8_t CapDA_ClibFlag = CALIB_CLEAR;    ///< 标定电容DA标志
#endif // USING_DAC
#ifdef USING_PWM
static uint16_t Calib_PWM0;                     ///< 标定电容 PWM 0点值
static uint16_t Calib_PWM1;                     ///< 标定电容 PWM 1点值
static uint16_t Calib_PWM2;                     ///< 标定电容 PWM 2点值
static uint16_t Calib_PWM3;                     ///< 标定电容 PWM 3点值
static uint16_t Calib_PWM4;                     ///< 标定电容 PWM 4点值
static uint16_t Calib_PWM5;                     ///< 标定电容 PWM 5点值
static uint8_t  CapPWM_ClibFlag = CALIB_CLEAR;  ///< 标定电容 PWM 标志
#endif // USING_PWM


const uint8_t Company_Name[] = {9, '7','3','8','8','3','0','6','7','6'};
const uint8_t Product_Code[] = {20, 'S','F','C','G','2','0','L','-','1','1','3','5','A',
                                '-','M','2','-','M','E','P'};
const uint8_t Hardware_Version[] = {7, 'H','V','2','.','0','.','0'};
const uint8_t Software_Version[] = {7, 'S','V','2','.','0','.','0'};
const uint8_t Device_ID[] = {11, '2','0','2','0','0','8','1','7','0','0','1'};
const uint8_t Customer_Code[] = {7, 'K','1','5','0','0','1','6'};

//Modbus 串口重新初始化回调函数
static int MB_USART_ReInit(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg);

//Modbus 设备冻结回调函数
static int MB_Freeze(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg);

//系统复位回调函数
static int MB_System_Reset(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg);

//无地址检查发送数据
static int MB_SendData_NoCheck(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg);

//备份数据
static uint8_t BankUp_Data(void);

//恢复出厂数据
static uint8_t Factory_Data_Reset(void);



/**@brief       Modbus 读预处理,从接收的原始数据中提取出消息ID和寄存器数量,将设备地址和功能码存入发送缓存中
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[out]   ReadAddr : 读取地址(消息ID)
* @param[out]   RegNum : 寄存器数量
* @return       函数执行结果
* - None
* @note         寄存器地址不能越界
*/
static void ModBus_ReadPreHandle(   ModBusBaseParam_TypeDef *ModBusBaseParam, 
                                    uint16_t *ReadAddr, 
                                    uint16_t *RegNum, 
                                    uint8_t *Err_Status)
{
    *Err_Status = OP_SUCCESS;
    
    if(ReadAddr != NULL)
    {
        //寄存器地址高字节
        *ReadAddr = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2];
        *ReadAddr <<= 8;
        //寄存器地址高字节
        *ReadAddr |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[3];
#if defined(SUBCODE_IS_DEVADDR)
        if((*ReadAddr >> 8) != ModBusBaseParam->Device_Addr)
        {
            *Err_Status = OP_FAILED;
        }        
#else
        if((*ReadAddr >> 8) != DEFAULT_SUBCODE)
        {
            *Err_Status = OP_FAILED;
            return;
        }
#endif // defined(SUBCODE_IS_DEVADDR)
        *ReadAddr &= 0x00FF;
    }

    if(RegNum != NULL)
    {
        //寄存器数据高字节
        *RegNum = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4];
        *RegNum <<= 8;
        //寄存器数量低字节
        *RegNum |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];
    }
    
    //拷贝设备地址、功能码
    memcpy( ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
            ModBusBaseParam->ModBus_TX_RX.Receive_Buf,
            2);
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 2;
}

/**@brief       Modbus 写预处理,从接收的原始数据中提取出消息ID,将设备地址、
* 功能码、消息ID和消息长度存入发送缓存中
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[out]   WriteAddr : 写入地址(消息ID)
* @param[out]   RegNum : 寄存器数量
* @return       函数执行结果
* - None
* @note         寄存器地址不能越界
*/
static void ModBus_WritePreHandle(  ModBusBaseParam_TypeDef *ModBusBaseParam, 
                                    uint16_t *WriteAddr, 
                                    uint16_t *RegNum, 
                                    uint8_t *Err_Status)
{
    *Err_Status = OP_SUCCESS;
    
    if(WriteAddr != NULL)
    {
        //寄存器地址高字节
        *WriteAddr = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2];
        *WriteAddr <<= 8;
        //寄存器地址高字节
        *WriteAddr |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[3];
#if defined(SUBCODE_IS_DEVADDR)
        if((*WriteAddr >> 8) != ModBusBaseParam->Device_Addr)
        {
            *Err_Status = OP_FAILED;
        }        
#else
        if((*WriteAddr >> 8) != DEFAULT_SUBCODE)
        {
            *Err_Status = OP_FAILED;
            return;
        }
#endif // defined(SUBCODE_IS_DEVADDR)
        *WriteAddr &= 0x00FF;
    }

    if(RegNum != NULL)
    {
        //寄存器数据高字节
        *RegNum = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4];
        *RegNum <<= 8;
        //寄存器数量低字节
        *RegNum |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];
    }

    //将响应数据存入发送缓存
    memcpy( ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
            ModBusBaseParam->ModBus_TX_RX.Receive_Buf,
            6);

    ModBusBaseParam->ModBus_TX_RX.Send_Len = 6;
}

/**@brief       Modbus 03功能码消息帧处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void ModbusFunc03(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    uint16_t Nr;
    //寄存器数据
    uint16_t DataBuf;
    //寄存器地址
    uint16_t ReadAddr;
    //寄存器数量
    uint16_t RegNum;
    //错误状态
    uint8_t Err_Status;
    //设备参数
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    //读预处理
    ModBus_ReadPreHandle(ModBusBaseParam, &ReadAddr, &RegNum, &Err_Status);
    //访问地址不在有效范围内
    if( ReadAddr < HOLDING_REG_REGION1_BGEIN 
        || (ReadAddr + RegNum) > (HOLDING_REG_REGION1_END + 1)
        || RegNum == 0)
    {
        ReadAddr = 0xFFFF;
    }
    if(Err_Status != OP_SUCCESS && ReadAddr != 0x0030)
    {
        ReadAddr = 0xFFFF;
    }

    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++]
        = RegNum * 2;
    for(Nr = 0; Nr < RegNum; Nr++, ReadAddr++)
    {
        switch(ReadAddr)
        {
            case 0x0030:
                //设备地址
                DataBuf = ModBusBaseParam->Device_Addr;
                if(BROADCASTADDR == ModBusBaseParam->ModBus_TX_RX.Receive_Buf[0])
                {
                    ModBusBaseParam->ModBus_CallBack = MB_SendData_NoCheck;
                }
            break;
                    
            case 0x0031:
                //波特率
                DataBuf = ModBusBaseParam->BaudRate;
            break;
                    
            case 0x0032:
                //奇偶校验
                DataBuf = ModBusBaseParam->Parity;
            break;

            case 0x0033:
                //空
                DataBuf = 0;
            break;

            case 0x0034:
                //补偿使能
                DataBuf = Device_Param->PCap_DataConvert->CompenEn;
            break;

            case 0x0035:
                //滤波系数
                if(Device_Param->DataFilter->FilterFactor == 0)        //实时
                {
                    DataBuf = 1;
                }
                else if(Device_Param->DataFilter->FilterFactor == 4)   //平滑
                {
                    DataBuf = 2;
                }
                else if(Device_Param->DataFilter->FilterFactor == 9)   //平稳
                {
                    DataBuf = 3;
                }
            break;

            case 0x0036:
                //自动上传周期
                DataBuf = (ModBusBaseParam->AutoUpload / 10) + 1;
            break;

            case 0x0037:
                //电容修正系数K
                DataBuf = (uint16_t)(Device_Param->PCap_DataConvert->Correct_K * 100.0);
            break;

            case 0x0038:
                //电容修正系数B
                DataBuf = (uint16_t)(Device_Param->PCap_DataConvert->Correct_B + 100.0);
            break;

            case 0x0039:
                //位置报警阀值1 保留
                DataBuf = 0;
            break;
            
            case 0x003A:
                //回差1 保留
                DataBuf = 0;
            break;
            
            case 0x003B:
                //位置报警阀值2 保留
                DataBuf = 0;
            break;
            
            case 0x003C:
                //回差2 保留
                DataBuf = 0;
            break;
            
            case 0x003D:
                //转换单位 保留
                DataBuf = 0;
            break;
            
            case 0x003E:
                //Pcap更新时间 保留
                DataBuf = 0;
            break;

            case 0x003F:
                //量程
                DataBuf = Device_Param->PCap_DataConvert->HeightRange * 10;
            break;

//            case 0x0040:
//                //空
//                DataBuf = 0;
//            break;

//            case 0x0041:
//                //燃料选择 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x0042:
//                //设置油箱形状 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x0047:
//                //加油时间阀值 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x0048:
//                //加油量阀值 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x0049:
//                //漏油时间阀值 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x004A:
//                //漏油量阀值 保留
//                DataBuf = 0;
//            break;
            
            case 0x0060:
                //输出方式
                DataBuf = ModBusBaseParam->Output_Mode;

            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                //访问地址无效
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)(DataBuf >> 8);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)DataBuf;       
    }
}

/**@brief       Modbus 04功能码消息帧处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void ModbusFunc04(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    uint16_t Nr;
    //寄存器地址
    uint16_t ReadAddr;
    //寄存器数量
    uint16_t RegNum;
    //错误状态
    uint8_t Err_Status;
    //寄存器数据
    uint32_t DataBuf;
    //设备参数
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    //读预处理
    ModBus_ReadPreHandle(ModBusBaseParam, &ReadAddr, &RegNum, &Err_Status);
    //寄存器地址无效
    if( Err_Status != OP_SUCCESS 
        || (ReadAddr > INPUT_REG_REGION1_END
        && (ReadAddr < INPUT_REG_REGION2_BEGIN || ReadAddr > INPUT_REG_REGION2_END))
        || (ReadAddr & 0x01) != 0 || (RegNum & 0x01) != 0
        || RegNum == 0)
    {
        ReadAddr = 0xFFFF;
    }
    
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++]
        = RegNum * 2;
    for(Nr = 0; Nr < RegNum; Nr += 2, ReadAddr += 2)
    {
        switch(ReadAddr)
        {
            case 0x0000:
                //液位高度AD值
                DataBuf = Device_Param->PCap_DataConvert_Out->LiquidHeightAD;
            break;
                
            case 0x0002:
                //液体温度 保留
                DataBuf = 0;
            break;
                
            case 0x0004:
                //环境温度
                DataBuf = (uint32_t)Device_Param->PCap_DataConvert_Out->PCap_Temper_Value;
            break;
            
//            case 0x0006:
//                //加油量 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x0008:
//                //漏油量 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x000A:
//                //油箱油量 保留
//                DataBuf = 0;
//            break;
            
            case 0x000C:
                //液位百分比
                DataBuf = Device_Param->PCap_DataConvert_Out->LiquidHeight_Percentage;
            break;
            
            case 0x000E:
                //液位高度
                DataBuf = Device_Param->PCap_DataConvert_Out->LiquidHeight * 10;
            break;
    #ifdef USING_DAC        
            case 0x0010:
                //液位DA值
                DataBuf = Device_Param->PCap_DataConvert_Out->PCap_DA_Value;
            break;
    #endif // USING_DAC
    #ifdef USING_PWM
            case 0x0012:
                //液位PWM值
                DataBuf = Device_Param->PCap_DataConvert_Out->PCap_PWM_Value;
            break;
    #endif // USING_PWM
            case 0x0080:
                //PCap原始值
                DataBuf = Device_Param->PCap_DataConvert_Out->PCap_ResultValue;
            break;
            
//            case 0x0082:
//                //PCap状态 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x0084:
//                //PCap写固件状态 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x0086:
//                //液位开关零点 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x0088:
//                //液位开关满点 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x008A:
//                //满点间距比 保留
//                DataBuf = 0;
//            break;
//            
//            case 0x008C:
//                //零点间距比 保留
//                DataBuf = 0;
//            break;

            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }

        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (DataBuf >> 24);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (DataBuf >> 16);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (DataBuf >> 8);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)DataBuf;        
    }
}

/**@brief       Modbus 05功能码消息帧处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void ModbusFunc05(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    //寄存器地址
    uint16_t WriteAddr;
    //错误状态
    uint8_t Err_Status;
    //寄存器数据
    uint16_t DataBuf;
    static uint8_t CalibFlag = CALIB_CLEAR;
    //设备参数
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    //写预处理
    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, NULL, &Err_Status);    
    //地址无效
    if(Err_Status != OP_SUCCESS 
        || WriteAddr < SINGLE_COIL_REGION1_BEGIN 
        || WriteAddr > SINGLE_COIL_REGION1_END)
    {
        WriteAddr = 0xFFFF;
    }

    //数据内容高字节
    DataBuf = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4];
    DataBuf <<= 8;
    //数据内容低字节
    DataBuf |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];

    //数据内容无效
    if((0x0000 != DataBuf)&&(0xFF00 != DataBuf))
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }
     
    switch(WriteAddr)
    {
        case 0x0050:
            if(ModBusBaseParam->InRomWrEn == IN_MEMORY_WR_ENABLE)
            {
                //电容标定
                if(0xFF00 == DataBuf)       //标定满量程
                {
                    Calib_CapMax = Device_Param->PCap_DataConvert_Out->PCap_ResultValue;
                    CalibFlag |= CALIB_CAPMAX_FLAG;
                }
                else                        //标定零点
                {
                    Calib_CapMin = Device_Param->PCap_DataConvert_Out->PCap_ResultValue;
                    CalibFlag |= CALIB_CAPMIN_FLAG;
                }
                //电容标定标志位有效就写入标定内容  
                if(CALIB_CAPEOC_FLAG == CalibFlag)
                {
                    MB_Cap_Calibration(arg);
                    CalibFlag = CALIB_CLEAR;
                }
            }
            else
            {
                goto __rom_error;
            }
        break;

        case 0x0051:
            if(ModBusBaseParam->InRomWrEn == IN_MEMORY_WR_ENABLE)
            {
                //恢复出厂数据
                if(0xFF00 == DataBuf)
                {
                    if(Factory_Data_Reset() != OP_SUCCESS)
                    {
                        goto __rom_error;
                    }
                    //恢复数据后复位重启
                    ModBusBaseParam->ModBus_CallBack = MB_System_Reset; 
                }
            }
            else
            {
                goto __rom_error;
            }
        break;

        case 0x0052:
            //设备冻结或解冻
            if(0xFF00 == DataBuf)
            {
                ModBusBaseParam->ModBus_CallBack = MB_Freeze;
            }
            else    
            {
                ModBusBaseParam->Freeze = FREEZE_DISABLE;
            }
        break;

        case 0x0053:
            //内部Flash使能或禁止
            if(0xFF00 == DataBuf)
            {
                ModBusBaseParam->InRomWrEn = IN_MEMORY_WR_ENABLE;
            }
            else
            {
                ModBusBaseParam->InRomWrEn = IN_MEMORY_WR_DISABLE;
            }
        break;
                
        default:
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
            ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
            return;
    }	
    return;
__rom_error:
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;    
}

/**@brief       Modbus 10功能码消息帧处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void ModbusFunc10(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    uint16_t Nr;
    //内容索引
    uint16_t Index;
    //寄存器地址
    uint16_t WriteAddr;
    //寄存器数量
    uint16_t RegNum;
    //错误状态
    uint8_t Err_Status;
    //数据长度
    uint16_t DataLen;
    //16位数据暂存
    uint16_t u16temp;
    //设备参数
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    
    //数据长度
    DataLen = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[6];
    //写预处理
    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, &RegNum, &Err_Status);    
    //寄存器地址无效
    if( Err_Status != OP_SUCCESS 
        || WriteAddr < MUL_REG_REGION1_BEGIN 
        || (WriteAddr + RegNum) > (MUL_REG_REGION1_END + 1)
        || RegNum == 0)
    {
        WriteAddr = 0xFFFF;
    }

    //内部ROM访问禁止
    if(IN_MEMORY_WR_ENABLE != ModBusBaseParam->InRomWrEn)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }

    Index = 0;
    for(Nr = 0; Nr < RegNum; Nr++, WriteAddr++)
    {
        u16temp = (uint16_t)(ModBusBaseParam->ModBus_TX_RX.Receive_Buf[7 + Index] << 8) 
            | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[8 + Index];

        switch(WriteAddr)
        {
            case 0x0030:
                //设备地址
                if(0 < u16temp && 0xF8 > u16temp)
                {
                    ModBusBaseParam->Device_Addr = u16temp;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->Device_Addr),
                                                (uint8_t *)&ModBusBaseParam->Device_Addr, 
                                                sizeof(ModBusBaseParam->Device_Addr));
                }
                else
                {
                    goto __error;
                }
            break;
                    
            case 0x0031:
                //波特率
                if(USART_BAUDRATE_115200_CODE >= u16temp)
                {
                    ModBusBaseParam->BaudRate = u16temp;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->BaudRate), 
                                                (uint8_t *)&ModBusBaseParam->BaudRate, 
                                                sizeof(ModBusBaseParam->BaudRate));
                    ModBusBaseParam->ModBus_CallBack = MB_USART_ReInit;
                }
                else
                {
                    goto __error;
                }
            break;
                    
            case 0x0032:
                //奇偶校验
                if(u16temp == USART_PARITY_NONE_CODE || u16temp == USART_PARITY_ODD_CODE 
                    || u16temp == USART_PARITY_EVEN_CODE)
                {
                    ModBusBaseParam->Parity = u16temp;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->Parity), 
                                                (uint8_t *)&ModBusBaseParam->Parity, 
                                                sizeof(ModBusBaseParam->Parity));
                    ModBusBaseParam->ModBus_CallBack = MB_USART_ReInit;
                }
                else
                {
                    goto __error;
                }
            break;
              
            case 0x0033:
                //空
            break;
              
            case 0x0034:
                //补偿使能
                Device_Param->PCap_DataConvert->CompenEn = u16temp;
                InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CompenEn), 
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CompenEn, 
                                            sizeof(Device_Param->PCap_DataConvert->CompenEn));
            break;
              
            case 0x0035:
                //滤波系数
                if(0x0A > u16temp)
                {
                    if(u16temp == 1)        //实时
                    {
                        Device_Param->DataFilter->FilterFactor = 0;
                    }
                    else if(u16temp == 2)   //平滑
                    {
                        Device_Param->DataFilter->FilterFactor = 4;
                    }
                    else if(u16temp == 3)   //平稳
                    {
                        Device_Param->DataFilter->FilterFactor = 9;
                    }
                    else                    //默认平滑
                    {
                        Device_Param->DataFilter->FilterFactor = 4;
                    }
                    InMemory_Write2T_MultiBytes((uint32_t)&(((DataFilterParam *)FILTERFACTOR_BASE_ADDRESS)->FilterFactor), 
                                                (uint8_t *)&Device_Param->DataFilter->FilterFactor, 
                                                sizeof(Device_Param->DataFilter->FilterFactor));
                    SwitchCurFilter(Device_Param->DataFilter->FilterFactor, Device_Param->DataFilter);
                }
                else
                {
                    goto __error;
                }
            break;
              
            case 0x0036:
                //自动上传周期
                if(u16temp >= 1 && u16temp <= 4)
                {                                    
                    ModBusBaseParam->AutoUpload = (u16temp - 1) * 10;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->AutoUpload), 
                                                (uint8_t *)&ModBusBaseParam->AutoUpload, 
                                                sizeof(ModBusBaseParam->AutoUpload));
                }
            break;
              
            case 0x0037:
                //电容修正系数K
                if(0 < u16temp)
                {
                    Device_Param->PCap_DataConvert->Correct_K = (float)u16temp / 100.0;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Correct_K), 
                                                (uint8_t *)&Device_Param->PCap_DataConvert->Correct_K, 
                                                sizeof(Device_Param->PCap_DataConvert->Correct_K));
                }
                else
                {
                    goto __error;
                }
            break;
              
            case 0x0038:
                //电容修正系数B
                Device_Param->PCap_DataConvert->Correct_B = (float)u16temp - 100.0;
                InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Correct_B), 
                                            (uint8_t *)&Device_Param->PCap_DataConvert->Correct_B, 
                                            sizeof(Device_Param->PCap_DataConvert->Correct_B));
            break;              
                
            case 0x0039:
                //位置报警阀值1 保留
            break;
            
            case 0x003A:
                //回差1 保留
            break;
            
            case 0x003B:
                //位置报警阀值2 保留
            break;
            
            case 0x003C:
                //回差2 保留
            break;
            
            case 0x003D:
                //转换单位 保留
            break;
            
            case 0x003E:
                //Pcap更新时间 保留
            break;
              
            case 0x003F:
                //量程
                if(u16temp > 0)
                {
                    Device_Param->PCap_DataConvert->HeightRange = u16temp / 10;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->HeightRange), 
                                                (uint8_t *)&Device_Param->PCap_DataConvert->HeightRange, 
                                                sizeof(Device_Param->PCap_DataConvert->HeightRange));
                }
                else
                {
                    goto __error;
                }
            break;
              
//            case 0x0040:
//                //空
//            break;              
//            
//            case 0x0041:
//                //燃料选择 保留
//            break;
//            
//            case 0x0042:
//                //设置油箱形状 保留
//            break;
//            
//            case 0x0047:
//                //加油时间阀值 保留
//            break;
//            
//            case 0x0048:
//                //加油量阀值 保留
//            break;
//            
//            case 0x0049:
//                //漏油时间阀值 保留
//            break;
//            
//            case 0x004A:
//                //漏油量阀值 保留
//            break;
            
            case 0x0060:
                //输出方式
                if(2 > u16temp)
                {
                    ModBusBaseParam->Output_Mode = u16temp;
                    InMemory_Write2T_MultiBytes((uint32_t)&(((ModBusBaseParam_TypeDef *)MODBUS_PARAM_BASE_ADDRESS)->Output_Mode), 
                                                (uint8_t *)&ModBusBaseParam->Output_Mode, 
                                                sizeof(ModBusBaseParam->Output_Mode));
                }
                else
                {
                    goto __error;
                }
            break;
              
            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }     
            
        Index += DataLen;        
    }
    return;
__error:
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3; 
}

/**@brief       Modbus 25功能码消息帧处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void ModbusFunc25(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    //寄存器地址
    uint16_t WriteAddr;
    //错误状态
    uint8_t Err_Status;
    //数据内容
    uint16_t DataBuf;
    //设备参数
    ModBus_Device_Param *Device_Param;    
#ifdef USING_ADC_TEMPER_SENSOR
    static uint8_t TempDA_ClibFlag = CALIB_CLEAR;
    static uint8_t TempDA_ClibEn = CLIB_DISABLE;
#endif // USING_ADC_TEMPER_SENSOR
        
    Device_Param = (ModBus_Device_Param *)arg;
    
    //写预处理
    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, NULL, &Err_Status);
    
    if(ModBusBaseParam->InRomWrEn != IN_MEMORY_WR_ENABLE)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }
    
    //寄存器地址无效
    if( Err_Status != OP_SUCCESS 
        || (WriteAddr > _25_FNUC_REG_REGION1_END))
    {
        WriteAddr = 0xFFFF;
    }

    //寄存器内容高字节
    DataBuf = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4];
    DataBuf <<= 8;
    //寄存器内容低字节
    DataBuf |= ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];

    //寄存内容无效
    if((0x0000 != DataBuf)&&(0xFF00 != DataBuf))
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }

    switch(WriteAddr)
    {
        case 0x0000:
        //空
        break;
#ifdef USING_DAC
        case 0x0001:
            //电容DA标定上下刻度线
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapDA_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapDA_ClibFlag |= CALIB_CAPDAHIH_FLAG;
                }
                else
                {
                    CapDA_ClibFlag |= CALIB_CAPDALOW_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;

        case 0x0002:
            //电容DA标定使能
            if(0xFF00 == DataBuf)
            {
                CapDA_ClibFlag = CALIB_CLEAR;
                Device_Param->PCap_DataConvert->CapDA_ClibEn = CLIB_ENABLE;
            }
            else
            {
                Device_Param->PCap_DataConvert->CapDA_ClibEn = CLIB_DISABLE;
            }

            if(CLIB_DISABLE == Device_Param->PCap_DataConvert->CapDA_ClibEn 
                && CapDA_ClibFlag != CALIB_CLEAR)
            {
                MB_CapDAOut_Calibration(arg);
                CapDA_ClibFlag = CALIB_CLEAR;
            }
        break;

        case 0x0003:
            //电容DA标定零点满量程
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapDA_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapDA_ClibFlag |= CALIB_CAPDAMAX_FLAG;
                }
                else
                {
                    CapDA_ClibFlag |= CALIB_CAPDAMIN_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;
#endif // USING_DAC
#ifdef USING_ADC_TEMPER_SENSOR
        case 0x0004:
            //温度标定使能
            if(0xFF00 == DataBuf)
            {
                TempDA_ClibFlag = CALIB_CLEAR;
                TempDA_ClibEn = CLIB_ENABLE;
            }
            else
            {
                TempDA_ClibEn = CLIB_DISABLE;
            }

            if((CLIB_DISABLE == TempDA_ClibEn) && (CALIB_TEMPDAEOC_FLAG == TempDA_ClibFlag))
            {
                MB_TempDAOut_Calibration(arg);
                TempDA_ClibFlag = CALIB_CLEAR;
            }
        break;

        case 0x0005:
            //温度标定
            if(CLIB_ENABLE == TempDA_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    TempDA_ClibFlag |= CALIB_TEMPDAMAX_FLAG;
                    Calib_TempDAMax = Device_Param->ADC_TemperOut->TemperInAirAD;
                }
                else
                {
                    TempDA_ClibFlag |= CALIB_TEMPDAMIN_FLAG;
                    Calib_TempDAMin = Device_Param->ADC_TemperOut->TemperInAirAD;
                }
            }
            else
            {
                goto __error;
            }
        break;
#endif // USING_ADC_TEMPER_SENSOR
#ifdef USING_PWM
        case 0x0006:
            //电容PWM标定使能
            if(0xFF00 == DataBuf)
            {
                CapPWM_ClibFlag = CALIB_CLEAR;
                Device_Param->PCap_DataConvert->CapPWM_ClibEn = CLIB_ENABLE;
            }
            else
            {
                Device_Param->PCap_DataConvert->CapPWM_ClibEn = CLIB_DISABLE;
            }

            if((CLIB_DISABLE == Device_Param->PCap_DataConvert->CapPWM_ClibEn) && (CALIB_CAPPWMEOC_FLAG == CapPWM_ClibFlag))
            {
                MB_CapPWMOut_Calibration(arg);
                CapPWM_ClibFlag = CALIB_CLEAR;
            }
        break;
            
        case 0x0007:
            //电容PWM标定 0点和 1点
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapPWM_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM1_FLAG;
                }
                else
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM0_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;
            
        case 0x0008:
            //电容PWM标定 2点和 3点
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapPWM_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM3_FLAG;
                }
                else
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM2_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;
            
        case 0x0009:
            //电容PWM标定 4点和 5点
            if(CLIB_ENABLE == Device_Param->PCap_DataConvert->CapPWM_ClibEn)
            {
                if(0xFF00 == DataBuf)
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM5_FLAG;
                }
                else
                {
                    CapPWM_ClibFlag |= CALIB_CAPPWM4_FLAG;
                }
            }
            else
            {
                goto __error;
            }
        break;
#endif // USING_PWM
            
        case 0x000A:
            //备份数据使能
            if(0xFF00 == DataBuf)
            {
                if(BankUp_Data() != OP_SUCCESS)
                {
                    goto __error;
                }
            }
        break;
            
        default:
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
            ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
            return;
    }
    return;
__error:
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3; 
}

/**@brief       Modbus 26功能码消息帧处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void ModbusFunc26(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    uint8_t i;
    //浮点数据
    float fBuf;
    uint16_t Nr;
    //寄存器地址
    uint16_t ReadAddr;
    //寄存器数量
    uint16_t RegNum;
    //错误状态
    uint8_t Err_Status;
    //寄存器数据内容
    uint32_t DataBuf;
    //设备参数
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    //读预处理
    ModBus_ReadPreHandle(ModBusBaseParam, &ReadAddr, &RegNum, &Err_Status);
    //寄存器访问地址无效
    if( Err_Status != OP_SUCCESS 
        || ReadAddr < DEF_MUL_REG_REGION1_BEGIN 
        || (ReadAddr + RegNum) > (DEF_MUL_REG_REGION1_END + 2)
        || (RegNum & 0x01) != 0 || (ReadAddr & 0x01) != 0
        || RegNum == 0)
    {
        ReadAddr = 0xFFFF;
    }

    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++]
        = RegNum * 2;
    for(Nr = 0; Nr < RegNum; Nr += 2, ReadAddr += 2)
    {
        DataBuf = 0;
        switch(ReadAddr)
        {        
            case 0x0080:                                                        //电容量程
                fBuf = (float)Device_Param->PCap_DataConvert->Cap_Calib.CapMax        
                        - (float)Device_Param->PCap_DataConvert->Cap_Calib.CapMin;        //电容零点
                DataBuf = *(uint32_t *)&fBuf;
            break;
                
            case 0x0082:
                fBuf = (float)Device_Param->PCap_DataConvert->Cap_Calib.CapMin;           //电容零点
                DataBuf = *(uint32_t *)&fBuf;
            break;
            
            case 0x0084:
                fBuf = (float)Device_Param->PCap_DataConvert->Cap_Calib.CapMax;           //电容满量程
                DataBuf = *(uint32_t *)&fBuf;
            break;
#ifdef USING_DAC
            case 0x0088:
                fBuf = (float)Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin;         //DA零点
                DataBuf = *(uint32_t *)&fBuf;
            break; 
            
            case 0x008A:
                fBuf = (float)Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow;         //DA下刻度
                DataBuf = *(uint32_t *)&fBuf;
            break; 
            
            case 0x008C:
                fBuf = (float)Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh;        //DA上刻度
                DataBuf = *(uint32_t *)&fBuf;
            break;
            
            case 0x008E:     
                fBuf = (float)Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax;         //DA满度
                DataBuf = *(uint32_t *)&fBuf;
            break;
                                                     
#endif // USING_DAC   
#ifdef USING_ADC_TEMPER_SENSOR            
            case 0x0090:                                                                
                fBuf = Device_Param->ADC_TemperParam->Temper_K1 * 100;          //环境温度修正系数K1   
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x0092:
                fBuf = Device_Param->ADC_TemperParam->Temper_B1 + 100;          //环境温度修正系数B1
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x0094:
                fBuf = Device_Param->ADC_TemperParam->Temper_K2 * 100;          //液体温度修正系数K2
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x0096:
                fBuf = Device_Param->ADC_TemperParam->Temper_B2 + 100;          //液体温度修正系数B2
                DataBuf = *(uint32_t *)&fBuf;
            break;
#endif // USING_ADC_TEMPER_SENSOR
            case 0x0098:
                fBuf = (float)Device_Param->PCap_DataConvert->CapAD_Calib.CapADMin;         //AD零点
                DataBuf = *(uint32_t *)&fBuf;
            break;    
            
            case 0x009A:
                fBuf = (float)Device_Param->PCap_DataConvert->CapAD_Calib.CapADLow;         //AD下刻度
                DataBuf = *(uint32_t *)&fBuf;
            break;  
            
            case 0x009C:
                fBuf = (float)Device_Param->PCap_DataConvert->CapAD_Calib.CapADHigh;        //AD上刻度
                DataBuf = *(uint32_t *)&fBuf;
            break;    
            
            case 0x009E: 
                fBuf = (float)Device_Param->PCap_DataConvert->CapAD_Calib.CapADMax;         //AD满度
                DataBuf = *(uint32_t *)&fBuf;
            break;                         
#ifdef USING_PWM
            case 0x00A0:     
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0;          //PWM 0点
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00A2:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1;          //PWM 1点
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00A4:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2;          //PWM 2点
                DataBuf = *(uint32_t *)&fBuf;
            break;  
              
            case 0x00A6:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3;          //PWM 3点
                DataBuf = *(uint32_t *)&fBuf;
            break; 
            
            case 0x00A8:     
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4;          //PWM 4点
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00AA:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5;          //PWM 5点
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00AC:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High0;     //PWM 0点高度
                DataBuf = *(uint32_t *)&fBuf;
            break;  
              
            case 0x00AE:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High1;     //PWM 1点高度
                DataBuf = *(uint32_t *)&fBuf;
            break; 
            
            case 0x00B0:     
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High2;     //PWM 2点高度
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00B2:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High3;     //PWM 3点高度
                DataBuf = *(uint32_t *)&fBuf;
            break;
              
            case 0x00B4:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High4;     //PWM 4点高度
                DataBuf = *(uint32_t *)&fBuf;
            break;  
              
            case 0x00B6:
                fBuf = (float)Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High5;     //PWM 5点高度
                DataBuf = *(uint32_t *)&fBuf;
            break; 
#endif // USING_PWM
            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }

        for(i = 4; i > 0; i--)
        {
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
                = (uint8_t)(DataBuf >> ((i - 1)*8));
        }
    }
}

/**@brief       Modbus 27功能码消息帧处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void ModbusFunc27(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    //浮点数据
    float fbuf;
    uint16_t Nr;
    //索引
    uint16_t index;
    //寄存器地址
    uint16_t WriteAddr;
    //寄存器数量
    uint16_t RegNum;
    //错误状态
    uint8_t Err_Status;
    //数据长度
    uint16_t DataLen;
    //设备参数
    ModBus_Device_Param *Device_Param;
          
    Device_Param = (ModBus_Device_Param *)arg;    
    //写预处理
    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, &RegNum, &Err_Status);  
    //内部ROM未使能访问失败
    if(IN_MEMORY_WR_ENABLE != ModBusBaseParam->InRomWrEn)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }

    //数据长度
    DataLen = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[6];
    //寄存器访问地址无效
    if( Err_Status != OP_SUCCESS 
        || WriteAddr < DEF_MUL_REG_REGION1_BEGIN 
        || (WriteAddr + RegNum) > (DEF_MUL_REG_REGION1_END + 2)
        || (RegNum & 0x01) != 0 || (WriteAddr & 0x01) != 0
        || RegNum == 0)
    {
        WriteAddr = 0xFFFF;
    }

    index = 0;
    for(Nr = 0; Nr < RegNum; Nr += 2, WriteAddr += 2)
    {
        fbuf = HexToFloat(&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[7 + index]);
        switch(WriteAddr)
        {
//            case 0x0080:                                        //电容量程 保留
//                
//            break;
                
            case 0x0082:            
                Device_Param->PCap_DataConvert->Cap_Calib.CapMin = (uint32_t)fbuf;
                Device_Param->DataFilter->InputRangeMin = (uint32_t)fbuf;            
                InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMin),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->Cap_Calib.CapMin, 
                                            sizeof(Device_Param->PCap_DataConvert->Cap_Calib.CapMin));
            break;
              
            case 0x0084:
                Device_Param->PCap_DataConvert->Cap_Calib.CapMax = (uint32_t)fbuf;
                Device_Param->DataFilter->InputRangeMax = (uint32_t)fbuf;
                InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib.CapMax),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->Cap_Calib.CapMax, 
                                            sizeof(Device_Param->PCap_DataConvert->Cap_Calib.CapMax));
            break;
#ifdef USING_DAC
            case 0x0088:
                if(Device_Param->PCap_DataConvert->CapDA_ClibEn == CLIB_ENABLE)
                {
                    Calib_DAMin = (uint16_t)fbuf;     
                    Device_Param->PCap_DataConvert_Out->PCap_DA_Value = (uint16_t)fbuf;
                }     
                else if(fbuf <= PCAP_DAC_MAX_VALUE && fbuf >= PCAP_DAC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin = (uint16_t)fbuf;              
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib.CapDAMin),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin, 
                                            sizeof(Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin));
                }
                else
                {
                    goto __error;
                }
            break;
                   
            case 0x008A:
                if(Device_Param->PCap_DataConvert->CapDA_ClibEn == CLIB_ENABLE)
                {
                    Calib_DALow = (uint16_t)fbuf;    
                    Device_Param->PCap_DataConvert_Out->PCap_DA_Value = (uint16_t)fbuf;                 
                }     
                else if(fbuf <= PCAP_DAC_MAX_VALUE && fbuf >= PCAP_DAC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow = (uint16_t)fbuf;              
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib.CapDALow),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow, 
                                            sizeof(Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow));
                }                
                else
                {
                    goto __error;
                }
            break;
              
            case 0x008C:
                if(Device_Param->PCap_DataConvert->CapDA_ClibEn == CLIB_ENABLE)
                {
                    Calib_DAHigh = (uint16_t)fbuf;        
                    Device_Param->PCap_DataConvert_Out->PCap_DA_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_DAC_MAX_VALUE && fbuf >= PCAP_DAC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh = (uint16_t)fbuf;              
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib.CapDAHigh),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh, 
                                            sizeof(Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh));
                }
                else
                {
                    goto __error;
                }
            break;
                  
            case 0x008E:                
                if(Device_Param->PCap_DataConvert->CapDA_ClibEn == CLIB_ENABLE)
                {
                    Calib_DAMax = (uint16_t)fbuf;        
                    Device_Param->PCap_DataConvert_Out->PCap_DA_Value = (uint16_t)fbuf;
                }          
                else if(fbuf <= PCAP_DAC_MAX_VALUE && fbuf >= PCAP_DAC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax = (uint16_t)fbuf;              
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib.CapDAMax),
                                            (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax, 
                                            sizeof(Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax));
                }
                else
                {
                    goto __error;
                }
            break; 
#endif // USING_DAC
#ifdef USING_ADC_TEMPER_SENSOR
            case 0x0090:                                                                
                Device_Param->ADC_TemperParam->Temper_K1 = fbuf / 100;              
                InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->Temper_K1),
                                            (uint8_t *)&Device_Param->ADC_TemperParam->Temper_K1, 
                                            sizeof(Device_Param->ADC_TemperParam->Temper_K1));
            break;
              
            case 0x0092:
                Device_Param->ADC_TemperParam->Temper_B1 = fbuf - 100;              
                InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->Temper_B1),
                                            (uint8_t *)&Device_Param->ADC_TemperParam->Temper_B1, 
                                            sizeof(Device_Param->ADC_TemperParam->Temper_B1));
            break;
              
            case 0x0094:
                Device_Param->ADC_TemperParam->Temper_K2 = fbuf / 100;              
                InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->Temper_K2),
                                            (uint8_t *)&Device_Param->ADC_TemperParam->Temper_K2, 
                                            sizeof(Device_Param->ADC_TemperParam->Temper_K2));
            break;
              
            case 0x0096:
                Device_Param->ADC_TemperParam->Temper_B2 = fbuf - 100;              
                InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->Temper_B2),
                                            (uint8_t *)&Device_Param->ADC_TemperParam->Temper_B2, 
                                            sizeof(Device_Param->ADC_TemperParam->Temper_B2));
            break;
#endif // USING_ADC_TEMPER_SENSOR
#ifdef USING_DAC
            case 0x0098:       
                if(fbuf <= PCAP_ADC_MAX_VALUE && fbuf >= PCAP_ADC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapAD_Calib.CapADMin = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapAD_Calib.CapADMin),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapAD_Calib.CapADMin, 
                                                sizeof(Device_Param->PCap_DataConvert->CapAD_Calib.CapADMin));
                }   
                else
                {
                    goto __error;
                }                
            break; 
                   
            case 0x009A:
                if(fbuf <= PCAP_ADC_MAX_VALUE && fbuf >= PCAP_ADC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapAD_Calib.CapADLow = (uint16_t)fbuf;                 
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapAD_Calib.CapADLow),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapAD_Calib.CapADLow, 
                                                sizeof(Device_Param->PCap_DataConvert->CapAD_Calib.CapADLow));
                }
                else
                {
                    goto __error;
                }
            break; 
              
            case 0x009C:
                if(fbuf <= PCAP_ADC_MAX_VALUE && fbuf >= PCAP_ADC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapAD_Calib.CapADHigh = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapAD_Calib.CapADHigh),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapAD_Calib.CapADHigh, 
                                                sizeof(Device_Param->PCap_DataConvert->CapAD_Calib.CapADHigh));
                }      
                else
                {
                    goto __error;
                }
            break; 
                   
            case 0x009E:
                if(fbuf <= PCAP_ADC_MAX_VALUE && fbuf >= PCAP_ADC_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapAD_Calib.CapADMax = (uint16_t)fbuf;      
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapAD_Calib.CapADMax),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapAD_Calib.CapADMax, 
                                                sizeof(Device_Param->PCap_DataConvert->CapAD_Calib.CapADMax));
                }         
                else
                {
                    goto __error;
                }  
            break;       
#endif // USING_DAC

#ifdef USING_PWM
            case 0x00A0:                                                            //PWM 0点
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM0 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM0),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0));
                }   
                else
                {
                    goto __error;
                }                                     
            break;
              
            case 0x00A2:                                                            //PWM 1点
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM1 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM1),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00A4:                                                            //PWM 2点
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM2 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM2),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2));
                }   
                else
                {
                    goto __error;
                }         
            break;  
              
            case 0x00A6:                                                            //PWM 3点
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM3 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM3),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3));
                }   
                else
                {
                    goto __error;
                }         
            break; 
            
            case 0x00A8:                                                            //PWM 4点
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM4 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM4),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00AA:                                                            //PWM 5点
                if(Device_Param->PCap_DataConvert->CapPWM_ClibEn == CLIB_ENABLE)     
                {
                    Calib_PWM5 = (uint16_t)fbuf;  
                    Device_Param->PCap_DataConvert_Out->PCap_PWM_Value = (uint16_t)fbuf;
                }
                else if(fbuf <= PCAP_PWM_MAX_VALUE && fbuf >= PCAP_PWM_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib.CapPWM5),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00AC:                                                            //PWM 0点高度
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High0 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High0),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High0, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High0));
                }   
                else
                {
                    goto __error;
                }         
            break;  
              
            case 0x00AE:                                                            //PWM 1点高度
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High1 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High1),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High1, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High1));
                }   
                else
                {
                    goto __error;
                }         
            break; 
            
            case 0x00B0:                                                            //PWM 2点高度
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High2 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High2),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High2, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High2));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00B2:                                                            //PWM 3点高度
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High3 = (uint16_t)fbuf;          
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High3),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High3, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High3));
                }   
                else
                {
                    goto __error;
                }         
            break;
              
            case 0x00B4:                                                            //PWM 4点高度
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High4 = (uint16_t)fbuf;         
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High4),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High4, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High4));
                }   
                else
                {
                    goto __error;
                }         
            break;  
              
            case 0x00B6:                                                            //PWM 5点高度
                if(fbuf <= PCAP_PWM_HIGH_MAX_VALUE && fbuf >= PCAP_PWM_HIGH_MIN_VALUE)
                {
                    Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High5 = (uint16_t)fbuf;        
                    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_High_Calib.CapPWM_High5),
                                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High5, 
                                                sizeof(Device_Param->PCap_DataConvert->CapPWM_High_Calib.CapPWM_High5));
                }   
                else
                {
                    goto __error;
                }         
            break; 

            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }
        index += DataLen;
#endif // USING_PWM
    }
    return;
__error:
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3; 
}

///**@brief       Modbus 2A功能码消息帧处理
//* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
//* @return       函数执行结果
//* - None
//*/
//void ModbusFunc2A(ModBusBaseParam_TypeDef *ModBusBaseParam)
//{
//    uint8_t i;
//    uint8_t j;
//    uint8_t objlen;
//    uint16_t RegNum;
//    //错误状态
//    uint8_t Err_Status;
//    uint16_t WriteAddr;

//    //写预处理
//    ModBus_WritePreHandle(ModBusBaseParam, &WriteAddr, &RegNum, &Err_Status);    
//    //内部ROM未使能访问失败
//    if(IN_MEMORY_WR_ENABLE != ModBusBaseParam->InRomWrEn)
//    {
//        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
//        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_DEVC_EXCEPTION;
//        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
//        return;
//    }
//    if( Err_Status != OP_SUCCESS 
//        || WriteAddr < MUL_VERSION_INF_BEGIN 
//        || (WriteAddr + RegNum) > (MUL_VERSION_INF_END + 1)
//        || RegNum == 0)
//    {
//        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
//        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
//        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
//        return;
//    }

//    j = 6;
//    for(i = 0; i < RegNum; i++, WriteAddr++)
//    {
//        objlen = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j];
//        switch(WriteAddr)
//        {
//            //机构名称
//            case 0x00E0:            
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(ORGANIZATION, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                                    
//                j += objlen;
//            break;
//            //产品代号    
//            case 0x00E1:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(PRODUCTION, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                               
//                j += objlen;
//            break;
//            //硬件版本    
//            case 0x00E2:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(HARDWAREVER, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                                  
//                j += objlen;
//            break;
//            //软件版本    
//            case 0x00E3:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(SOFTWAREVER, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                                  
//                j += objlen;
//            break;
//            //设备ID    
//            case 0x00E4:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(DEVICENUM, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                                     
//                j += objlen;
//            break;
//            //客户编码    
//            case 0x00E5:
//                if(30 < objlen)
//                {
//                    goto __error;
//                }
//                InMemory_Write_MultiBytes(CUSTOMER, 
//                    (const uint8_t *)&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[j], objlen);                               
//                j += objlen;
//            break;
//                
//            default:
//                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
//                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
//                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
//                return;
//        }        
//    }
//    return;
//__error:
//    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
//    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
//    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3; 
//}

/**@brief       Modbus 2B功能码消息帧处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @return       函数执行结果
* - None
*/
void ModbusFunc2B(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
    uint8_t i;
    uint8_t objlen;
    uint16_t RegNum;
    uint16_t ReadAddr;
    //错误状态
    uint8_t Err_Status;

    //读预处理
    ModBus_ReadPreHandle(ModBusBaseParam, &ReadAddr, &RegNum, &Err_Status);    
    
    if( Err_Status != OP_SUCCESS 
        || ReadAddr < MUL_VERSION_INF_BEGIN
        || (ReadAddr + RegNum) > (MUL_VERSION_INF_END + 1)
        || RegNum == 0)
    {
        ReadAddr = 0xFFFF;
    }

    memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[2], &ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4], 2);
    memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[4], &ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2], 2);
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 6;
    for(i = 0; i < RegNum; i++, ReadAddr++)
    {
        switch(ReadAddr)
        {
            //机构名称  
            case 0x00E0:
//                objlen = InMemory_Read_OneByte(ORGANIZATION);                         
                objlen = Company_Name[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((ORGANIZATION + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Company_Name[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //产品代号  
            case 0x00E1:
//                objlen = InMemory_Read_OneByte(PRODUCTION);                 
                objlen = Product_Code[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((PRODUCTION + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Product_Code[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //硬件版本  
            case 0x00E2:
//                objlen = InMemory_Read_OneByte(HARDWAREVER);  
                objlen = Hardware_Version[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((HARDWAREVER + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Hardware_Version[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //软件版本 
            case 0x00E3:                        
//                objlen = InMemory_Read_OneByte(SOFTWAREVER);
                objlen = Software_Version[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((SOFTWAREVER + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Software_Version[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //设备ID  
            case 0x00E4:
//                objlen = InMemory_Read_OneByte(DEVICENUM);   
                objlen = Device_ID[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((DEVICENUM + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Device_ID[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;
            //客户编码  
            case 0x00E5:
//                objlen = InMemory_Read_OneByte(CUSTOMER);       
                objlen = Customer_Code[0];
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = objlen;
                if((objlen > (SEND_SIZE * 2 / 3))||(0 == objlen))
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len - 1] = 1;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0;
                    break;
                }
//                InMemory_Read_MultiBytes((CUSTOMER + 1), 
//                    &ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len], objlen);
                memcpy(&ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len],
                        &Customer_Code[1],
                        objlen);
                ModBusBaseParam->ModBus_TX_RX.Send_Len += objlen;
            break;

            default:
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
                ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
                return;
        }        
    }
}
#endif // BOOT_PROGRAM

/* 使用soway上位机升级程序(Boot程序), BOOT_PROGRAM在main.h中定义 */
#ifdef BOOT_PROGRAM
#include "common.h"
#include "flash_if.h"
#endif // BOOT_PROGRAM

/**@brief       Modbus 41功能码消息帧处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @return       函数执行结果
* - None
*/
void ModbusFunc41(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
/* 使用soway上位机升级程序(Boot程序), BOOT_PROGRAM在main.h中定义 */
#ifdef BOOT_PROGRAM
    
    uint16_t WriteAddr;                     //寄存器地址    
    uint16_t DataLen;                       //数据长度
    /* 擦除保存在内部EEPROM的参数标志, 对于STM32L0系列以外的芯片则表示擦除保存在内部FLASH的参数标志 */
    uint16_t Erase_EEPROM_Flag = 0;         
    uint16_t packetnum;                     //总包数
    uint16_t packetcnt;                     //包序号
    uint16_t prt;                           //计数
    uint32_t tpcksum;                       //包校验和
    uint32_t *ramdata;                      //数据指针
    static uint16_t PacketCnt;              // 包序号
    static uint16_t PacketNum;              // 总包数
    static uint32_t Flashadrdst;            // FLASH地址
    static uint32_t FileCheckSum;           // 升级文件校验和
    static uint32_t FileRunCheckSum;        // 升级文件实时校验和
    
    WriteAddr = (ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2] << 8)
                | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[3];
    DataLen = (ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4] << 8)
                | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];
    
    //寄存器地址无效
    if( 
#if defined(SUBCODE_IS_DEVADDR)
        (WriteAddr >> 8) != ModBusBaseParam->Device_Addr ||
#endif // defined(SUBCODE_IS_DEVADDR)
        (WriteAddr & 0xFF) < 1 
        || (WriteAddr & 0xFF) > 4
        || (DataLen != (ModBusBaseParam->ModBus_TX_RX.Receive_Len - 6)))
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] |= MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_ADDR_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        return;
    }
    
    WriteAddr &= 0x00FF;
    
    //存储待发送信息
    memcpy( ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
            ModBusBaseParam->ModBus_TX_RX.Receive_Buf,
            4);
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 4;

    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0x00;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = 0x01;

    switch(WriteAddr)
    {
        case 0x0001:                                                                        //开始升级      
            ModBusBaseParam->UpgradeWaitTime = -1;        
            if((0 != DataLen) && (2 != DataLen))
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
                break;
            }
            else 
            {
                if(DataLen == 2)
                {
                    Erase_EEPROM_Flag = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[6] * 256 
                                        + ModBusBaseParam->ModBus_TX_RX.Receive_Buf[7];    //获取擦除标志
                }
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
            }  
            PacketNum = 0;
            PacketCnt = 0;

            if(Erase_EEPROM_Flag)
            {         
            #if defined(STM32F0)                
                InFlash_Erase_Page(0, 4);
            #elif defined(STM32L0)
                InEEPROM_Erase_MultiWord(0, (IN_EEPROM_END - IN_EEPROM_START + 1) / 4);
            #endif // defined(STM32F0) or defined(STM32L0)
            }
            break;
          
        case 0x0002:                                                                                //清除源程序
            if(0 != DataLen)
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
                break;
            }          
            FLASH_If_Init();                                                                          //FLASH解锁
            if(FLASH_If_GetWriteProtectionStatus() != FLASHIF_PROTECTION_NONE)
            {
                if(FLASH_If_WriteProtectionConfig(FLASHIF_WRP_DISABLE) == FLASHIF_OK)
                {
                    HAL_FLASH_OB_Launch();
                }
            }
            FLASH_If_Erase(APPLICATION_ADDRESS);
            Flashadrdst = APPLICATION_ADDRESS;
            ModBusBaseParam->ProgErase = ERASE_FLAG;
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
            InMemory_Write_OneByte(ADDR_ERASEFLAG, ERASE_FLAG);
            break;

        case 0x0003:                //传输升级文件
            packetnum = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[6] * 256 + ModBusBaseParam->ModBus_TX_RX.Receive_Buf[7];    //获取总包数
            packetcnt = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[8] * 256 + ModBusBaseParam->ModBus_TX_RX.Receive_Buf[9];
            if((0 == PacketNum) && (1 < packetnum) && (0 == packetcnt))
            {
                FileCheckSum = 0;
                FileRunCheckSum = 0;
                PacketNum = packetnum;
                PacketCnt = packetcnt;
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;

                for(prt = 0; prt < 4; prt++)
                {
                    FileCheckSum <<= 8;
                    FileCheckSum += ModBusBaseParam->ModBus_TX_RX.Receive_Buf[10 + prt];
                }
            }
            else if((PacketNum == packetnum) && (1 < packetnum) 
                && (PacketCnt == (packetcnt - 1)) && (PacketNum > packetcnt))
            {
                tpcksum = 0;
                DataLen = DataLen - 4;

                for(prt = 0; prt < DataLen; prt++)
                {
                    tpcksum += ModBusBaseParam->ModBus_TX_RX.Receive_Buf[10 + prt];
                }

                Decoding(&ModBusBaseParam->ModBus_TX_RX.Receive_Buf[10], DataLen);

                for(prt = 0; prt < DataLen; prt++)
                {
                    ModBusBaseParam->ModBus_TX_RX.Receive_Buf[prt] = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[10 + prt];
                }
                ramdata = (uint32_t*)ModBusBaseParam->ModBus_TX_RX.Receive_Buf;

                if(FLASH_If_Write(Flashadrdst, ramdata, DataLen/4)  == 0)
                {
                    PacketCnt++;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
                    FileRunCheckSum += tpcksum;
                    Flashadrdst += DataLen;
                }
                else
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
                }
            }
            else if((PacketNum == packetnum) && (1 < packetnum) 
                && (PacketCnt == packetcnt) && (PacketNum > packetcnt))
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
            }
            else
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
            }
            break;

        case 0x0004:                                                                                  //执行应用程序
            if((((FileRunCheckSum == FileCheckSum) && ((PacketCnt + 1) == PacketNum)) 
                || (0 == PacketNum))&&(0 == DataLen))
            {
                if(0 != PacketNum)
                {
                    ModBusBaseParam->UpgradeWaitTime = 0;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
                }
                else if(ModBusBaseParam->ProgErase == ERASE_FLAG)
                {
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
                }
                else
                {
                    ModBusBaseParam->UpgradeWaitTime = 0;
                    ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_ERR_NONE;
                }
            }
            else
            {
                ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_LRC_ERR;
            }
            break;

        default:
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] = RESPONSE_REC_ERR;
            break;
    }
    
#else
    uint16_t temp[2];    
    
    temp[0] = ((uint16_t)ModBusBaseParam->ModBus_TX_RX.Receive_Buf[2] << 8) 
                | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[3];
    temp[1] = ((uint16_t)ModBusBaseParam->ModBus_TX_RX.Receive_Buf[4] << 8) 
                | ModBusBaseParam->ModBus_TX_RX.Receive_Buf[5];
    if(temp[0] != 0x0001 || temp[1] != 0x0000)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[0] = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[0];
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[1] 
                                                    | MB_REQ_FAILURE;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_VALU_EXCEPTION;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
    }
    else
    {
        InMemory_Write_OneByte(ADDR_UPGRADEFLAG, 0xFF);
        memcpy(ModBusBaseParam->ModBus_TX_RX.Send_Buf, ModBusBaseParam->ModBus_TX_RX.Receive_Buf, 5);
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[5] = 0x01;
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[6] = 0x00;
        ModBusBaseParam->ModBus_TX_RX.Send_Len = 7;        
        ModBusBaseParam->ModBus_CallBack = MB_System_Reset;
    }
#endif // BOOT_PROGRAM
}

/* 使用soway上位机升级程序(Boot程序), BOOT_PROGRAM在main.h中定义 */
#ifndef BOOT_PROGRAM

/**@brief       Modbus 消息帧自动上传处理
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void ModbusAutoUpload(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    float fbuf;
    uint8_t Nr;
    uint32_t DataBuf;
    //设备参数
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;

    ModBusBaseParam->ModBus_TX_RX.Send_Buf[0] = ModBusBaseParam->Device_Addr;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] = 0x04;
    ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = 0x08;

    ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
    fbuf = (float)Device_Param->PCap_DataConvert_Out->LiquidHeightAD;
    DataBuf = *(uint32_t *)&fbuf;
    for(Nr = 4; Nr > 0; Nr--)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)(DataBuf >> ((Nr - 1)*8));
    }

    DataBuf = *(uint32_t *)&Device_Param->PCap_DataConvert_Out->PCap_Temper_Value;
    for(Nr = 4; Nr > 0; Nr--)
    {
        ModBusBaseParam->ModBus_TX_RX.Send_Buf[ModBusBaseParam->ModBus_TX_RX.Send_Len++] 
            = (uint8_t)(DataBuf >> ((Nr - 1)*8));
    }
#if defined(USING_MODBUS_RTU)
    MODBUS_RTU_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        NO_CHECK_ADDRESS);
#elif defined(USING_MODBUS_ASCII)
    MODBUS_ASCII_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        NO_CHECK_ADDRESS);
#endif // defined(USING_MODBUS_RTU) or defined(USING_MODBUS_ASCII)
}

/**@brief       电容标定
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void MB_Cap_Calibration(void *arg)
{
    //设备参数
    ModBus_Device_Param *Device_Param;
        
    Device_Param = (ModBus_Device_Param *)arg;
    
    if(Calib_CapMin < Calib_CapMax)
    {
        //电容零点
        Device_Param->PCap_DataConvert->Cap_Calib.CapMin = Calib_CapMin;
        //电容满点
        Device_Param->PCap_DataConvert->Cap_Calib.CapMax = Calib_CapMax;
        
        Device_Param->DataFilter->InputRangeMin = Calib_CapMin;
        Device_Param->DataFilter->InputRangeMax = Calib_CapMax;
        //电容标定数据写入内部Flash
        InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->Cap_Calib),
                                    (uint8_t *)&Device_Param->PCap_DataConvert->Cap_Calib, 
                                    sizeof(Device_Param->PCap_DataConvert->Cap_Calib));
    }
}

#ifdef USING_DAC
/**@brief       电容DA值标定
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void MB_CapDAOut_Calibration(void *arg)
{
    //设备参数
    ModBus_Device_Param *Device_Param;
    
    Device_Param = (ModBus_Device_Param *)arg;
    if(CapDA_ClibFlag == (CALIB_CAPDAMAX_FLAG | CALIB_CAPDAMIN_FLAG))
    {
        if(Calib_DAMax <= Calib_DAMin)
        {
            return;
        }
        Calib_DAHigh = 0;
        Calib_DALow = 0;        
    }
    else
    {
        if((Calib_DAMax <= Calib_DAHigh) 
            || (Calib_DAHigh <= Calib_DALow) 
            || (Calib_DALow <= Calib_DAMin))
        {
            return;
        }
    }    
    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMin = Calib_DAMin;
    Device_Param->PCap_DataConvert->CapDA_Calib.CapDALow = Calib_DALow;
    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAHigh = Calib_DAHigh;
    Device_Param->PCap_DataConvert->CapDA_Calib.CapDAMax = Calib_DAMax;                
    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapDA_Calib),
                                (uint8_t *)&Device_Param->PCap_DataConvert->CapDA_Calib, 
                                sizeof(Device_Param->PCap_DataConvert->CapDA_Calib));            
}
#endif // USING_DAC

#ifdef USING_ADC_TEMPER_SENSOR
/**@brief       温度DA值标定功能
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void MB_TempDAOut_Calibration(void *arg)
{
    //设备参数
    ModBus_Device_Param *Device_Param;
    
    Device_Param = (ModBus_Device_Param *)arg;
    //如果温度DA标定值有效就写入内部Flash
    if((Calib_TempDAMax > Calib_TempDAMin) && (Calib_TempDAMax <= DAC_VALUE_MAX))
    {
        Device_Param->ADC_TemperParam->ADC_Temper_Calib.TempDAMin = Calib_TempDAMin;
        Device_Param->ADC_TemperParam->ADC_Temper_Calib.TempDAMax = Calib_TempDAMax;
        Device_Param->ADC_TemperParam->TempDARange = Device_Param->ADC_TemperParam->ADC_Temper_Calib.TempDAMax 
                                                - Device_Param->ADC_TemperParam->ADC_Temper_Calib.TempDAMin;
        InMemory_Write2T_MultiBytes((uint32_t)&(((ADC_TemperParam_TypeDef *)ADC_TEMPERPARAM_BASE_ADDRESS)->ADC_Temper_Calib),
                                    (uint8_t *)&Device_Param->ADC_TemperParam->ADC_Temper_Calib, 
                                    sizeof(Device_Param->ADC_TemperParam->ADC_Temper_Calib));               
    }
}
#endif // USING_ADC_TEMPER_SENSOR

#ifdef USING_PWM
/**@brief       电容PWM值标定
* @param[in]    arg : 用户自定义的参数,这里为设备参数
* @return       函数执行结果
* - None
*/
void MB_CapPWMOut_Calibration(void *arg)
{
    //设备参数
    ModBus_Device_Param *Device_Param;
    
    Device_Param = (ModBus_Device_Param *)arg;
    if(CapPWM_ClibFlag == CALIB_CAPPWMEOC_FLAG)
    {
        if(Calib_PWM0 <= Calib_PWM1 || Calib_PWM1 <= Calib_PWM2
             || Calib_PWM2 <= Calib_PWM3 || Calib_PWM3 <= Calib_PWM4
             || Calib_PWM4 <= Calib_PWM5)
        {
            return;
        }     
    }
    else
    {
        return;
    }                    
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM0 = Calib_PWM0;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM1 = Calib_PWM1;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM2 = Calib_PWM2;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM3 = Calib_PWM3;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM4 = Calib_PWM4;
    Device_Param->PCap_DataConvert->CapPWM_Calib.CapPWM5 = Calib_PWM5;
    InMemory_Write2T_MultiBytes((uint32_t)&(((PCap_DataConvert_Param *)PCAP_PARAM_BASE_ADDRESS)->CapPWM_Calib),
                                (uint8_t *)&Device_Param->PCap_DataConvert->CapPWM_Calib, 
                                sizeof(Device_Param->PCap_DataConvert->CapPWM_Calib));      
}

#endif // USING_PWM

/**@brief       Modbus 串口重新初始化
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;  
* @return       函数执行结果
* - int整型值(OP_SUCCESS)
* @note         本函数赋给ModBusBaseParam->ModBus_CallBack后,可在ModBus响应上位机后调用
*/
static int MB_USART_ReInit(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{        
/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS
    rt_sem_take(ModBusBaseParam->TX_Lock, RT_WAITING_FOREVER);    //获取信号量
#else
    while(Sensor_USART_Get_TX_Cplt_Flag() == 0);    //等待串口发送完毕
#endif // USING_RT_THREAD_OS
    
    UNUSED(arg);
    Sensor_USART_Init(  ModBusBaseParam->BaudRate, 
                        ModBusBaseParam->Parity);
    
/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS
    rt_sem_release(ModBusBaseParam->TX_Lock);    //释放信号量
#endif  // USING_RT_THREAD_OS
    return OP_SUCCESS;
}

/**@brief       Modbus 设备冻结
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;  
* @return       函数执行结果
* - int整型值(OP_SUCCESS)
* @note         本函数赋给ModBusBaseParam->ModBus_CallBack后,可在ModBus响应上位机后调用
*/
static int MB_Freeze(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{           
    UNUSED(arg);
    ModBusBaseParam->Freeze = FREEZE_ENABLE;
    
    return OP_SUCCESS;
}

/**@brief       系统复位
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;  
* @return       函数执行结果
* - int整型值(OP_SUCCESS)
* @note         本函数赋给ModBusBaseParam->ModBus_CallBack后,可在ModBus响应上位机后调用
*/
static int MB_System_Reset(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS
    rt_sem_take(ModBusBaseParam->TX_Lock, RT_WAITING_FOREVER);    //获取信号量
#else
    while(Sensor_USART_Get_TX_Cplt_Flag() == 0);    //等待串口发送完毕
#endif // USING_RT_THREAD_OS
    
    UNUSED(arg);
    HAL_NVIC_SystemReset();
    
    return OP_SUCCESS;
}

/**@brief       发送ModBus数据且不检查设备地址
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;  
* @return       函数执行结果
* - int整型值(OP_SUCCESS)
* @note         本函数赋给ModBusBaseParam->ModBus_CallBack后,可在ModBus响应上位机后调用
*/
static int MB_SendData_NoCheck(struct _ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    //发送Modbus RTU
#if defined(USING_MODBUS_RTU)
    MODBUS_RTU_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        NO_CHECK_ADDRESS);
#elif defined(USING_MODBUS_ASCII)
    UNUSED(arg);
    MODBUS_ASCII_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        NO_CHECK_ADDRESS);
#endif // defined(USING_MODBUS_RTU) or defined(USING_MODBUS_ASCII)
    
    return OP_SUCCESS;
}

uint8_t System_Param_Buf[SYSTEM_PARAM_LEN];

/**@brief       将当前所有参数作备份
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;  
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         本函数赋给ModBusBaseParam->ModBus_CallBack后,可在ModBus响应上位机后调用
*/
static uint8_t BankUp_Data(void)
{
    InMemory_Read_MultiBytes(SYSTEM_PARAM_BASE_ADDRESS, System_Param_Buf, SYSTEM_PARAM_LEN);
    
    return InMemory_Write_MultiBytes(SYSTEM_PARAM_BAK2_BASE_ADDRESS,
                                    System_Param_Buf,
                                    SYSTEM_PARAM_LEN);
}

/**@brief       恢复出厂数据(将备份参数读取出来覆盖当前系统参数)
* @param[in]    ModBusBaseParam : ModBus处理的基本参数结构体;  
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         本函数赋给ModBusBaseParam->ModBus_CallBack后,可在ModBus响应上位机后调用
*/
static uint8_t Factory_Data_Reset(void)
{
    InMemory_Read_MultiBytes(SYSTEM_PARAM_BAK2_BASE_ADDRESS, System_Param_Buf, SYSTEM_PARAM_LEN);
    
    return InMemory_Write2T_MultiBytes(SYSTEM_PARAM_BASE_ADDRESS,
                                        System_Param_Buf,
                                        SYSTEM_PARAM_LEN);
}


#endif // BOOT_PROGRAM

#endif // __PICOCAP_APP_H
