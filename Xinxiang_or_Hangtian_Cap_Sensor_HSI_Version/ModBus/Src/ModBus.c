/**@file        Modbus.c
* @brief        Modbus ͨ��Э��
* @details      �궨���ܡ��ɼ����ݽ���
* @author       ׯ��Ⱥ
* @date         2020-08-05
* @version      V2.1.0
* @copyright    2020-2030,��������Ϊ�Ƽ���չ���޹�˾
**********************************************************************************
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author  <th>Maintainer  <th>Description
* <tr><td>2020/08/05  <td>2.1.0    <td>ׯ��Ⱥ  <td>���      <td>ɾ�����Զ��ϴ��߳�, 
* ʹ�� ModBus �����̴߳�����Ϣ֡���Զ��ϴ�
* </table>
*
**********************************************************************************
*/

#include "ModBus.h"


#if defined(USING_MODBUS_RTU)
#include "ModBus_RTU.h"
#elif defined(USING_MODBUS_ASCII)
#include "ModBus_ASCII.h"
#endif // defined(USING_MODBUS_RTU) or defined(USING_MODBUS_ASCII)


static uint8_t ModBus_Receive_Buf[RECEIVE_SIZE];    ///< ModBus���ջ���(ModBus��������ʱ,ʹ���������)
static uint8_t ModBus_Send_Buf[SEND_SIZE];          ///< ModBus���ͻ���



/**@brief       ModBus��ʼ��
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��
* @return       ����ִ�н��
* - None
*/
void ModBus_Init(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
   
    InMemory_Read_MultiBytes(MODBUS_PARAM_BASE_ADDRESS, 
                            (uint8_t *)ModBusBaseParam, 
                            sizeof(ModBusBaseParam_TypeDef));               //��ModBus����
/* ʹ��soway��λ����������(Boot����), BOOT_PROGRAM��main.h�ж��� */
#ifdef BOOT_PROGRAM 
    ModBusBaseParam->ProgErase      = InMemory_Read_OneByte(ADDR_ERASEFLAG);
    ModBusBaseParam->BaudRate       = USART_BAUDRATE_9600_CODE;
    ModBusBaseParam->Parity         = USART_PARITY_NONE_CODE;
#endif // BOOT_PROGRAM
        
    ModBusBaseParam->ModBus_CallBack = NULL;                                //ModBus�ص�������ΪNULL
    
    ModBusBaseParam->ModBus_TX_RX.Receive_Buf = ModBus_Receive_Buf;         //ModBus�������ݻ���
    ModBusBaseParam->ModBus_TX_RX.Send_Buf = ModBus_Send_Buf;               //ModBus�������ݻ���
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 0;                             //ModBus�������ݵĳ���
    
    Sensor_USART_Init(ModBusBaseParam->BaudRate, ModBusBaseParam->Parity);  //��ʼ������
}


/**@brief       ��������
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    Msg : ��Ϣ�׵�ַ;
* @param[in]    MsgLen : ��Ϣ֡����;
* @return       ����ִ�н��
* - OP_SUCCESS(�ɹ�)
* - OP_FAILED(ʧ��)
*/
uint8_t Send_Data(ModBusBaseParam_TypeDef *ModBusBaseParam, uint8_t *pMsg, uint16_t len)
{
#ifdef  USART_USING_485
    //����ʹ��
    TX_ON;
#endif // USART_USING_485

    while(Sensor_USART_Get_TX_Cplt_Flag() == 0);
    Sensor_USART_Clear_TX_Cplt_Flag();

    //DMA����
    return BSP_UART_Transmit_DMA(pMsg, len);        
}

/**@brief       Modbus��Ϣ֡��������
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,�����豸����
* @return       ����ִ�н��
* - None
*/
void ModbusHandle(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    uint8_t recv_status;
        
#if defined(USING_MODBUS_RTU)
    //��ȡһ֡ModBus RTU����
    recv_status = MODBUS_RTU_RecvData(ModBusBaseParam->ModBus_TX_RX.Receive_Buf, 
                                    &ModBusBaseParam->ModBus_TX_RX.Receive_Len);
#elif defined(USING_MODBUS_ASCII)
    //��ȡһ֡ModBus ASCII����
    recv_status = MODBUS_ASCII_RecvData(ModBusBaseParam->ModBus_TX_RX.Receive_Buf, 
                                    &ModBusBaseParam->ModBus_TX_RX.Receive_Len);
#endif // defined(USING_MODBUS_RTU) or defined(USING_MODBUS_ASCII)
    //У����󣬵�ַ���󶼲�������Ϣ֡
    if( recv_status != ERR_NONE 
        || ((ModBusBaseParam->Device_Addr != ModBusBaseParam->ModBus_TX_RX.Receive_Buf[0]) 
/* ʹ��soway��λ����������(Boot����), BOOT_PROGRAM��main.h�ж��� */
#ifdef BOOT_PROGRAM 
        && (0xFF != ModBusBaseParam->ModBus_TX_RX.Receive_Buf[0])
#endif // BOOT_PROGRAM
        && (BROADCASTADDR != ModBusBaseParam->ModBus_TX_RX.Receive_Buf[0])))
    {     
        return;
    }
    switch(ModBusBaseParam->ModBus_TX_RX.Receive_Buf[1])    //�����봦��
    {
        case 0x03:
            ModbusFunc03(ModBusBaseParam, arg);
        break;

        case 0x04:
            ModbusFunc04(ModBusBaseParam, arg);
        break;

        case 0x05:
            ModbusFunc05(ModBusBaseParam, arg);
        break;
     
        case 0x10:
            ModbusFunc10(ModBusBaseParam, arg);
        break;

        case 0x25:
            ModbusFunc25(ModBusBaseParam, arg);
        break;
      
        case 0x26:
            ModbusFunc26(ModBusBaseParam, arg);
        break;
       
        case 0x27:
            ModbusFunc27(ModBusBaseParam, arg);
        break;
     
        case 0x2A:
            ModbusFunc2A(ModBusBaseParam);
        break;

        case 0x2B:
            ModbusFunc2B(ModBusBaseParam);
        break;
          
        case 0x41:
            ModbusFunc41(ModBusBaseParam);
        break;   

        default:
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[0] 
                = ModBusBaseParam->ModBus_TX_RX.Receive_Buf[0];
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[1] 
                = (ModBusBaseParam->ModBus_TX_RX.Receive_Buf[1] | MB_REQ_FAILURE);
            ModBusBaseParam->ModBus_TX_RX.Send_Buf[2] = MB_FUNC_EXCEPTION;
            ModBusBaseParam->ModBus_TX_RX.Send_Len = 3;
        break;
    }
#if defined(USING_MODBUS_RTU)
    //����һ֡ModBus RTU����
    MODBUS_RTU_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        CHECK_ADDRESS);
#elif defined(USING_MODBUS_ASCII)
    //����һ֡ModBus ASCII����
    MODBUS_ASCII_SendData(ModBusBaseParam, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Buf, 
                        ModBusBaseParam->ModBus_TX_RX.Send_Len,
                        CHECK_ADDRESS);
#endif // defined(USING_MODBUS_RTU) or defined(USING_MODBUS_ASCII)   
    if(ModBusBaseParam->ModBus_CallBack != NULL)
    {        
        ModBusBaseParam->ModBus_CallBack(ModBusBaseParam, arg);
        ModBusBaseParam->ModBus_CallBack = NULL;
    }    
    ModBusBaseParam->ModBus_TX_RX.Send_Len = 0;
}

/**@brief       Modbus 03��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,�����豸����
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc03(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    UNUSED(ModBusBaseParam);
    UNUSED(arg);
}

/**@brief       Modbus 04��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,�����豸����
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc04(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    UNUSED(ModBusBaseParam);
    UNUSED(arg);
}

/**@brief       Modbus 05��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,�����豸����
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc05(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    UNUSED(ModBusBaseParam);
    UNUSED(arg);
}

/**@brief       Modbus 10��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,�����豸����
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc10(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    UNUSED(ModBusBaseParam);
    UNUSED(arg);
}

/**@brief       Modbus 25��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,�����豸����
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc25(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    UNUSED(ModBusBaseParam);
    UNUSED(arg);
}

/**@brief       Modbus 26��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,�����豸����
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc26(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    UNUSED(ModBusBaseParam);
    UNUSED(arg);
}

/**@brief       Modbus 27��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,�����豸����
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc27(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    UNUSED(ModBusBaseParam);
    UNUSED(arg);
}

/**@brief       Modbus 2A��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc2A(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
    UNUSED(ModBusBaseParam);
}

/**@brief       Modbus 2B��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc2B(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
    UNUSED(ModBusBaseParam);
}

/**@brief       Modbus 41��������Ϣ֡����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusFunc41(ModBusBaseParam_TypeDef *ModBusBaseParam)
{
    UNUSED(ModBusBaseParam);
}

/**@brief       Modbus ��Ϣ֡�Զ��ϴ�����
* @param[in]    ModBusBaseParam : ModBus�����Ļ��������ṹ��;
* @param[in]    arg : �û��Զ���Ĳ���,�����豸����
* @return       ����ִ�н��
* - None
* @note         ����ʹ��������,ʵ�ʺ�����ModBus_Conf.c���涨��
*/
__weak void ModbusAutoUpload(ModBusBaseParam_TypeDef *ModBusBaseParam, void *arg)
{
    UNUSED(ModBusBaseParam);
    UNUSED(arg);
}

/* ʹ��RT-Thread����ϵͳ,USING_RT_THREAD_OS��main.h�ж��� */
#ifdef USING_RT_THREAD_OS
#include "rtthread.h"

#define MODBUS_HANDLE_THREAD_STACK          512
#define MODBUS_HANDLE_THREAD_PRIORITY       RT_THREAD_PRIORITY_MAX - 4

static ModBusBaseParam_TypeDef     ModBusBase_Param;     //ModBus�����Ļ��������ṹ
static ModBus_Device_Param         ModBus_Device;       //ModBus�����豸�Ĳ����ṹ��


/**@brief       ModBus�����߳�,���ڴ���ModBus��Ϣ֡
* @param[in]    parameter : �̵߳Ĳ���,���ﲻʹ��
* @return       ����ִ�н��
* - None
*/
void modbus_handle_thread_entry(void *parameter)
{                
    while(1)
    {
        // �жϴ����Ƿ������ݽ��ղ�����ȴ�ʱ��(��λΪ ms)��������ĵȴ�ʱ��ת����ϵͳʱ�ӽ���
        if(Sensor_USART_Get_RX_Updata_Flag(
            rt_tick_from_millisecond(
            (ModBusBase_Param.AutoUpload ? ModBusBase_Param.AutoUpload : 1) // �ж��Զ��ϴ������Ƿ�Ϊ0����Ϊ0��ʹ��
            * 1000)))                                                       // �Զ��ϴ����� * 1000 ms ������ʹ�� 1000 ms
        {           
            // Modbus ��Ϣ֡����
            ModbusHandle(&ModBusBase_Param, &ModBus_Device);            
            Sensor_USART_Clear_RX_Updata_Flag();                        
        } 
        else if(ModBusBase_Param.AutoUpload != 0)
        {
            // ModBus ֡�Զ��ϴ�
            ModbusAutoUpload(&ModBusBase_Param, &ModBus_Device); 
        }
    }
}

/**@brief       ModBus������ʼ��,����ModBus�����̺߳�ModBus�Զ��ϴ������̲߳�����
* @return       ����ִ�н��
* - int����ֵ(RT_EOK)
* @note         ������ʹ��RT-Thread���Զ���ʼ�����INIT_COMPONENT_EXPORT
* ����ִ��,ϵͳ��λ���Զ���ʼ��
*/
static int modbus_init(void)
{
#ifdef USING_ADC_TEMPER_SENSOR
    rt_device_t                 adc_device;                 // ADC�豸
#endif // USING_ADC_TEMPER_SENSOR
    rt_device_t                 pcap_device;                // Pcap�豸
#ifdef USING_ADC_TEMPER_SENSOR
    struct rt_adc_device_obj    *adc_device_obj;            // ADC�豸����(�����豸�������������ֵ)
#endif // USING_ADC_TEMPER_SENSOR
    struct rt_pcap_device_obj   *pcap_device_obj;           // Pcap�豸����(�����豸�������������ֵ)
    rt_thread_t                 modbus_handle_thread;       // ModBus �����߳̾��
    
/************************************** ����ADC�豸����(����) ***************************************/
    // ����ADC�豸
#ifdef USING_ADC_TEMPER_SENSOR
    adc_device = rt_device_find(ADC_DEVICE_NAME);           
    if(adc_device != RT_NULL)
    {   
        // ��ȡADC�豸������Ϣ
        adc_device_obj = (struct rt_adc_device_obj *)adc_device->user_data; 
        // ��(����)ADC�豸
        rt_device_open(adc_device, RT_DEVICE_OFLAG_RDONLY); 
    }
#endif // USING_ADC_TEMPER_SENSOR
/*******************************************************************************************************/
    
    
/************************************** ����Pcap�豸����(����) ***************************************/
    // ����Pcap�豸
    pcap_device = rt_device_find(PCAP_DEVICE_NAME);         
    if(pcap_device != RT_NULL)
    {
        // ��ȡPcap�豸������Ϣ
        pcap_device_obj = (struct rt_pcap_device_obj *)pcap_device->user_data;  
        // ��(����)Pcap�豸
        rt_device_open(pcap_device, RT_DEVICE_OFLAG_RDWR);  
    }  
/*******************************************************************************************************/      
           
    
/******************************************* �豸������ʼ�� ********************************************/
    /* Ϊ ModBus ���� ADC �� Pcap �豸���� */
#ifdef USING_ADC_TEMPER_SENSOR
    if(adc_device != RT_NULL)
    {
        ModBus_Device.ADC_TemperParam         = &adc_device_obj->ADC_TemperParam;
        ModBus_Device.ADC_TemperOut           = &adc_device_obj->ADC_TemperOut;
    }
#endif // USING_ADC_TEMPER_SENSOR
    if(pcap_device != RT_NULL)
    {
        ModBus_Device.DataFilter              = &pcap_device_obj->DataFilter;
        ModBus_Device.PCap_DataConvert        = &pcap_device_obj->PCap_DataConvert; 
        ModBus_Device.PCap_DataConvert_Out    = &pcap_device_obj->PCap_DataConvert_Out;
    }
/*******************************************************************************************************/
    
    
/******************************************** ModBus��ʼ�� *********************************************/
    // ModBus��ʼ��(�������ڳ�ʼ��)    
    ModBus_Init(&ModBusBase_Param);                      
    // ���Ҵ��ڷ�����
    ModBusBase_Param.TX_Lock                            
        = (rt_sem_t)rt_object_find(USART_TX_LOCK_NAME, RT_Object_Class_Semaphore);    
    
    /* ���� ModBus �����߳� */
    modbus_handle_thread = rt_thread_create("mb_hand",
                                            modbus_handle_thread_entry,
                                            RT_NULL,
                                            MODBUS_HANDLE_THREAD_STACK,
                                            MODBUS_HANDLE_THREAD_PRIORITY,
                                            20);
    RT_ASSERT(modbus_handle_thread != RT_NULL);
    // ���� ModBus �����߳�
    rt_thread_startup(modbus_handle_thread);        
/*******************************************************************************************************/
    
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(modbus_init);

#endif // USING_RT_THREAD_OS
