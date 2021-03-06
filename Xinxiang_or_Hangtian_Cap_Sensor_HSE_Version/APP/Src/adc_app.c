/**@file        adc_app.c
* @brief        ADC模块的应用
* @details      ADC DMA 功能配置 ADC温度数据获取与处理
* @author       杨春林
* @date         2020-05-06
* @version      V1.0.1
* @copyright    2020-2030,深圳市信为科技发展有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2020/07/06  <td>1.0.1    <td>杨春林    <td>添加L0系列支持程序
* </table>
*
**********************************************************************************
*/

#include "adc_app.h"
#include "In_Memory_app.h"
#include "string.h"



static uint16_t gADC_GetData[AD_GROUP_MAX][AD_CHANNEL_MAX];     ///< ADC的DMA数据缓存区
static uint16_t uADC_GetResult[AD_GROUP_MAX][AD_CHANNEL_MAX];   ///< ADC结果缓存,用于数据处理

static uint8_t ADC_Updata_Flag = 0;                             ///< ADC数据更新标志



/**@brief       初始化温度转换需要的参数结构
* @param[out]   ADC_TemperParam : 温度转换需要的参数结构指针; 
* @return       函数执行结果
* - None
* @note         要使用本函数,要加入In_Memory_app.c、In_Memory_app.h、
* In_Flash.c和In_Flash.h文件(STM32L0系列则加入In_EEPROM.c和In_EEPROM.h文件)
*/
void Sensor_ADC_TemperParam_Init(ADC_TemperParam_TypeDef *ADC_TemperParam)
{    
    InMemory_Read_MultiBytes(ADC_TEMPERPARAM_BASE_ADDRESS, 
                            (uint8_t *)ADC_TemperParam, 
                            sizeof(ADC_TemperParam_TypeDef));
    ADC_TemperParam->TempDARange = ADC_TemperParam->ADC_Temper_Calib.TempDAMax
                                    - ADC_TemperParam->ADC_Temper_Calib.TempDAMin;
}


/**@brief       初始化ADC，求出stm32芯片内部温度传感器的温度变化斜率，启动ADC的DMA传输
* @return       函数执行结果
* - None
*/
void Sensor_ADC_Init(void)
{    
    //ADC驱动初始化
    BSP_ADC_Init();
    //开始ADC DMA传输
    BSP_ADC_Start_DMA((uint32_t *)gADC_GetData,  AD_SEQBUFF_MAX);
}

/**@brief       用户获取ADC通道上平均滤波后的数据
* @param[in]    Channel_Num : 通道号,指定获取的通道;
* @return       函数执行结果
* - 平均滤波后的ADC值
*/
uint16_t Sensor_ADC_GetChn_Value(AD_CHANNEL_NUM AD_Channel_Num)
{
    uint8_t cn;
    uint32_t AD_Sum = 0;

    if(AD_Channel_Num >= AD_CHANNEL_MAX)
        return 0;
    
    for(cn = 0; cn < AD_GROUP_MAX; cn++)
    {
        //对通道的所有数据累加
        AD_Sum += uADC_GetResult[cn][AD_Channel_Num];
    }
    //计算平均值
    return AD_Sum / AD_GROUP_MAX;
}

/**@brief       获取ADC温度数据
* @return       函数执行结果
* - 温度值
* @note         温度计算可参考STM32F072数据手册
*/
float Sensor_ADC_Get_TemperData(void)
{
    uint8_t cn;
    float Result;
    uint32_t AD_Sum = 0;
    uint16_t AD_Average;

    for(cn = 0; cn < AD_GROUP_MAX; cn++)
    {
        //对温度通道的所有数据累加
        AD_Sum += uADC_GetResult[cn][AD_CHANNEL_TEMPSENSOR];
    }
    //计算平均值
    AD_Average = AD_Sum / AD_GROUP_MAX;
    
#if defined(STM32F0)
    /* 将获得的温度ADC值转换成实际温度 */
    Result = (AD_Average - (int32_t) *VREF30ADDR );
    Result = Result * (int32_t)(110 - 30);
    Result = Result / (int32_t)(*VREF110ADDR - *VREF30ADDR);
    Result = Result + 30;
#elif defined(STM32L0)
    /* 将获得的温度ADC值转换成实际温度, 这里参考了芯片参考手册的代码示例编写的 */
    Result = ((AD_Average * VDD_APPLI / VDD_CALIB) - (int32_t) *VREF30ADDR );
    Result = Result * (int32_t)(130 - 30);
    Result = Result / (int32_t)(*VREF130ADDR - *VREF30ADDR);
    Result = Result + 30;
#endif // defined(STM32F0) or defined(STM32L0)
    
    return Result;
}

/**@brief       获取ADC数据更新标志
* @return       函数执行结果
* - ADC更新标志
*/
uint8_t Sensor_ADC_Get_Updata_Flag(void)
{
    return ADC_Updata_Flag;
}

/**@brief       清除ADC更新标志
* @return       函数执行结果
* - None
*/
void Sensor_ADC_Clean_Updata_Flag(void)
{    
    ADC_Updata_Flag = UPDATA_NONE;
    BSP_ADC_DMA_Enable_IT();
}

/**@brief       ADC转换完成回调,在这里停止ADC转换,拷贝DMA数据到
* 结果缓存uADC_GetResult中
* @param[in]    hadc : ADC处理对象指针
* @return       函数执行结果
* - None
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{    
    BSP_ADC_DMA_Disable_IT();
    memcpy(uADC_GetResult, gADC_GetData, sizeof(uADC_GetResult));
    
    ADC_Updata_Flag = UPDATA_OK;
}

/* 使用RT-Thread操作系统,USING_RT_THREAD_OS在main.h中定义 */
#ifdef USING_RT_THREAD_OS

#define ADC_THREAD_STACK        512
#define ADC_THREAD_PRIORITY     RT_THREAD_PRIORITY_MAX - 2

/**@struct ADC设备对象 */
static struct rt_adc_device_obj adc_device_obj;     

/**@brief       ADC设备初始化,本函数被rt_device_register注册后,被
* rt_device_init或rt_device_open调用
* @param[in]    dev : 设备句柄
* @return       函数执行结果
* - RT_EOK : 设备初始化成功; 
* - 其他 : 设备初始化失败
*/
static rt_err_t stm32_adc_init(rt_device_t dev)
{    
    Sensor_ADC_Init();
    
    return RT_EOK;
}

/**@brief       ADC设备打开,本函数被rt_device_register注册后,被
* rt_device_open调用
* @param[in]    dev : 设备句柄; 
* @param[in]    oflag : 设备访问模式标志
* @return       函数执行结果
* - RT_EOK : 设备打开成功; 
* - -RT_EBUSY : 如果设备注册时指定的参数中包括 RT_DEVICE_FLAG_STANDALONE 
* 参数，此设备将不允许重复打开;
* - 其他:设备打开失败
*/
static rt_err_t  stm32_adc_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

/**@brief       ADC设备关闭,本函数被rt_device_register注册后,被
* rt_device_close调用
* @param[in]    dev : 设备句柄; 
* @return       函数执行结果
* - RT_EOK:设备关闭成功; 
* - -RT_ERROR:设备已经完全关闭，不能重复关闭设备;
* - 其他:设备关闭失败
* @note         这里不使用设备关闭,故只返回一个RT_EOK
*/
static rt_err_t  stm32_adc_close(rt_device_t dev)
{
    return RT_EOK;
}

/**@brief       ADC设备读数据,本函数被rt_device_register注册后,被
* rt_device_read调用
* @param[in]    dev : 设备句柄;
* @param[in]    pos : 读取数据偏移量;
* @param[out]   buffer : 内存缓冲区指针，读取的数据将会被保存在缓冲区中;
* @param[in]    size : 读取数据的大小
* @return       函数执行结果
* - size:读到数据的实际大小，如果是字符设备，返回大小以字节为
* 单位，如果是块设备，返回的大小以块为单位;
* - 0:需要读取当前线程的 errno 来判断错误状态
*/
static rt_size_t stm32_adc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    uint8_t i;
    uint16_t *adc_buf;
    
    if(pos == AD_CHANNEL_TEMPSENSOR)
    {
        *(float *)buffer = Sensor_ADC_Get_TemperData();
        return 1;
    }
    else
    {
        adc_buf = (uint16_t *)buffer;
        for(i = pos; i < (size + pos); i++)
        {
            if(i >= AD_CHANNEL_TEMPSENSOR)
                return i - pos;
            *adc_buf = Sensor_ADC_GetChn_Value((AD_CHANNEL_NUM)i);
            adc_buf += 2;            
        }
    }
    return size;
}

/**@brief       ADC设备控制,本函数被rt_device_register注册后,被
* rt_device_control调用
* @param[in]    dev:设备句柄;
* @param[in]    cmd:命令控制字，这个参数通常与设备驱动程序相关;
* @param[in]    arg:控制的参数
* @return       函数执行结果
* - RT_EOK:函数执行成功;
* - -RT_ENOSYS:执行失败，dev 为空;
* - 其他错误码:执行失败
* @note         这里不使用设备控制,故只返回一个RT_EOK
*/
static rt_err_t  stm32_adc_control(rt_device_t dev, int cmd, void *args)
{
    return RT_EOK;
}

/**@brief       ADC线程,用于读取ADC更新的数据
* @param[in]    parameter : 线程的参数,这里不使用
* @return       函数执行结果
* - None
*/
void adc_thread_entry(void *parameter)
{
    rt_device_t adc_device;
    struct rt_adc_device_obj *adc_obj;
    
    adc_device = rt_device_find(ADC_DEVICE_NAME);
    if(adc_device != RT_NULL)
    {
        adc_obj = (struct rt_adc_device_obj *)adc_device->user_data;
        rt_device_open(adc_device, RT_DEVICE_OFLAG_RDONLY);
    }
    
    while(1)
    {
        //判断ADC是否被更新
        if(Sensor_ADC_Get_Updata_Flag() == UPDATA_OK && adc_device != RT_NULL)
        {
            //获取AD转换出的温度值
            rt_device_read(adc_device, AD_CHANNEL_TEMPSENSOR, 
                            &adc_obj->ADC_TemperOut.TemperInAir, 1);
            //清除ADC更新标志,并打开ADC转换
            Sensor_ADC_Clean_Updata_Flag();
        }
        rt_thread_mdelay(10);
    }
}

/**@brief       ADC设备参数初始化,注册ADC设备,创建ADC线程并启动
* @return       函数执行结果
* - int整型值(RT_EOK)
*/
static int adc_device_init(void)
{
    rt_thread_t adc_thread;
    
    adc_device_obj.dev.type     = RT_Device_Class_Miscellaneous;
    adc_device_obj.dev.init     = stm32_adc_init;
    adc_device_obj.dev.open     = stm32_adc_open;
    adc_device_obj.dev.close    = stm32_adc_close;
    adc_device_obj.dev.read     = stm32_adc_read;
    adc_device_obj.dev.write    = RT_NULL;
    adc_device_obj.dev.control  = stm32_adc_control;    
    
    adc_device_obj.dev.user_data = &adc_device_obj;
    
    Sensor_ADC_TemperParam_Init(&adc_device_obj.ADC_TemperParam);   // ADC温度处理参数初始化    
    rt_device_register(&adc_device_obj.dev, ADC_DEVICE_NAME,        // 注册ADC设备
                        RT_DEVICE_FLAG_RDONLY
                        | RT_DEVICE_FLAG_STANDALONE);
    /* 创建 ADC 处理线程 */
    adc_thread = rt_thread_create("adc",
                                    adc_thread_entry,
                                    RT_NULL,
                                    ADC_THREAD_STACK,
                                    ADC_THREAD_PRIORITY,
                                    20);
    RT_ASSERT(adc_thread != RT_NULL);
    // 启动 ADC 处理线程
    rt_thread_startup(adc_thread);
    
    return RT_EOK;
}
INIT_DEVICE_EXPORT(adc_device_init);

#endif // USING_RT_THREAD_OS


