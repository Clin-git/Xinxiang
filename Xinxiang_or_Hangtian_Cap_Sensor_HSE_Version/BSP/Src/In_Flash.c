/**@file        In_Flash.c
* @brief        读写内部Flash驱动程序
* @details      向内部Flash读取或写入数据
* @author       杨春林
* @date         2020-04-29
* @version      V1.0.0
* @copyright    2020-2030,深圳市信为科技发展有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2020/04/29  <td>1.0.0    <td>杨春林    <td>创建初始版本
* </table>
*
**********************************************************************************
*/

#include "In_Flash.h"


static uint64_t Flash_Buf[FLASH_PAGE_SIZE / 8];    ///< 临时缓存


/**@brief       向内部Flash指定位置写入一字节数据
* @param[in]    RWAddr : 写入地址
* @param[in]    WrData : 写入数据
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         地址不能越界,最大值为 FLASH_END
*/
uint8_t InFlash_Write_OneByte(uint16_t RWAddr, uint8_t WrData)
{
    uint32_t WrAddr;
    uint16_t i;
    uint8_t *buf = (uint8_t *)Flash_Buf;
    HAL_StatusTypeDef Halstatus;
    uint8_t Erase_Flag = 0;

    if(RWAddr >= IN_FLASH_SIZE)
    {
        return OP_FAILED;
    }
    WrAddr = (RWAddr / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    InFlash_Read_MultiBytes(WrAddr, buf, FLASH_PAGE_SIZE);
        
    if(Flash_Buf[RWAddr / 8] != 0xFFFFFFFFFFFFFFFF)
    {
        Halstatus = (HAL_StatusTypeDef)InFlash_Erase_Page(RWAddr, 1);
        if(HAL_OK != Halstatus)
            return OP_FAILED;
        Erase_Flag = 1;
    }
    buf[RWAddr % FLASH_PAGE_SIZE] = WrData;
        
    //__disable_irq();    
    HAL_FLASH_Unlock();
    if(Erase_Flag)
    {
        for(i = 0; i < FLASH_PAGE_SIZE; i += 8)
        {
            Halstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                                            (WrAddr + i) + IN_FLASH_BASE_ADDRESS, 
                                            (uint64_t)Flash_Buf[i / 8]);
            if(HAL_OK != Halstatus)
                break;
        }
    }
    else
    {
        Halstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                                        ((RWAddr / 8) * 8) + IN_FLASH_BASE_ADDRESS, 
                                        (uint64_t)Flash_Buf[(RWAddr / 8)]);
    }
    HAL_FLASH_Lock();
    //__enable_irq();
    if(HAL_OK != Halstatus)
    {
        return OP_FAILED;
    }

    return OP_SUCCESS;
}

/**@brief       向内部Flash指定位置读取一字节数据
* @param[in]    RWAddr : 读取地址
* @return       函数执行结果
* - 1个字节数据
* @note         地址不能越界,最大值为 FLASH_END
*/
uint8_t InFlash_Read_OneByte(uint16_t RWAddr)
{
  uint8_t RdData;
  uint32_t RealAddr;
  
  RdData = 0;
  RealAddr = IN_FLASH_BASE_ADDRESS + RWAddr;
  assert_param(IS_FLASH_PROGRAM_ADDRESS(RdAddr));
  RdData = *(__IO uint8_t *)RealAddr;
  return RdData;
}

/**@brief       向内部Flash指定位置写多个字节
* @param[in]    RWAddr : 写起始地址
* @param[in]    pWrbuf : 数据缓存指针
* @param[in]    Wrlen : 写数据长度
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         地址不能越界,最大值为 FLASH_END
*/
uint8_t InFlash_Write_MultiBytes(uint16_t RWAddr, uint8_t const *pWrbuf, uint16_t Wrlen)
{
    uint16_t WrAddr;
    uint16_t i;
    uint8_t j;
    uint16_t remain;
    uint16_t off;
    uint8_t *wbuf = (uint8_t *)pWrbuf;
    uint8_t *flash_buf = (uint8_t *)Flash_Buf;
    HAL_StatusTypeDef Halstatus;

    if(RWAddr >= IN_FLASH_SIZE)
    {
        return OP_FAILED;
    }
    WrAddr = (RWAddr / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    off = RWAddr % FLASH_PAGE_SIZE;
    remain = FLASH_PAGE_SIZE - off;
    if(Wrlen <= remain)
        remain = Wrlen;
    for(j = 0; j < (Wrlen / FLASH_PAGE_SIZE) + 1; j++)
    {        
        InFlash_Read_MultiBytes(WrAddr, flash_buf, FLASH_PAGE_SIZE);
        for(i = 0; i < remain; i++)
        {
            flash_buf[i + off] = wbuf[i];
        }
        
        Halstatus = (HAL_StatusTypeDef)InFlash_Erase_Page(WrAddr, 1);
        if(HAL_OK != Halstatus)
            break;
        //__disable_irq();        
        HAL_FLASH_Unlock();                        
        for(i = 0; i < FLASH_PAGE_SIZE; i += 8)
        {
            Halstatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                                        (WrAddr + i) + IN_FLASH_BASE_ADDRESS, (uint64_t)Flash_Buf[i / 8]);
            if(HAL_OK != Halstatus)
            {
                HAL_FLASH_Lock();
                goto __exit;
            }
        }
        HAL_FLASH_Lock();
        //__enable_irq();
        WrAddr += FLASH_PAGE_SIZE;
        off = 0;
        wbuf += remain;
        Wrlen -= remain;
        if(Wrlen > FLASH_PAGE_SIZE)
            remain = FLASH_PAGE_SIZE;
        else
            remain = Wrlen;
    }
__exit:
    if(HAL_OK != Halstatus)
    {
        return OP_FAILED;
    }

    return OP_SUCCESS;
}

/**@brief       从内部Flash指定位置读多个字节
* @param[in]    RWAddr : 读起始地址
* @param[in]    pRdbuf : 数据缓存指针
* @param[in]    Rdlen : 读数据长度
* @return       函数执行结果
* - None
* @note         地址不能越界,最大值为 FLASH_END
*/
void InFlash_Read_MultiBytes(uint16_t RWAddr, uint8_t *pRdbuf, uint16_t Rdlen)
{
  uint16_t FlashCnt;
  uint16_t RWAddress;
  
  RWAddress = RWAddr;
  for(FlashCnt = 0; FlashCnt < Rdlen; FlashCnt++)
  {
    *(pRdbuf + FlashCnt) = InFlash_Read_OneByte(RWAddress);
    
    RWAddress++;
  }
}

#if defined(STM32G0)
/**@brief  根据输入的地址得到 Flash 页编号
* @param[in]  Addr: 内部 FLASH 的地址
* @return       函数执行结果
* - Flash 页编号
  */
static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}
#endif // defined(STM32G0)
/**@brief       向内部Flash指定位置擦除页
* @param[in]    RWAddr : 擦除起始地址
* @param[in]    PageNb : 擦除页数目
* @return       函数执行结果
* - OP_SUCCESS(成功)
* - OP_FAILED(失败)
* @note         地址不能越界,最大值为 FLASH_END
*/
uint8_t InFlash_Erase_Page(uint16_t RWAddr, uint8_t PageNb)
{
    uint8_t sta = OP_SUCCESS;
    uint32_t EepAddress;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

    EepAddress = (RWAddr / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    EepAddress += IN_FLASH_BASE_ADDRESS;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined(STM32G0)
    EraseInitStruct.Page        = GetPage(EepAddress);
#else
    EraseInitStruct.PageAddress = EepAddress;
#endif // defined(STM32G0)
    EraseInitStruct.NbPages = PageNb;
    HAL_FLASH_Unlock();    
    sta = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    HAL_FLASH_Lock();    

    return sta;
}
