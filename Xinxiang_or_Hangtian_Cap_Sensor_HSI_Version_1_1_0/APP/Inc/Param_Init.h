#ifndef __PARAM_INIT_H
#define __PARAM_INIT_H

#include "In_Memory_app.h"
#include "ModBus.h"
#include "Picocap_app.h"
#include "adc_app.h"

/**@brief       ModBus������ʼ��
* @param[in]    ModBusBaseParam : ModBus����Ļ��������ṹ��
* @return       ����ִ�н��
* - None
*/
void ModBus_Param_Init(ModBusBaseParam_TypeDef *ModBusBaseParam);

/**@brief       ��ʼ�������˲���Ҫ�Ĳ����ṹ
* @param[in]    Filterfactor_CountMax : �˲�ϵ�����ۼ���������ֵ,�˲�ϵ���ڸ�8λ,
* �ۼ���������ֵ�ڵ�8λ;
* @param[out]   FilterParam : �����˲���Ҫ�Ĳ����ṹָ��;
* @return       ����ִ�н��
* - None
* @note         Ҫʹ�ñ�����,Ҫ����In_Memory_app.c��In_Memory_app.h��
* In_Flash.c��In_Flash.h�ļ�(STM32L0ϵ�������In_EEPROM.c��In_EEPROM.h�ļ�)
*/
void DataFilterParam_Init(DataFilterParam *FilterParam, uint16_t Filterfactor_CountMax);

/**@brief       ��ʼ������ת����Ҫ�Ĳ����ṹ
* @param[out]   DataConvert_Param : ����ת����Ҫ�Ĳ����ṹָ��;
* @return       ����ִ�н��
* - None
* @note         Ҫʹ�ñ�����,Ҫ����In_Memory_app.c��In_Memory_app.h��
* In_Flash.c��In_Flash.h�ļ�(STM32L0ϵ�������In_EEPROM.c��In_EEPROM.h�ļ�)
*/
void DataConvertParam_Init(PCap_DataConvert_Param *DataConvert_Param);

/**@brief       ��ʼ���¶�ת����Ҫ�Ĳ����ṹ
* @param[out]   ADC_TemperParam : �¶�ת����Ҫ�Ĳ����ṹָ��; 
* @return       ����ִ�н��
* - None
* @note         Ҫʹ�ñ�����,Ҫ����In_Memory_app.c��In_Memory_app.h��
* In_Flash.c��In_Flash.h�ļ�(STM32L0ϵ�������In_EEPROM.c��In_EEPROM.h�ļ�)
*/
void Sensor_ADC_TemperParam_Init(ADC_TemperParam_TypeDef *ADC_TemperParam);

#endif // __PARAM_INIT_H
