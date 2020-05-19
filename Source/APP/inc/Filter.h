#ifndef __FILTER_H__
#define __FILTER_H__


#define TIM3_COUNTER_CLOCK        24000000                 //����ʱ��(24M��/��)
                                                           //Ԥ��Ƶֵ
#define TIM3_PRESCALER_VALUE      (SystemCoreClock/TIM3_COUNTER_CLOCK - 1)

void TIM3_PWM_INIT(void);
void TIM3_CH2_PWM(uint32_t arr);
//void TIM3_CH2_PWM(uint32_t Freq, uint16_t Dutycycle);  //��ֲ�ڣ�F:\�ҵ�����\F0��һ�׶��������\STM32F0xx_TIM���PWM������ϸ����

void App_Filter_Task (void *p_arg);

#endif
