# README 

## ���

���ĵ�Ϊ ���ݴ������� ��ҪӲ������ �� ����(BSP) ˵����

��Ҫ�������£�

- Ŀǰ֧�ֵ�����оƬ
- �����������

ͨ���Ķ����ĵ����Դ�ŵ��˽���ݳ������������

## ֧�ֵ�����оƬ

Ŀǰ�������򹤳�֧�ֵ�����оƬΪ��
ST��˾��
STM32F0ϵ��
STM32L0ϵ��
����˾Ŀǰ�ڵ��ݴ�������ʹ������оƬΪ���֣��ֱ�Ϊ STM32F072CB, STM32L072CB

������ϸ��Ϣ��ο� ST �� �ٷ���վ(https://www.st.com/)��

## ��������

�� BSP Ŀǰ�������֧��������£�

| **Ƭ������**      | **֧�����** | **��ע**                              |
| :----------------- | :----------: | :------------------------------------- |
| UART              |     ֧��     | USART1                            |
| SPI               |     ֧��     | SPI1                              |
| ADC | ֧�� |  |
| DAC | ֧�� |  |
| RTC               |   ֧��   | ֧���ڲ�����ʱ�� |
| FLASH | ֧�� |  |
| IWG               |   ֧��   |                               |

����UART��SPI��ADC��DAC��FLASH(STM32L0ϵ��Ϊ�ڲ�EEPROM)��IWG���ѷ�װ��ģ�飬
�ײ��������ϲ�Ӧ���Ƿֿ��ġ�

UART�� STM32F0ϵ�� �� STM32L0ϵ�� �����ModBus��2���¼���������2���¼�
�ֱ�Ϊ�������ݳ�ʱ�¼������յ��ض��ַ��¼�����ͨ������UART������2���¼��ֱ�
��Ϊ RTU �� ASCII Э�������֡������ɱ�־�����磺ʹ��RTUЭ��ʱ���������ݽ�
�ճ�ʱʱ��Ϊ 3.5 ���ַ�(�Ĵ���)ʱ����Ϊ������ɱ�־��ʹ��ASCIIЭ��ʱ������
���յ����з� '/n'��Ϊ������ɱ�־�����ݵķ��ͺͽ��ն�ʹ��DMA��ʽ��

SPI�� ��װ�˳�ʼ�������ݶ�д��API������ʹ��ʱ��ֱ�ӵ�����ЩAPI���������ݵķ���
�ͽ��ն�ʹ����ͨ��ʽ��

ADC�� ������ADC�ɼ�ͨ����ͨ���У�ͨ��0��1��2��3��6��7��8��9���ڲ��¶ȴ�����ͨ����
���ݴ���ʹ��DMA��ʽ��

DAC�� ������DACת��ͨ����ͨ���У�ͨ��1��2�����ݴ���ʹ����ͨ��ʽ��

FLASH(��EEPROM)����װ�˶���д�Ͳ�����API������ʹ��ʱ��ֱ�ӵ�����ЩAPI��������
ʵ�� 1 �ֽڻ���ֽڵĶ�д�Ĺ��ܣ��Լ�ָ��ĳ��������в�����

IWG�� ��װ�˳�ʼ����ι��(���Ź���ʱ����װ��)��API������ʹ��ʱ��ֱ�ӵ�����ЩAPI
��������ʵ�ֳ�ʼ���������Ź���ι���Ĺ��ܡ�

��Ҫ�����ú꣺ 
- IN_EEPROM_BASE_ADDRESS----(�����ڲ�EEPROM����ַ)
- IN_EEPROM_START-----------(EEPROM��ʼ��ַ)
- IN_EEPROM_END-------------(EEPROM������ַ)
- IN_FLASH_BASE_ADDRESS-----(�����ڲ�FLASH����ַ)
- IN_FLASH_START------------(FLASH��ʼ��ַ)
- IN_FLASH_END--------------(FLASH������ַ)
- USING_UART_TIMEOUT--------(ʹ�ý��ճ�ʱ)
- USING_CHARMATCH-----------(ʹ���ַ�ƥ��)
- USART_USING_485-----------(����485�շ����ƹܽ�ʹ��)
- DEFAULT_UART_TIMEOUT------(Ĭ�ϳ�ʱʱ�䣬һ��, 10��ʾ1���ֽ�ʱ��)
- DEFAULT_UART_MATCHCHAR----(Ĭ��ƥ����ַ�)
- USING_MODBUS_RTU----------(ʹ��ModBus RTUЭ��, ����Ҫ����USING_UART_TIMEOUT)
- USING_MODBUS_ASCII--------(ʹ��ModBus ASCIIЭ��, ����Ҫ����USING_CHARMATCH)
- SUBCODE_IS_DEVADDR--------(����ModBus����Ϊ�豸��ַ����������Ĭ��Ϊ0)
- AUTOUPLOAD_CYCLE----------(�Զ��ϴ�����,��λ ms)

## ע������

- UART �� ModBus ��2���¼�����������STM32F0��STM32L0ϵ����֧�֣������ֲᷢ�֣�
Ŀǰ��STM32F1��STM32F4ϵ��û����2�����ܡ�

## ��ϵ����Ϣ

ά����: ���

-  QQ��371921187
-  ���䣺371921187@qq.com