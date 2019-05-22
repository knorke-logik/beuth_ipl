
#include <stm32f30x.h>
#include <stm32f30x_conf.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_dac.h>

int main(void)
{

GPIO_InitTypeDef GPIO_InitStructure;
DAC_InitTypeDef DAC_InitStructure;

SystemInit();

RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);


GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;

DAC_StructInit(&DAC_InitStructure);
GPIO_Init(GPIOA, &GPIO_InitStructure);
DAC_Init(DAC1,DAC_Channel_1, &DAC_InitStructure);
DAC_SetChannel1Data(DAC1,DAC_Align_12b_R, 868);// 0x7FF);
DAC_Cmd(DAC1,DAC_Channel_1, ENABLE);

while(1){};
return 0;
}
