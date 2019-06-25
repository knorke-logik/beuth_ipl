
#include <stm32f30x.h>
#include <stm32f30x_conf.h>

// #define PAGE_SIZE             ((uint32_t)0x0400)  	/* Page size = 1KByte */
 #define  SAMPLE_BASE_ADDRESS  ((uint32_t)  0x08009C40 + 0x2C)
 #define  SAMPLE1_SIZE_ADDRESS  ((uint32_t) (0x08009C40 + 0xFF)) //(SAMPLE_BASE_ADDRESS+0x40))
 #define  END_ADDR (uint32_t) (0x08009C40 + 0x53D0)


void flash2dac()
{
	TIM_TimeBaseInitTypeDef timerInitStructure;
	int8_t 					sample_8b 		= 0xBB;
	int16_t 				data_from_flash = 0xAAAA;
	uint32_t				addr   = SAMPLE_BASE_ADDRESS;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	timerInitStructure.TIM_Prescaler = 9;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 49;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_GenerateEvent(TIM2,TIM_EventSource_CC1);
	TIM_SelectOCxM(TIM2,TIM_Channel_1,TIM_OCMode_Toggle);
	TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Enable);
	TIM2->CCR1 = 10;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);

  FLASH_Unlock();

  while(1){
	  if (FLASH_GetStatus() == FLASH_COMPLETE && FLASH_GetFlagStatus(FLASH_SR_BSY) == 0)
	  {
		  uint32_t tim = TIM_GetCounter(TIM2);
		  if (tim == 49)
		  {
			data_from_flash =(int8_t)( *(__IO uint16_t*)addr  + 0x80);
			sample_8b =  data_from_flash  ;
			DAC_SetChannel1Data(DAC1,DAC_Align_8b_R, sample_8b);
			addr = addr+2;
			if(addr == END_ADDR) addr = SAMPLE_BASE_ADDRESS;
		  }
	  }
  }
  FLASH_Lock();
}


int main(void)
{
  GPIO_InitTypeDef 	gpio_init_tim;
  DAC_InitTypeDef 	dac_init;

  // SystemInit();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  gpio_init_tim.GPIO_Mode = GPIO_Mode_AF;
  gpio_init_tim.GPIO_Pin = GPIO_Pin_0;
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_1);
  GPIO_Init(GPIOA, &gpio_init_tim);

  DAC_StructInit(&dac_init);
  DAC_Init(DAC1,DAC_Channel_1, &dac_init);
  DAC_Cmd(DAC1,DAC_Channel_1, ENABLE);

  flash2dac();

}
