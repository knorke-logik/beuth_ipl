
#include <stm32f30x.h>
#include <stm32f30x_conf.h>

#define  SAMPLE_BASE_ADDRESS       (uint32_t) 0x08009C40
#define  DATABLOCK_BASE_ADDRESS    (uint32_t) (SAMPLE_BASE_ADDRESS + 0x2C)
#define  SAMPLE_SIZE_OFFSET        (uint32_t) (SAMPLE_BASE_ADDRESS + 0xFF) //(DATABLOCK_BASE_ADDRESS+0x40))
#define  END_ADDR                  (uint32_t) (SAMPLE_BASE_ADDRESS + 0x53D0)

void flash2dac()
{
  TIM_TimeBaseInitTypeDef timerInitStructure;
  uint32_t				addr   = DATABLOCK_BASE_ADDRESS;

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
	  if ((TIM2->SR & 0x2) == 0x2 )
	    {
		   TIM2->SR &= (uint32_t) 0xfffd;
		   DAC_SetChannel1Data(DAC1,DAC_Align_8b_R, (int8_t) *(__IO int16_t*)addr);
		   addr += 1;
		   if(addr == END_ADDR) addr = DATABLOCK_BASE_ADDRESS;
		  }
	  }
  }
  FLASH_Lock();
}

void dacinit()
{
 DAC_InitTypeDef dac_init;
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
 DAC_StructInit(&dac_init);
 DAC_Init(DAC1,DAC_Channel_1, &dac_init);
 DAC_Cmd(DAC1,DAC_Channel_1, ENABLE);
}

void gpioinit()
{
 GPIO_InitTypeDef gpio_init;
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
 gpio_init.GPIO_Mode = GPIO_Mode_AF;
 gpio_init.GPIO_Pin = GPIO_Pin_0;
 GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_1);
 GPIO_Init(GPIOA, &gpio_init);
}

int main(void)
{
  // *(uint32_t*) 0xE0042008 |= (uint32_t) 0x1;  // timer debug mode
  gpioinit();
  dacinit();
  flash2dac();
}
