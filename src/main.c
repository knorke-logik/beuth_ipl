
#include <stm32f30x.h>
#include <stm32f30x_conf.h>

#define  SAMPLE_BASE_ADDRESS       (uint32_t)  0x08009C40
#define  DATABLOCK_BASE_ADDRESS    (uint32_t) (SAMPLE_BASE_ADDRESS + 0x2C)
#define  SAMPLE_SIZE_OFFSET        (uint32_t) (SAMPLE_BASE_ADDRESS + 0xFF)
#define  END_ADDR                  (uint32_t) (SAMPLE_BASE_ADDRESS + 0x53D0)
//#define CCMR_OFFSET        ((uint16_t)0x0018)
/* DHR registers offsets */
#define DHR12R1_OFFSET             ((uint32_t)0x00000008)
//#define DHR12R2_OFFSET             ((uint32_t)0x00000014)
//#define DHR12RD_OFFSET             ((uint32_t)0x00000020)

void tim2_cfg()
{
	  TIM2->CCER  &= 0x0;					 // disable tim2
	  TIM2-> CR1  &= 0xFFFE;                 // timer enable
	  TIM2->CCMR1 &= (uint32_t) 0xFEFF8FFF;  // reset state?
	  TIM2->CCMR1 &= (uint32_t) 0xFFFFFFFC;  //
	  TIM2->CCMR1 |= (uint32_t) 0x00000030;  // toggling output
	  //  *(__IO uint32_t *) ( TIM2 + CCMR_OFFSET ) |= (uint32_t) 0x00010030 ; // toggling output

	  TIM2->EGR |= (uint16_t) 0x0002;        // event source channel1 ..?
	  TIM2->CR1 &= (uint16_t) 0xFFEF;        // up-counter mode
	  TIM2->CR1 &= (uint16_t) 0xFCFF;        // clock division 1
	  TIM2->RCR = 0;

	  TIM2->CCR1 = 10;
	  TIM2->PSC = 9;
	  TIM2->ARR = 49;

	  TIM2->CCER |= 0x1;  // channel 1 enable
	  TIM2 -> CR1 |= 0x1; // timer enable
}

void flash2dac()
{
  uint32_t addr = DATABLOCK_BASE_ADDRESS;
  RCC->APB1ENR |= 1;  //  TIM2 clocking enable

  tim2_cfg();

  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;

  while (1)
  {
   if (FLASH_GetStatus() == FLASH_COMPLETE && FLASH_GetFlagStatus(FLASH_SR_BSY) == 0)
   {
	  if ((TIM2->SR & 0x2) == 0x2 )  // if compare event is occured ...
	    {
		   TIM2->SR &= (uint32_t) 0xfffd; //...reset the flag
//		   int16_t tmp = *(__IO int16_t*) addr ;
//		   int16_t tmp1 = tmp + 0x80;
//		   int8_t tmp2 = (int8_t) tmp1 ;
//		   uint8_t tmp_uint = (uint8_t) tmp2;
		   *(__IO uint32_t *) ((uint32_t)DAC1 + DHR12R1_OFFSET + DAC_Align_8b_R) = (uint8_t) ( *(__IO int16_t*) addr ) + 0x80;
		   addr += 1;
		   if(addr == END_ADDR) addr = DATABLOCK_BASE_ADDRESS;
		}
	}
  }
  FLASH->CR |= FLASH_CR_LOCK;
}

void dacinit()
{
 RCC->APB1ENR |= RCC_APB1Periph_DAC;
 RCC->APB1RSTR |= RCC_APB1Periph_DAC;
 RCC->APB1RSTR &= ~RCC_APB1Periph_DAC;
 DAC1->CR = 0x0003;
}

void hupe_dacinit()
{
 uint32_t tmp_cnt = 0;
 RCC->APB1ENR |= RCC_APB1Periph_DAC | RCC_APB1Periph_TIM2;

 RCC->APB1RSTR |= RCC_APB1Periph_TIM2;
 RCC->APB1RSTR &= ~RCC_APB1Periph_TIM2;

// TIM2->EGR = TIM_EventSource_CC1;
 TIM2->CCER = 0x1;
 TIM2->PSC  = 0x0;
 TIM2->ARR  = 0x1;
 TIM2->CNT  = 0x0;
 TIM2->CR2  = 0x0020;       // compare pulse TGO MMS-bit
 TIM2->CR1  = 0x0005;

 RCC->APB1RSTR |= RCC_APB1Periph_DAC;
 RCC->APB1RSTR &= ~RCC_APB1Periph_DAC;

 DAC1->CR |= (uint32_t)0x0FA5;

 while(tmp_cnt < 900000){

  if ((TIM2->SR & 0x02) == 0x2 )
  {
   TIM2->SR &= (uint32_t) 0xfffd;
   tmp_cnt = tmp_cnt + 1;
  }
 }
}

void gpioinit()
{
// GPIO_InitTypeDef gpio_init;

 RCC->AHBENR   |= RCC_AHBPeriph_GPIOA;
 RCC->APB1RSTR |= RCC_AHBPeriph_GPIOA;
 RCC->APB1RSTR &= ~RCC_AHBPeriph_GPIOA;

 GPIOA->MODER  |= (uint32_t) 0x00000302;
 GPIOA->AFR[0] |= (uint8_t)  0x01;
// GPIOA->AFR[1] |= 0x0;
// gpio_init.GPIO_Mode = GPIO_Mode_AF;
// gpio_init.GPIO_Pin = GPIO_Pin_0;


// GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_1);

// GPIO_Init(GPIOA, &gpio_init);

}

void configure_PC13(void) {

//    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOC */
    RCC->AHBENR   |= RCC_AHBPeriph_GPIOC;
    /* Enable clock for SYSCFG */
    RCC->APB2ENR |= RCC_APB2Periph_SYSCFG;
    /* Reset GPIOC */
    RCC->APB2RSTR |= RCC_AHBPeriph_GPIOC;
    RCC->APB2RSTR &= ~RCC_AHBPeriph_GPIOC;

    /* Set pin as input */
    GPIOC->PUPDR   |= 0x04000000;
    GPIOC->OSPEEDR |= 0x0C000000;

    /* Tell system that you will use PC13 for EXTI_Line13 */
    SYSCFG->EXTICR[3] |= 0x0020;

    /* PC13 is connected to EXTI_Line13 */
    EXTI->IMR  |= 0x00002000; // unmask the 13th interrupt line
    EXTI->RTSR |= 0x00002000; // rising edge

//    /* Add IRQ vector to NVIC */
//    /* PC13 is connected to EXTI_Line13, which has EXTI0_IRQn vector */
//    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
//    /* Set priority */
//    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
//    /* Set sub priority */
//    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
//    /* Enable interrupt */
//    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//    /* Add to NVIC */
//    NVIC_Init(&NVIC_InitStruct);
    NVIC->ISER[1] |= 0x0100;
}

void EXTI15_10_IRQHandler()
{
	 EXTI->PR &= (uint32_t) 0x00002000; // reset the interrupt pending flag

	 hupe_dacinit();
	 dacinit();
	 tim2_cfg();
}

int main(void)
{
  // *(uint32_t*) 0xE0042008 |= (uint32_t) 0x1;  // timer debug mode
  gpioinit();

  // dac init snippet
  RCC->APB1ENR |= RCC_APB1Periph_DAC;
  DAC1->CR = 0x0003;

  configure_PC13();
  tim2_cfg();
  flash2dac();
}
