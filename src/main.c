
#include <stm32f30x.h>
#include <stm32f30x_conf.h>

#define  WAV_DATA_OFFSET           (uint32_t)  0x2C
#define  WAV_DATA_SIZE_OFFSET      (uint32_t)  0x28
#define  SAMPLE_BASE_ADDRESS       (uint32_t)  0x08009C40
#define  DATABLOCK_BASE_ADDRESS    (uint32_t) (SAMPLE_BASE_ADDRESS + WAV_DATA_OFFSET)
#define  SAMPLE_SIZE_OFFSET        (uint32_t) (SAMPLE_BASE_ADDRESS + WAV_DATA_SIZE_OFFSET)
#define  END_ADDR                  (uint32_t) (SAMPLE_BASE_ADDRESS + 0x53D0)

#define  SAMPLE2_BASE_ADDRESS       (uint32_t)  0x08008CA0
#define  DATABLOCK2_BASE_ADDRESS    (uint32_t) (SAMPLE2_BASE_ADDRESS + WAV_DATA_OFFSET)
#define  SAMPLE2_SIZE_OFFSET        (uint32_t) (SAMPLE2_BASE_ADDRESS + WAV_DATA_SIZE_OFFSET)
#define  END_ADDR2                  (uint32_t) (SAMPLE2_BASE_ADDRESS + 0xE1D)
//#define CCMR_OFFSET        ((uint16_t)0x0018)
/* DHR registers offsets */
#define DHR12R1_OFFSET             ((uint32_t)0x00000008)
//#define DHR12R2_OFFSET             ((uint32_t)0x00000014)
//#define DHR12RD_OFFSET             ((uint32_t)0x00000020)

void tim2_8khz_cfg()
{
	  RCC->APB1ENR |= 1;
	  RCC->APB1RSTR |= 1;
	  RCC->APB1RSTR &= 0;

	  TIM2->CCER  &= 0x0;					 // disable tim2
	  TIM2->CR1   &= 0xFFFE;                 // timer enable
	  TIM2->CCMR1 &= (uint32_t) 0xFEFF8FFF;  // reset state?
	  TIM2->CCMR1 &= (uint32_t) 0xFFFFFFFC;  //
	  TIM2->CCMR1 |= (uint32_t) 0x00000030;  // toggling output
	  //  *(__IO uint32_t *) ( TIM2 + CCMR_OFFSET ) |= (uint32_t) 0x00010030 ; // toggling output

	  TIM2->EGR |= (uint16_t) 0x0002;        // event source channel1 ..?
	  TIM2->CR1 &= (uint16_t) 0xFFEF;        // up-counter mode
	  TIM2->CR1 &= (uint16_t) 0xFCFF;        // clock division 1
	  TIM2->RCR  = 0x0;
	  // 8kHz configuration
	  TIM2->PSC  = 9;  // +1
	  TIM2->ARR  = 99; // +1
	  TIM2->CCR1 = 10;

	  TIM2->CCER  |= 0x1;  // channel 1 enable
	  TIM2->CR1   |= 0x1;  // timer enable
}

void hupefahrt2dac()
{
  uint32_t addr = DATABLOCK_BASE_ADDRESS;

  tim2_8khz_cfg();

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
		   *(__IO uint32_t *) ((uint32_t)DAC1 + DHR12R1_OFFSET + DAC_Align_8b_R) = (int8_t) *(__IO int16_t*) addr  ;//+ 0x80;
		   addr += 1;
		   if(addr == END_ADDR) addr = DATABLOCK_BASE_ADDRESS;
		}
	 }
  }
  FLASH->CR |= FLASH_CR_LOCK;
}

void fahrt()
{
  uint32_t addr = DATABLOCK2_BASE_ADDRESS;

  tim2_8khz_cfg();

  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;

  while (1)
  {
   if (FLASH_GetStatus() == FLASH_COMPLETE && FLASH_GetFlagStatus(FLASH_SR_BSY) == 0)
   {
	  if ((TIM2->SR & 0x2) == 0x2 )  // if compare event is occured ...
	    {
		   TIM2->SR &= (uint32_t) 0xfffd; //...reset the flag
		   *(__IO uint32_t *) ((uint32_t)DAC1 + DHR12R1_OFFSET + DAC_Align_8b_R) = (int8_t) *(__IO int16_t*) addr;
		   addr += 1;
		   if(addr == END_ADDR2) addr = DATABLOCK2_BASE_ADDRESS;
		}
	 }
  }
  FLASH->CR |= FLASH_CR_LOCK;
}

void gpioinit()
{
 // configure the timer output gpio
 RCC->AHBENR   |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC ;
 /* Enable clock for SYSCFG */
 RCC->APB2ENR |= RCC_APB2Periph_SYSCFG;

 RCC->AHBRSTR |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC;
 RCC->AHBRSTR &= ~RCC_AHBPeriph_GPIOA;
 RCC->AHBRSTR &= ~RCC_AHBPeriph_GPIOB;
 RCC->AHBRSTR &= ~RCC_AHBPeriph_GPIOC;

 GPIOA->MODER  |= (uint32_t) 0x00000302;
 GPIOA->AFR[0] |= (uint8_t)  0x01;       // timer output

 GPIOB->MODER      = 0x00;
 GPIOB->OTYPER	   = 0x0;
 GPIOB->PUPDR      |= 0x000000AA; // PB0,1,2,3 pull-down
 GPIOB->OSPEEDR    |= 0x000000FF; // PB0,1,2,3 highspeed


 GPIOC->MODER  |= (uint32_t) 0x00000055;    //configure PC0,1,2,3 as output
 GPIOC->PUPDR      |= 0x000000AA;        // PB0,1,2,3 pull-down
 GPIOC->OSPEEDR    |= 0x000000FF;        // PB0,1,2,3 highspeed
 GPIOC->PUPDR   |= 0x04000000; // PC13 pull-up
 GPIOC->OSPEEDR |= 0x0C000000;

 SYSCFG->EXTICR[0] = 0x1111;  // exti lines belegen
 SYSCFG->EXTICR[3] |= 0x0020; /* Tell system that you will use PC13 for EXTI_Line13 */

 EXTI->IMR  |= 0x0000200F; // unmask the interrupt lines: 13,0,1,2,3
 EXTI->RTSR |= 0x0000200F; // rising edge

 /* Add IRQ vector to NVIC */
 NVIC->ISER[1] |= 0x0100;
 NVIC->ISER[0] |= 0x03C0;
}
void EXTI15_10_IRQHandler()
{
	 EXTI->PR &= (uint32_t) 0x00002000; // reset the interrupt pending flag
}

void EXTI0_IRQHandler()
{    EXTI->PR &= (uint32_t) 0x00000001; // reset the interrupt pending flag

	 uint32_t debounce_buf = 0;
     for(uint32_t i = 0; i<100000; i++)
     {
       if((GPIOB->IDR & 0x1) == 0x1)
       {
        debounce_buf += 1;
       }
       else debounce_buf = 0;
     }

     if (debounce_buf == 80000)
     {
      hupefahrt2dac();
     }
}

void EXTI1_IRQHandler()
{
	EXTI->PR &= (uint32_t) 0x00000002; // reset the interrupt pending flag
	uint32_t debounce_buf = 0;

    for(uint32_t i = 0; i<50000; i++)
    {
      if((GPIOB->IDR & 0x2) == 0x2)
      {
       debounce_buf += 1;
      }
      else debounce_buf = 0;
    }
    if (debounce_buf >= 35000)
    {

   	 GPIOC->ODR ^= 0x00000001; // toggle the sound bit
    }

}

void EXTI2_TS_IRQHandler()
{
	 EXTI->PR &= (uint32_t) 0x00000004; // reset the interrupt pending flag
	 uint32_t debounce_buf = 0;

	     for(uint32_t i = 0; i<50000; i++)
	     {
	       if((GPIOB->IDR & 0x2) == 0x2)
	       {
	        debounce_buf += 1;
	       }
	       else debounce_buf = 0;
	     }
	     if (debounce_buf >= 35000)
	     {
	      GPIOC->ODR ^= 0x00000002; // toggle the light bit
	     }

}

void EXTI3_IRQHandler()
{
	 EXTI->PR &= (uint32_t) 0x00000008; // reset the interrupt pending flag
	 uint32_t debounce_buf = 0;

	 for(uint32_t i = 0; i<50000; i++)
	 {
	  if((GPIOB->IDR & 0x2) == 0x2)
	  {
	   debounce_buf += 1;
	  }
	  else debounce_buf = 0;
	 }
	 if (debounce_buf >= 35000)
	 {
      GPIOC->ODR ^= 0x00000004; // toggle the light direction
	 }

}

//void convert_test(){
//	dacinit();
//	tim2_8khz_cfg();
//	for (int8_t i = 0x80 ; i < 0x7F; i ++ )
//	{		int z = 0;
//			while (z == 0){
//			  if ((TIM2->SR & 0x2) == 0x2 )  // if compare event is occured ...
//			    {
//
//				   TIM2->SR &= (uint32_t) 0xfffd; //...reset the flag
//
//				   *(__IO uint32_t *) ((uint32_t)DAC1 + DHR12R1_OFFSET + DAC_Align_8b_R) = i + 0x80;//(int8_t) ( *(__IO int16_t*) i ) ;//+ 0x80;
//
//				   if(i == 0x7E) i = 0x80;
//				   z = 1;
//				}
//			}
//	}
//}

int main(void)
{
  // *(uint32_t*) 0xE0042008 |= (uint32_t) 0x1;  // timer debug mode

  RCC->APB1ENR |= RCC_APB1Periph_DAC;
  RCC->APB1RSTR |= RCC_APB1Periph_DAC;
  RCC->APB1RSTR &= ~RCC_APB1Periph_DAC;
  DAC1->CR = 0x0003;

  gpioinit();
  tim2_8khz_cfg();
  hupefahrt2dac();
//  fahrt();
}
