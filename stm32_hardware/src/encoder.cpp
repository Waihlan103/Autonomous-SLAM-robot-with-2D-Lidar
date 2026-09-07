#include "encoder.hpp"
#include <Arduino.h>

volatile int32_t encoder_left = 0;
volatile int32_t encoder_right = 0;

void encoder_init() {
     RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
     RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;

     // PA0, PA1 pull-up
     GPIOA->CRL &= ~((0xF << 0) | (0xF << 4));
     GPIOA->CRL |=  (0x4 << 0) | (0x4 << 4); // input floating
    
     // PB6, PB7 pull-up
     GPIOB->CRL &= ~((0xF << 24) | (0xF << 28));
     GPIOB->CRL |=  (0x4 << 24) | (0x4 << 28);

     /* TIM2 encoder */
     // Encoder Mode 3 for Timer2
     TIM2->SMCR &= ~TIM_SMCR_SMS;
     TIM2->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

     // CC1, CC2 mapped to TI1, TI2
     TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
     TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
     
     // Rising edge for both channels
     TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);

     TIM2->PSC = 0;
     TIM2->ARR = 0xFFFF;
     TIM2->CNT = 0;

     TIM2->CR1 |= TIM_CR1_CEN;

     /* TIM4 encoder */
     // Encoder Mode 3 for Timer4
     TIM4->SMCR &= ~TIM_SMCR_SMS;
     TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

     // CC1, CC2 mapped to TI1, TI2
     TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
     TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
     
     // Rising edge for both channels
     TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);

     TIM4->PSC = 0;
     TIM4->ARR = 0xFFFF;
     TIM4->CNT = 0;

     TIM4->CR1 |= TIM_CR1_CEN;
}

/* ================= ENCODER UPDATE (DELTA) ================= */
void encoder_update() {
  static uint16_t last_cnt2 = 0;
  uint16_t now_cnt2 = TIM2->CNT;
  int16_t diff2 = (int16_t)(now_cnt2 - last_cnt2);
  encoder_left += diff2;
  last_cnt2 = now_cnt2;
  
  static uint16_t last_cnt4 = 0;
  uint16_t now_cnt4 = TIM4->CNT;
  int16_t diff4 = (int16_t)(now_cnt4 - last_cnt4);
  encoder_right -= diff4;
  last_cnt4 = now_cnt4;
}