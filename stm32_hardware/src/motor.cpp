#include <Arduino.h>
#include "motor.hpp"

void pwm_init() {
  /* Initialize Motor Pins */
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  /* Enable clocks */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  /* PA6, PA7 -> Alternate Function Push Pull */
  GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
                  GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
  GPIOA->CRL |=  (GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0 | GPIO_CRL_CNF6_1); // 50MHz AF
  GPIOA->CRL |=  (GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1);

  /* Timer config
     72MHz / (PSC+1) = 1MHz
     1MHz / 1000 = 1kHz (stable for testing)
     👉 change ARR for higher freq if needed
  */
  TIM3->PSC = 71;
  TIM3->ARR = 3600 - 1;

  /* PWM mode 1 on CH1 & CH2 */
  TIM3->CCMR1 &= ~((7 << 4) | (7 << 12));   // CH1 CH2 PWM1 PWM2
  TIM3->CCMR1 |= (6 << 4) | (6 << 12); 

  TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;

  TIM3->CR1 |= TIM_CR1_CEN;
}

void motor_set(int left, int right) {
  /* Direction */
  digitalWrite(IN1, left >= 0);
  digitalWrite(IN2, left < 0);
  digitalWrite(IN3, right >= 0);
  digitalWrite(IN4, right < 0);

  left  = constrain(abs(left), 0, 3600 - 1);
  right = constrain(abs(right), 0, 3600 - 1);

  TIM3->CCR1 = left;
  TIM3->CCR2 = right;
}