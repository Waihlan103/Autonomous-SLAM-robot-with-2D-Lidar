#ifndef MOTOR_HPP
#define MOTOR_HPP

/* -------- L298N PINS -------- */
#define IN1 PB12
#define IN2 PB13
#define IN3 PB14
#define IN4 PB15

#define PWM_L PA6   // TIM3 CH1
#define PWM_R PA7   // TIM3 CH2

void pwm_init();
void motor_set(int, int);
#endif