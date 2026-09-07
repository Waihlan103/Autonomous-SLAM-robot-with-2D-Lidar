#ifndef ENCODER_HPP
#define ENCODER_HPP
#include <cstdint>

extern volatile int32_t encoder_left;
extern volatile int32_t encoder_right;

void encoder_init();
void encoder_update();

#endif