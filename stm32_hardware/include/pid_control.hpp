#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP

extern float LKp, LKi, LKd;
extern float RKp, RKi, RKd;
extern float prevErrorLeft, prevErrorRight;
extern float integralLeft, integralRight;
extern unsigned long lastTime;
extern const int left_PPR; // Pulses per revolution of the motor
extern const int right_PPR;
extern float LcmdRPM, RcmdRPM;
extern int motorSpeedLeft, motorSpeedRight;
extern float LtargetRPM, RtargetRPM;
extern float prevLtarget, prevRtarget;

#include <Arduino.h>
#include "math_constants.hpp"

void PID_control();
#endif