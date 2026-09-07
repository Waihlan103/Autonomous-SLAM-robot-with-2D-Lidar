#include "pid_control.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "math_constants.hpp"

// PID constants
float LKp = 3, LKi = 0.75, LKd = 0.001;
float RKp = 3, RKi = 0.75, RKd = 0.001;

// PID internal variables
float previousErrorLeft = 0.0, previousErrorRight = 0.0;
float integralLeft = 0.0, integralRight = 0.0;
unsigned long lastTime = 0;

// Motor output
int motorSpeedLeft = 0, motorSpeedRight = 0;

const int left_PPR = 4391; // pulses per revolution
const int right_PPR = 4391;

// Target velocity (set from ROS2 command)
float LtargetRPM = 0;
float RtargetRPM = 0;

float prevLtarget = 0.0;
float prevRtarget = 0.0;

void PID_control() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 50) { // every 50ms
        // Calculate delta counts for PID
        //float dt = (currentTime - lastTime) / 1000.0;

        static int32_t last_left = 0;
        static int32_t last_right = 0;

        int32_t delta_left  = encoder_left  - last_left;
        int32_t delta_right = encoder_right - last_right;

        last_left  = encoder_left;
        last_right = encoder_right;

        // Convert encoder counts to RPM
        //float leftRPM  = (delta_left / (float)left_PPR) * (60.0 / dt);   // counts → RPM
        //float rightRPM = (delta_right / (float)right_PPR) * (60.0 / dt);
        float leftRPM = (delta_left / (float)left_PPR) * 1200;   // counts → RPM
        float rightRPM = (delta_right / (float)right_PPR) * 1200;

        // ---------------- DIRECTION CHANGE RESET ----------------
        bool directionChanged =
            (LtargetRPM * prevLtarget < 0) ||
            (RtargetRPM * prevRtarget < 0);

        if (directionChanged) {
            integralLeft = 0;
            integralRight = 0;
            previousErrorLeft = 0;
            previousErrorRight = 0;
        }

        prevLtarget = LtargetRPM;
        prevRtarget = RtargetRPM;

        // Errors
        float errorLeft  = LtargetRPM - leftRPM;
        float errorRight = RtargetRPM - rightRPM;

        // Proportional
        float P_termLeft  = LKp * errorLeft;
        float P_termRight = RKp * errorRight;

        // Integral
        integralLeft  += errorLeft;
        integralRight += errorRight;

        // anti-windup
        integralLeft = constrain(integralLeft, -1200, 1200);
        integralRight = constrain(integralRight, -1200, 1200);

        float I_termLeft  = LKi * integralLeft;
        float I_termRight = RKi * integralRight;

        // Derivative
        float D_termLeft  = LKd * (errorLeft - previousErrorLeft);
        float D_termRight = RKd * (errorRight - previousErrorRight);

        // PID output to PWM (0-255)
        motorSpeedLeft  = constrain(P_termLeft + I_termLeft + D_termLeft, -3599, 3599);
        motorSpeedRight = constrain(P_termRight + I_termRight + D_termRight, -3599, 3599);

        // ---------------- MINIMUM START PWM ----------------
        //if(abs(motorSpeedLeft) < 600 && LcmdRPM != 0)
        //    motorSpeedLeft = 600 * (motorSpeedLeft > 0 ? 1 : -1);

        //if(abs(motorSpeedRight) < 600 && RcmdRPM != 0)
        //    motorSpeedRight = 600 * (motorSpeedRight > 0 ? 1 : -1);
        // ---------------- DEAD BAND ----------------
        if (abs(LtargetRPM) < 1) {
            motorSpeedLeft = 0;
            integralLeft = 0;
        }

        if (abs(RtargetRPM) < 1) {
            motorSpeedRight = 0;
            integralRight = 0;
        }

        // Apply motor output
        motor_set(motorSpeedLeft, motorSpeedRight);

        // Update previous error
        previousErrorLeft  = errorLeft;
        previousErrorRight = errorRight;

        lastTime = currentTime;
    }
}
