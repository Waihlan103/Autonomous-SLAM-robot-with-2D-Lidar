#ifndef MPU_6050_HPP
#define MPU_6050_HPP
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

extern float yaw;
extern float heading;
extern float gyroBiasZ;

void mpu_init();
void mpu_update();
#endif