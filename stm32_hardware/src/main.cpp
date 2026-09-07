#include <Arduino.h>
#include "motor.hpp"
#include "encoder.hpp"
#include "pid_control.hpp"
#include "serial_update.hpp"
#include "mpu_6050.hpp"

unsigned long last_tx = 0;
const unsigned long tx_interval = 20;

void setup() {
  Serial.begin(115200);
  pwm_init();
  encoder_init();
  mpu_init();
}

void loop() {
  serial_read();
  PID_control();
  encoder_update();
  mpu_update();
  
  if(millis() - last_tx > tx_interval){
    serial_write();
    last_tx = millis();
  }
}
