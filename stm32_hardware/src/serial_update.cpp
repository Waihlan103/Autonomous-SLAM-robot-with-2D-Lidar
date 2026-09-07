#include "serial_update.hpp"
#include "pid_control.hpp"
#include "encoder.hpp"
#include "mpu_6050.hpp"
#include "math_constants.hpp"

void serial_read() {
    static uint8_t rx_buf[6];
    static uint8_t rx_idx = 0;

    while (Serial.available()) {
        uint8_t b = Serial.read();

        // wait for header
        if (rx_idx == 0) {
            if (b != 0xAA) continue;
        }

        rx_buf[rx_idx++] = b;

        if (rx_idx == 6) {
            // compute checksum
            uint8_t sum = 0;
            for (int i = 0; i < 5; i++) sum += rx_buf[i];

            if (rx_buf[5] == sum) {
                // reconstruct int16_t wheel commands
                int16_t left_raw  = rx_buf[1] | (rx_buf[2] << 8);
                int16_t right_raw = rx_buf[3] | (rx_buf[4] << 8);

                // scaled → rad/s
                float left_rad_s = left_raw  / 1000.0f;
                float right_rad_s = right_raw / 1000.0f;

                // convert → RPM for PID
                LtargetRPM = left_rad_s  * 60.0f / (2.0f * math_const::PI_F);
                RtargetRPM = right_rad_s * 60.0f / (2.0f * math_const::PI_F);
            }
            rx_idx = 0; // reset for next packet
        }
    }
}

void serial_write(){
    uint8_t tx[14];

    tx[0] = 0x55;

    int32_t enc_l = encoder_left;
    int32_t enc_r = encoder_right;
    float yaw_f = yaw;

    memcpy(&tx[1], &enc_l, 4);
    memcpy(&tx[5], &enc_r, 4);
    memcpy(&tx[9], &yaw_f, 4);

    uint8_t checksum = 0;
    for(int i=1;i < 13;i++){
        checksum += tx[i];
    }
    tx[13] = checksum;

    Serial.write(tx, sizeof(tx));
}