#include <mpu_6050.hpp>

float yaw = 0.0f;
float heading = 0.0;
float gyroBiasZ = 0.0;
unsigned long prevTime = 0;

static Adafruit_MPU6050 mpu;

void calibrateGyro()
{
    Serial.println("Calibrating gyroscope...");
    float sum = 0.0;
    int samples = 1000;

    for (int i = 0; i < samples; i++)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.z;
        delay(5);
    }

    gyroBiasZ = sum / samples;
}

void mpu_init(){
     Wire.setSCL(PB8);
     Wire.setSDA(PB9);
     Wire.begin();

     Serial.println("Adafruit MPU6050 test!");
     if(!mpu.begin()){
          Serial.println("Failed to find MPU6050 chip");
          while(1){
               delay(10);
          }
     }
     Serial.println("MPU6050 initialized!");
     
     mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
     mpu.setGyroRange(MPU6050_RANGE_500_DEG);
     mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

     delay(100);
     calibrateGyro();
     prevTime = millis();
}

void mpu_update(){
     sensors_event_t a, g, temp;
     mpu.getEvent(&a, &g, &temp);

     float gyroZ = (g.gyro.z - gyroBiasZ);

     unsigned long now = millis();
     float dt = (now - prevTime) / 1000.0f;  // seconds
     prevTime = now;

     // integrate
     heading += gyroZ * dt;

     yaw = heading;

     // Serial.print("Yaw (rad): ");
     // Serial.println(yaw, 6);
}