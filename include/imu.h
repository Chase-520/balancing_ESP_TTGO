#pragma once

#include <Adafruit_BNO08x.h>

// Setup IMU
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1
long reportIntervalUs = 5000; //200 Hz
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

void init_IMU(){

    if(!bno08x.begin_I2C()){
        Serial.println("Failed to find BNO08x chip");
        while(1){delay(10);}
    }

    Serial.println("IMU Connected!");
    bno08x.enableReport(reportType,reportIntervalUs);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);
  
  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  
  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void imuLoop(){
    if (bno08x.getSensorEvent(&sensorValue)) {
    // 将四元数转换为欧拉角
        switch (sensorValue.sensorId) {
            case SH2_ARVR_STABILIZED_RV:
            quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
            break;
            case SH2_GYRO_INTEGRATED_RV:
            quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
            break;
            }  
    }     
}