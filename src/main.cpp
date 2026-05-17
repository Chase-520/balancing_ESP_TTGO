#include <Arduino.h>
#include <joint.h>
#include <scara_ik.h>
#include <Adafruit_BNO08x.h>
#include <PID_v1.h>
#include <N20.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
// --- Configuration ---
static const int J1_SERVO_PIN    = 4;
static const int J1_NEUTRAL   = 1720;  
static const double J1_STEP = 324.6760839;  

static const int J2_SERVO_PIN    = 25;
static const int J2_NEUTRAL   = 3110;  
static const double J2_STEP = -534.7606088; 

static const int R_DIR_PIN = 2; //9
static const int R_PWM_PIN = 15; //7
static const bool R_INVERSED = false;

String inputBuffer = "";
// Setup joints
Joint J1 = Joint(J1_SERVO_PIN,J1_STEP,J1_NEUTRAL,-1);
Joint J2 = Joint(J2_SERVO_PIN,J2_STEP,J2_NEUTRAL,1);

// Setup wheel
N20 RN20 = N20(R_DIR_PIN,R_PWM_PIN,R_INVERSED);

// Setup inverse kinematic solver
ScaraIK solver = ScaraIK();

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

//Track last loop time
unsigned long lastTime = 0;
float sampleTime = 0.01; //10ms sample time

// Joint PID
double J_input, J_output, J_setpoint;
double Kp=0.4, Ki=0.05, Kd=0.01;
PID myPID(&J_input, &J_output, &J_setpoint, Kp, Ki, Kd, DIRECT);

// ---------- ESPNow wifi receive
typedef struct {
  int value;
} Data;

Data data;

void onReceive(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));

  Serial.print("Received: ");
  Serial.println(data.value);
}

// Robot states
double r_height = 10; //10cm above the ground

void init_IMU(){

    if(!bno08x.begin_I2C()){
        Serial.println("Failed to find BNO08x chip");
        while(1){delay(10);}
    }

    Serial.println("Connect to IMU!");
    bno08x.enableReport(reportType,reportIntervalUs);
    lastTime = millis();
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
    J_input = ypr.roll;    //Update J_input

}

void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 Servo – Serial end effector Control ===");
    init_IMU();

    J_setpoint = -90;         // target pitch (e.g., level)
    J_input = ypr.roll;    // current pitch from IMU
    myPID.SetMode(AUTOMATIC);
    // Add this in setup()
    myPID.SetSampleTime(10); // match your 10ms intention
    // Add this in setup() - set to whatever x range your IK solver expects
    myPID.SetOutputLimits(-20, 20); // example: 
    
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
    }

    esp_now_register_recv_cb(onReceive);

}

void loop() {
    imuLoop();
    unsigned long currentTime = millis();
    // float deltaTime = (currentTime - lastTime) / 1000.0; // into seconds

    RN20.setSpeed(60);
    ScaraIKResult desire_pos;
    if(myPID.Compute()){
        // Calculate Inverse Kinematics
        if(solver.solve(J_output,r_height, desire_pos)){    
            //Write to servos
            J1.move_to_pos(desire_pos.theta2);
            J2.move_to_pos(desire_pos.theta1);
        }
    }

    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime > 500) {
        Serial.print("Roll: ");        Serial.println(ypr.roll);
        Serial.print("Setpoint: ");    Serial.println(J_setpoint);
        Serial.print("J_input: ");     Serial.println(J_input);
        Serial.print("J_output: ");    Serial.println(J_output);
        Serial.print("PID mode: ");    Serial.println(myPID.GetMode());
        lastPrintTime = currentTime;
    }

    lastTime = currentTime; // add this at the end of loop()
}