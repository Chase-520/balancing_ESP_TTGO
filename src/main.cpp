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
static const int J1_NEUTRAL   = 2270;  
static const double J1_STEP = -315.1267873;  

static const int J2_SERVO_PIN    = 25;
static const int J2_NEUTRAL   = 1484;  
static const double J2_STEP = 307.4873501; 

static const int J3_SERVO_PIN    = 2;
static const int J3_NEUTRAL   = 2920;  
static const double J3_STEP = -534.7606088; 

static const int J4_SERVO_PIN    = 15;
static const int J4_NEUTRAL   = 1770;  
static const double J4_STEP = 534.7606088; 

static const int R_DIR_PIN = 14; //10
static const int R_PWM_PIN = 35; //8
static const bool R_INVERSED = false;

static const int L_DIR_PIN = 13; //9
static const int L_PWM_PIN = 12; //7
static const bool L_INVERSED = true;

String inputBuffer = "";
// Setup joints
Joint J1 = Joint(J1_SERVO_PIN,J1_STEP,J1_NEUTRAL,1);
Joint J2 = Joint(J2_SERVO_PIN,J2_STEP,J2_NEUTRAL,-1);
Joint J3 = Joint(J3_SERVO_PIN,J3_STEP,J3_NEUTRAL,1);
Joint J4 = Joint(J4_SERVO_PIN,J4_STEP,J4_NEUTRAL,-1);

// Setup wheel
N20 RN20 = N20(R_DIR_PIN,R_PWM_PIN,R_INVERSED);
N20 LN20 = N20(L_DIR_PIN,L_PWM_PIN,L_INVERSED);

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
double J_L_input, J_L_output, J_L_setpoint;
double J_R_input, J_R_output, J_R_setpoint;
double J_Kp=0.1, J_Ki=0.0, J_Kd=0.01;
PID right_leg_pid(&J_R_input, &J_R_output, &J_R_setpoint, J_Kp, J_Ki, J_Kd, AUTOMATIC);
PID left_leg_pid(&J_L_input, &J_L_output, &J_L_setpoint, J_Kp, J_Ki, J_Kd, AUTOMATIC);

// Wheel PID
double W_L_input, W_L_output, W_L_setpoint;
double W_R_input, W_R_output, W_R_setpoint;
double W_Kp=5, W_Ki=0.05, W_Kd=0.5;
PID left_wheel_pid(&W_L_input, &W_L_output, &W_L_setpoint, W_Kp, W_Ki,W_Kd,AUTOMATIC);
PID right_wheel_pid(&W_R_input, &W_R_output, &W_R_setpoint, W_Kp, W_Ki,W_Kd,AUTOMATIC);

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
double r_height = 8; //10cm above the ground

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
    

}

void init_PIDs(){
  J_L_setpoint = -90;         // target pitch (e.g., level)
  J_L_input = -90;    // current pitch from IMU
  J_R_setpoint = -90;        
  J_R_input = -90; 

  left_leg_pid.SetMode(AUTOMATIC);
  // Add this in setup()
  left_leg_pid.SetSampleTime(10); // match your 10ms intention
  // Add this in setup() - set to whatever x range your IK solver expects
  left_leg_pid.SetOutputLimits(-20, 20); // example: 

  right_leg_pid.SetMode(AUTOMATIC);
  // Add this in setup()
  right_leg_pid.SetSampleTime(10); // match your 10ms intention
  // Add this in setup() - set to whatever x range your IK solver expects
  right_leg_pid.SetOutputLimits(-20, 20); // example: 

  W_L_setpoint = -90;         // target pitch (e.g., level)
  W_L_input = -90;    // current pitch from IMU
  W_R_setpoint = -90;        
  W_R_input = -90; 

  left_wheel_pid.SetMode(AUTOMATIC);
  left_wheel_pid.SetSampleTime(5); 
  left_wheel_pid.SetOutputLimits(-100, 100); 

  right_wheel_pid.SetMode(AUTOMATIC);
  right_wheel_pid.SetSampleTime(5); 
  right_wheel_pid.SetOutputLimits(-100, 100); 
}

void init_WIFI(){
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
  Serial.println("ESP-NOW init failed");
  return;
  }

  esp_now_register_recv_cb(onReceive);
}

void comput_loop(){
  // RN20.setSpeed(60);
    ScaraIKResult desire_pos;
    if(right_leg_pid.Compute()){
        // Calculate Inverse Kinematics
        if(solver.solve(-J_R_output,r_height, desire_pos)){    
            //Write to servos
            J1.move_to_pos(desire_pos.theta1);
            J2.move_to_pos(desire_pos.theta2);
        }
    }

    if(left_leg_pid.Compute()){
        // Calculate Inverse Kinematics
        if(solver.solve(J_L_output,r_height, desire_pos)){    
            //Write to servos
            J3.move_to_pos(desire_pos.theta1);
            J4.move_to_pos(desire_pos.theta2);
        }
    }

    if(right_wheel_pid.Compute()){
        RN20.setSpeed(W_R_output);
    }

    if(left_wheel_pid.Compute()){
      LN20.setSpeed(W_L_output);
    }

}
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 Servo – Serial end effector Control ===");
  init_IMU();
  init_PIDs();

}

void loop() {
    imuLoop();
    // update pid inputs
    J_L_input = ypr.roll;    
    J_R_input = ypr.roll; 
    W_L_input = ypr.roll;
    W_R_input = ypr.roll;
    unsigned long currentTime = millis();
    // float deltaTime = (currentTime - lastTime) / 1000.0; // into seconds
    comput_loop();
    

    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime > 500) {
        Serial.print("Roll: ");        Serial.println(ypr.roll);
        Serial.print("Setpoint: ");    Serial.println(J_R_setpoint);
        Serial.print("J_input: ");     Serial.println(J_R_input);
        Serial.print("J_output: ");    Serial.println(J_R_output);
        lastPrintTime = currentTime;
    }

    lastTime = currentTime; // add this at the end of loop()
}