#include <Arduino.h>
#include <joint.h>
#include <scara_ik.h>
#include <imu.h>
#include <N20.h>
#include <PID_v1.h>
// --- Configuration ---
static const int J1_SERVO_PIN    = 4;
static const int J1_NEUTRAL   = 1720;  
static const double J1_STEP = 324.6760839;  

static const int J2_SERVO_PIN    = 25;
static const int J2_NEUTRAL   = 3110;  
static const double J2_STEP = -534.7606088; 

static const int L_DIR_PIN = 5;
static const int L_PWM_PIN = 24;
static const bool L_INVERSED = false;

static const int R_DIR_PIN = 5;
static const int R_PWM_PIN = 24;
static const bool R_INVERSED = false;

String inputBuffer = "";
// Setup joints
Joint J1 = Joint(J1_SERVO_PIN,J1_STEP,J1_NEUTRAL,-1);
Joint J2 = Joint(J2_SERVO_PIN,J2_STEP,J2_NEUTRAL,1);

// Setup N20 motors
N20 LN20 = N20(L_DIR_PIN,L_PWM_PIN,L_INVERSED);
N20 RN20 = N20(L_DIR_PIN,L_PWM_PIN,L_INVERSED);

// Setup inverse kinematic solver
ScaraIK solver = ScaraIK();

//Track last loop time
unsigned long lastTime = 0;
float sampleTime = 0.01; //10ms sample time

// Joint PID
double J_input, J_output, J_setpoint;
double Kp=0.4, Ki=0.05, Kd=0.01;
PID myPID(&J_input, &J_output, &J_setpoint, Kp, Ki, Kd, DIRECT);

// Robot states
double r_height = 10; //10cm above the ground


void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 Servo – Serial end effector Control ===");
    init_IMU();
    J_setpoint = -90;         // target pitch (e.g., level)
    J_input = -90;    // initialize with 0 error
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(10); //  10ms loop time for outer PID, ~100 Hz
    myPID.SetOutputLimits(-20, 20); 
    lastTime = millis();
}

void loop() {
    imuLoop(); // update ypr
    J_input = ypr.roll; // give latest pitch of robot to controller
    unsigned long currentTime = millis();
    // float deltaTime = (currentTime - lastTime) / 1000.0; // into seconds

    
    ScaraIKResult desire_pos;
    // computer PID for joint
    if(myPID.Compute()){
        // Calculate Inverse Kinematics
        if(solver.solve(J_output,r_height, desire_pos)){
  
            //Write to servos
            J1.move_to_pos(desire_pos.theta2);
            J2.move_to_pos(desire_pos.theta1);
        }
    }


    // System debug
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