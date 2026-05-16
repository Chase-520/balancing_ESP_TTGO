#include <Arduino.h>
#include <joint.h>
#include <scara_ik.h>
#include <imu.h>
#include <N20.h>
#include <leg.h>
#include <PID_v1.h>

// stuctures
struct PID_config{
    double Kp;
    double Ki;
    double Kd;
};
// --- Configuration ---
static const int J1_SERVO_PIN    = 4;
static const int J1_NEUTRAL   = 1720;  
static const double J1_STEP = 324.6760839;  
static const int J1_DIR = -1;

static const int J2_SERVO_PIN    = 25;
static const int J2_NEUTRAL   = 3110;  
static const double J2_STEP = -534.7606088; 
static const int J2_DIR = 1;

static const int L_DIR_PIN = 5;
static const int L_PWM_PIN = 24;
static const bool L_INVERSED = false;

static const int R_DIR_PIN = 5;
static const int R_PWM_PIN = 24;
static const bool R_INVERSED = false;

static const double leg_kp = 0.4;
static const double leg_ki = 0.05;
static const double leg_kd = 0.01;

static const double wheel_kp = 8.0;
static const double wheel_ki = 1.0;
static const double wheel_kd = 0.5;

Joint_config J1_config{J1_SERVO_PIN,J1_NEUTRAL,J1_STEP,J1_DIR};
Joint_config J2_config{J2_SERVO_PIN,J2_NEUTRAL,J2_STEP,J2_DIR};

N20_config N20_left_config{L_DIR_PIN,L_PWM_PIN,L_INVERSED};
N20_config N20_right_config{R_DIR_PIN,R_PWM_PIN,R_INVERSED};

PID_config leg_pid_config{leg_kp,leg_ki,leg_kd};
ScaraIKResult left_leg_desire_pos;
ScaraIKResult right_leg_desire_pos;

// Setup Legs
Leg right_leg = Leg(J2_config,J1_config);

// Setup N20 motors
N20 LN20 = N20(L_DIR_PIN,L_PWM_PIN,L_INVERSED);
N20 RN20 = N20(L_DIR_PIN,L_PWM_PIN,L_INVERSED);

// Setup inverse kinematic solver
ScaraIK solver = ScaraIK();

//Track last loop time
unsigned long lastTime = 0;
float sampleTime = 0.01; //10ms sample time

// Joint PID
double R_L_input, R_L_output, R_L_setpoint;
PID right_leg_PID(&R_L_input, &R_L_output, &R_L_setpoint, leg_pid_config.Kp, leg_pid_config.Ki, leg_pid_config.Kd, AUTOMATIC);


// Robot states
struct RS{
    double left_leg_height;
    double right_leg_height;
};

RS robot_state{12,12};

// --------------func declarations ----------------
void init_PID_all();
void update_PID_input();
void compute_PIDs();

// ---------------func realization --------------------
void init_PID_all(){
    R_L_setpoint = -90;         // target pitch (e.g., level)
    R_L_input = -90;    // initialize with 0 error
    right_leg_PID.SetMode(AUTOMATIC);
    right_leg_PID.SetSampleTime(10); //  10ms loop time for outer PID, ~100 Hz
    right_leg_PID.SetOutputLimits(-20, 20); 
}

void update_PID_input(){
    R_L_input = ypr.roll;
}

void compute_PIDs(){
    if(right_leg_PID.Compute()){
        right_leg.set_end_effctor(R_L_output,robot_state.right_leg_height);
    }
}

// ---------------Main func ------------------
void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 Servo - Serial end effector Control ===");
    init_IMU();
    init_PID_all();
    lastTime = millis();
}

void loop() {
    imuLoop(); // update ypr
    update_PID_input();
    unsigned long currentTime = millis();
    // float deltaTime = (currentTime - lastTime) / 1000.0; // into seconds
    compute_PIDs();

    // System debug
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime > 500) {
        Serial.print("Roll: ");        Serial.println(ypr.roll);
        Serial.print("Setpoint: ");    Serial.println(R_L_setpoint);
        Serial.print("J_input: ");     Serial.println(R_L_input);
        Serial.print("J_output: ");    Serial.println(R_L_output);
        Serial.print("PID mode: ");    Serial.println(right_leg_PID.GetMode());
        lastPrintTime = currentTime;
    }

    lastTime = currentTime; // add this at the end of loop()
}