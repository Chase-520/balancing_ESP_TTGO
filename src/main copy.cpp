// #include <Arduino.h>
// #include <joint.h>
// #include <scara_ik.h>
// // --- Configuration ---
// static const int J1_SERVO_PIN    = 4;
// static const int J1_NEUTRAL   = 1720;  
// static const double J1_STEP = 324.6760839;  


// String inputBuffer = "";
// // Setup joints
// Joint J1 = Joint(J1_SERVO_PIN,J1_STEP,J1_NEUTRAL,-1);

// // Setup inverse kinematic solver
// ScaraIK solver = ScaraIK();

// void setup() {
//     Serial.begin(115200);
//     Serial.println("=== ESP32 Servo – Serial end effector Control ===");
// }
// void loop() {
//     while (Serial.available()) {
//         char c = (char)Serial.read();

//         if (c == '\n' || c == '\r') {
//             inputBuffer.trim();

//             if (inputBuffer.length() > 0) {

//                 int commaIndex = inputBuffer.indexOf(',');

//                 if (commaIndex != -1) {

//                     String firstStr  = inputBuffer.substring(0, commaIndex);
//                     String secondStr = inputBuffer.substring(commaIndex + 1);

//                     float value1 = firstStr.toFloat();
//                     float value2 = secondStr.toFloat();

//                     Serial.print("Value 1: ");
//                     Serial.println(value1);

//                     Serial.print("Value 2: ");
//                     Serial.println(value2);

//                     // Calculate Inverse Kinematics
//                     ScaraIKResult desire_pos;
//                     if(solver.solve(value1,value2, desire_pos)){
//                         // debug
//                         Serial.print("Desire angle (rad): ");
//                         Serial.println(desire_pos.theta2);

//                         //Write to servos
//                         J1.move_to_pos(desire_pos.theta2);
//                     }else{
//                         Serial.println("unreachable");
//                     }

                    


//                 } else {
//                     Serial.println("Invalid format. Use: x,y");
//                 }
//             }

//             inputBuffer = "";
//         } else {
//             inputBuffer += c;
//         }
//     }
// }