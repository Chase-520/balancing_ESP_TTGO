// #include <Arduino.h>
// #include <ESP32Servo.h>

// // --- Configuration ---
// static const int SERVO_PIN    = 4;
// static const int PWM_MIN_US   = 900;   // minimum accepted pulse width (µs)
// static const int PWM_MAX_US   = 2100;  // maximum accepted pulse width (µs)
// static const int PWM_CENTER_US = (PWM_MIN_US + PWM_MAX_US) / 2;  // 1500 µs

// Servo servo;
// String inputBuffer = "";

// void applyPwm(int pwmUs) {
//     // Clamp to valid range
//     pwmUs = constrain(pwmUs, PWM_MIN_US, PWM_MAX_US);
//     servo.writeMicroseconds(pwmUs);
//     Serial.print("PWM set to: ");
//     Serial.print(pwmUs);
//     Serial.println(" µs");
// }

// void setup() {
//     Serial.begin(115200);
//     Serial.println("=== ESP32 Servo – Serial PWM Control ===");
//     Serial.print("Send a pulse width in µs (");
//     Serial.print(PWM_MIN_US);
//     Serial.print(" – ");
//     Serial.print(PWM_MAX_US);
//     Serial.println(") followed by Enter.");

//     servo.setPeriodHertz(50);                           // standard 50 Hz servo signal
//     servo.attach(SERVO_PIN, PWM_MIN_US, PWM_MAX_US);

//     // Centre on startup
//     applyPwm(PWM_CENTER_US);
// }

// void loop() {
//     // Read serial input one character at a time; act on newline
//     while (Serial.available()) {
//         char c = (char)Serial.read();

//         if (c == '\n' || c == '\r') {
//             inputBuffer.trim();
//             if (inputBuffer.length() > 0) {
//                 int pwmUs = inputBuffer.toInt();

//                 if (pwmUs < PWM_MIN_US || pwmUs > PWM_MAX_US) {
//                     Serial.print("Error: value out of range. Enter ");
//                     Serial.print(PWM_MIN_US);
//                     Serial.print(" – ");
//                     Serial.print(PWM_MAX_US);
//                     Serial.println(".");
//                 } else {
//                     applyPwm(pwmUs);
//                 }
//             }
//             inputBuffer = "";
//         } else {
//             inputBuffer += c;
//         }
//     }
// }