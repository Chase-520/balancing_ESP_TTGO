// /**
//  * @file    main.cpp
//  * @brief   Example usage of ScaraIK for a dual-arm SCARA robot on PlatformIO
//  *
//  * @details Demonstrates:
//  *            1. Basic IK solve and reading joint angles
//  *            2. Forward kinematics verification (round-trip)
//  *            3. Reachability guard before commanding motors
//  *            4. Elbow-down configuration
//  *            5. Custom geometry (runtime)
//  *            6. Commanding two stepper drivers (conceptual)
//  *
//  * @hardware  Any Arduino-compatible board (Uno, Mega, ESP32, STM32, RP2040 …)
//  *            Two stepper/servo drivers on the driver joints O1 and O2.
//  *
//  * Wiring assumption (adapt to your hardware):
//  *   STEP_PIN_L / DIR_PIN_L  →  left  driver (O1)
//  *   STEP_PIN_R / DIR_PIN_R  →  right driver (O2)
//  */

// #include <Arduino.h>
// #include "scara_ik.h"

// /* ─────────────────────────────────────────────────────────────────────────────
//  * Hardware pin definitions  (edit to match your board)
//  * ───────────────────────────────────────────────────────────────────────────*/
// static constexpr uint8_t STEP_PIN_L = 2;
// static constexpr uint8_t DIR_PIN_L  = 3;
// static constexpr uint8_t STEP_PIN_R = 4;
// static constexpr uint8_t DIR_PIN_R  = 5;

// /* ─────────────────────────────────────────────────────────────────────────────
//  * Motor constants  (edit to match your hardware)
//  * ───────────────────────────────────────────────────────────────────────────*/
// static constexpr float STEPS_PER_REV    = 3200.0f;  // steps/rev (16× microstepping)
// static constexpr float STEPS_PER_RADIAN = STEPS_PER_REV / (2.0f * M_PI);

// /* ─────────────────────────────────────────────────────────────────────────────
//  * Global solver instance — default geometry:
//  *   d=7.5 cm   l1=5.0 cm   l2=12.5 cm
//  * ───────────────────────────────────────────────────────────────────────────*/
// ScaraIK ik;

// /* Track current joint angles so we can compute relative moves */
// float currentTheta1 = 0.0f;
// float currentTheta2 = 0.0f;


// /* ─────────────────────────────────────────────────────────────────────────────
//  * Helper: move both drivers to absolute joint angles
//  *
//  * In real firmware you would hand these step counts to an AccelStepper
//  * or similar library.  Here we just print and toggle pins as a placeholder.
//  * ───────────────────────────────────────────────────────────────────────────*/
// void moveToAngles(float theta1, float theta2)
// {
//     // This funciton
// }


// /* ─────────────────────────────────────────────────────────────────────────────
//  * Core command: move end-effector to (px, py) in cm
//  *
//  * Returns true if the move was executed, false if unreachable.
//  * ───────────────────────────────────────────────────────────────────────────*/
// bool moveTo(float px, float py, ScaraElbow elbow = ELBOW_UP)
// {
//     Serial.print(F("\n[moveTo] target X="));
//     Serial.print(px, 3);
//     Serial.print(F(" cm  Y="));
//     Serial.print(py, 3);
//     Serial.println(F(" cm"));

//     ScaraIKResult result;

//     /* ── 1. Solve IK ─────────────────────────────────────────────────── */
//     if (!ik.solve(px, py, result, elbow)) {
//         Serial.println(F("  ✗ Target is UNREACHABLE — move aborted."));
//         return false;
//     }

//     Serial.print(F("  theta1 = "));
//     Serial.print(degrees(result.theta1), 3);
//     Serial.print(F("°   theta2 = "));
//     Serial.print(degrees(result.theta2), 3);
//     Serial.println(F("°"));

//     /* ── 2. Command motors ───────────────────────────────────────────── */
//     moveToAngles(result.theta1, result.theta2);

//     Serial.println(F("  ✓ Move complete."));
//     return true;
// }


// /* ─────────────────────────────────────────────────────────────────────────────
//  * setup() — runs once at power-on
//  * ───────────────────────────────────────────────────────────────────────────*/
// void setup()
// {
//     Serial.begin(115200);
//     while (!Serial) { /* wait for USB-CDC on Leonardo / ESP32-S2 etc. */ }
//     delay(500);

//     pinMode(STEP_PIN_L, OUTPUT);
//     pinMode(DIR_PIN_L,  OUTPUT);
//     pinMode(STEP_PIN_R, OUTPUT);
//     pinMode(DIR_PIN_R,  OUTPUT);

//     Serial.println(F("\n========================================"));
//     Serial.println(F("  Dual-Arm SCARA  —  ScaraIK Examples  "));
//     Serial.println(F("========================================"));
//     Serial.print(F("Geometry:  d="));  Serial.print(ik.baseDistance());
//     Serial.print(F(" cm   l1=")); Serial.print(ik.upperArm());
//     Serial.print(F(" cm   l2=")); Serial.print(ik.lowerArm());
//     Serial.println(F(" cm"));
//     Serial.print(F("Workspace: min reach=")); Serial.print(ik.minReach(), 2);
//     Serial.print(F(" cm   max reach="));     Serial.print(ik.maxReach(), 2);
//     Serial.println(F(" cm"));

//     /* ════════════════════════════════════════════════════════════════════
//      * EXAMPLE 1 — Basic IK solve
//      *   Ask for joint angles without moving any motor.
//      * ══════════════════════════════════════════════════════════════════*/
//     Serial.println(F("\n--- Example 1: Basic IK solve ---"));
//     {
//         ScaraIKResult r;
//         float targetX = 0.0f, targetY = 10.0f;

//         if (ik.solve(targetX, targetY, r)) {
//             Serial.print(F("Target (0, 10) cm  →  theta1="));
//             Serial.print(degrees(r.theta1), 2);
//             Serial.print(F("°   theta2="));
//             Serial.print(degrees(r.theta2), 2);
//             Serial.println(F("°"));
//         } else {
//             Serial.println(F("Unreachable."));
//         }
//     }

//     /* ════════════════════════════════════════════════════════════════════
//      * EXAMPLE 2 — Forward kinematics round-trip
//      *   Solve IK, then feed angles back into FK and verify P matches.
//      * ══════════════════════════════════════════════════════════════════*/
//     Serial.println(F("\n--- Example 2: FK round-trip verification ---"));
//     {
//         const float tx = 4.0f, ty = 9.0f;
//         ScaraIKResult ik_r;

//         if (ik.solve(tx, ty, ik_r)) {
//             ScaraFKResult fk_r;
//             if (ik.forwardK(ik_r.theta1, ik_r.theta2, fk_r)) {
//                 float errX = fabsf(fk_r.px - tx);
//                 float errY = fabsf(fk_r.py - ty);

//                 Serial.print(F("IK input : X="));  Serial.print(tx, 4);
//                 Serial.print(F("  Y="));           Serial.println(ty, 4);

//                 Serial.print(F("FK output: X="));  Serial.print(fk_r.px, 6);
//                 Serial.print(F("  Y="));           Serial.println(fk_r.py, 6);

//                 Serial.print(F("Elbow L  : ("));
//                 Serial.print(fk_r.e1x, 3); Serial.print(F(", "));
//                 Serial.print(fk_r.e1y, 3); Serial.println(F(")"));

//                 Serial.print(F("Elbow R  : ("));
//                 Serial.print(fk_r.e2x, 3); Serial.print(F(", "));
//                 Serial.print(fk_r.e2y, 3); Serial.println(F(")"));

//                 Serial.print(F("Error    : X="));
//                 Serial.print(errX, 2e-7);
//                 Serial.print(F("  Y="));
//                 Serial.println(errY, 2e-7);
//             }
//         }
//     }

//     /* ════════════════════════════════════════════════════════════════════
//      * EXAMPLE 3 — Reachability guard
//      *   Always check before commanding hardware.
//      * ══════════════════════════════════════════════════════════════════*/
//     Serial.println(F("\n--- Example 3: Reachability guard ---"));
//     {
//         struct { float x, y; } points[] = {
//             { 0.0f,  10.0f },   // reachable
//             { 0.0f,  20.0f },   // too far (> l1+l2 = 17.5)
//             { 0.0f,   1.0f },   // too close (< |l2-l1| = 7.5)
//             { 6.0f,  12.0f },   // reachable
//         };

//         for (auto &pt : points) {
//             Serial.print(F("  ("));
//             Serial.print(pt.x, 1); Serial.print(F(", "));
//             Serial.print(pt.y, 1); Serial.print(F(") -> "));
//             Serial.println(ik.isReachable(pt.x, pt.y)
//                            ? F("reachable") : F("UNREACHABLE"));
//         }
//     }

//     /* ════════════════════════════════════════════════════════════════════
//      * EXAMPLE 4 — Elbow-down configuration
//      *   Useful when the elbow-up path would collide with a fixture.
//      * ══════════════════════════════════════════════════════════════════*/
//     Serial.println(F("\n--- Example 4: Elbow-down vs elbow-up ---"));
//     {
//         ScaraIKResult up, down;
//         float tx = 2.0f, ty = 11.0f;

//         ik.solve(tx, ty, up,   ELBOW_UP);
//         ik.solve(tx, ty, down, ELBOW_DOWN);

//         Serial.print(F("Target (2, 11):"));
//         Serial.print(F("  ELBOW_UP   theta1="));
//         Serial.print(degrees(up.theta1),   2);
//         Serial.print(F("°  theta2="));
//         Serial.print(degrees(up.theta2),   2);
//         Serial.println(F("°"));

//         Serial.print(F("               ELBOW_DOWN theta1="));
//         Serial.print(degrees(down.theta1), 2);
//         Serial.print(F("°  theta2="));
//         Serial.print(degrees(down.theta2), 2);
//         Serial.println(F("°"));
//     }

//     /* ════════════════════════════════════════════════════════════════════
//      * EXAMPLE 5 — Custom geometry at runtime
//      *   Instantiate a second solver with different arm lengths.
//      * ══════════════════════════════════════════════════════════════════*/
//     Serial.println(F("\n--- Example 5: Custom geometry ---"));
//     {
//         ScaraIK customIK(10.0f, 8.0f, 15.0f);   // d=10, l1=8, l2=15

//         ScaraIKResult r;
//         if (customIK.solve(0.0f, 14.0f, r)) {
//             Serial.print(F("Custom robot solve(0,14): theta1="));
//             Serial.print(degrees(r.theta1), 2);
//             Serial.print(F("°  theta2="));
//             Serial.print(degrees(r.theta2), 2);
//             Serial.println(F("°"));
//         }
//     }

//     /* ════════════════════════════════════════════════════════════════════
//      * EXAMPLE 6 — Move through a sequence of waypoints
//      *   The moveTo() helper calls IK + drives the steppers.
//      * ══════════════════════════════════════════════════════════════════*/
//     Serial.println(F("\n--- Example 6: Waypoint sequence ---"));

//     struct Waypoint { float x, y; };
//     Waypoint path[] = {
//         { 0.0f, 10.0f },
//         { 4.0f,  9.0f },
//         { 0.0f, 13.0f },
//         {-4.0f,  9.0f },
//         { 0.0f, 10.0f },   // return home
//     };

//     for (auto &wp : path) {
//         moveTo(wp.x, wp.y);
//         delay(200);   // brief pause between waypoints
//     }

//     Serial.println(F("\n========================================"));
//     Serial.println(F("  All examples complete."));
//     Serial.println(F("  Send any character to re-run loop()."));
//     Serial.println(F("========================================"));
// }


// /* ─────────────────────────────────────────────────────────────────────────────
//  * loop() — interactive Serial command
//  *
//  * Send:   X Y<newline>     e.g.  "3.5 11.0"
//  * The robot will solve IK and drive to that position.
//  * ───────────────────────────────────────────────────────────────────────────*/
// void loop()
// {
//     if (Serial.available() == 0) return;

//     float px = Serial.parseFloat();
//     float py = Serial.parseFloat();
//     Serial.read();   // consume newline

//     moveTo(px, py);
// }