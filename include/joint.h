#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#ifndef PI_INVERSE
#define PI_INVERSE 0.318309886f
#endif
class Joint{
public:
    Joint(){}
    Joint(int pin, double step, int station_pwm ,int dir){
        /*
            int pin: pin for this servo
            double step: conversion between pwm and radians (pwm/rad)
            int station_pwm: pwm value when joint pointing at 0 rad
            int dir: placement of joint, right is -1, left is 1
        */
        attachServo(pin);
        _pwm_at_zero = station_pwm;
        _step = step;
        _dir = dir;
    }

    void move_to_pos(double ang){
        /*Drive servo to desire angle in randians, assuming +x is 0*/
        radToPWM(ang);
        // Serial.print("moving to ");
        // Serial.print(ang);
        // Serial.print(" with pwm ");
        // Serial.println(_pos);
        _servo.writeMicroseconds(constrain(_pos,900,2100));
    }

private:
    /*
    servo: Servo object
    _step: store step (pwm/rad)
    _dir: store dir
    _pos: store pwm to write to the servo
    */
    Servo _servo;
    double _step;
    int _pwm_at_zero;
    int _dir;
    int _pos;

    void attachServo(int p){
        _servo.setPeriodHertz(50);
        _servo.attach(p,900,2100);
    }

    void radToPWM(double rad){
        _pos = _pwm_at_zero + _step * _dir * rad;
    }
};