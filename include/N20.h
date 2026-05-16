#pragma once

#include <Arduino.h>

class N20{

public:
    N20(int dir, int pwm, bool inverse=false){
        /*
        int _dir_pin: direction pin, HIGH or LOW
        int _pwm_pin: pwm pin, range 0-255
        bool _inverse: whether to inver motor dirction or not
        */
        _dir_pin = dir;
        _pwm_pin = pwm;
        _inverse = inverse;       

        _attach();

    }

    void setSpeed(int speed) {
        // 确保速度在0-255范围内
        bool cur_dir = true;
        if (speed > 255) {
            speed = 255;
        }
        if (speed < -255) {
            speed = -255;
            cur_dir = false;
        }
        
        if(_inverse)cur_dir=!cur_dir;
        // 设置方向
        digitalWrite(_dir_pin, cur_dir ? HIGH : LOW);
        
        // 设置PWM速度
        analogWrite(_pwm_pin, speed);
    }



private:
    /*
    int _dir_pin: direction pin, HIGH or LOW
    int _pwm_pin: pwm pin, range 0-255
    bool _inverse: whether to inver motor dirction or not
    */
    int _dir_pin;
    int _pwm_pin;
    bool _inverse;

    void _attach(){
        pinMode(_dir_pin, OUTPUT);
        pinMode(_pwm_pin, OUTPUT);
    }
}