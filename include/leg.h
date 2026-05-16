#pragma once
#include <Arduino.h>
#include <joint.h>
#include <scara_ik.h>

struct Joint_config{
    int servo_pin;
    int neutral;
    double step;
    int dir
};

class Leg{

public:
    Leg(Joint_config configL, Joint_config configR){
        
    }

private:
    Joint_config config;
    Joint JL;
    Joint JR;
    ScaraIK sovler;

    void _init_joint(Joint_config config, Joint &servo){
        servo = Joint(config.servo_pin,config.step,config.neutral, config.dir);
    }

}