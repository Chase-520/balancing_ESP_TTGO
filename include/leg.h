#pragma once
#include <Arduino.h>
#include <joint.h>
#include <scara_ik.h>

struct Joint_config{
    int servo_pin;
    int neutral;
    double step;
    int dir;
};

class Leg{

public:
    Leg(Joint_config& configL, Joint_config& configR){
        _init_joint(configL,JL);
        _init_joint(configR,JR);
    }

    void set_end_effctor(double& px, double& py){
        solver.solve(px,py, desire_pos);
        //Write to servos
        JL.move_to_pos(desire_pos.theta2);
        JR.move_to_pos(desire_pos.theta1);
    }

private:
    Joint_config config;
    Joint JL;
    Joint JR;
    ScaraIK solver;
    ScaraIKResult desire_pos;

    void _init_joint(Joint_config& config, Joint& servo){
        servo = Joint(config.servo_pin,config.step,config.neutral, config.dir);
    }

};