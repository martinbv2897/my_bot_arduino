/*
  Motor.h - Library for working with the Cytron SPG30E-30K.
  Created by Vinay Lanka, January 27, 2021.
*/
#ifndef pid_h
#define pid_h

#include "Arduino.h"

class pid {
  public:
    //Constructor - Plus and Minus are the Motor output / en_a and en_b are the encoder inputs
    
    pid(double kp ,double ki, double kd);
    //Spin the motor with a percentage value
    int compute(double sp,double vel);
    //control Inputs- plus is one direction and minus is the other
    double kp;
    double ki;
    double kd;
    unsigned long t_act,t_ant;
    
};

#endif
