#include "Arduino.h"
#include "pid.h"


pid::pid() {
  pid::kp = kp;
  pid::ki = ki;
  pid::kd = kd;
  pid::moving =moving;
  unsigned long t_act,t_ant;
  
}

double pid::compute(double sp,double vel) {
   
   double out,out1;
   double rate_error,last_error,error,acum_error;
   if (moving == 1) {
   //Serial.print("kp: ");Serial.print(kp);Serial.print(" kd: ");Serial.print(kd);Serial.print("  ki: ");Serial.println(ki);
   //while(vel!=sp){
   t_act = millis();
   error = sp - vel;
   //error = abs(error);
   rate_error = (error - last_error)/(t_act-t_ant);
   acum_error += error*(t_act-t_ant); 
   out = (kp*error + kd*rate_error + ki*acum_error);///100000;
   //Serial.print("out ");Serial.println(out);
   
   }else{
    out = sp;
   }
   last_error = error;
   t_ant = t_act;
   return out;
}
