#include "Arduino.h"
#include "pid.h"


pid::pid(double kp ,double ki, double kd) {

  pid::kp = kp;
  pid::ki= ki;
  pid::kd = kd;
  unsigned long t_act,t_ant;
  
}

int pid::compute(double sp,double vel) {
   
   double out,out1;
   double rate_error,last_error,error,acum_error;
   //while(vel!=sp){
   t_act = millis();
   error = sp - vel;
   //error = abs(error);
   rate_error = (error - last_error)/(t_act-t_ant);
   acum_error += error*(t_act-t_ant); 
   out = kp*error + kd*rate_error + ki*acum_error;
   //Serial.print("out");Serial.print(out);
   if (sp>0)
   out = map(out,0,5000,0,100);
   else
   out = map(out,0,-5000,0,-100);
   //Serial.print("/////out_pond");Serial.println(out);
   last_error = error;
   t_ant = t_act;
   return out;
//    if(vel < sp){
//      out = vel+kp;
//      //Serial.println("sumando");
//      return out;
//    }
//    else if(vel > sp){
//      out = vel-kp;
//      //Serial.print("restando");
//      return out;
//    }
     
    
   //}
}
