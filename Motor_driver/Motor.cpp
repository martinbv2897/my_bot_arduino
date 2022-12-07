#include "Arduino.h"
#include "Motor.h"


Motor::Motor(int in1, int in2, int en, int en_a,int en_b) {
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(en,OUTPUT);
  pinMode(en_a,INPUT);
  pinMode(en_b,INPUT);
  Motor::in1 = in1;
  Motor::in2 = in2;
  Motor::en = en;
  Motor::en_a = en_a;
  Motor::en_b = en_b;
}

void Motor::rotate(int value) {
  if(value>=0){
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
//    Serial.println("called");
//    Serial.println(plus);
    
    //Serial.print("value");Serial.println(value);
    
    int out = map(value, 0, 100, 0, 225);
    //Serial.print("put");Serial.println(out);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en,out);
    
  }else{
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
    int out = map(value, 0, -100, 0, 225);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en,out);
  }
}
