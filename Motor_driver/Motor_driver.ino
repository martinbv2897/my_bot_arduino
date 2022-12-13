

//Se importan todas las librerias 
#include "Motor.h"
#include "pid.h"
#include "commands.h"
///////////////////////

#define BAUDRATE 115200

//variables para decodificar los comandos recibidos en el serial
int arg = 0;
int index = 0;

// variable para retener un char
char chr;

char cmd;
char cmd1[16];

char argv1[16];
char argv2[16];
String auxstr;
String strs[4];
double arg1;
double arg2;

///////////////////////
float long_eje = 0.2275; // distancias entre ruedas en metros
const byte MOTOR_IZQ = 2;  // izq//Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_DER = 3;  // Motor 1 Interrupt Pin - INT 0 - Left Motor


#define LOOPTIME 10 
////////////////////
//     (in1,in2,enb,encoder)
//(       in1, in2, en, en_a, en_b)
Motor right(9,   10,  8 ,  3,  48);
Motor left( 6,    7,  5 ,  2,  49);

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2
/// Constantes para el control PID/////////
volatile double sp1,sp2; ///     
                                         
double left_kp =10, left_ki = 0.95 , left_kd = 10.2;             // modify for optimal performance
double right_kp = 14.1 , right_ki = 1.03 , right_kd = 10;

double right_input = 0, right_output = 0, right_setpoint = 0;
pid control_der;

double left_input = 0, left_output = 0, left_setpoint = 0;
pid control_izq;
////////////////////////////////////////////////////////////////////////////////////////
/// Variables para medir las velocidades del motor//////////////////////////////////////
double v_lineal = 0;
double v_ang = 0;
double R = 0;
double v_med,v_ang_med;

unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff;
float encoder1Diff;

float encoder0Count;
float encoder1Count;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

double num_enc_count_0 =0;
double num_enc_count_1 =0;

double speed_act_left = 0;   //Actual speed for left wheel in m/s
double speed_act_right = 0;  //Command speed for left wheel in m/s 

double fre_ang_left = 0;   //Actual ang speed for left wheel in hz
double fre_ang_right = 0;  //Command  speed for left wheel in hz

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  strs[1].toCharArray(argv1,16);
  strs[2].toCharArray(argv2,16);
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    //Serial.println(Ping(arg1));
    break;

  case SERVO_WRITE:
    //servos[arg1].setTargetPosition(arg2);
    //Serial.println("OK");
    break;
  case SERVO_READ:
    
    break;

  case READ_ENCODERS:
    
    Serial.print(encoder0Pos );//cuenta contador encoder izq
    Serial.print(" ");
    Serial.println(encoder1Pos);//cuenta contador encoder der
    break;
   case RESET_ENCODERS:
    
    encoder0Pos = 0;
    encoder1Pos = 0;
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    

    if (arg1 == 0 && arg2 == 0) {
      //setMotorSpeeds(0, 0);
      //resetPID();
      
      control_izq.moving = 0;
      control_der.moving = 0;
      Serial.println("OK"); 
      sp1=0;
      sp2=0;
    }
    else{
      //Serial.print("arg1: ");Serial.print(arg1);Serial.print("   encoder0Count: ");Serial.println(encoder0Count);
        control_izq.moving = 1;
        control_der.moving = 1;
        sp1=arg1;
        sp2=arg2;
      
      
      Serial.println("OK"); 
    }
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
     
    control_izq.moving = 0;
    control_der.moving = 0;
    sp1 = arg1;
    sp2 = arg2;
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
//    control_izq.kp = 10;control_izq.kd = 0;control_izq.ki = 10;
//    control_izq.ko = 1;
//    control_der.kp = 10;control_der.kd = 0;control_der.ki = 10;
//    control_der.ko = 1;
    Serial.println("Parametros fijos, revisar el arduino ide");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}
 
void setup() {
  Serial.begin(BAUDRATE);
    control_izq.kp = 10/*5*/;control_izq.kd = 1;control_izq.ki = 0.0001;
    control_izq.ko = 1;               //1                 0.91
    control_der.kp = 10/*5*/;control_der.kd = 1;control_der.ki = 0.0001;
    control_der.ko = 1;               //1                 0.91
  attachInterrupt(digitalPinToInterrupt(left.en_a), change_left_a, RISING);
  attachInterrupt(digitalPinToInterrupt(right.en_a), change_right_a, RISING);
//  attachInterrupt(digitalPinToInterrupt(right.encoder), change_right, CHANGE);
   
}

void loop() {



    int StringCount = 0;
  if (Serial.available() > 0) {
    auxstr= Serial.readString();
    while(auxstr.length()>0){
//      chr = Serial.read();
      int auxindex =auxstr.indexOf(' ');
      if (auxindex == -1)
        {
        strs[StringCount++]=auxstr;
        break;
        }
       else
        {
        strs[StringCount++]=auxstr.substring(0,auxindex);
        auxstr = auxstr.substring(auxindex+1);
        }
    }
      strs[0].toCharArray(cmd1,16);
      cmd = cmd1[0];
      runCommand();
      resetCommand();
  }
  
  
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    prevMillis = currentMillis;
  
    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;

    encoder0Count = encoder0Diff*100;//cantidad de ticks por cada periodo del loop
    encoder1Count = encoder1Diff*100;
    


    
    encoder0Diff = encoder0Diff/2958;//distancia en metros cada 10ms
    encoder1Diff = encoder1Diff/2958;
    
    speed_act_left = encoder0Diff*100;// (m/s)                  
    speed_act_right = encoder1Diff*100;// (m/s)

    fre_ang_left = (speed_act_left/(0.0665/2.0))/(2*3.14);// radio en metros 
    fre_ang_right =(speed_act_right/(0.0665/2.0))/(2*3.14);// radio en metros
    //Serial.print("fre_ang_left : ");Serial.print(fre_ang_left);Serial.print(" fre_ang_right: ");Serial.println(fre_ang_right);
//Serial.print("speed_act_left: ");Serial.print(speed_act_left);Serial.print(" speed_act_right: ");Serial.println(speed_act_right);    
                                                           // 2958 ticks en 1m = 8.83 en 10 ms izq
                                                            //                    8.42 en 10 ms der
    v_med= (speed_act_left + speed_act_right)/2;
    v_ang_med = (speed_act_right - speed_act_left)/(long_eje);   

                                                           
  /////*********** Se manda a la funcion PID los valores de referencia y velocidad medidad*******/////////////////////////////////////////////////////////////
    left_output = control_izq.compute(sp1,encoder0Count);
    right_output = control_der.compute(sp2,encoder1Count);
    
  //////**** Salida del PID********************////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    left.rotate(left_output);
    right.rotate(right_output);

    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
  }

   
}



// ************** encoders interrupts **************

// ************** encoder 1 *********************
void change_left_a(){  
 if(digitalRead(left.en_b) == HIGH) 
  encoder0Pos = encoder0Pos - 1;
 else
  encoder0Pos = encoder0Pos + 1;
}
// ************** encoder 2 *********************
void change_right_a(){  
  if(digitalRead(right.en_b) == LOW) 
  encoder1Pos = encoder1Pos - 1;
 else
  encoder1Pos = encoder1Pos + 1;
}
