

//Se importan todas las librerias 
#include "Motor.h"
#include "pid.h"
#include "commands.h"

float long_eje = 0.2275; // distancias entre ruedas en metros
const byte MOTOR_IZQ = 2;  // izq//Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_DER = 3;  // Motor 1 Interrupt Pin - INT 0 - Left Motor


#define LOOPTIME 10

////////////////////
//     (in1,in2,enb,encoder)
//(       in1, in2, en, en_a, en_b)
Motor right(9,   10,  8 ,  3,  48);
Motor left(6,  7,   5 ,  2,  49);

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2
/// Constantes para el control PID/////////
double left_kp =10, left_ki = 0.95 , left_kd = 10.2;             // modify for optimal performance
double right_kp = 14.1 , right_ki = 1.03 , right_kd = 10;

double right_input = 0, right_output = 0, right_setpoint = 0;
pid control_der(right_kp, right_ki, right_kd);

double left_input = 0, left_output = 0, left_setpoint = 0;
pid control_izq  (left_kp, left_ki, left_kd);
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

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

double speed_act_left = 0;   //Actual speed for left wheel in m/s
double speed_act_right = 0;  //Command speed for left wheel in m/s 


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
    Serial.println(Ping(arg1));
    break;

  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;

    

  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}
 
void setup() {
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(left.en_a), change_left_a, RISING);
  attachInterrupt(digitalPinToInterrupt(right.en_a), change_right_a, RISING);
//  attachInterrupt(digitalPinToInterrupt(right.encoder), change_right, CHANGE);
   
}

void loop() {
 
    while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    prevMillis = currentMillis;
  
    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;
    encoder0Diff = encoder0Diff/2958;//distancia en metros cada 10ms
    encoder1Diff = encoder1Diff/2958;

    
    speed_act_left = encoder0Diff*100;// (m/s)                  
    speed_act_right = encoder1Diff*100;// (m/s)
                                                           // 2958 ticks en 1m = 8.83 en 10 ms izq
                                                            //                    8.42 en 10 ms der
    v_med= (speed_act_left + speed_act_right)/2;
    v_ang_med = (speed_act_right - speed_act_left)/(long_eje);                                                          
  
    left_input = encoder0Diff;  //Input to PID controller is the current difference
    right_input = encoder1Diff;
    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
   
   /// Calculo de los setpoints para cada motor 
    left_setpoint = (2*v_lineal-v_ang*long_eje)/(2*R);
    right_setpoint = (2/R)*v_lineal-left_setpoint ; 
    left_setpoint = left_setpoint*30/3.14159265;
    right_setpoint = right_setpoint*30/3.14159265;

  }
   /// Salidad del control pid////
  right_output = control_der.compute(right_setpoint,speed_act_right);
  left_output = control_izq.compute(left_setpoint,speed_act_left);
   /// señales de salida para los motores
  left.rotate(left_output);
  right.rotate(right_output);
   
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
