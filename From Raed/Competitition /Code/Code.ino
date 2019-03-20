#include <Servo.h>
#include "QSerial.h"
QSerial myIRserial;
#define rxpin 4
#define txpin -1


Servo pan, tilt, grab;

class Robot {
  public:
    int R_MotorSpeed = 250;
    int L_MotorSpeed = 250;
    int R_SlowMotorSpeed = 100;
    int L_SlowMotorSpeed = 100;
    
    int IR_Right_Thresh = 950;
    int IR_Middle_Thresh = 900;
    int IR_Left_Thresh = 950;

    int Gripper_Thresh = 1000;
    int Gripper_Start_Position = 40;
    int Pan_Default_Position = 0;
    int Tilt_Grab_Position = 180;
    int Tilt_Up_Position = 0;

    int Grab_Distance_Thresh = 500;

};

int R_MOTORSPEED_PIN = 6;
int R_MOTORDIR_PIN = 7;
int L_MOTORSPEED_PIN = 5;
int L_MOTORDIR_PIN = 4;

int IR_R_SENSORPIN = 1;
int IR_M_SENSORPIN = 0;
int IR_L_SENSORPIN = 2;

int FRONT_DISTANCE_SENSORPIN = 3;

int R_BUMPERPIN = 13;
int L_BUMPERPIN = 10;

int PAN_SERVO_PIN = 3;
int TILT_SERVO_PIN = 8;
int GRIPPER_PIN = 9;
int GRIPPER_SENSOR_PIN = A2;

int START_BUTTONPIN = 3;

void moveBot(Robot object, char input)
{
  if (input == 'F'){
   digitalWrite(R_MOTORDIR_PIN, HIGH);
   digitalWrite(L_MOTORDIR_PIN, HIGH);
   analogWrite(R_MOTORSPEED_PIN, object.R_MotorSpeed);
   analogWrite(L_MOTORSPEED_PIN, object.L_MotorSpeed);
  }
  else if (input == 'B'){
    
  }
  else if (input == 'L'){
    
  }
  else if (input == 'R'){
    
  }
  else if (input == 'U'){  //180 Turn
    
  }
  else if (input == 'G'){  //Grab Ball (Stop when distance sensor reaches threshold, then grab ball)
    
  }
  else if (input == 'D'){ //Drop Ball
    
  }
 
}


void setup() {

  Robot robot = Robot();
  
  if (Serial.available() == 1) {
    Serial.write("ready");
    char iByte = Serial.read();
    Serial.println(iByte);
    if (iByte == 'Y') {
       int robotId = 1;
    }
    if (iByte == 'X') {
       int robotId = 2;
    }
    if (iByte == 'Z') {
       int robotId = 3;
    }
  }
  
  int robotId = 1; //or 2 or 3. This number should be a return value from the detectBeacon Function
  
  pinMode(R_MOTORDIR_PIN, OUTPUT);
  pinMode(L_MOTORDIR_PIN, OUTPUT);
  pinMode(R_MOTORSPEED_PIN , OUTPUT);
  pinMode(L_MOTORSPEED_PIN, OUTPUT);

  pinMode (R_BUMPERPIN, INPUT);
  pinMode (L_BUMPERPIN, INPUT);

  pan.attach(PAN_SERVO_PIN);
  tilt.attach(TILT_SERVO_PIN);
  grab.attach(GRIPPER_PIN); 
  pinMode(GRIPPER_SENSOR_PIN, INPUT);

  pinMode (FRONT_DISTANCE_SENSORPIN,INPUT);

  pinMode (START_BUTTONPIN, INPUT);
  
  pan.write(robot.Pan_Default_Position);
  tilt.write(robot.Tilt_Up_Position);
  grab.write(robot.Gripper_Start_Position);
  
  
  Serial.begin(9600);
  
  //Starting Sequence vvvvvvv
  
  int val = digitalRead(START_BUTTONPIN);
  while ( val == HIGH){
      val = digitalRead(START_BUTTONPIN);
      if (val == LOW) {
        while (val == LOW){
          val = digitalRead(START_BUTTONPIN);
        }
        break;
      }
  }
}

void loop() {

}
