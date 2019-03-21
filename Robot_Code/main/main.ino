
$ //IR SENSOR BOTTOM


bkito@BENK-PC MINGW64 ~/Documents/Projects/ELEC299Bot
$ int IR_R_SENSORPIN = 1;
/*Program describing behaviour of warehouse robot prototype
 * Created by Adib Chisty on March 20, 2019
 * For ELEC299
 */

//======== INCLUSIONS =======
#include <Servo.h>
#include "QSerial.h"
QSerial myIRserial;
//===========================

//========PIN DEFINITIONS====
//MOTORS
int R_MOTORSPEED_PIN = 6;
int R_MOTORDIR_PIN = 7;
int L_MOTORSPEED_PIN = 5;
int L_MOTORDIR_PIN = 4;

//IR SENSOR BOTTOM
int IR_R_SENSORPIN = 1;
int IR_M_SENSORPIN = 0;
int IR_L_SENSORPIN = 2;

//IR SENSOR FRONT
int FRONT_DISTANCE_SENSORPIN = 3;

//BUMPERS
int R_BUMPERPIN = 13;
int L_BUMPERPIN = 10;

//SERVOS
int PAN_SERVO_PIN = 3;
int TILT_SERVO_PIN = 8;
int GRIPPER_PIN = 9;
int GRIPPER_SENSOR_PIN = A2;
Servo pan, tilt, grip;

//MISC
int START_BUTTONPIN = 3;
#define rxpin 4
#define txpin -1

//===========================

//========MISC DEFINITIONS===

//===========================

//========THRESHOLDS=========
//===========================

//========CLASSES============
class Robot {
  public:
    int R_MotorSpeed = 250;
    int L_MotorSpeed = 250;
    int R_SlowMotorSpeed = 100;
    int L_SlowMotorSpeed = 100;

    int IR_Right_Thresh = 950;
    int IR_Middle_Thresh = 900;
    int IR_Left_Thresh = 950;

    int Gripper_Pressure_Threshold = ;//int for comparing grip.analogRead(GRIPPER_SENSOR_PIN) used in grab()
    int Ball_Pick_Up_Angle = ;//angle for tilt servo when picking up ball, used in grab()
    int Ball_Drop_Off_Angle = ;//angle for tilt servo when droping of ball, used in drop()

    int Gripper_Thresh = 1000;
    int Gripper_Start_Position = 40;
    int Pan_Default_Position = 0;
    int Tilt_Grab_Position = 160;
    int Tilt_Up_Position = 0;

    int Grab_Distance_Thresh = 500;

};
//===========================


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

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

  pinMode(R_BUMPERPIN, INPUT);
  pinMode(L_BUMPERPIN, INPUT);

  pan.attach(PAN_SERVO_PIN);
  tilt.attach(TILT_SERVO_PIN);
  grip.attach(GRIPPER_PIN);
  pinMode(GRIPPER_SENSOR_PIN, INPUT);

  pinMode(FRONT_DISTANCE_SENSORPIN,INPUT);

  pinMode(START_BUTTONPIN, INPUT);

  pan.write(robot.Pan_Default_Position);
  tilt.write(robot.Tilt_Up_Position);
  grip.write(robot.Gripper_Start_Position);




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
  // put your main code here, to run repeatedly:

}

//=====MOVEMENT FUNCTIONS====



void forwards(Robot R){
  //TO DO
}

void backwards(Robot R){
  //TO DO
}

void turn(Robot R, int dir){
  //TO DO
}

void approach(Robot R){
  //TO DO
}

//===========================

//=====SERVO FUNCTIONS=======

//angles tilt server over the ball
//closes gripper comparing untill pressure pin tells it to stop
//moves tilt into a vertical position
void grab(Robot R){
  //TO DO
  tilt.write(R.Ball_Pick_Up_Angle);//Find int for proper angle for pick up
  grip.write(50);
  for(int gripperDistance = 50;analogRead(GRIPPER_SENSOR_PIN)>R.Gripper_Pressure_Threshold; gripperDistance+=2){
    grip.write(gripperDistance);
  }
  tilt.write(160);
}

void drop(Robot R){
  //TO DO
  tilt.write(R.Ball_Drop_Off_Angle);
  grip.write(40);
  tilt.write(160);
}

//===========================

//=====HELPER FUNCTIONS======

//===========================
