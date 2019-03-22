


/*Program describing behaviour of warehouse robot prototype
 * Created by Adib Chisty on March 20, 2019
 * Inspired by Husnain Haider's Coordinate System
 * For ELEC299
 */

//======== INCLUSIONS =======
#include <Servo.h>
#include "QSerial.h"
QSerial myIRserial;
//===========================

//========PIN DEFINITIONS====
//MOTORS
int R_Dir= 7;       //M1
int R_Speed = 6;    //E1
int L_Speed = 5;    //E2
int L_Dir = 4;      //M2
  
//IR SENSOR BOTTOM
int R_IR = A5;
int M_IR = A4;
int L_IR = A3;

//IR SENSOR FRONT
int F_IR = A2;

//BUMPERS
int R_Bumper = 3;
int L_Bumper = 12;

//SERVOS
int pan_PIN = 8;
int tilt_PIN = 9;
int grip_PIN = 10;
int pressure_PIN = A0;

Servo pan, tilt, grip;

//MISC
int start_PIN = 11;

/*
#define rxpin 4      IDK what this is for
#define txpin -1
*/

//===========================

//========MISC DEFINITIONS===

//cordinate and cardinality system
int cy = 0;
int cx = 0;
int cd = 0; //direction 0 = 'NORTH' 1,2,3 = E,S,W

//integers to hold IR sensor analog values
//bottom facing
int lVal = 0;
int cVal = 0;
int rVal = 0;

//previous value holders
int plVal = 0;
int pcVal = 0;
int prVal = 0;

//front facing
int IRVal = 0;

//timer thing
int lastInter = 0;

//===========================

//========THRESHOLDS=========
int thresh = 750;
//===========================

//========CLASSES============ //No need for using a class, move thresholds into definition area
class Robot { 
  public:
    int R_MotorSpeed = 250;
    int L_MotorSpeed = 250;
    int R_SlowMotorSpeed = 100;
    int L_SlowMotorSpeed = 100;

    int IR_Right_Thresh = 950;
    int IR_Middle_Thresh = 900;
    int IR_Left_Thresh = 950;

    int Gripper_Pressure_Threshold = 0;//int for comparing grip.analogRead(GRIPPER_SENSOR_PIN) used in grab()
    int Ball_Pick_Up_Angle = 0;//angle for tilt servo when picking up ball, used in grab()
    int Ball_Drop_Off_Angle = 0;//angle for tilt servo when droping of ball, used in drop()

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

  //=================== IR BEACON detection goes here
/*

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
*/
//=====================================================

  int robotId = 1; //or 2 or 3. This number should be a return value from the detectBeacon Function

  pinMode(R_Dir, OUTPUT);
  pinMode(L_Dir, OUTPUT);
  pinMode(R_Speed , OUTPUT);
  pinMode(L_Speed, OUTPUT);

  pinMode(R_Bumper, INPUT);
  pinMode(L_Bumper, INPUT);

  pan.attach(pan_PIN);
  tilt.attach(tilt_PIN);
  grip.attach(grip_PIN);
  pinMode(pressure_PIN, INPUT);

  pinMode(F_IR,INPUT);
  pinMode(L_IR,INPUT);
  pinMode(M_IR,INPUT);
  pinMode(R_IR,INPUT);

  pinMode(start_PIN, INPUT);

  pan.write(robot.Pan_Default_Position);
  tilt.write(robot.Tilt_Up_Position);
  grip.write(robot.Gripper_Start_Position);




  //Starting Sequence vvvvvvv

  int val = digitalRead(start_PIN);
  while ( val == HIGH){
      val = digitalRead(start_PIN);
      if (val == LOW) {
        while (val == LOW){
          val = digitalRead(start_PIN);
        }
        break;
      }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

//=====MOVEMENT FUNCTIONS====

void goToCoord(Robot R,int x, int y, int d){
  //TO DO
}

void forward(Robot R, int numOfIntersections){
  //TO DO
}

void backward(Robot R){ //kinda redundant because no sensors on the back, will REMOVE
  //TO DO
}

void turn(Robot R, int dir){
  //TO DO
}

void approach(Robot R){
  //TO DO
}

void correctCourse(Robot R){
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
  for(int gripperDistance = 50;analogRead(pressure_PIN)>R.Gripper_Pressure_Threshold; gripperDistance+=2){
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
