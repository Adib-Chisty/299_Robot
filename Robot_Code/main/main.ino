


/*Program describing behaviour of warehouse robot prototype
* Created by Adib Chisty on March 20, 2019
* Inspired by Husnain Haider's Coordinate System
* Expanded with Raed Fayad's CLass
* Modularity added
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
int mVal = 0;
int rVal = 0;

//previous value holders
int plVal = 0;
int pmVal = 0;
int prVal = 0;

//front facing
int IRVal = 0;

//timer thing
int lastInter = 0;

//===========================

//========THRESHOLDS=========
int thresh = 750;
//===========================

//========CLASSES============ //
class Robot {
public:
  int R_FastMotorSpeed = 250;
  int L_FastMotorSpeed = 250;
  int R_MedMotorSpeed = 150;
  int L_MedMotorSpeed = 150;
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

//PINMODE SETUPS
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

  pinMode(F_IR, INPUT);
  pinMode(L_IR, INPUT);
  pinMode(M_IR, INPUT);
  pinMode(R_IR, INPUT);

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

void goToCoord(Robot R,int x, int y, int d){ //x,y is destination coord, d is the cardinality of the ball
  if (cy<y){ //if current y coordinate is less than the objective y then move forward until cy = y
    forward(R,y-cy);
    cy = y;
    Serial.println("Reached Y component");
  }

  if(cx!=x){ //if current x coordinate is less than the objective x, turn, then move forward until cx = x

    if(cx<x){
      Serial.println("Turning Right");
      turn(R,0);
      forward(R,x-cx);
      cx = x;
    }
    else if(cx>x){
      Serial.println("Turning Left");
      turn(R,1);
      forward(R,cx-x);
      cx = x;
    }
  }
  if(cd!=d){ //if the robot is not facing the ball, then turn towards the ball
    if(abs(cd-d) == 2){ //ball is 180deg behind robot, turn 90deg twice [Cases: Many]
      turn(R,1);
      turn(R,1);
    }
    else if(cd-d ==-3){ //ball is -90deg to robot, turn -90deg(Left) once [Case: facing 0, ball at 3]
      turn (R,1);
    }
    else if(cd-d ==3){  //ball is 90deg to robot, turn 90deg(Right) once  [Case: facing 3, ball at 0]
      turn (R,0);
    }
    //if didnt catch above cases, adjust direction
    else if(cd-d>0)  //ball is -90deg to robot, turn -90deg(Left)   [Cases: Many]
    {
      turn(R,1);
    }
    else if(cd-d<0) //ball is 90deg to robot, turn 90deg(Right)     [Cases: Many]
    {
      turn(R,0);
    }
  }

}

void forward(Robot R, int numOfIntersections){
  int intersectionCount = 0;
  digitalWrite(L_Dir, HIGH); //Set left motor direction to forward
  digitalWrite(R_Dir, HIGH); //Set right motor direction to forward

  //While not at the desired intersection, keep moving forward
  while(intersectionCount < numOfIntersections){
    lVal = analogRead(L_IR);
    mVal = analogRead(M_IR);
    rVal = analogRead(R_IR);

    correctCourse(R); //correct leaning

    //correctly detect intersections, prevents accidental detection of false intersection
    if((lVal > thresh) && (mVal > thresh) && (rVal > thresh) && (plVal > thresh) && (pmVal > thresh) && (prVal > thresh)){ //At an intersection. Increment intersection counter
      if((millis()-lastInter)>200){
        Serial.println("INTERSECTION DETECTED");

        intersectionCount++;
        Serial.println(intersectionCount);
        lastInter = millis();

      }
    }
    plVal = lVal;
    pmVal = mVal;
    prVal = rVal;
  }
  stop(R);
}


void backward(Robot R){ //kinda redundant because no sensors on the back, will REMOVE
  //TO DO
}

void stop(Robot R){
  analogWrite(L_Speed, 0);
  analogWrite(R_Speed, 0);
}

void turn(Robot R, int dir){
  //drive a little past the intersection
  delay(50);
  analogWrite(L_Speed, R.L_SlowMotorSpeed);
  analogWrite(R_Speed, R.R_SlowMotorSpeed);
  digitalWrite(L_Dir, HIGH);
  digitalWrite(R_Dir, HIGH);
  delay(700);

  if(dir == 1){//Begin turning counter-clockwise to ensure that center line sensor is not scaning the black tape
    analogWrite(L_Speed, R.L_SlowMotorSpeed);
    analogWrite(R_Speed, R.R_SlowMotorSpeed);
    digitalWrite(L_Dir, LOW);
    digitalWrite(R_Dir, HIGH);
    delay(200);
  }
  else{//Begin turning clockwise to ensure that center line sensor is not scaning the black tape
    analogWrite(L_Speed, R.L_SlowMotorSpeed);
    analogWrite(R_Speed, R.R_SlowMotorSpeed);
    digitalWrite(L_Dir, HIGH);
    digitalWrite(R_Dir, LOW);
    delay(200);
  }

  mVal = analogRead(M_IR);
  while(mVal<thresh){ //Rotate in specified direction until the middle line sensor reads the black tape value
    mVal = analogRead(M_IR);
    analogWrite(L_Speed, R.L_SlowMotorSpeed);
    analogWrite(R_Speed, R.R_SlowMotorSpeed);
    if(dir = 1){
      digitalWrite(L_Dir, LOW);
      digitalWrite(R_Dir, HIGH);
    }else{
      digitalWrite(L_Dir, HIGH);
      digitalWrite(R_Dir, LOW);
    }
  }

  //Update current direction of the robot
  if(dir == 1){
    if(cd ==0){ //Update direction robot is facing
      cd =3;
    }else{
      cd =cd-1;
    }
  }
  else{
    cd = (cd+1)%4; //Update direction robot is facing
  }

  stop(R);
}


void approachWall(Robot R){
  //TO DO
}

void correctCourse(Robot R){
  if((lVal < thresh) && (mVal > thresh) && (rVal < thresh)){ //SET MOTORS TO DRIVE FORWARD
    analogWrite(L_Speed, R.L_MedMotorSpeed);
    analogWrite(R_Speed, R.R_MedMotorSpeed);
  }else if((lVal > thresh) && (mVal < thresh) && (rVal < thresh)){//LEANING INTO THE RIGHT...SPEED UP RIGHT MOTOR (CALIBRATE)
    analogWrite(L_Speed, R.L_SlowMotorSpeed);
    analogWrite(R_Speed, R.R_MedMotorSpeed);
  }else if((lVal < thresh) && (mVal < thresh) && (rVal > thresh)){//LEANING INTO THE LEFT...SPEED UP RIGHT MOTOR (CALIBRATE)
    analogWrite(L_Speed, R.L_MedMotorSpeed);
    analogWrite(R_Speed, R.L_SlowMotorSpeed);
  }
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
