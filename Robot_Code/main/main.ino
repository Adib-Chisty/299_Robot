


/*Program describing behaviour of warehouse robot prototype
  Created by Adib Chisty on March 20, 2019
  For ELEC299
*/

//======== INCLUSIONS =======
#include <Servo.h>
#include "QSerial.h"
QSerial myIRserial;
//===========================

//========PIN DEFINITIONS====
//MOTORS
int R_Dir = 7;      //M1
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
unsigned long lastInter = 0;


//===========================

//========THRESHOLDS=========
int thresh = 800;
//===========================

//========CLASSES============ //
class Robot {
  public:
    int R_FastMotorSpeed = 144;
    int L_FastMotorSpeed = 136;
    int R_MedMotorSpeed = 100;
    int L_MedMotorSpeed = 110;
    int R_SlowMotorSpeed = 94;
    int L_SlowMotorSpeed = 86;

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

Robot robot = Robot();
//===========================


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);



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

  pan.write(90);
  tilt.write(160);
  grip.write(100);

  pinMode(pressure_PIN, INPUT);

  pinMode(F_IR, INPUT);
  pinMode(L_IR, INPUT);
  pinMode(M_IR, INPUT);
  pinMode(R_IR, INPUT);

  pinMode(start_PIN, INPUT);

  /*
    pan.write(robot.Pan_Default_Position);
    tilt.write(robot.Tilt_Up_Position);
    grip.write(robot.Gripper_Start_Position);
  */



  //Starting Sequence vvvvvvv

  int val = digitalRead(start_PIN);
  while ( val == HIGH) {
    val = digitalRead(start_PIN);
    if (val == LOW) {
      while (val == LOW) {
        val = digitalRead(start_PIN);
      }
      break;
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  /*
    //snake n back
    forward(robot, 1);
    turn(robot, 1,false);
    forward(robot, 2);
    turn(robot, 0,false);
    forward(robot, 1);
    turn(robot, 0,false);
    forward(robot, 2);
    turn(robot, 1,false);
    forward(robot, 1);
    turn(robot, 1,false);
    forward(robot, 2);
    turn(robot, 0,false);
    forward(robot, 1);
    turn(robot,  0,false);
    forward(robot, 2);
    turn(robot,  0,false);
    forward(robot,4);
    exit(0);
  */

  /* //circuit
    forward(robot, 4);
    turn(robot, 1);
    forward(robot, 2);
    turn(robot, 1);
    forward(robot, 4);
    turn(robot, 1);
    forward(robot, 2);
    turn(robot, 1);
  */

  /* 180 tester
    Serial.println("Robot Current Direction");
    Serial.println(cd);
    forward(robot, 2);
    turn(robot, 0, true);
  */

  /*
    cx = 1;
    cy = -1;
    goToCoord(robot, 4, 4, 1);
    approachWall(robot);
    grab(robot);
    //turn(robot, 1, true);
    RTB(robot, 1);
    approachWall(robot);
    drop(robot);
    exit(0);
  */
  grab(robot);
  exit(0);
}


//=====MOVEMENT FUNCTIONS====

void goToCoord(Robot R, int x, int y, int d) { //x,y is destination coord, d is the cardinality of the ball
  if (cy < y) { //if current y coordinate is less than the objective y then move forward until cy = y
    forward(R, y - cy);
    cy = y;
    Serial.println("Reached Y component");
  }

  if (cx != x) { //if current x coordinate is less than the objective x, turn, then move forward until cx = x

    if (cx < x) {
      Serial.println("Turning Right");
      turn(R, 0, false);
      forward(R, x - cx);
      cx = x;
    }
    else if (cx > x) {
      Serial.println("Turning Left");
      turn(R, 1, false);
      forward(R, cx - x);
      cx = x;
    }
  }
  if (cd != d) { //if the robot is not facing the ball, then turn towards the ball
    if (abs(cd - d) == 2) { //ball is 180deg behind robot, turn 90deg twice [Cases: Many]
      turn(R, 1, true);
      //turn(R, 1, true);
    }
    else if (cd - d == -3) { //ball is -90deg to robot, turn -90deg(Left) once [Case: facing 0, ball at 3]
      turn (R, 1, false);
    }
    else if (cd - d == 3) { //ball is 90deg to robot, turn 90deg(Right) once  [Case: facing 3, ball at 0]
      turn (R, 0, false);
    }
    //if didnt catch above cases, adjust direction
    else if (cd - d > 0) //ball is -90deg to robot, turn -90deg(Left)   [Cases: Many]
    {
      turn(R, 1, false);
    }
    else if (cd - d < 0) //ball is 90deg to robot, turn 90deg(Right)     [Cases: Many]
    {
      turn(R, 0, false);
    }
  }

}

void RTB(Robot R, int homeX) {
  int y = 0;
  int x = homeX;

  if (cx != x) {
    if (cx < x) {
      forward(R, (x - cx));
      cx = x;
      turn(R, 0, false); //turn right
    }
    else if (cx > x) {
      forward(R, (cx - x));
      cx = x;
      turn(R, 1, false); //turn left
    }

  }
  if (cy != y) {
    forward(R, (cy - y));
    cy = y;
  }
}

void forward(Robot R, int numOfIntersections) {
  int intersectionCount = 0;
  digitalWrite(L_Dir, HIGH); //Set left motor direction to forward
  digitalWrite(R_Dir, HIGH); //Set right motor direction to forward

  //While not at the desired intersection, keep moving forward
  while (intersectionCount < numOfIntersections) {
    lVal = analogRead(L_IR);
    mVal = analogRead(M_IR);
    rVal = analogRead(R_IR);

    correctCourse(R); //correct leaning

    //correctly detect intersections, prevents accidental detection of false intersection
    if ((lVal > thresh) && (mVal > thresh) && (rVal > thresh) ) { //At an intersection. Increment intersection counter
      if ((millis() - lastInter) > 500) {
        Serial.println("INTERSECTION DETECTED");

        intersectionCount++;
        Serial.println(intersectionCount);
        lastInter = millis();

      }
    }

  }
  Serial.println("STOP");
  stop(R);

}


void backward(Robot R) { //kinda redundant because no sensors on the back, will REMOVE
  //TO DO
}

void stop(Robot R) {
  analogWrite(L_Speed, 0);
  analogWrite(R_Speed, 0);
}

void turn(Robot R, int dir, bool pi) { //pi means a 180
  //drive a little past the intersection
  int Val = 0;
  int turns = 0;

  if (pi) {
    turns = 2;
  }
  else {
    turns = 1;
  }
  //Serial.println("Driving Past intersection");

  delay(50);
  analogWrite(L_Speed, 130);
  analogWrite(R_Speed, 105);
  digitalWrite(L_Dir, HIGH);
  digitalWrite(R_Dir, HIGH);
  delay(400);

  for (int i = 0; i < turns; i++) {
    if (dir == 1) { //Begin turning counter-clockwise to ensure that center line sensor is not scaning the black tape
      //Serial.println("Pre turning left");
      analogWrite(L_Speed, 120);
      analogWrite(R_Speed, 105);
      digitalWrite(L_Dir, LOW);
      digitalWrite(R_Dir, HIGH);
      delay(500);
    }
    else { //Begin turning clockwise to ensure that center line sensor is not scaning the black tape
      //Serial.println("Pre turning right");
      analogWrite(L_Speed, 105);
      analogWrite(R_Speed, 120);
      digitalWrite(L_Dir, HIGH);
      digitalWrite(R_Dir, LOW);
      delay(480); //<-- PRE TURN RIGHT DELAY

    }

    if (dir == 1) {
      Val = analogRead(L_IR);
    } else {
      Val = analogRead(R_IR);
    }

    while (Val < thresh) { //Rotate in specified direction until the middle line sensor reads the black tape value

      if (dir == 1) {
        Val = analogRead(L_IR);
      } else {
        Val = analogRead(R_IR);
      }

      //Serial.println("Middle IR:");
      //Serial.println(Val);
      analogWrite(L_Speed, 114);
      analogWrite(R_Speed, 110);


      if (dir == 1) {
        //Serial.println("Turning Left");
        digitalWrite(L_Dir, LOW);
        digitalWrite(R_Dir, HIGH);
      } else {
        //Serial.println("Turning Right");
        digitalWrite(L_Dir, HIGH);
        digitalWrite(R_Dir, LOW);
      }

    }

  }
  //Update current direction of the robot

  if (pi) {

    switch (cd) {
      case 0:
        cd = 2;
        break;
      case 1:
        cd = 3;
        break;
      case 2:
        cd = 0;
        break;
      case 3:
        cd = 1;
        break;
      default:
        break;
    }

  } else {

    if (dir == 1) {
      if (cd == 0) {
        cd = 3;
      } else {
        cd = cd - 1;
      }
    }
    else {
      cd = (cd + 1) % 4; //Update direction robot is facing
    }

  }
  Serial.println("STOP");
  //delay(50); //adjustment delay

  stop(R);
}


void approachWall(Robot r) {

  digitalWrite(L_Dir, HIGH);
  digitalWrite(R_Dir, HIGH);
  int lbump = digitalRead(L_Bumper);
  int rbump = digitalRead(R_Bumper);
  r.L_MedMotorSpeed = r.L_MedMotorSpeed - 50;
  r.R_MedMotorSpeed = r.R_MedMotorSpeed - 50;

  correctCourse(r);

  while (!(lbump == LOW || rbump == LOW)) {

    digitalWrite(L_Dir, HIGH);
    digitalWrite(R_Dir, HIGH);
    lbump = digitalRead(L_Bumper);
    rbump = digitalRead(R_Bumper);
    correctCourse(r);
  }
  r.L_MedMotorSpeed = r.L_MedMotorSpeed + 50;
  r.R_MedMotorSpeed = r.R_MedMotorSpeed + 50;
  digitalWrite(L_Dir, LOW);
  digitalWrite(R_Dir, LOW);
  analogWrite(L_Speed, r.L_MedMotorSpeed - 35);
  analogWrite(R_Speed, r.R_MedMotorSpeed - 30);
  delay(390);
  analogWrite(L_Speed, 0);
  analogWrite(R_Speed, 0);

}

void correctCourse(Robot R) {
  lVal = analogRead(L_IR);
  mVal = analogRead(M_IR);
  rVal = analogRead(R_IR);

  if ((lVal < thresh) && (mVal > thresh) && (rVal < thresh)) { //SET MOTORS TO DRIVE FORWARD
    //Serial.println("Driving Forward");
    analogWrite(L_Speed, 110);
    analogWrite(R_Speed, 100);

  } else if ((lVal > thresh) && (mVal < thresh) && (rVal < thresh)) { //LEANING INTO THE RIGHT...SPEED UP RIGHT MOTOR (CALIBRATE)
    //Serial.println("Leaning right, pulling left");
    //    analogWrite(L_Speed, 100);
    //    analogWrite(R_Speed, 120);
    analogWrite(L_Speed, 100);
    analogWrite(R_Speed, 130);
  } else if ((lVal < thresh) && (mVal < thresh) && (rVal > thresh)) { //LEANING INTO THE LEFT...SPEED UP RIGHT MOTOR (CALIBRATE)
    //Serial.println("Leaning left, pulling right");
    //    analogWrite(L_Speed, 130);
    //    analogWrite(R_Speed, 100);
    analogWrite(L_Speed, 140);
    analogWrite(R_Speed, 100);
  }
  else {
    //SET MOTORS TO DRIVE FORWARD
    //Serial.println("Driving Forward");
    analogWrite(L_Speed, 110);
    analogWrite(R_Speed, 100);
  }
}



//===========================

//=====SERVO FUNCTIONS=======

//angles tilt server over the ball
//closes gripper comparing untill pressure pin tells it to stop
//moves tilt into a vertical position
void grab(Robot R) {
  //TO DO

  //To grab position
  tilt.write(70);
  pan.write(90);
  delay(2000);

  //grab it forreal
  int fullyOpen = 0;
  while (analogRead(pressure_PIN) < 200) {
    grip.write(fullyOpen);
    fullyOpen += 1;
    delay(100);
  }

  //put ball back up
  tilt.write(160);
  delay(2000);

  //back up
  delay(50);
  analogWrite(L_Speed, 130);
  analogWrite(R_Speed, 105);
  digitalWrite(L_Dir, LOW);
  digitalWrite(R_Dir, LOW);
  delay(700);

  //do 180
  turn(R, 0, false);
  //drive upto intersection
  forward(R, 1);
  //move up a lil
  delay(50);
  analogWrite(L_Speed, 130);
  analogWrite(R_Speed, 105);
  digitalWrite(L_Dir, HIGH);
  digitalWrite(R_Dir, HIGH);
  delay(400);

  //stahp
  stop(R);
}

void drop(Robot r) {
  //TO DO                    //Call this after drop function after calling approachWall, This will move it forward until it hits the wall
  digitalWrite(L_Dir, HIGH);
  digitalWrite(R_Dir, HIGH);
  int lbump = digitalRead(L_Bumper);
  int rbump = digitalRead(R_Bumper);
  analogWrite(L_Speed, r.L_MedMotorSpeed - 35);
  analogWrite(R_Speed, r.R_MedMotorSpeed - 30);
  while (!(lbump == LOW || rbump == LOW)) {
    lbump = digitalRead(L_Bumper);
    rbump = digitalRead(R_Bumper);
  }
  stop(r);

  // vvv Actual code for dropping ball.
  tilt.write(r.Ball_Drop_Off_Angle);
  grip.write(40);
  tilt.write(160);
}

//===========================

//=====HELPER FUNCTIONS======

//===========================
