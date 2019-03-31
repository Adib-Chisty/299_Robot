


/*Program describing behaviour of warehouse robot prototype
  Created by Adib Chisty on March 20, 2019
  For ELEC299
*/

//======== INCLUSIONS =======
#include <Servo.h>
#include "QSerial.h"
QSerial myIRserial;
//===========================
QSerial IRSerial;

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
int beacon_PIN = 11;
int button_PIN = A2;
int LED_PIN = 13;
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

int homeX = 0;
int homeY = 0;

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

//path arrays
int roboId; //or 2 or 3. This number should be a return value from the detectBeacon Function
int paths[3][15] = {
  {0, 3, 3, 4, 4, 1, 0, 1, 3, 0, 2, 3, 0, 4, 0},
  {4, 0, 1, 0, 0, 3, 1, 4, 0, 2, 4, 0, 3, 4, 0},
  {0, 4, 3, 4, 3, 1, 4, 4, 0, 4, 1, 1, 4, 2, 1}
};

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

  pinMode(LED_PIN, OUTPUT);
  //=================== IR BEACON detection goes here

  IRSerial.attach(beacon_PIN, -1);

  do {
    roboId = IRSerial.receive(200);
    Serial.print("Raw character:");
    Serial.print(roboId);
    Serial.print("\n");
  } while (validateIRIn(roboId));

  roboId -= 48;
  identifyRobot(roboId);
  Serial.print("ID Number:");
  Serial.print(roboId);
  Serial.print("\n");
  //=====================================================

  Serial.println("Valid Character Detected");
  //flash led
  flash(roboId);

  int read_ = 1;
  bool pressed = false;
  while (true) {
    read_ = digitalRead(button_PIN);
    if (read_ == 0) {
      Serial.println("Button Depressed");
      pressed = true;
    }
    if (pressed == true && read_ == 1) {
      Serial.println("Button Released");
      break;
    }
  }


}

//MAIN STUFF
void loop() {

  int counter = 0;
  for (int i = 0; i < 15; i += 3) {

    goToCoord(robot, paths[roboId][i], paths[roboId][i + 1], paths[roboId][i + 2]);
    approachWall(robot, false);
    grab(robot);
    RTB(robot, homeX);
    approachWall(robot, true);
    drop(robot);

    counter++;
    Serial.print("Got a Dice:");
    Serial.print(counter);
    Serial.print("/5\n");
  }
  Serial.println("ROUTE COMPLETE.");
  celebrate();
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

void RTB(Robot R, int home_) {
  int y = 0;
  int x = home_;

  if (cd == 2) {
    if (cx < x) {
      turn(R, 1, false);
    } else if (cx > x) {
      turn(R, 0, false);
    } else {
      //do nothing
    }
  }

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
      delay(650);
      //      delay(500);
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
      analogWrite(L_Speed, 120);
      analogWrite(R_Speed, 130);


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


void approachWall(Robot r, bool base) {

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

  if (!base) {
    digitalWrite(L_Dir, LOW);
    digitalWrite(R_Dir, LOW);
    analogWrite(L_Speed, r.L_MedMotorSpeed - 35);
    analogWrite(R_Speed, r.R_MedMotorSpeed - 30);
    delay(1000); //blazeit
  }
  stop(r);

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
  pan.write(90);
  grip.write(0);
  delay(500);

  int tiltVal = 160;
  while (tiltVal >= 65) {
    tilt.write(tiltVal);
    tiltVal -= 1;
    delay(25);
  }

  //grab it forreal
  int fullyOpen = 0;
  while (analogRead(pressure_PIN) < 200) {
    grip.write(fullyOpen);
    fullyOpen += 1;
    delay(25);
  }
  //put ball back up
  tilt.write(180);
  delay(1000);

  //back up
  delay(50);
  analogWrite(L_Speed, 110);
  analogWrite(R_Speed, 120);
  digitalWrite(L_Dir, LOW);
  digitalWrite(R_Dir, LOW);
  delay(800);

  //do 180
  turn(R, 0, true);

  //stahp
  stop(R);
}

void drop(Robot R) {
  //Call this after drop function after calling approachWall, This will move it forward until it hits the wall

  int tiltValue = 180;
  while (tiltValue >= 90) {
    tilt.write(tiltValue);
    if (tiltValue == 95) {
      grip.write(0);
    }
    tiltValue -= 1;
  }

  //backup
  delay(50);
  analogWrite(L_Speed, 110);
  analogWrite(R_Speed, 120);
  digitalWrite(L_Dir, LOW);
  digitalWrite(R_Dir, LOW);
  delay(800);

  //turn
  turn(R, 0, false);

  // realign servo
  tilt.write(180);
  grip.write(100);
  delay(1000);

  stop(R);

  cd = 0;
  cx = homeX;
  cy = homeY;
}

//===========================

//=====HELPER FUNCTIONS======

//used when reading IRSerial to identiy robot
bool validateIRIn(char i) {
  return !(i == '0' || i == '1' || i == '2');
}

void identifyRobot(int roboId) {
  switch (roboId) {
    case 0:
      cx = 1;
      cy = -1;
      homeX = 1;
      homeY = -1;
      break;
    case 1:
      cx = 2;
      cy = -1;
      homeX = 2;
      homeY = -1;
      break;
    case 2:
      cx = 3;
      cy = -1;
      homeX = 3;
      homeY = -1;
      break;
    default:
      Serial.print("roboId was a bad value");
      exit(0);
      break;
  }
}


void celebrate() {
  tilt.write(180);
  digitalWrite(L_Dir, HIGH); //Set left motor direction to forward
  digitalWrite(R_Dir, LOW); //Set right motor direction to forward
  analogWrite(L_Speed, 120);
  analogWrite(R_Speed, 120);

  int asdf = 4;
  int asdfg = 4;
  bool switcher = true;
  int gripSize = 40;
  int tiltSize = 70;
  while (true) {
    digitalWrite(LED_PIN, switcher);
    switcher = !switcher;

    if (gripSize > 160 || gripSize < 50) {
      asdf *= -1;
    }

    if (tiltSize > 150 || tiltSize < 80) {
      asdfg *= -1;
    }

    tilt.write(tiltSize);
    tiltSize += asdfg;
    delay(500);

    grip.write(gripSize);
    gripSize += asdf;

    delay(500);

  }

}

void flash(int ID) {
  for (int i = 0; i < ID; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
  digitalWrite(LED_PIN, HIGH);
}
//===========================
