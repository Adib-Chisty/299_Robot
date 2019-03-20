void setup() { 
  Servo pan, tilt, grab;
}



bool detectObstacle (Robot(object)) { // for detecting when the robot hits the wall to grab the ball 
  int E1 = R_MOTORSPEED_PIN;
  int E2 = L_MOTORSPEED_PIN;
  bumper_right = digitalRead(object.LEFT_BUMPERPIN);
  bumper_left = digitalRead(object.RIGHT_BUMPERPIN);
  while((bumper_right == HIGH) && (bumper_left == HIGH)) {
  return false
  }
  if (bumper_right == LOW) && (bumper_left == HIGH){
  digitalWrite (E1, object.R_SlowMotorSpeed);
  digitalWrite (E2, object.L_SlowMotorSpeed);
  }
  else if ((bumper_right == HIGH) && (bumper_left == LOW)) { 
  digitalWrite (E1, object.R_SlowMotorSpeed);
  digitalWrite (E2, object.L_SlowMotorSpeed);
  else { 
        return true
        digitalWrite (E1, 0);
        digitalWrite (E2, 0);
        
    }
  }
}

bool detectIntersection (Robot(object)) { 
             irr = analogRead(object.IR_R_SENSORPIN);
             irm = analogRead(object.IR_M_SENSORPIN);
             irl = analogRead(object.IR_L_SENSORPIN); 
if (irm > object.IR_Middle_Thresh) { 
  if (irr > object.IR_Right_Thresh) && (irl > object.IR_Left_Thresh) {
  }
  return true 
  else { 
    return false
  }
}
}

void grabBall(Robot(object)) {
int E1 = R_MOTORSPEED_PIN;
int E2 = L_MOTORSPEED_PIN;
int closing = Gripper_Start_Position;
int gripperForce = 0;
distance_sensor_grab = analogRead(object.FRONT_DISTANCE_SENSORPIN);
  if (distance_sensor_grab > grab_distance_threshold) {
    digitalWrite(E1,0);
    digitalWrite(E2,0);
    tilt.write(object.Tilt_Grab_Position);
    gripper.write(closing);
  gripperForce = analogRead(object.GRIPPER_PIN);
  if (gripperForce >= object.Gripper_thresh) { 
    tilt.write(object.Tilt_Start_Position);
    break;
  else {
    closing = closing + 2;
        }
      }
    }
}


void dropBall() {
int E1 = R_MOTORSPEED_PIN;
int E2 = L_MOTORSPEED_PIN;
distance_sensor_grab = analogRead(object.FRONT_DISTANCE_SENSORPIN);
  if (distance_sensor_grab > grab_distance_threshold) {
    digitalWrite(E1,0);
    digitalWrite(E2,0);
    if (detectObstacle(robot(object)) == True) {
      tilt.write(object.Tilt_Grab_Position);
      gripper.write(Gripper_Start_Position);
      delay(1000);
      tilt.write(object.Tile_Start_Position);
      return; // after ball is dropped write 
  }
  }
}
