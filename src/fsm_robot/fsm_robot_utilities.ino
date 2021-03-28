void initializeRobot(int soft_start_level) {
  // initializeRobot(SOFT_START_DISABLED) the Softstart is disabled and you can use the pin 12
  if (soft_start_level != SOFT_START_DISABLED) {
    pinMode(SOFT_START_CONTROL_PIN, OUTPUT);
    digitalWrite(SOFT_START_CONTROL_PIN, LOW);
  }

  // Initialization pin Servo motors
  for (int i = 0; i < QNUM; i++) {
    q[i].attach(qPin[i]);
  }

  // For each step motor this set up the initial degree
  for (int i = 0; i < QNUM; i++) {
    q[i].write(homePosition[i]);
    actualPosition[i] = homePosition[i];
  }
 
  // Turn ON the Braccio softly and save it from brokes.
  if (soft_start_level != SOFT_START_DISABLED) {
    _softStart(soft_start_level);
  }
}

void _softwarePWM(int high_time, int low_time) {
  digitalWrite(SOFT_START_CONTROL_PIN, HIGH);
  delayMicroseconds(high_time);
  digitalWrite(SOFT_START_CONTROL_PIN, LOW);
  delayMicroseconds(low_time);
}

/*
  This function, used only with the Braccio Shield V4 and greater,
  turn ON the Braccio softly and save it from brokes.
  The SOFT_START_CONTROL_PIN is used as a software PWM
  @param soft_start_level: the minimum value is -70, default value is 0 (SOFT_START_DEFAULT)
*/
void _softStart(int soft_start_level) {
  long int tmp = millis();
  while (millis() - tmp < LOW_LIMIT_TIMEOUT)
    _softwarePWM(80 + soft_start_level, 450 - soft_start_level); //the sum should be 530usec

  while (millis() - tmp < HIGH_LIMIT_TIMEOUT)
    _softwarePWM(75 + soft_start_level, 430 - soft_start_level); //the sum should be 505usec

  digitalWrite(SOFT_START_CONTROL_PIN, HIGH);
}

void releaseRobot(int soft_start_level) {
  // Turn off Servo motors
  for (int i = 0; i < QNUM; i++) {
    q[i].detach();
  }
  
  if (soft_start_level != SOFT_START_DISABLED) {
    digitalWrite(SOFT_START_CONTROL_PIN, LOW);
    pinMode(SOFT_START_CONTROL_PIN, INPUT);
  }
}

bool executeLoadedTrajectory() {
  
  bool end_task;
  
  for (int i = 0; i < MAXPOINTS; i++) {
    for (int j = 0; j < QNUM; j++) {
      if (checkBoundaries(j, trajectory[i][j])) {
        q[j].write(trajectory[i][j]);
      }
      else {
        end_task = false;
        return end_task;
      }
    }
    delay(DELTA_T_LOADED_TRAJECTORY);
  }

  end_task = true;
  return end_task;
}

bool executeTrivialTrajectory(byte targetPosition[]) {

  bool done = false;

  //Until the all motors are in the desired position
  while (!done) {     
    //For each servo motor if next degree is not the same of the previuos than do the movement   
    for (int i = 0; i < QNUM; i++) {
      //One step ahead
      if (targetPosition[i] > actualPosition[i]) {
        actualPosition[i]++;
      }
      //One step beyond
      else if (targetPosition[i] < actualPosition[i]) {
        actualPosition[i]--;
      }
      if (checkBoundaries(i, actualPosition[i])) {
        q[i].write(actualPosition[i]);
      }
      else {
        done = false;
        return done;
      }
    }
    
    //delay between each movement
    delay(DELTA_T_TRIVIAL_TRAJECTORY);
    
    // It checks if all the servo motors are in the desired position
    for (int i = 0; i < QNUM; i++) {
      if (targetPosition[i] != actualPosition[i]) {
        done = false;
        break;
      }
      done = true;
    }
  }

  return done;
}

void transition_ACK(int prevState, int nextState) {
  Serial.print((char) prevState);
  Serial.print((char) nextState);
}

bool checkBoundaries(int joint, byte jointPosition) {
  /*
    Step Delay: a milliseconds delay between the movement of each servo. Allowed values from 10 to 30 msec.
    M1 (q[0]) = base degrees. Allowed values from 0 to 180 degrees
    M2 (q[1]) = shoulder degrees. Allowed values from 15 to 165 degrees
    M3 (q[2]) = elbow degrees. Allowed values from 0 to 180 degrees
    M4 (q[3]) = wrist vertical degrees. Allowed values from 0 to 180 degrees
    M5 (q[4]) = wrist rotation degrees. Allowed values from 0 to 180 degrees
    M6 (q[5]) = gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */
  bool ans = true;
  
  // switch (joint) {
  //   case BASE:       ans = jointPosition >  0 && jointPosition < 180; break;
  //   case SHOULDER:   ans = jointPosition > 15 && jointPosition < 165; break;
  //   case ELBOW:      ans = jointPosition >  0 && jointPosition < 180; break;
  //   case WRIST_ROT:  ans = jointPosition >  0 && jointPosition < 180; break;
  //   case WRIST_VER:  ans = jointPosition >  0 && jointPosition < 180; break;
  //   case GRIPPER:    ans = jointPosition > 10 && jointPosition <  73; break;
  //   default:         ans = false;
  // }

  return ans;
}
