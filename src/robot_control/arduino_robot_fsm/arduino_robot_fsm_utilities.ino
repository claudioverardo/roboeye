void initializeRobot(int soft_init_level) {
  // initializeRobot(SOFT_INIT_DISABLED) the softInit is disabled and you can use the pin 12
  if (soft_init_level != SOFT_INIT_DISABLED) {
    pinMode(SOFT_INIT_CONTROL_PIN, OUTPUT);
    digitalWrite(SOFT_INIT_CONTROL_PIN, LOW);
  }

  // Initialization pin Servo motors
  for (int i = 0; i < QNUM; i++) {
    q[i].attach(qPin[i]);
  }

  // For each step motor this set up the initial degree
  for (int i = 0; i < QNUM; i++) {
    q[i].write(homePosition[i]);
  }
 
  // Turn ON the Braccio softly and save it from brokes.
  if (soft_init_level != SOFT_INIT_DISABLED) {
    _softInit(soft_init_level);
  }
}

void _softwarePWM(int high_time, int low_time) {
  digitalWrite(SOFT_INIT_CONTROL_PIN, HIGH);
  delayMicroseconds(high_time);
  digitalWrite(SOFT_INIT_CONTROL_PIN, LOW);
  delayMicroseconds(low_time);
}

/*
  This function, used only with the Braccio Shield V4 and greater,
  turn ON the Braccio softly and save it from brokes.
  The SOFT_INIT_CONTROL_PIN is used as a software PWM
  @param soft_init_level: the minimum value is -70, default value is 0 (SOFT_INIT_DEFAULT)
*/
void _softInit(int soft_init_level) {
  long int tmp = millis();
  while (millis() - tmp < DELTA_T_INIT_LOW_LIMIT)
    _softwarePWM(80 + soft_init_level, 450 - soft_init_level); //the sum should be 530usec

  while (millis() - tmp < DELTA_T_INIT_HIGH_LIMIT)
    _softwarePWM(75 + soft_init_level, 430 - soft_init_level); //the sum should be 505usec

  digitalWrite(SOFT_INIT_CONTROL_PIN, HIGH);
}

void releaseRobot(int soft_init_level) {
  // Turn off Servo motors
  for (int i = 0; i < QNUM; i++) {
    q[i].detach();
  }
  
  if (soft_init_level != SOFT_INIT_DISABLED) {
    digitalWrite(SOFT_INIT_CONTROL_PIN, LOW);
    pinMode(SOFT_INIT_CONTROL_PIN, INPUT);
  }
}

bool executePointwiseTrajectory(int deltaT) {
  
  bool end_task;
  
  for (int i = 0; i < trajectoryNumPoints; i++) {
    if (checkBoundaries(trajectory[i])) {
      for (int j = 0; j < QNUM; j++) {
        q[j].write(trajectory[i][j]);
      }
      delay(deltaT);
    }
    else {
      end_task = false;
      return end_task;
    }
  }

  end_task = true;
  return end_task;
}

bool braccioServoMovement(byte targetPosition[], int deltaT) {

  bool done = false;

  // Check if the target position is permitted
  if (checkBoundaries(targetPosition)) {

    // Until the all motors are in the desired position
    while (!done) {
         
      // For each servo motor if next degree is not the same of the previuos than do the movement   
      for (int i = 0; i < QNUM; i++) {
        // One step ahead
        if (targetPosition[i] > currentPosition[i]) {
          currentPosition[i]++;
        }
        // One step beyond
        else if (targetPosition[i] < currentPosition[i]) {
          currentPosition[i]--;
        }
        // Move
        q[i].write(currentPosition[i]);
      }
      
      // Delay between each movement
      delay(deltaT);
      
      // Check if all the servo motors are in the desired position
      for (int i = 0; i < QNUM; i++) {
        if (targetPosition[i] != currentPosition[i]) {
          done = false;
          break;
        }
        done = true;
      }
    }
    
  }
  
  return done;
}

bool checkBoundaries(byte jointPositions[]) {

  bool ans = true;

  /*
    M1 (q[0]) = base degrees. Allowed values from 0 to 180 degrees
    M2 (q[1]) = shoulder degrees. Allowed values from 15 to 165 degrees
    M3 (q[2]) = elbow degrees. Allowed values from 0 to 180 degrees
    M4 (q[3]) = wrist vertical degrees. Allowed values from 0 to 180 degrees
    M5 (q[4]) = wrist rotation degrees. Allowed values from 0 to 180 degrees
    M6 (q[5]) = gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */

  /*
  ans =  ( jointPositions[BASE]      >=  0 && jointPositions[BASE]      <= 180 )
      && ( jointPositions[SHOULDER]  >= 15 && jointPositions[SHOULDER]  <= 165 )
      && ( jointPositions[ELBOW]     >=  0 && jointPositions[ELBOW]     <= 180 )
      && ( jointPositions[WRIST_ROT] >=  0 && jointPositions[WRIST_ROT] <= 180 )
      && ( jointPositions[WRIST_VER] >=  0 && jointPositions[WRIST_VER] <= 180 )
      && ( jointPositions[GRIPPER]   >= 10 && jointPositions[GRIPPER]   <= 73  );
  */

  return ans;
}

void commandACK(byte command) {
  Serial.write( command);
}

void transitionACK(byte prevState, byte nextState) {
  Serial.write(prevState);
  Serial.write(nextState);
}

void printTrajectory() {
  // Print the trajectory on the serial line in order to check it on Matlab
  Serial.write(trajectoryType);
  Serial.write(trajectoryNumPoints);
  Serial.write(trajectoryDeltaT);
  for (int i = 0; i < trajectoryNumPoints; i++) {
    for (int j = 0; j < QNUM; j++) {
      Serial.write(trajectory[i][j]);
    }
  }
}

void finalizeTrajectory(byte finalPosition[]) {
  // Update the current position
  currentPositionChanged = true;
  for (int j = 0; j < QNUM; j++) {
    currentPosition[j] = finalPosition[j];
  }
  
  // Reset trajectory informations
  trajectoryType = 0;
  trajectoryTypeLoaded = false;
  trajectoryNumPoints = 0;
  trajectoryNumPointsLoaded = false;
  trajectoryDeltaT = 30;
  trajectoryDeltaTLoaded = false;
  trajectoryBytesCounter = 0;
}
