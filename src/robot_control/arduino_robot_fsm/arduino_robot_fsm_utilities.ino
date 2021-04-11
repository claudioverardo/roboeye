/*
 * Write on the serial the last command received as ACK for Matlab.
 * @param command: command to be written on the serial
 */
void commandACK(byte command) {
  Serial.write(command);
}

/*
 * Write on the serial the last state transition as ACK for Matlab.
 * @param prevState: state before the transition
 * @param nextState: state after the transition
 */
void transitionACK(byte prevState, byte nextState) {
  Serial.write(prevState);
  Serial.write(nextState);
}

/*
 * This function turn ON the Braccio robot softly and save it from brokes.
 * The SOFT_INIT_CONTROL_PIN is used as a software PWM.
 * @param soft_init_level: the minimum value is -70, default value is 0 (SOFT_INIT_DEFAULT)
 */
void softInit(int soft_init_level) {
  long int tmp = millis();
  while (millis() - tmp < DELTA_T_INIT_LOW_LIMIT)
    softwarePWM(80 + soft_init_level, 450 - soft_init_level); //the sum should be 530usec

  while (millis() - tmp < DELTA_T_INIT_HIGH_LIMIT)
    softwarePWM(75 + soft_init_level, 430 - soft_init_level); //the sum should be 505usec

  digitalWrite(SOFT_INIT_CONTROL_PIN, HIGH);
}

/*
 * Software implementation of the PWM for the SOFT_START_CONTROL_PIN.
 * @param high_time: the time in the logic level high
 * @param low_time: the time in the logic level low
 */
void softwarePWM(int high_time, int low_time) {
  digitalWrite(SOFT_INIT_CONTROL_PIN, HIGH);
  delayMicroseconds(high_time);
  digitalWrite(SOFT_INIT_CONTROL_PIN, LOW);
  delayMicroseconds(low_time);
}

/*
 * Execute a pointwise trajectory.
 * @param deltaT: timestep of each point of the trajectory
 * @return end_task: true if the trajectory is completed without errors, false otherwise
 */
bool executePointwiseTrajectory(int deltaT) {
  
  bool end_task;
  
  // For all the points of the trajectory
  for (int i = 0; i < trajectoryNumPoints; i++) {
    // Check if the target position is permitted
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

/*
 * Interpolate and execute the trajectory between the current position and a target keypoint.
 * @param targetPosition: target keypoint of the trajectory
 * @param deltaT: timestep of each point of the trajectory
 * @return end_task: true if the trajectory is completed without errors, false otherwise
 */
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

/*
 * Check if a joints position satisfy the robot constraints.
 * @param jointsPosition: joints position under test
 * @return ans: true if the robot constraints are satisfied, false otherwise
 */
bool checkBoundaries(byte jointsPosition[]) {

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
  ans =  ( jointsPosition[BASE]      >=  0 && jointsPosition[BASE]      <= 180 )
      && ( jointsPosition[SHOULDER]  >= 15 && jointsPosition[SHOULDER]  <= 165 )
      && ( jointsPosition[ELBOW]     >=  0 && jointsPosition[ELBOW]     <= 180 )
      && ( jointsPosition[WRIST_ROT] >=  0 && jointsPosition[WRIST_ROT] <= 180 )
      && ( jointsPosition[WRIST_VER] >=  0 && jointsPosition[WRIST_VER] <= 180 )
      && ( jointsPosition[GRIPPER]   >= 10 && jointsPosition[GRIPPER]   <= 73  );
  */

  return ans;
}

/*
 * Print the loaded trajectory on the serial line in order to check it on Matlab.
 */
void printTrajectory() {
  Serial.write(trajectoryType);
  Serial.write(trajectoryNumPoints);
  Serial.write(trajectoryDeltaT);
  for (int i = 0; i < trajectoryNumPoints; i++) {
    for (int j = 0; j < QNUM; j++) {
      Serial.write(trajectory[i][j]);
    }
  }
}

/*
 * After the execution of a trajectory, update the current position and reset the trajectory data.
 * @param finalPosition: last position of the executed trajectory
 */
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
