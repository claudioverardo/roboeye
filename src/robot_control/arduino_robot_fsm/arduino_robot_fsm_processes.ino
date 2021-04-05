void processStart() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Init serial communication with Matlab
  Serial.begin(BAUD_RATE);

  // Send robot informations to Matlab
  Serial.write(QNUM);
  Serial.write(MAXPOINTS);
  Serial.write(homePosition,QNUM);
  Serial.write(DELTA_T_START/1000);
  Serial.write(DELTA_T_INIT_HIGH_LIMIT/1000);
  Serial.write(DELTA_T_RELEASE/1000);
  
  delay(DELTA_T_START);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Next state transition -> NOP
  transitionACK(START, NOP);
  state = NOP;
}

void processNOP() {
  // Wait for user commands, do nothing in the meanwhile
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    // Next state transition -> INITIALIZE
    if (incomingByte == 1) {
      commandACK(incomingByte);
      transitionACK(NOP,INITIALIZE);
      state = INITIALIZE;
    }
    // Next state transition -> END
    else if (incomingByte == 0) {
      commandACK(incomingByte);
      transitionACK(NOP,END);
      // Close serial communication with Matlab
      Serial.end();
      state = END;
    }
  }
}

void processInitialize() {
  // Initialization functions and set up the initial position for Braccio
  // All the servo motors will be positioned in the "home" position (cf above)
  initializeRobot(SOFT_INIT_DEFAULT);

  // Reset trajectory informations, update current position
  finalizeTrajectory(homePosition);
  
  // Next state transition -> READY
  transitionACK(INITIALIZE,READY);
  state = READY;
}

void processReady(){
  // Plot the actual position of the robot
  if (currentPositionChanged) {
    for (int i=0; i<QNUM; i++) {
      Serial.write(currentPosition[i]);
    }
    currentPositionChanged = false;
  }
  
  // Robot in home position, wait commands
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    // Next state transition -> RELEASE
    if (incomingByte == 0) {
      commandACK(incomingByte);
      transitionACK(READY,RELEASE);
      state = RELEASE;
    }
    // Next state transition -> LOAD_TRAJECTORY
    else if (incomingByte == 1 || incomingByte == 3 || incomingByte == 4 || incomingByte == 5 || 
             incomingByte == 6 || incomingByte == 7 || incomingByte == 8) {
      commandACK(incomingByte);
      transitionACK(READY,LOAD_TRAJECTORY);
      state = LOAD_TRAJECTORY;
    }
    // Next state transition -> READY
    else if (incomingByte == 2) {
      commandACK(incomingByte);
      for (int i = 0; i < QNUM; i++) {
        q[i].write(homePosition[i]);
        currentPosition[i] = homePosition[i];
      }
      currentPositionChanged = true;
      transitionACK(READY,READY);
      state = READY;
    }
  
  }
}

void processLoadTrajectory() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    if (!trajectoryTypeLoaded) {
      trajectoryType = incomingByte;
      trajectoryTypeLoaded = true;
    }
    else if (!trajectoryNumPointsLoaded) {
      trajectoryNumPoints = incomingByte;
      trajectoryNumPointsLoaded = true;
    }
    else if (!trajectoryDeltaTLoaded) {
      trajectoryDeltaT = incomingByte;
      trajectoryDeltaTLoaded = true;
    }
    else {
      // Load trajectory array
      trajectory[trajectoryBytesCounter / QNUM][trajectoryBytesCounter % QNUM] = incomingByte;
      trajectoryBytesCounter++;
  
      // Check if the trajectory is complete
      if (trajectoryBytesCounter >= QNUM * trajectoryNumPoints) {
  
        // Next state transition -> POINTWISE_TRAJECTORY
        if (trajectoryType == 1) {
          pinMode(LED_BUILTIN, OUTPUT);
          digitalWrite(LED_BUILTIN, HIGH);
          printTrajectory();
          transitionACK(LOAD_TRAJECTORY,POINTWISE_TRAJECTORY);
          state = POINTWISE_TRAJECTORY;
        }
        // Next state transition -> KEYPOINTS_TRAJECTORY
        else if (trajectoryType == 2) {
          printTrajectory();
          transitionACK(LOAD_TRAJECTORY,KEYPOINTS_TRAJECTORY);
          state = KEYPOINTS_TRAJECTORY;
        }
        // Next state transition -> ERROR_STATE
        else {
          transitionACK(LOAD_TRAJECTORY,ERROR_STATE);
          state = ERROR_STATE;
        }
        
      }
    }
  }
}

void processPointwiseTrajectory() {
  // Wait data check from Matlab
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    // ACK from Matlab of the data check
    if (incomingByte == 1) {
      commandACK(incomingByte);
      
      // Follow the loaded trajectory
      bool end_task;
      end_task = executePointwiseTrajectory(trajectoryDeltaT);
          
      // Next state transition -> ERROR_STATE
      if (!end_task) {
        transitionACK(POINTWISE_TRAJECTORY,ERROR_STATE);
        state = ERROR_STATE;
        return;
      }
    
      // Reset trajectory informations, update current position
      finalizeTrajectory(trajectory[trajectoryNumPoints-1]);
      
      // Next state transition -> READY
      transitionACK(POINTWISE_TRAJECTORY,READY);
      state = READY;
    }
    // Next state transition -> ERROR_STATE
    else {
      commandACK(incomingByte);
      transitionACK(POINTWISE_TRAJECTORY,ERROR_STATE);
      state = ERROR_STATE;
    }
  }
}

void processKeypointsTrajectory() {
  // Wait data check from Matlab
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    // ACK from Matlab of the data check
    if (incomingByte == 1) {
      commandACK(incomingByte);
      
      // Follow the loaded trajectory
      bool end_task;
      for (int i=0; i<trajectoryNumPoints; i++) {
        // Reach the target position with constant-velocity motion
        end_task = braccioServoMovement(trajectory[i], trajectoryDeltaT);
  
        // Next state transition -> ERROR_STATE
        if (!end_task) {
          transitionACK(KEYPOINTS_TRAJECTORY,ERROR_STATE);
          state = ERROR_STATE;
          return;
        }
      }

      // Reset trajectory informations, update current position
      finalizeTrajectory(trajectory[trajectoryNumPoints-1]);
      
      // Next state transition -> READY
      transitionACK(KEYPOINTS_TRAJECTORY,READY);
      state = READY;
    }
    // Next state transition -> ERROR_STATE
    else {
      commandACK(incomingByte);
      transitionACK(KEYPOINTS_TRAJECTORY,ERROR_STATE);
      state = ERROR_STATE;
    }
  }
  
}

void processRelease() {
  // Turn off all the servo motors
  releaseRobot(SOFT_INIT_DEFAULT);
  
  delay(DELTA_T_RELEASE);
  
  // Next state transition -> NOP
  transitionACK(RELEASE,NOP);
  state = NOP;
}

void processErrorState() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    
    // Next state transition -> RELEASE
    if (incomingByte == 0) {
      commandACK(incomingByte);
      transitionACK(ERROR_STATE,RELEASE);
      state = RELEASE;
    }
  }
}

void processEnd() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(DELTA_T_END);
  digitalWrite(LED_BUILTIN, LOW);
  delay(DELTA_T_END);
}
