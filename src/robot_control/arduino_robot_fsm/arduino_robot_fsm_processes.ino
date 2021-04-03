void processStart() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Init serial communication with Matlab
  Serial.begin(BAUD_RATE);
  
  delay(DELTA_T_START);

  digitalWrite(LED_BUILTIN, LOW);
  
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
  initializeRobot(SOFT_START_DEFAULT);
  
  // Next state transition -> READY
  currentPositionPlotted = false;
  transitionACK(INITIALIZE,READY);
  state = READY;
}

void processReady(){
  // Plot the actual position of the robot
  if (!currentPositionPlotted) {
    for (int i=0; i<QNUM; i++) {
      Serial.print((char) currentPosition[i]);
    }
    currentPositionPlotted = true;
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
    // Next state transition -> BUILT_IN_TRAJECTORY
    else if (incomingByte == 1 || incomingByte == 4 || incomingByte == 5) {
      commandACK(incomingByte);
      transitionACK(READY,BUILT_IN_TRAJECTORY);
      state = BUILT_IN_TRAJECTORY;
    }
    // Next state transition -> READY
    else if (incomingByte == 2) {
      commandACK(incomingByte);
      for (int i = 0; i < QNUM; i++) {
        q[i].write(homePosition[i]);
        currentPosition[i] = homePosition[i];
      }
      currentPositionPlotted = false;
      transitionACK(READY,READY);
      state = READY;
    }
    // Next state transition -> LOAD_TRAJECTORY
    else if (incomingByte == 3) {
      commandACK(incomingByte);
      transitionACK(READY,LOAD_TRAJECTORY);
      state = LOAD_TRAJECTORY;
    }
  
  }
}

void processLoadTrajectory() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    
    // Load trajectory array
    trajectory[trajectoryBytesCounter / QNUM][trajectoryBytesCounter % QNUM] = incomingByte;
    trajectoryBytesCounter++;

    // Check if the trajectory is complete
    if (trajectoryBytesCounter >= QNUM * MAXPOINTS) {

      // Reset the counter
      trajectoryBytesCounter = 0;

      // Print the trajectory on the serial line to check it on Matlab
      for (int i = 0; i < MAXPOINTS; i++) {
        for (int j = 0; j < QNUM; j++) {
          Serial.print((char) trajectory[i][j]);
        }
      }
      
      // Next state transition -> FOLLOW_TRAJECTORY
      transitionACK(LOAD_TRAJECTORY,FOLLOW_TRAJECTORY);
      state = FOLLOW_TRAJECTORY;
    }
  }
}

void processFollowTrajectory() {
  // Follow the loaded trajectory
  bool end_task;
  end_task = executeLoadedTrajectory(DELTA_T_EXECUTE_LOADED_TRAJECTORY);
      
  // Next state transition -> ERROR
  if (!end_task) {
    transitionACK(FOLLOW_TRAJECTORY,ERROR_STATE);
    state = ERROR_STATE;
    return;
  }

  // Update the actual position
  for (int j = 0; j < QNUM; j++) {
    currentPosition[j] = trajectory[MAXPOINTS-1][j];
  }
  
  // Next state transition -> READY
  currentPositionPlotted = false;
  transitionACK(FOLLOW_TRAJECTORY,READY);
  state = READY;
}

void processBuiltInTrajectory() {
  
  if (Serial.available() > 0) {

    byte targetPosition[QNUM];
    byte numTargetPositions;

    numTargetPositions = Serial.read();
  
    for (int i=0; i<numTargetPositions; i++) {
    
      Serial.readBytes(targetPosition, QNUM);
      
      // Reach the target position with constant-velocity motion
      bool end_task;
      end_task = executeBuiltInTrajectory(targetPosition, DELTA_T_EXECUTE_BUILT_IN_TRAJECTORY);
    
      // Next state transition -> ERROR
      if (!end_task) {
         transitionACK(BUILT_IN_TRAJECTORY,ERROR_STATE);
         state = ERROR_STATE;
         return;
      }
    }
  
    // Next state transition -> READY
    currentPositionPlotted = false;
    transitionACK(BUILT_IN_TRAJECTORY,READY);
    state = READY;
  }
}

void processRelease() {
  // Turn off all the servo motors
  releaseRobot(SOFT_START_DEFAULT);
  
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
