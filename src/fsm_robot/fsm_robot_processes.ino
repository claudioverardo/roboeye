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
      cmdACK();
      transitionACK(NOP,INITIALIZE);
      state = INITIALIZE;
    }
    else if (incomingByte == 0) {
      cmdACK();
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
  
  // Next state transition -> HOME
  transitionACK(INITIALIZE,HOME);
  state = HOME;
}

void processHome(){
  // Robot in home position, wait commands
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    // Next state transition -> RELEASE
    if (incomingByte == 0) {
      cmdACK();
      transitionACK(HOME,RELEASE);
      state = RELEASE;
    }
    // Next state transition -> HOME
    else if (incomingByte == 1) {
      cmdACK();
      for (int i = 0; i < QNUM; i++) {
        q[i].write(homePosition[i]);
      }  
      transitionACK(HOME,HOME);
      state = HOME;
    }
    // Next state transition -> LOAD_TRAJECTORY
    else if (incomingByte == 2) {
      cmdACK();
      transitionACK(HOME,LOAD_TRAJECTORY);
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
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    if (incomingByte == 2) {
      cmdACK();
      
      // Follow the loaded trajectory
      bool end_task;
      end_task = executeLoadedTrajectory();
          
      // Next state transition -> ERROR
      if (!end_task) {
        transitionACK(FOLLOW_TRAJECTORY,ERROR_STATE);
        state = ERROR_STATE;
        return;
      }

      // Update the actual position
      for (int j = 0; j < QNUM; j++) {
        actualPosition[j] = trajectory[MAXPOINTS-1][j];
      }
      
      // Next state transition -> DONE
      transitionACK(FOLLOW_TRAJECTORY,DONE);
      state = DONE;
    }
    // Next state transition -> BACK_HOME
    else if (incomingByte == 1) {
      cmdACK();
      transitionACK(FOLLOW_TRAJECTORY,BACK_HOME);
      state = BACK_HOME;
    }
    // Next state transition -> RELEASE
    else if (incomingByte == 0) {
      cmdACK();
      transitionACK(FOLLOW_TRAJECTORY,RELEASE);
      state = RELEASE;
    }
  }
}

void processDone() {  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  
    // Next state transition -> RELEASE
    if (incomingByte == 0) {
      cmdACK();
      transitionACK(DONE,RELEASE);
      state = RELEASE;
    }
    // Next state transition -> BACK_HOME
    else if (incomingByte == 1) {
      cmdACK();
      transitionACK(DONE,BACK_HOME);
      state = BACK_HOME;
    }
  }
}

void processBackHome() {
  // For each step motor this set up the initial degree
  // for (int i = 0; i < QNUM; i++) {
  //   q[i].write(homePosition[i]);
  // }
  
  // Reach the home position with constant-velocity motion
  bool end_task;
  end_task = executeTrivialTrajectory(homePosition);

  // Next state transition -> ERROR
  if (!end_task) {
     transitionACK(BACK_HOME,ERROR_STATE);
     state = ERROR_STATE;
     return;
  }
  
  delay(DELTA_T_BACK_HOME);

  // Next state transition -> HOME
  transitionACK(BACK_HOME,HOME);
  state = HOME;
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
      cmdACK();
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
