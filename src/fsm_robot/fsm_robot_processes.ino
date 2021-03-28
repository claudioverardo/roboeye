void processSetup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Init serial communication with Matlab
  Serial.begin(BAUD_RATE);
  
  delay(DELTA_T_SETUP);

  digitalWrite(LED_BUILTIN, LOW);
  
  transition_ACK(SETUP, NOP);
  state = NOP;
}

void processNOP() {
  // Wait for user commands, do nothing in the meanwhile
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }

  // Next state transition -> INITIALIZE
  if (incomingByte == 1) {
    transition_ACK(NOP,INITIALIZE);
    state = INITIALIZE;
  } 
}

void processInitialize() {
  // Initialization functions and set up the initial position for Braccio
  // All the servo motors will be positioned in the "home" position (cf above)
  initializeRobot(SOFT_START_DEFAULT);
  
  // Next state transition -> HOME
  transition_ACK(INITIALIZE,HOME);
  state = HOME;
}

void processHome(){
  // Robot in home position, wait commands
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    // Next state transition -> RELEASE
    if (incomingByte == 0) {
      transition_ACK(HOME,RELEASE);
      state = RELEASE;
    }
    // Next state transition -> HOME
    else if (incomingByte == 1) {
      for (int i = 0; i < QNUM; i++) {
        q[i].write(homePosition[i]);
      }  
      transition_ACK(HOME,HOME);
      state = HOME;
    }
    // Next state transition -> LOAD_TRAJECTORY
    else if (incomingByte == 2) {
      transition_ACK(HOME,LOAD_TRAJECTORY);
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
      transition_ACK(LOAD_TRAJECTORY,FOLLOW_TRAJECTORY);
      state = FOLLOW_TRAJECTORY;
    }
  }
}

void processFollowTrajectory() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    if (incomingByte == 2) {
      
      // Follow the loaded trajectory
      bool end_task;
      end_task = executeLoadedTrajectory();
          
      if (!end_task) {
        transition_ACK(FOLLOW_TRAJECTORY,ERROR_STATE);
        state = ERROR_STATE;
        return;
      }

      // Update the actual position
      for (int j = 0; j < QNUM; j++) {
        actualPosition[j] = trajectory[MAXPOINTS-1][j];
      }
      
      // Next state transition -> DONE
      transition_ACK(FOLLOW_TRAJECTORY,DONE);
      state = DONE;
    }
    // Next state transition -> BACK_HOME
    else if (incomingByte == 1) {
      transition_ACK(FOLLOW_TRAJECTORY,BACK_HOME);
      state = BACK_HOME;
    }
    // Next state transition -> RELEASE
    else if (incomingByte == 0) {
      transition_ACK(FOLLOW_TRAJECTORY,RELEASE);
      state = RELEASE;
    }
  }
}

void processDone() {  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  
    // Next state transition -> RELEASE
    if (incomingByte == 0) {
      transition_ACK(DONE,RELEASE);
      state = RELEASE;
    }
    // Next state transition -> BACK_HOME
    else if (incomingByte == 1) {
      transition_ACK(DONE,BACK_HOME);
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

  if (!end_task) {
     transition_ACK(BACK_HOME,ERROR_STATE);
     state = ERROR_STATE;
     return;
  }
  
  delay(DELTA_T_BACK_HOME);

  // Next state transition -> HOME
  transition_ACK(BACK_HOME,HOME);
  state = HOME;
}

void processRelease() {
  // Turn off all the servo motors
  releaseRobot(SOFT_START_DEFAULT);
  
  delay(DELTA_T_RELEASE);
  
  // Next state transition -> NOP
  transition_ACK(RELEASE,NOP);
  state = NOP;
}

void processErrorState() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    
    // Next state transition -> RELEASE
    if (incomingByte == 0) {
      transition_ACK(ERROR_STATE,RELEASE);
      state = RELEASE;
    }
  }
}
