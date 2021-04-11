/*
 * START state of the FSM.
 * Start serial connection with Matlab, send robot informations and go to the NOP state. 
 */
void processStart() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Initialize serial communication with Matlab
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

/*
 * NOP state of the FSM.
 * Robot turned off. Wait for user commands, do nothing in the meanwhile.
 */
void processNOP() {
  // Robot turned off, wait commands
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

/*
 * INITIALIZE state of the FSM.
 * Initialize the robot and go to the READY state.
 */
void processInitialize() {
  // if SOFT_INIT_LEVEL = SOFT_INIT_DISABLED the softInit is disabled and you can use the pin 12
  if (SOFT_INIT_LEVEL != SOFT_INIT_DISABLED) {
    pinMode(SOFT_INIT_CONTROL_PIN, OUTPUT);
    digitalWrite(SOFT_INIT_CONTROL_PIN, LOW);
  }

  // Initialization of all the pin servo motors
  for (int i = 0; i < QNUM; i++) {
    q[i].attach(qPin[i]);
  }

  // Position the robot in the "home" position
  for (int i = 0; i < QNUM; i++) {
    q[i].write(homePosition[i]);
  }
 
  // Turn ON the robot softly and save it from brokes
  if (SOFT_INIT_LEVEL != SOFT_INIT_DISABLED) {
    softInit(SOFT_INIT_LEVEL);
  }

  // Reset trajectory informations, update current position
  finalizeTrajectory(homePosition);
  
  // Next state transition -> READY
  transitionACK(INITIALIZE,READY);
  state = READY;
}

/*
 * READY state of the FSM.
 * Robot initialized and ready to load/perform a trajectory.
 */
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

/*
 * LOAD_TRAJECTORY state of the FSM.
 * Load a trajectory from Matlab into the robot.
 */
void processLoadTrajectory() {
  // Wait commands/data
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    // Type of the trajectory (0: unknown, 1: pointwise, 2: keypoints)
    if (!trajectoryTypeLoaded) {
      trajectoryType = incomingByte;
      trajectoryTypeLoaded = true;
    }
    // Number of points of the trajectory
    else if (!trajectoryNumPointsLoaded) {
      trajectoryNumPoints = incomingByte;
      trajectoryNumPointsLoaded = true;
      // Next state transition -> ERROR_STATE
      if (trajectoryNumPoints > MAXPOINTS) {
        transitionACK(LOAD_TRAJECTORY,ERROR_STATE);
        state = ERROR_STATE;
      }
    }
    // Timestep of the trajectory
    else if (!trajectoryDeltaTLoaded) {
      trajectoryDeltaT = incomingByte;
      trajectoryDeltaTLoaded = true;
    }
    // Points of the trajectory
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

/*
 * POINTWISE_TRAJECTORY state of the FSM.
 * Check the loaded trajectory and execute it as pointwise trajectory.
 */
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

/*
 * KEYPOINTS_TRAJECTORY state of the FSM.
 * Check the loaded trajectory and execute it as keypoints trajectory.
 */
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

/*
 * RELEASE state of the FSM.
 * Turn off the robot and go to the NOP state.
 */
void processRelease() {
  // Turn off all the servo motors
  for (int i = 0; i < QNUM; i++) {
    q[i].detach();
  }
  
  if (SOFT_INIT_LEVEL != SOFT_INIT_DISABLED) {
    digitalWrite(SOFT_INIT_CONTROL_PIN, LOW);
    pinMode(SOFT_INIT_CONTROL_PIN, INPUT);
  }
  
  delay(DELTA_T_RELEASE);
  
  // Next state transition -> NOP
  transitionACK(RELEASE,NOP);
  state = NOP;
}

/*
 * ERROR_STATE state of the FSM.
 * Some error occured. The robot does not receive commands and can be turned off only. 
 */
void processErrorState() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Wait commands
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

/*
 * END state of the FSM.
 * Robot turned off and serial connection closed. Do nothing until the next reboot.
 */
void processEnd() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(DELTA_T_END);
  digitalWrite(LED_BUILTIN, LOW);
  delay(DELTA_T_END);
}
