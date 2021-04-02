#include <Servo.h>

#define BAUD_RATE 115200

#define BASE 0
#define SHOULDER 1
#define ELBOW 2
#define WRIST_ROT 3
#define WRIST_VER 4
#define GRIPPER 5

#define QNUM 6
#define MAXPOINTS 170

#define SOFT_START_DISABLED -999
#define SOFT_START_DEFAULT 0
#define SOFT_START_CONTROL_PIN 12
#define LOW_LIMIT_TIMEOUT 2000
#define HIGH_LIMIT_TIMEOUT 6000

#define DELTA_T_START 1000 // [ms]
#define DELTA_T_BACK_HOME 500 // [ms]
#define DELTA_T_EXECUTE_LOADED_TRAJECTORY 10 // [ms]
#define DELTA_T_EXECUTE_BUILT_IN_TRAJECTORY 30 // [ms]
#define DELTA_T_BUILT_IN_TRAJECTORY 500 // [ms]
#define DELTA_T_RELEASE 2000 // [ms]
#define DELTA_T_END 500 // [ms]

enum State {
  START,
  NOP,
  INITIALIZE,
  READY,
  LOAD_TRAJECTORY,
  FOLLOW_TRAJECTORY,
  BUILT_IN_TRAJECTORY,
  RELEASE,
  ERROR_STATE,
  END
};
State state = START;

Servo q[QNUM];
const int qPin[QNUM] = {11, 10, 9, 6, 5, 3};

// const byte homePosition[QNUM] = {90, 90, 90, 90, 90, 73}; // nominal
const byte homePosition[QNUM] = {90, 83, 98, 97, 90, 0}; // corrected
byte currentPosition[QNUM];
bool currentPositionPlotted = false;

byte trajectory[MAXPOINTS][QNUM];
int trajectoryBytesCounter = 0;

byte incomingByte;

void setup() {}

void loop() {
  // Run finite-state machine (FSM)
  switch (state) {
    case START:
      processStart();
      break;
    case NOP:
      processNOP();
      break;
    case INITIALIZE:
      processInitialize();
      break;
    case READY:
      processReady();
      break;
    case LOAD_TRAJECTORY:
      processLoadTrajectory();
      break;
    case FOLLOW_TRAJECTORY:
      processFollowTrajectory();
      break;
    case BUILT_IN_TRAJECTORY:
      processBuiltInTrajectory();
      break;
    case RELEASE:
      processRelease();
      break;
    case ERROR_STATE:
      processErrorState();
      break;
    case END:
      processEnd();
      break;
  }
}
