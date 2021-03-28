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

#define DELTA_T_SETUP 2000 // [ms]
#define DELTA_T_BACK_HOME 500 // [ms]
#define DELTA_T_RELEASE 2000 // [ms]
#define DELTA_T_LOADED_TRAJECTORY 10 // [ms]
#define DELTA_T_TRIVIAL_TRAJECTORY 30 // [ms]

enum State {
  SETUP,
  NOP,
  INITIALIZE,
  HOME,
  LOAD_TRAJECTORY,
  FOLLOW_TRAJECTORY,
  DONE,
  BACK_HOME,
  RELEASE,
  ERROR_STATE
};
State state = SETUP;

Servo q[QNUM];
const int qPin[QNUM] = {11, 10, 9, 6, 5, 3};

// const byte homePosition[QNUM] = {90, 90, 90, 90, 90, 73}; // nominal
const byte homePosition[QNUM] = {90, 83, 98, 97, 90, 0}; // corrected
byte actualPosition[QNUM];

byte trajectory[MAXPOINTS][QNUM];
int trajectoryBytesCounter = 0;

byte incomingByte;

void setup() {}

void loop() {
  // Run finite-state machine (FSM)
  switch (state) {
    case SETUP:
      processSetup();
      break;
    case NOP:
      processNOP();
      break;
    case INITIALIZE:
      processInitialize();
      break;
    case HOME:
      processHome();
      break;
    case LOAD_TRAJECTORY:
      processLoadTrajectory();
      break;
    case FOLLOW_TRAJECTORY:
      processFollowTrajectory();
      break;
    case DONE:
      processDone();
      break;
    case BACK_HOME:
      processBackHome();
      break;
    case RELEASE:
      processRelease();
      break;
    case ERROR_STATE:
      processErrorState();
      break;
  }
}
