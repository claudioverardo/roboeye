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

#define SOFT_INIT_DISABLED -999
#define SOFT_INIT_DEFAULT 0
#define SOFT_INIT_CONTROL_PIN 12

#define DELTA_T_START 1000 // [ms]
#define DELTA_T_INIT_LOW_LIMIT  2000 // [ms] -- softStart
#define DELTA_T_INIT_HIGH_LIMIT 6000 // [ms] -- softStart
#define DELTA_T_RELEASE 1000 // [ms]
#define DELTA_T_END 500 // [ms]

enum State {
  START,
  NOP,
  INITIALIZE,
  READY,
  LOAD_TRAJECTORY,
  POINTWISE_TRAJECTORY,
  KEYPOINTS_TRAJECTORY,
  RELEASE,
  ERROR_STATE,
  END
};
State state = START;

Servo q[QNUM];
const int qPin[QNUM] = {11, 10, 9, 6, 5, 3};

// const byte homePosition[QNUM] = {90, 90, 90, 90, 90, 73}; // nominal
// const byte homePosition[QNUM] = {90, 83, 98, 97, 90, 0}; // corrected old
const byte homePosition[QNUM] = {90, 84, 99, 95, 90, 0}; // corrected
byte currentPosition[QNUM];
bool currentPositionChanged = false;

byte trajectory[MAXPOINTS][QNUM];
byte trajectoryType = 0; // 0: unknown 1: pointwise 2: keypoints
bool trajectoryTypeLoaded = false;
byte trajectoryNumPoints = 0;
bool trajectoryNumPointsLoaded = false;
byte trajectoryDeltaT = 30;
bool trajectoryDeltaTLoaded = false;
int  trajectoryBytesCounter = 0;

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
    case POINTWISE_TRAJECTORY:
      processPointwiseTrajectory();
      break;
    case KEYPOINTS_TRAJECTORY:
      processKeypointsTrajectory();
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
