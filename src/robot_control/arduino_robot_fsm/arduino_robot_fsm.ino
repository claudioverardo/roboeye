/*
 * Low-level finite state machine (FSM) for Tinkerkit Braccio Robot
 * Written by Claudio Verardo, Mattia Balutto
 */

#include <Servo.h>

// Baud rate of the serial connection
#define BAUD_RATE 115200

// Number of joints
#define QNUM 6

// Identifiers of the joints
#define BASE      0
#define SHOULDER  1
#define ELBOW     2
#define WRIST_ROT 3
#define WRIST_VER 4
#define GRIPPER   5

// Max number of points of a trajectory
#define MAXPOINTS 170

// Software PWM parameters for soft start of the robot
#define SOFT_INIT_DISABLED    -999
#define SOFT_INIT_DEFAULT        0
#define SOFT_INIT_LEVEL          0
#define SOFT_INIT_CONTROL_PIN   12

// Delays and timeouts of the FSM
#define DELTA_T_START           1000 // [ms]
#define DELTA_T_INIT_LOW_LIMIT  2000 // [ms] -- softStart
#define DELTA_T_INIT_HIGH_LIMIT 6000 // [ms] -- softStart
#define DELTA_T_RELEASE         1000 // [ms]
#define DELTA_T_END              500 // [ms]

// States of the FSM
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

// Servos and control pins
Servo q[QNUM];
const int qPin[QNUM] = {11, 10, 9, 6, 5, 3};

// Home position of the robot
// const byte homePosition[QNUM] = {90, 90, 90, 90, 90, 73}; // nominal
const byte homePosition[QNUM] = {87, 81, 99, 95, 90, 0};     // corrected

// Current position of the robot
byte currentPosition[QNUM];
bool currentPositionChanged = false;

// Data of the loaded trajectory
byte trajectory[MAXPOINTS][QNUM];
byte trajectoryType = 0;
bool trajectoryTypeLoaded = false;
byte trajectoryNumPoints = 0;
bool trajectoryNumPointsLoaded = false;
byte trajectoryDeltaT = 30;
bool trajectoryDeltaTLoaded = false;
int  trajectoryBytesCounter = 0;

// Incoming byte from Matlab
byte incomingByte;

void setup() {}

/*
 * Run the finite-state-machine (FSM) of the robot.
 */
void loop() {
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
