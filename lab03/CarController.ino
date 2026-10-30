
#include <math.h>

// =====================================================
// ARDUINO UNO R4 WIFI - RIGHT WALL FOLLOWING ROBOT
// No external LEDs
// New FRONT = motor-wheel end of robot
// Right-facing ultrasonic sensor
// =====================================================

const int buttonPin = 2;

// ---------------- MOTOR DRIVER PINS ------------------
// Retain the existing L298N pin wiring.

const int leftMotorEnablePin = 10;
const int leftMotorInput1 = A4;
const int leftMotorInput2 = A3;

const int rightMotorEnablePin = 9;
const int rightMotorInput1 = A0;
const int rightMotorInput2 = A1;

// ---------------- ENCODER PINS -----------------------

const int leftEncoderPin = 12;
const int rightEncoderPin = 11;

// ---------------- ULTRASONIC PINS --------------------

const int frontTrigPin = 5;
const int frontEchoPin = 6;

// This sensor now faces RIGHT.
const int rightTrigPin = 7;
const int rightEchoPin = 8;

// =====================================================
// GENERAL SETTINGS
// =====================================================

int speed = 85;
char movement = ' ';

const unsigned long controlInterval = 100;
unsigned long previousControlTime = 0;

const unsigned long telemetryInterval = 500;
unsigned long previousTelemetryTime = 0;

// Autonomous mode starts after five seconds
// if no serial command is received.
//
// Set false to disable automatic starting
// during setup or testing.

const bool enableAutoStart = false;

const unsigned long autoStartDelayMS = 5000;

bool autoStartPending = true;

// =====================================================
// MOTOR DIRECTION CONFIGURATION
// =====================================================

// The robot's NEW front is the opposite end
// from the previous version.
//
// The old code used negative motor commands
// for forward movement.
//
// This version uses positive motor commands
// to move toward the NEW front.
//
// If W drives in the wrong direction during
// the manual test, change this to -1.

const int motorForwardPolarity = 1;

// =====================================================
// FOLLOW ME SETTINGS
// =====================================================

const float followSetpointCM = 25.0;
const float followToleranceCM = 2.0;

const float followKp = 8.0;

const int maxFollowPWM = 200;
const int minMovingPWM = 125;

// =====================================================
// RIGHT WALL FOLLOWING - P CONTROLLER
// =====================================================

const float wallSetpointCM = 25.0;

const float wallKp = 3.0;

const int maxWallBasePWM = 95;
const int maxWallCorrectionPWM = 10;

const float sideFilterAlpha = 0.35;
const float maxCorrectionStep = 12.0;

// =====================================================
// CORNER NAVIGATION SETTINGS
// =====================================================

// Start a turn when an obstacle is approaching.
const float insideCornerDistanceCM = 30.0;

// Front must be this clear before continuing.
const float frontClearDistanceCM = 32.0;

// Right wall considered missing beyond this.
const float wallLostDistanceCM = 55.0;

// Right wall detected again within this distance.
const float wallFoundDistanceCM = 38.0;

// Consecutive samples to confirm an opening.
const int wallLostConfirmations = 3;

// Motor speed during turns.
const int insideTurnPWM = 105;
const int outsideTurnPWM = 105;

// Minimum time before accepting a completed turn.
const unsigned long minTurnTimeMS = 300;

// Maximum permitted turning time.
const unsigned long maxTurnTimeMS = 5000;

// Allow limited invalid front measurements
// during a turn.
const int maxInvalidFrontReadings = 5;

// Require two clear-front measurements
// before considering the turn complete.
const int frontClearConfirmations = 2;

// =====================================================
// BACKUP SETTINGS
// =====================================================

// If the front sensor reports less than 15 cm
// at the start of a turn, back up briefly.

const float minSpinClearanceCM = 15.0;

const float backupClearDistanceCM = 22.0;

const int backupPWM = 105;

const unsigned long maxBackupMS = 700;

// No rear sensor is installed. Backup is
// strictly time-limited and requires testing.

// =====================================================
// OPTIONAL ENCODER STALL DETECTION
// =====================================================

// Leave disabled until you have confirmed
// both encoders count reliably during turns.
//
// Turn timeouts still remain active.

const bool enableEncoderStallCheck = false;

const unsigned long spinStallTimeMS = 700;

// =====================================================
// SENSOR AND NAVIGATION VARIABLES
// =====================================================

float frontDistanceCM = NAN;
float rightDistanceCM = NAN;

float filteredRightCM = NAN;

float previousCorrection = 0;

int wallLostCounter = 0;
int invalidFrontCounter = 0;
int frontClearCounter = 0;

const char* stopReason = "none";

enum NavigationState {
  FOLLOW_WALL,
  BACK_UP,
  INSIDE_TURN,
  OUTSIDE_TURN,
  NAV_STOPPED
};

NavigationState navState = NAV_STOPPED;

// Which turn to perform after backing up.
NavigationState pendingTurn = INSIDE_TURN;

unsigned long stateStartTime = 0;

// =====================================================
// ENCODER VARIABLES
// =====================================================

unsigned long leftPulseCount = 0;
unsigned long rightPulseCount = 0;

unsigned long previousLeftPulseCount = 0;
unsigned long previousRightPulseCount = 0;

int previousLeftEncoderState = HIGH;
int previousRightEncoderState = HIGH;

unsigned long lastMotionTime = 0;
unsigned long lastMotionPulseTotal = 0;

// Forward declaration, used during sensor delay.
void updateEncoders();

// =====================================================
// MOTOR CONTROL
// =====================================================

void setMotor(
  int enablePin,
  int input1,
  int input2,
  int motorPWM
) {

  motorPWM = constrain(motorPWM, -255, 255);

  // Disable before changing motor direction.
  analogWrite(enablePin, 0);

  digitalWrite(
    input1,
    motorPWM < 0 ? HIGH : LOW
  );

  digitalWrite(
    input2,
    motorPWM > 0 ? HIGH : LOW
  );

  analogWrite(enablePin, abs(motorPWM));
}

// Drive using the NEW front of the robot.
// Positive PWM means forward toward sensors.

void drive(int leftPWM, int rightPWM) {

  setMotor(
    leftMotorEnablePin,
    leftMotorInput1,
    leftMotorInput2,
    motorForwardPolarity * leftPWM
  );

  setMotor(
    rightMotorEnablePin,
    rightMotorInput1,
    rightMotorInput2,
    motorForwardPolarity * rightPWM
  );
}

// =====================================================
// BASIC MOVEMENT
// =====================================================

void forward(int pwm) {
  drive(pwm, pwm);
}

void backward(int pwm) {
  drive(-pwm, -pwm);
}

void stopMotors() {
  drive(0, 0);
}

// Gentle steering toward the new left.
void turnLeft(int pwm) {
  drive(pwm / 2, pwm);
}

// Gentle steering toward the new right.
void turnRight(int pwm) {
  drive(pwm, pwm / 2);
}

// Counterclockwise rotation.
void spinLeft(int pwm) {
  drive(-pwm, pwm);
}

// Clockwise rotation.
void spinRight(int pwm) {
  drive(pwm, -pwm);
}

// =====================================================
// ULTRASONIC SENSOR FUNCTIONS
// =====================================================

float readDistanceCM(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration =
    pulseIn(echoPin, HIGH, 25000UL);

  if (duration == 0) {
    return NAN;
  }

  float distanceCM =
    duration * 0.0343 / 2.0;

  if (distanceCM < 2.0 ||
      distanceCM > 350.0) {

    return NAN;
  }

  return distanceCM;
}

void readBothSensors() {

  frontDistanceCM =
    readDistanceCM(frontTrigPin, frontEchoPin);

  // Give ultrasonic echoes time to settle.
  // Continue checking encoder inputs.

  unsigned long waitStart = millis();

  while (millis() - waitStart < 35) {
    updateEncoders();
  }

  rightDistanceCM =
    readDistanceCM(rightTrigPin, rightEchoPin);
}

// =====================================================
// NAVIGATION HELPERS
// =====================================================

void resetWallController() {

  filteredRightCM = NAN;
  previousCorrection = 0;

  wallLostCounter = 0;
  invalidFrontCounter = 0;
  frontClearCounter = 0;
}

void changeNavState(NavigationState nextState) {

  navState = nextState;

  stateStartTime = millis();

  previousCorrection = 0;
  invalidFrontCounter = 0;
  frontClearCounter = 0;

  // Reset encoder-based stall monitoring.
  lastMotionTime = stateStartTime;

  lastMotionPulseTotal =
    leftPulseCount + rightPulseCount;
}

void failSafeStop(const char* reason) {

  stopMotors();

  navState = NAV_STOPPED;

  stopReason = reason;
}

bool wheelsStalled() {

  unsigned long pulseTotal =
    leftPulseCount + rightPulseCount;

  if (pulseTotal != lastMotionPulseTotal) {

    lastMotionPulseTotal = pulseTotal;
    lastMotionTime = millis();

    return false;
  }

  return millis() - lastMotionTime >
         spinStallTimeMS;
}

// =====================================================
// CORNER TURNING HELPERS
// =====================================================

// Following a RIGHT wall:
//
// INSIDE corner (wall ahead, right wall present)
//   -> Turn LEFT, away from the right wall.
//
// OUTSIDE corner (right wall disappears)
//   -> Turn RIGHT, toward the opening.

void performTurn(NavigationState turnState) {

  if (turnState == INSIDE_TURN) {

    spinLeft(insideTurnPWM);

  } else if (turnState == OUTSIDE_TURN) {

    spinRight(outsideTurnPWM);
  }
}

// Reverse briefly to create turning clearance.
void backAwayFromFrontWall() {

  backward(backupPWM);
}

// Begin a left or right maneuver.
void beginTurn(NavigationState requestedTurn) {

  pendingTurn = requestedTurn;

  if (!isnan(frontDistanceCM) &&
      frontDistanceCM < minSpinClearanceCM) {

    changeNavState(BACK_UP);

    backAwayFromFrontWall();

    return;
  }

  changeNavState(requestedTurn);

  performTurn(requestedTurn);
}

// =====================================================
// NAVIGATION STATUS FOR PROCESSING
// =====================================================

const char* navStateName() {

  if (movement != 'g') {
    return "inactive";
  }

  switch (navState) {

    case FOLLOW_WALL:
      return "following";

    case BACK_UP:
      return "backing_up";

    case INSIDE_TURN:
      return "inside_turn";

    case OUTSIDE_TURN:
      return "outside_turn";

    case NAV_STOPPED:
      return stopReason;

    default:
      return "invalid_state";
  }
}

// =====================================================
// FOLLOW ME MODE - H
// =====================================================

void updateFollowMe() {

  frontDistanceCM =
    readDistanceCM(frontTrigPin, frontEchoPin);

  rightDistanceCM = NAN;

  if (isnan(frontDistanceCM)) {
    stopMotors();
    return;
  }

  float error =
    frontDistanceCM - followSetpointCM;

  if (fabs(error) <= followToleranceCM) {

    stopMotors();
    return;
  }

  int pwm = (int)round(followKp * error);

  int direction = pwm > 0 ? 1 : -1;

  int magnitude = constrain(
    abs(pwm),
    minMovingPWM,
    maxFollowPWM
  );

  pwm = direction * magnitude;

  drive(pwm, pwm);
}

// =====================================================
// RIGHT WALL FOLLOWING - P CONTROLLER
// =====================================================

void updateFollowWall() {

  // No reliable front measurement.
  if (isnan(frontDistanceCM)) {

    failSafeStop("front_no_echo");
    return;
  }

  // Determine whether the RIGHT wall is missing.
  bool wallMissing =
    isnan(rightDistanceCM) ||
    rightDistanceCM > wallLostDistanceCM;

  if (wallMissing) {

    wallLostCounter++;

  } else {

    wallLostCounter = 0;
  }

  // -------------------------------------------------
  // FRONT OBSTACLE
  // -------------------------------------------------

  if (frontDistanceCM <= insideCornerDistanceCM) {

    // If the right side might be open, stop briefly
    // while confirming the opening. Don't turn left
    // immediately into the wrong branch.

    if (wallMissing &&
        wallLostCounter < wallLostConfirmations) {

      stopMotors();
      return;
    }

    // Confirmed right-hand opening: turn RIGHT.
    if (wallMissing &&
        wallLostCounter >= wallLostConfirmations) {

      beginTurn(OUTSIDE_TURN);
      return;
    }

    // Right wall present: turn LEFT.
    beginTurn(INSIDE_TURN);
    return;
  }

  // -------------------------------------------------
  // OUTSIDE CORNER / RIGHT WALL DISAPPEARS
  // -------------------------------------------------

  if (wallMissing) {

    if (wallLostCounter >= wallLostConfirmations) {

      beginTurn(OUTSIDE_TURN);
      return;
    }

    // Wait for confirmation before turning.
    forward(maxWallBasePWM);
    return;
  }

  // -------------------------------------------------
  // NORMAL WALL FOLLOWING
  // -------------------------------------------------

  // Exponential smoothing of right distance.
  if (isnan(filteredRightCM)) {

    filteredRightCM = rightDistanceCM;

  } else {

    filteredRightCM =
      sideFilterAlpha * rightDistanceCM +
      (1.0 - sideFilterAlpha) * filteredRightCM;
  }

  // Positive error: too far from RIGHT wall.
  float error =
    filteredRightCM - wallSetpointCM;

  // Proportional correction.
  float desiredCorrection = constrain(
    wallKp * error,
    -maxWallCorrectionPWM,
    maxWallCorrectionPWM
  );

  // Rate limit the correction.
  float correction = constrain(
    desiredCorrection,
    previousCorrection - maxCorrectionStep,
    previousCorrection + maxCorrectionStep
  );

  previousCorrection = correction;

  int basePWM = constrain(
    speed,
    0,
    maxWallBasePWM
  );

  // IMPORTANT: Right-wall correction.
  //
  // Too far from right wall:
  // left wheel faster -> steer RIGHT.
  //
  // Too close to right wall:
  // right wheel faster -> steer LEFT.

  int leftPWM = constrain(
    (int)round(basePWM + correction),
    0,
    255
  );

  int rightPWM = constrain(
    (int)round(basePWM - correction),
    0,
    255
  );

  drive(leftPWM, rightPWM);
}

// =====================================================
// BACK UP BEFORE TURNING
// =====================================================

void updateBackUp() {

  unsigned long elapsed =
    millis() - stateStartTime;

  if (isnan(frontDistanceCM)) {

    failSafeStop("front_no_echo");
    return;
  }

  // Sufficient clearance achieved.
  if (frontDistanceCM >= backupClearDistanceCM) {

    changeNavState(pendingTurn);

    stopMotors();

    // Next control cycle starts the planned turn.
    return;
  }

  // Backing up too long is unsafe because
  // there is no rear-facing sensor.
  if (elapsed >= maxBackupMS) {

    failSafeStop("backup_timeout");
    return;
  }

  backAwayFromFrontWall();
}

// =====================================================
// INSIDE CORNER - TURN LEFT
// =====================================================

void updateInsideTurn() {

  unsigned long elapsed =
    millis() - stateStartTime;

  if (elapsed > maxTurnTimeMS) {

    failSafeStop("turn_timeout");
    return;
  }

  if (enableEncoderStallCheck && wheelsStalled()) {

    failSafeStop("turn_stall");
    return;
  }

  if (isnan(frontDistanceCM)) {

    invalidFrontCounter++;

    if (invalidFrontCounter >= maxInvalidFrontReadings) {

      failSafeStop("front_no_echo");
      return;
    }

    spinLeft(insideTurnPWM);
    return;
  }

  invalidFrontCounter = 0;

  // Require consecutive clear measurements.
  if (frontDistanceCM >= frontClearDistanceCM) {

    frontClearCounter++;

  } else {

    frontClearCounter = 0;
  }

  if (elapsed >= minTurnTimeMS &&
      frontClearCounter >= frontClearConfirmations) {

    changeNavState(FOLLOW_WALL);

    resetWallController();

    stopMotors();
    return;
  }

  // Continue counterclockwise rotation.
  spinLeft(insideTurnPWM);
}

// =====================================================
// OUTSIDE CORNER - TURN RIGHT
// =====================================================

void updateOutsideTurn() {

  unsigned long elapsed =
    millis() - stateStartTime;

  if (elapsed > maxTurnTimeMS) {

    failSafeStop("turn_timeout");
    return;
  }

  if (enableEncoderStallCheck && wheelsStalled()) {

    failSafeStop("turn_stall");
    return;
  }

  if (isnan(frontDistanceCM)) {

    invalidFrontCounter++;

    if (invalidFrontCounter >= maxInvalidFrontReadings) {

      failSafeStop("front_no_echo");
      return;
    }

    spinRight(outsideTurnPWM);
    return;
  }

  invalidFrontCounter = 0;

  // The forward path must be clear.
  bool frontClear =
    frontDistanceCM >= frontClearDistanceCM;

  // Look for the RIGHT wall again.
  bool foundRightWall =
    !isnan(rightDistanceCM) &&
    rightDistanceCM <= wallFoundDistanceCM;

  // Require both conditions for two
  // consecutive samples.

  if (frontClear && foundRightWall) {

    frontClearCounter++;

  } else {

    frontClearCounter = 0;
  }

  if (elapsed >= minTurnTimeMS &&
      frontClearCounter >= frontClearConfirmations) {

    changeNavState(FOLLOW_WALL);

    resetWallController();

    stopMotors();
    return;
  }

  // Continue clockwise rotation.
  spinRight(outsideTurnPWM);
}

// =====================================================
// AUTONOMOUS MODE CONTROLLER
// =====================================================

void updateAutonomousMode() {

  if (movement != 'h' &&
      movement != 'g') {

    return;
  }

  unsigned long currentTime = millis();

  if (currentTime - previousControlTime <
      controlInterval) {

    return;
  }

  previousControlTime = currentTime;

  // FOLLOW ME mode.
  if (movement == 'h') {

    updateFollowMe();
    return;
  }

  // Hold the fail-safe stop until restarted.
  if (navState == NAV_STOPPED) {

    stopMotors();
    return;
  }

  // Update ultrasonic distances.
  readBothSensors();

  switch (navState) {

    case FOLLOW_WALL:
      updateFollowWall();
      break;

    case BACK_UP:
      updateBackUp();
      break;

    case INSIDE_TURN:
      updateInsideTurn();
      break;

    case OUTSIDE_TURN:
      updateOutsideTurn();
      break;

    default:
      failSafeStop("invalid_state");
      break;
  }
}

// =====================================================
// MANUAL MOVEMENT COMMANDS
// =====================================================

void applyMovement() {

  switch (movement) {

    case 'w':
      forward(speed);
      break;

    case 's':
      backward(speed);
      break;

    case 'a':
      turnLeft(speed);
      break;

    case 'd':
      turnRight(speed);
      break;

    case 'q':
      spinLeft(speed);
      break;

    case 'e':
      spinRight(speed);
      break;

    default:
      stopMotors();
      break;
  }
}

// =====================================================
// ENCODER TRACKING
// =====================================================

void updateEncoders() {

  int leftState =
    digitalRead(leftEncoderPin);

  int rightState =
    digitalRead(rightEncoderPin);

  if (previousLeftEncoderState == HIGH &&
      leftState == LOW) {

    leftPulseCount++;
  }

  if (previousRightEncoderState == HIGH &&
      rightState == LOW) {

    rightPulseCount++;
  }

  previousLeftEncoderState = leftState;
  previousRightEncoderState = rightState;
}

// =====================================================
// TELEMETRY - PROCESSING COMPATIBLE
// =====================================================

// Keep the existing 10-column CSV format:
//
// t_ms,movement,pwm,left_count,right_count,
// left_pps,right_pps,front_cm,side_cm,nav_state
//
// "side_cm" now represents RIGHT distance.

void printDistance(float distance) {

  if (isnan(distance)) {

    Serial.print("NA");

  } else {

    Serial.print(distance, 2);
  }
}

void printTelemetry() {

  unsigned long now = millis();

  unsigned long elapsed =
    now - previousTelemetryTime;

  if (elapsed < telemetryInterval) {
    return;
  }

  unsigned long leftPulses =
    leftPulseCount - previousLeftPulseCount;

  unsigned long rightPulses =
    rightPulseCount - previousRightPulseCount;

  float leftPPS =
    (leftPulses * 1000.0) / elapsed;

  float rightPPS =
    (rightPulses * 1000.0) / elapsed;

  // Timestamp.
  Serial.print(now);
  Serial.print(",");

  // Current command.
  if (movement == ' ') {

    Serial.print("stop");

  } else {

    Serial.print(movement);
  }

  Serial.print(",");

  // Configured speed.
  Serial.print(speed);
  Serial.print(",");

  // Encoder totals.
  Serial.print(leftPulseCount);
  Serial.print(",");

  Serial.print(rightPulseCount);
  Serial.print(",");

  // Encoder pulses per second.
  Serial.print(leftPPS, 2);
  Serial.print(",");

  Serial.print(rightPPS, 2);
  Serial.print(",");

  // Front distance.
  printDistance(frontDistanceCM);
  Serial.print(",");

  // Right-side distance, in side_cm column.
  printDistance(rightDistanceCM);
  Serial.print(",");

  // Navigation state or stop reason.
  Serial.println(navStateName());

  previousLeftPulseCount = leftPulseCount;
  previousRightPulseCount = rightPulseCount;

  previousTelemetryTime = now;
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  pinMode(buttonPin, INPUT_PULLUP);

  // Motor pins.
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorInput1, OUTPUT);
  pinMode(leftMotorInput2, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorInput1, OUTPUT);
  pinMode(rightMotorInput2, OUTPUT);

  // Encoder pins.
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);

  previousLeftEncoderState =
    digitalRead(leftEncoderPin);

  previousRightEncoderState =
    digitalRead(rightEncoderPin);

  // Ultrasonic sensors.
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);

  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);

  digitalWrite(frontTrigPin, LOW);
  digitalWrite(rightTrigPin, LOW);

  stopMotors();

  // Serial for Processing / USB diagnostics.
  Serial.begin(9600);

  previousTelemetryTime = millis();
  previousControlTime = millis();

  Serial.println(
    "t_ms,movement,pwm,left_count,right_count,"
    "left_pps,right_pps,front_cm,side_cm,nav_state"
  );
}

// =====================================================
// AUTONOMOUS STARTUP
// =====================================================

void startWallFollow() {

  movement = 'g';

  stopMotors();

  stopReason = "none";

  resetWallController();

  changeNavState(FOLLOW_WALL);

  previousControlTime =
    millis() - controlInterval;
}

void updateAutoStart() {

  if (!autoStartPending) {
    return;
  }

  if (!enableAutoStart) {

    autoStartPending = false;
    return;
  }

  if (millis() < autoStartDelayMS) {
    return;
  }

  autoStartPending = false;

  startWallFollow();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  updateEncoders();

  // Process all received serial commands.
  while (Serial.available() > 0) {

    char command = Serial.read();

    if (command == '\r' ||
        command == '\n') {

      continue;
    }

    // Receiving a command cancels auto-start.
    if (autoStartPending) {

      autoStartPending = false;
    }

    switch (command) {

      // Manual driving.
      case 'w':
      case 's':
      case 'a':
      case 'd':
      case 'q':
      case 'e':

        movement = command;

        navState = NAV_STOPPED;

        applyMovement();
        break;

      // Stop.
      case ' ':

        movement = ' ';

        navState = NAV_STOPPED;

        stopReason = "manual_stop";

        stopMotors();
        break;

      // Follow Me.
      case 'h':

        movement = 'h';

        navState = NAV_STOPPED;

        stopReason = "none";

        stopMotors();

        previousControlTime =
          millis() - controlInterval;
        break;

      // Autonomous right-wall following.
      case 'g':

        startWallFollow();
        break;

      // Increase speed.
      case '+':

        speed = constrain(
          speed + 15, 0, 255
        );

        if (movement != 'h' &&
            movement != 'g') {

          applyMovement();
        }
        break;

      // Decrease speed.
      case '-':

        speed = constrain(
          speed - 15, 0, 255
        );

        if (movement != 'h' &&
            movement != 'g') {

          applyMovement();
        }
        break;

      default:
        break;
    }
  }

  // Start automatically after power-up
  // if enabled and no command was received.
  updateAutoStart();

  // Sensor-based navigation.
  updateAutonomousMode();

  // Telemetry.
  printTelemetry();
}
