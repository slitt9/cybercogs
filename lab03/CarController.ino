#include <math.h>

// ==========================================
// MOTOR PIN CONFIGURATION
// ==========================================

// Left motor
const int leftMotorEnablePin = 9;
const int leftMotorInput1 = A4;
const int leftMotorInput2 = A3;

// Right motor
const int rightMotorEnablePin = 10;
const int rightMotorInput1 = A0;
const int rightMotorInput2 = A1;


// ==========================================
// LED PIN CONFIGURATION
// ==========================================

const int leftLED = 2;
const int rightLED = 3;
const int yellowLED = 4;


// ==========================================
// ENCODER PIN CONFIGURATION
// ==========================================

const int leftEncoderPin = 12;
const int rightEncoderPin = 11;


// ==========================================
// ULTRASONIC SENSOR PIN CONFIGURATION
// ==========================================

// IMPORTANT:
// Change -1 to the Arduino pin you connect.
//
// Front ultrasonic sensor
const int frontTrigPin = 5;
const int frontEchoPin = 6;

// No side ultrasonic sensor connected yet
const int sideTrigPin = 7;
const int sideEchoPin = 8;
//
// Both sensors:
// VCC -> Arduino 5V
// GND -> Arduino GND
//
// -1 means the pin is not configured.


// ==========================================
// GLOBAL VARIABLES
// ==========================================

int speed = 255;

// Manual commands:
// w, s, a, d, q, e, space
//
// Autonomous modes:
// h = Follow Me
// g = Wall Follow

char movement = ' ';

// Actual LED movement indicator
char indicatorMovement = ' ';

// LED blinking
const unsigned long blinkInterval = 400;

unsigned long previousBlinkTime = 0;
bool blinkState = true;
char previousMovement = '\0';


// ==========================================
// ENCODER VARIABLES
// ==========================================

unsigned long leftPulseCount = 0;
unsigned long rightPulseCount = 0;

unsigned long previousLeftPulseCount = 0;
unsigned long previousRightPulseCount = 0;

int previousLeftEncoderState = HIGH;
int previousRightEncoderState = HIGH;


// ==========================================
// TELEMETRY CONFIGURATION
// ==========================================

const unsigned long telemetryInterval = 500;

unsigned long previousTelemetryTime = 0;


// ==========================================
// PID CONFIGURATION
// ==========================================

// Desired distance from the object
const float followSetpointCM = 25.0;

// Desired distance from the side wall
const float wallSetpointCM = 25.0;

// Follow Me PID gains
float followKp = 8.0;
float followKi = 0.0;
float followKd = 0.0;

// Wall Follow PID gains
float wallKp = 3.0;
float wallKi = 0.0;
float wallKd = 0.0;

// General controller settings
const unsigned long controlInterval = 100;

// Distance tolerance
const float distanceToleranceCM = 2.0;

// Stop wall following if an obstacle is too close
const float frontSafetyDistanceCM = 15.0;

// Choose which side of the robot faces the wall
const bool wallOnLeft = true;

// Maximum Follow Me motor command
const int maxFollowPWM = 200;

// Minimum PWM to overcome motor friction.
// Tune experimentally for your robot.
const int minMovingPWM = 125;

// Wall-following speed settings
const int maxWallBasePWM = 170;
const int maxWallCorrectionPWM = 90;

unsigned long previousControlTime = 0;

// Store latest sensor measurements
float frontDistanceCM = NAN;
float sideDistanceCM = NAN;


// ==========================================
// PID CONTROLLER STRUCTURE
// ==========================================

struct PIDController {

  float kp;
  float ki;
  float kd;

  float integral;
  float previousError;

  bool initialized;
};

// Independent controller for each mode
PIDController followPID = {
  8.0, 0.0, 0.0,
  0.0, 0.0, false
};

PIDController wallPID = {
  3.0, 0.0, 0.0,
  0.0, 0.0, false
};


// ==========================================
// PID FUNCTIONS
// ==========================================

void resetPID(PIDController &pid) {

  pid.integral = 0;
  pid.previousError = 0;
  pid.initialized = false;
}


float calculatePID(
  PIDController &pid,
  float error,
  float dt,
  float maxOutput
) {

  if (dt <= 0) {
    return 0;
  }

  // Proportional component
  float P = pid.kp * error;

  // Integral component
  pid.integral += error * dt;

  // Prevent excessive integral accumulation
  if (pid.ki > 0) {

    float maxIntegral = maxOutput / pid.ki;

    pid.integral = constrain(
      pid.integral,
      -maxIntegral,
      maxIntegral
    );

  } else {

    pid.integral = 0;
  }

  float I = pid.ki * pid.integral;

  // Derivative component
  float derivative = 0;

  if (pid.initialized) {

    derivative =
      (error - pid.previousError) / dt;
  }

  float D = pid.kd * derivative;

  // Save current error
  pid.previousError = error;
  pid.initialized = true;

  // PID output
  float output = P + I + D;

  // Limit output
  return constrain(
    output,
    -maxOutput,
    maxOutput
  );
}


// ==========================================
// MOTOR CONTROL
// ==========================================

void setMotor(
  int enablePin,
  int input1,
  int input2,
  int motorPWM
) {

  motorPWM = constrain(motorPWM, -255, 255);

  // Disable motor before direction changes
  analogWrite(enablePin, 0);

  // Set motor direction
  digitalWrite(input1, motorPWM < 0 ? HIGH : LOW);
  digitalWrite(input2, motorPWM > 0 ? HIGH : LOW);

  // Apply PWM
  analogWrite(enablePin, abs(motorPWM));
}


void drive(int leftMotorSpeed, int rightMotorSpeed) {

  // Negative signs preserve existing motor calibration

  setMotor(
    leftMotorEnablePin,
    leftMotorInput1,
    leftMotorInput2,
    -leftMotorSpeed
  );

  setMotor(
    rightMotorEnablePin,
    rightMotorInput1,
    rightMotorInput2,
    -rightMotorSpeed
  );
}


// ==========================================
// ROBOT MOVEMENT FUNCTIONS
// ==========================================

void forward(int motorSpeed) {
  drive(motorSpeed, motorSpeed);
}

void backward(int motorSpeed) {
  drive(-motorSpeed, -motorSpeed);
}

void stop() {
  drive(0, 0);
}

void turnLeft(int motorSpeed) {
  drive(motorSpeed / 2, motorSpeed);
}

void turnRight(int motorSpeed) {
  drive(motorSpeed, motorSpeed / 2);
}

void spinLeft(int motorSpeed) {
  drive(-motorSpeed, motorSpeed);
}

void spinRight(int motorSpeed) {
  drive(motorSpeed, -motorSpeed);
}


// ==========================================
// ULTRASONIC SENSOR FUNCTIONS
// ==========================================

bool validUltrasonicPins(int trigPin, int echoPin) {

  return trigPin >= 0 &&
         echoPin >= 0 &&
         trigPin != echoPin;
}


float readDistanceCM(int trigPin, int echoPin) {

  // Invalid configuration
  if (!validUltrasonicPins(trigPin, echoPin)) {
    return NAN;
  }

  // Generate ultrasonic trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  // Measure echo duration
  // Timeout after 25 milliseconds
  unsigned long duration =
    pulseIn(echoPin, HIGH, 25000UL);

  // No valid echo
  if (duration == 0) {
    return NAN;
  }

  // Speed of sound ~0.0343 cm/us
  float distanceCM =
    duration * 0.0343 / 2.0;

  // Reject invalid measurements
  if (distanceCM < 2.0 || distanceCM > 350.0) {
    return NAN;
  }

  return distanceCM;
}


// ==========================================
// MOTOR STARTUP PWM
// ==========================================

// Compensates for the minimum PWM needed
// to overcome friction.

int applyMinimumPWM(int requestedPWM) {

  if (requestedPWM == 0) {
    return 0;
  }

  int direction = requestedPWM > 0 ? 1 : -1;

  int magnitude = abs(requestedPWM);

  magnitude = max(magnitude, minMovingPWM);

  return direction * constrain(magnitude, 0, 255);
}


// ==========================================
// FOLLOW ME CONTROLLER
// ==========================================

void updateFollowMe(float dt) {

  // Measure distance to object
  frontDistanceCM =
    readDistanceCM(frontTrigPin, frontEchoPin);

  // Stop if no valid measurement
  if (isnan(frontDistanceCM)) {

    stop();
    indicatorMovement = ' ';

    resetPID(followPID);
    return;
  }

  // Calculate distance error
  float error =
    frontDistanceCM - followSetpointCM;

  // Stop close to desired distance
  if (abs(error) <= distanceToleranceCM) {

    stop();
    indicatorMovement = ' ';

    resetPID(followPID);
    return;
  }

  // Calculate PID motor command
  float output = calculatePID(
    followPID,
    error,
    dt,
    maxFollowPWM
  );

  int motorPWM = (int)round(output);

  // Apply minimum usable PWM
  motorPWM = applyMinimumPWM(motorPWM);

  // Limit maximum forward/backward speed
  motorPWM = constrain(
    motorPWM,
    -maxFollowPWM,
    maxFollowPWM
  );

  // Drive both wheels equally
  drive(motorPWM, motorPWM);

  // Update LED indicators
  indicatorMovement =
    motorPWM > 0 ? 'w' : 's';
}


// ==========================================
// WALL FOLLOW CONTROLLER
// ==========================================

void updateWallFollow(float dt) {

  // Check for obstacles in front
  frontDistanceCM =
    readDistanceCM(frontTrigPin, frontEchoPin);

  // Measure distance to side wall
  sideDistanceCM =
    readDistanceCM(sideTrigPin, sideEchoPin);

  // Stop if either sensor reading is invalid
  if (isnan(frontDistanceCM) ||
      isnan(sideDistanceCM)) {

    stop();
    indicatorMovement = ' ';

    resetPID(wallPID);
    return;
  }

  // Stop at an obstacle or inside corner
  if (frontDistanceCM <= frontSafetyDistanceCM) {

    stop();
    indicatorMovement = ' ';

    resetPID(wallPID);
    return;
  }

  // Calculate wall distance error
  float error =
    sideDistanceCM - wallSetpointCM;

  // Calculate steering correction
  float correction = calculatePID(
    wallPID,
    error,
    dt,
    maxWallCorrectionPWM
  );

  // Avoid unnecessary tiny corrections
  if (abs(error) <= distanceToleranceCM) {
    correction = 0;
  }

  // Wall-following forward speed
  int basePWM = constrain(
    speed,
    0,
    maxWallBasePWM
  );

  // Reverse correction for a right-side wall
  if (!wallOnLeft) {
    correction = -correction;
  }

  // Differential steering
  int leftPWM = constrain(
    (int)round(basePWM - correction),
    0,
    255
  );

  int rightPWM = constrain(
    (int)round(basePWM + correction),
    0,
    255
  );

  // Move along the wall
  drive(leftPWM, rightPWM);

  // LED indicators
  if (basePWM == 0) {

    indicatorMovement = ' ';

  } else if (correction > 10) {

    indicatorMovement = 'a';

  } else if (correction < -10) {

    indicatorMovement = 'd';

  } else {

    indicatorMovement = 'w';
  }
}


// ==========================================
// AUTONOMOUS MODE CONTROLLER
// ==========================================

void updateAutonomousMode() {

  if (movement != 'h' && movement != 'g') {
    return;
  }

  unsigned long currentTime = millis();

  unsigned long elapsed =
    currentTime - previousControlTime;

  if (elapsed < controlInterval) {
    return;
  }

  float dt = elapsed / 1000.0;

  previousControlTime = currentTime;

  if (movement == 'h') {

    updateFollowMe(dt);

  } else if (movement == 'g') {

    updateWallFollow(dt);
  }
}


// ==========================================
// MOVEMENT CONTROL
// ==========================================

void applyMovement() {

  switch (movement) {

    case 'w':
      forward(speed);
      indicatorMovement = 'w';
      break;

    case 's':
      backward(speed);
      indicatorMovement = 's';
      break;

    case 'a':
      turnLeft(speed);
      indicatorMovement = 'a';
      break;

    case 'd':
      turnRight(speed);
      indicatorMovement = 'd';
      break;

    case 'q':
      spinLeft(speed);
      indicatorMovement = 'q';
      break;

    case 'e':
      spinRight(speed);
      indicatorMovement = 'e';
      break;

    case 'h':
    case 'g':
      // Autonomous control runs separately
      break;

    default:
      stop();
      indicatorMovement = ' ';
      break;
  }
}


// ==========================================
// LED CONTROL
// ==========================================

void updateLEDs() {

  unsigned long currentTime = millis();

  // Reset blinking when indicator changes
  if (indicatorMovement != previousMovement) {

    previousMovement = indicatorMovement;
    previousBlinkTime = currentTime;
    blinkState = true;
  }

  if (currentTime - previousBlinkTime >= blinkInterval) {

    previousBlinkTime = currentTime;
    blinkState = !blinkState;
  }

  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
  digitalWrite(yellowLED, LOW);

  switch (indicatorMovement) {

    case 'w':
      digitalWrite(yellowLED, HIGH);
      break;

    case 's':
      digitalWrite(
        yellowLED,
        blinkState ? HIGH : LOW
      );
      break;

    case 'a':
      digitalWrite(
        leftLED,
        blinkState ? HIGH : LOW
      );
      break;

    case 'd':
      digitalWrite(
        rightLED,
        blinkState ? HIGH : LOW
      );
      break;

    case 'q':
      digitalWrite(
        leftLED,
        blinkState ? HIGH : LOW
      );
      digitalWrite(yellowLED, HIGH);
      break;

    case 'e':
      digitalWrite(
        rightLED,
        blinkState ? HIGH : LOW
      );
      digitalWrite(yellowLED, HIGH);
      break;

    default:
      break;
  }
}


// ==========================================
// ENCODER TRACKING
// ==========================================

void updateEncoders() {

  int leftEncoderState =
    digitalRead(leftEncoderPin);

  int rightEncoderState =
    digitalRead(rightEncoderPin);

  // Count falling edges
  if (previousLeftEncoderState == HIGH &&
      leftEncoderState == LOW) {

    leftPulseCount++;
  }

  if (previousRightEncoderState == HIGH &&
      rightEncoderState == LOW) {

    rightPulseCount++;
  }

  previousLeftEncoderState =
    leftEncoderState;

  previousRightEncoderState =
    rightEncoderState;
}


// ==========================================
// TELEMETRY OUTPUT
// ==========================================

void printTelemetry() {

  unsigned long currentTime = millis();

  unsigned long elapsedTime =
    currentTime - previousTelemetryTime;

  if (elapsedTime < telemetryInterval) {
    return;
  }

  unsigned long leftPulses =
    leftPulseCount - previousLeftPulseCount;

  unsigned long rightPulses =
    rightPulseCount - previousRightPulseCount;

  float leftPPS =
    (leftPulses * 1000.0) / elapsedTime;

  float rightPPS =
    (rightPulses * 1000.0) / elapsedTime;

  // Preserve existing 7-column CSV format
  Serial.print(currentTime);
  Serial.print(",");

  if (movement == ' ') {
    Serial.print("stop");
  } else {
    Serial.print(movement);
  }

  Serial.print(",");
  Serial.print(speed);
  Serial.print(",");
  Serial.print(leftPulseCount);
  Serial.print(",");
  Serial.print(rightPulseCount);
  Serial.print(",");
  Serial.print(leftPPS, 2);
  Serial.print(",");
  Serial.println(rightPPS, 2);

  previousLeftPulseCount = leftPulseCount;
  previousRightPulseCount = rightPulseCount;

  previousTelemetryTime = currentTime;
}


// ==========================================
// SETUP
// ==========================================

void setup() {

  // Motors
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorInput1, OUTPUT);
  pinMode(leftMotorInput2, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorInput1, OUTPUT);
  pinMode(rightMotorInput2, OUTPUT);

  // LEDs
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);

  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
  digitalWrite(yellowLED, LOW);

  // Encoders
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);

  previousLeftEncoderState =
    digitalRead(leftEncoderPin);

  previousRightEncoderState =
    digitalRead(rightEncoderPin);

  // Ultrasonic sensors
  if (validUltrasonicPins(frontTrigPin, frontEchoPin)) {

    pinMode(frontTrigPin, OUTPUT);
    pinMode(frontEchoPin, INPUT);

    digitalWrite(frontTrigPin, LOW);
  }

  if (validUltrasonicPins(sideTrigPin, sideEchoPin)) {

    pinMode(sideTrigPin, OUTPUT);
    pinMode(sideEchoPin, INPUT);

    digitalWrite(sideTrigPin, LOW);
  }

  // Initially stop
  stop();

  // Serial telemetry
  Serial.begin(9600);

  previousTelemetryTime = millis();
  previousControlTime = millis();

  Serial.println(
    "t_ms,movement,pwm,left_count,right_count,left_pps,right_pps"
  );
}


// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  updateEncoders();

  // Process Serial commands
  if (Serial.available() > 0) {

    char command = Serial.read();

    if (command != '\n' && command != '\r') {

      switch (command) {

        case 'w':
        case 's':
        case 'a':
        case 'd':
        case 'q':
        case 'e':
        case ' ':

          movement = command;

          resetPID(followPID);
          resetPID(wallPID);

          applyMovement();
          break;

        case 'h':

          // Follow Me requires front sensor
          if (validUltrasonicPins(
                frontTrigPin,
                frontEchoPin)) {

            stop();
            indicatorMovement = ' ';

            movement = 'h';

            resetPID(followPID);
            resetPID(wallPID);

            previousControlTime =
              millis() - controlInterval;

          } else {

            movement = ' ';
            stop();
            indicatorMovement = ' ';
          }

          break;

        case 'g':

          // Wall Follow requires both sensors
          if (validUltrasonicPins(
                frontTrigPin,
                frontEchoPin) &&
              validUltrasonicPins(
                sideTrigPin,
                sideEchoPin)) {

            stop();
            indicatorMovement = ' ';

            movement = 'g';

            resetPID(followPID);
            resetPID(wallPID);

            previousControlTime =
              millis() - controlInterval;

          } else {

            movement = ' ';
            stop();
            indicatorMovement = ' ';
          }

          break;

        case '+':

          speed = constrain(
            speed + 15,
            0,
            255
          );

          if (movement != 'h' &&
              movement != 'g') {

            applyMovement();
          }

          break;

        case '-':

          speed = constrain(
            speed - 15,
            0,
            255
          );

          if (movement != 'h' &&
              movement != 'g') {

            applyMovement();
          }

          break;

        default:

          movement = ' ';
          stop();
          indicatorMovement = ' ';

          resetPID(followPID);
          resetPID(wallPID);

          break;
      }
    }
  }

  // Autonomous distance control
  updateAutonomousMode();

  // LED indicators
  updateLEDs();

  // Encoder telemetry
  printTelemetry();
}