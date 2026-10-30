#include <math.h>

 // Arduino UNO R4 WiFi + L298N. Front is the motor-wheel end.
const int leftMotorEnablePin = 10, leftMotorDirectionPin1 = A4, leftMotorDirectionPin2 = A3;
const int rightMotorEnablePin = 9, rightMotorDirectionPin1 = A0, rightMotorDirectionPin2 = A1;
const int frontSensorTriggerPin = 5, frontSensorEchoPin = 6;
const int rightSensorTriggerPin = 7, rightSensorEchoPin = 8;

const int motorForwardPolarity = 1;  // Change to -1 if both motors run backward.
const int baseForwardSpeedPwm = 85;
const int turningSpeedPwm = 90;
const int maximumSteeringCorrectionPwm = 20;
const float targetRightWallDistanceCm = 10.0;
 // Initial tuning values; tune on the physical robot.
const float wallDistanceProportionalGain = 3.0;
const float wallDistanceIntegralGain = 0.4;
const float wallDistanceDerivativeGain = 0.8;
const float derivativeFilterTimeConstantSeconds = 0.15;

float accumulatedWallDistanceErrorCmSeconds = 0.0;
float previousWallDistanceErrorCm = 0.0;
float filteredWallDistanceErrorRateCmPerSecond = 0.0;
unsigned long previousWallPidUpdateTimeMs = 0;
bool wallPidHasPreviousMeasurement = false;
const float frontObstacleStopDistanceCm = 20.0;
const float frontClearanceToResumeCm = 23.0;  // Hysteresis prevents repeated turns.
const unsigned long maximumTurnDurationMs = 5000;

bool isTurningAwayFromObstacle = false;
bool isStoppedAfterFault = false;
int consecutiveClearFrontReadings = 0;
unsigned long turnStartTimeMs = 0;

void setMotorDirectionAndSpeed(int motorEnablePin, int motorDirectionPin1, int motorDirectionPin2, int motorSpeedPwm) {
  motorSpeedPwm *= motorForwardPolarity;
  analogWrite(motorEnablePin, 0);
  digitalWrite(motorDirectionPin1, motorSpeedPwm < 0 ? HIGH : LOW);
  digitalWrite(motorDirectionPin2, motorSpeedPwm > 0 ? HIGH : LOW);
  analogWrite(motorEnablePin, abs(motorSpeedPwm));
}

void setBothMotorSpeeds(int leftMotorSpeedPwm, int rightMotorSpeedPwm) {
  setMotorDirectionAndSpeed(leftMotorEnablePin, leftMotorDirectionPin1, leftMotorDirectionPin2, leftMotorSpeedPwm);
  setMotorDirectionAndSpeed(rightMotorEnablePin, rightMotorDirectionPin1, rightMotorDirectionPin2, rightMotorSpeedPwm);
}

float measureUltrasonicDistanceCm(int sensorTriggerPin, int sensorEchoPin) {
  digitalWrite(sensorTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorTriggerPin, LOW);
  unsigned long echoDurationMicroseconds = pulseIn(sensorEchoPin, HIGH, 25000UL);
  float measuredDistanceCm = echoDurationMicroseconds * 0.0343f / 2.0f;
  return echoDurationMicroseconds == 0 || measuredDistanceCm < 2.0f || measuredDistanceCm > 350.0f ? NAN : measuredDistanceCm;
}

 // Call whenever wall following is interrupted.
void resetWallFollowingPid() {
  accumulatedWallDistanceErrorCmSeconds = 0.0;
  previousWallDistanceErrorCm = 0.0;
  filteredWallDistanceErrorRateCmPerSecond = 0.0;
  previousWallPidUpdateTimeMs = 0;
  wallPidHasPreviousMeasurement = false;
}

 // Positive correction steers right; negative correction steers left.
 // Call only with a valid right-wall distance, once per control cycle.
int calculateWallFollowingPidCorrection(float measuredRightWallDistanceCm) {
  unsigned long currentUpdateTimeMs = millis();
  float wallDistanceErrorCm = measuredRightWallDistanceCm - targetRightWallDistanceCm;
  float proportionalCorrectionPwm =
      wallDistanceProportionalGain * wallDistanceErrorCm;

  if (wallPidHasPreviousMeasurement) {
    float elapsedTimeSeconds =
        (currentUpdateTimeMs - previousWallPidUpdateTimeMs) / 1000.0f;

    if (elapsedTimeSeconds > 0.0f) {
      float measuredErrorRateCmPerSecond =
          (wallDistanceErrorCm - previousWallDistanceErrorCm) /
          elapsedTimeSeconds;
      float derivativeFilterWeight = elapsedTimeSeconds /
          (derivativeFilterTimeConstantSeconds + elapsedTimeSeconds);
      filteredWallDistanceErrorRateCmPerSecond += derivativeFilterWeight *
          (measuredErrorRateCmPerSecond -
           filteredWallDistanceErrorRateCmPerSecond);

      float proposedAccumulatedErrorCmSeconds =
          accumulatedWallDistanceErrorCmSeconds +
          wallDistanceErrorCm * elapsedTimeSeconds;
      float proposedCorrectionPwm = proportionalCorrectionPwm +
          wallDistanceIntegralGain * proposedAccumulatedErrorCmSeconds +
          wallDistanceDerivativeGain *
              filteredWallDistanceErrorRateCmPerSecond;

       // Anti-windup: integrate within limits, or to unwind saturation.
      if ((proposedCorrectionPwm >= -maximumSteeringCorrectionPwm &&
           proposedCorrectionPwm <= maximumSteeringCorrectionPwm) ||
          (proposedCorrectionPwm > maximumSteeringCorrectionPwm &&
           wallDistanceErrorCm < 0.0f) ||
          (proposedCorrectionPwm < -maximumSteeringCorrectionPwm &&
           wallDistanceErrorCm > 0.0f)) {
        accumulatedWallDistanceErrorCmSeconds =
            proposedAccumulatedErrorCmSeconds;
      }
    }
  }

  previousWallDistanceErrorCm = wallDistanceErrorCm;
  previousWallPidUpdateTimeMs = currentUpdateTimeMs;
  wallPidHasPreviousMeasurement = true;

  // P: current distance error. I: accumulated error. D: rate of change.
  float integralCorrectionPwm =
      wallDistanceIntegralGain * accumulatedWallDistanceErrorCmSeconds;
  float derivativeCorrectionPwm =
      wallDistanceDerivativeGain * filteredWallDistanceErrorRateCmPerSecond;
  float steeringCorrectionPwm = proportionalCorrectionPwm +
      integralCorrectionPwm + derivativeCorrectionPwm;

  return (int)round(constrain(steeringCorrectionPwm,
                             -(float)maximumSteeringCorrectionPwm,
                             (float)maximumSteeringCorrectionPwm));
}

void driveForwardUsingWallFollowingPid(float measuredRightWallDistanceCm) {
  int steeringCorrectionPwm =
      calculateWallFollowingPidCorrection(measuredRightWallDistanceCm);
  int leftMotorSpeedPwm = constrain(
      baseForwardSpeedPwm + steeringCorrectionPwm, -255, 255);
  int rightMotorSpeedPwm = constrain(
      baseForwardSpeedPwm - steeringCorrectionPwm, -255, 255);
  setBothMotorSpeeds(leftMotorSpeedPwm, rightMotorSpeedPwm);
}

void setup() {
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorDirectionPin1, OUTPUT);
  pinMode(leftMotorDirectionPin2, OUTPUT);
  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorDirectionPin1, OUTPUT);
  pinMode(rightMotorDirectionPin2, OUTPUT);
  pinMode(frontSensorTriggerPin, OUTPUT);
  pinMode(frontSensorEchoPin, INPUT);
  pinMode(rightSensorTriggerPin, OUTPUT);
  pinMode(rightSensorEchoPin, INPUT);
  digitalWrite(frontSensorTriggerPin, LOW);
  digitalWrite(rightSensorTriggerPin, LOW);
  setBothMotorSpeeds(0, 0);
  delay(5000);  // Automatic start five seconds after power-up/reset.
}

void loop() {
  if (isStoppedAfterFault) return;  // Reset the board to restart after a fault.

  float frontDistanceCm = measureUltrasonicDistanceCm(frontSensorTriggerPin, frontSensorEchoPin);
   // Stop as soon as the front reading detects a blockage or fails.
  if (isnan(frontDistanceCm) || (!isTurningAwayFromObstacle && frontDistanceCm <= frontObstacleStopDistanceCm)) {
    setBothMotorSpeeds(0, 0);
  }
  delay(35);  // Separate ultrasonic pings to reduce interference.
  float rightWallDistanceCm = measureUltrasonicDistanceCm(rightSensorTriggerPin, rightSensorEchoPin);

   // Unknown distance is not evidence of a clear path.
  if (isnan(frontDistanceCm) || isnan(rightWallDistanceCm)) {
    setBothMotorSpeeds(0, 0);
    resetWallFollowingPid();
    isStoppedAfterFault = true;
    return;
  }

  if (!isTurningAwayFromObstacle && frontDistanceCm <= frontObstacleStopDistanceCm) {
    setBothMotorSpeeds(0, 0);
    resetWallFollowingPid();
    isTurningAwayFromObstacle = true;
    consecutiveClearFrontReadings = 0;
    turnStartTimeMs = millis();
    delay(65);
    return;  // Begin rotating on the next sensor cycle.
  }

  if (isTurningAwayFromObstacle) {
    if (millis() - turnStartTimeMs >= maximumTurnDurationMs) {
      setBothMotorSpeeds(0, 0);
      resetWallFollowingPid();
      isStoppedAfterFault = true;
      return;
    }

    if (frontDistanceCm >= frontClearanceToResumeCm) {
      setBothMotorSpeeds(0, 0);  // Hold this angle while confirming clearance.
      if (++consecutiveClearFrontReadings >= 2) isTurningAwayFromObstacle = false;
    } else {
      consecutiveClearFrontReadings = 0;
      setBothMotorSpeeds(-turningSpeedPwm, turningSpeedPwm);  // Turn left, away from right wall.
    }
  } else {
    driveForwardUsingWallFollowingPid(rightWallDistanceCm);
  }

  delay(65);  // About 100 ms per cycle, plus sensor measurement time.
}
