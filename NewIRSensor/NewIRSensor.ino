#include <math.h>

// ================= Pins =================
const int leftEnablePin = 10;
const int leftDirPin1 = A4;
const int leftDirPin2 = A3;

const int rightEnablePin = 9;
const int rightDirPin1 = A0;
const int rightDirPin2 = A1;

const int leftIrPin = A5;
const int centerIrPin = 4;
const int rightIrPin = A2;

const int frontTrigPin = 5;
const int frontEchoPin = 6;
const int rightTrigPin = 7;
const int rightEchoPin = 8;

// ================= Settings =================
const int LINE_SEEN = LOW;

// Both wheels must propel the robot forward for positive commands.
// Change one value to -1 if that wheel runs backward.
const int leftPolarity = 1;
const int rightPolarity = 1;

// Line-following starting values.
const int lineSpeed = 60;
const int gentleInner = 40;
const int gentleOuter = 75;
const int strongInner = 0;
const int strongOuter = 85;

// Brief curved search through a gap; no reversing.
const unsigned long gapSearchMs = 700;

// Wall following.
const float wallEnterCm = 20.0;
const float targetWallCm = 10.0;
const float frontTurnCm = 20.0;
const float frontClearCm = 23.0;

const int wallSpeed = 70;
const int wallTurnSpeed = 75;
const int maxWallCorrection = 20;

const float Kp = 3.0;
const float Ki = 0.4;
const float Kd = 0.8;

// ================= State =================
enum CourseMode { LINE_MODE, WALL_MODE };
CourseMode courseMode = LINE_MODE;

int lastTapeSide = 0; // -1 left, +1 right, 0 unknown
bool inGap = false;
unsigned long gapStart = 0;

float frontCm = NAN;
float rightCm = NAN;

unsigned long lastPing = 0;
unsigned long frontTime = 0;
unsigned long rightTime = 0;

bool pingFrontNext = true;
bool newFront = false;
bool newRight = false;

int frontNearCount = 0;
int rightNearCount = 0;
bool wallStageCompleted = false;

unsigned long wallStart = 0;
bool exitMarkerArmed = false;
bool timingExit = false;
unsigned long exitStart = 0;

bool turningFromWall = false;
int clearFrontCount = 0;

// PID state.
float integral = 0;
float previousError = 0;
float filteredRate = 0;
bool pidReady = false;
unsigned long pidTime = 0;
int wallCorrection = 0;

// ================= Motors =================
void setMotor(int enablePin, int pin1, int pin2, int speed) {
  speed = constrain(speed, -255, 255);

  // Disable while changing direction.
  analogWrite(enablePin, 0);

  digitalWrite(pin1, speed < 0 ? HIGH : LOW);
  digitalWrite(pin2, speed > 0 ? HIGH : LOW);

  analogWrite(enablePin, abs(speed));
}

void drive(int left, int right) {
  setMotor(leftEnablePin, leftDirPin1, leftDirPin2,
           left * leftPolarity);

  setMotor(rightEnablePin, rightDirPin1, rightDirPin2,
           right * rightPolarity);
}

// ================= Ultrasonic =================
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 15000UL);

  if (duration == 0) return NAN;

  float cm = duration * 0.0343f / 2.0f;
  if (cm < 2.0f || cm > 250.0f) return NAN;

  return cm;
}

void updateDistances() {
  newFront = false;
  newRight = false;

  // Alternate sensors to separate their pings.
  if (millis() - lastPing < 35) return;

  if (pingFrontNext) {
    frontCm = readDistance(frontTrigPin, frontEchoPin);
    frontTime = millis();
    newFront = true;
  } else {
    rightCm = readDistance(rightTrigPin, rightEchoPin);
    rightTime = millis();
    newRight = true;
  }

  pingFrontNext = !pingFrontNext;
  lastPing = millis();
}

// ================= Wall PID =================
void resetPid() {
  integral = 0;
  previousError = 0;
  filteredRate = 0;
  pidReady = false;
  wallCorrection = 0;
}

void updatePid(float distance) {
  unsigned long now = millis();
  float error = distance - targetWallCm;

  if (pidReady) {
    float dt = (now - pidTime) / 1000.0f;

    if (dt > 0 && dt < 0.5f) {
      float rate = (error - previousError) / dt;
      float weight = dt / (0.15f + dt);
      filteredRate += weight * (rate - filteredRate);

      float proposedIntegral = integral + error * dt;
      float proposedOutput =
        Kp * error + Ki * proposedIntegral + Kd * filteredRate;

      // Prevent integral buildup while output is saturated.
      if (fabs(proposedOutput) <= maxWallCorrection ||
          (proposedOutput > maxWallCorrection && error < 0) ||
          (proposedOutput < -maxWallCorrection && error > 0)) {
        integral = proposedIntegral;
      }
    } else {
      integral = 0;
      filteredRate = 0;
    }
  }

  float output = Kp * error + Ki * integral + Kd * filteredRate;

  wallCorrection = (int)round(constrain(
    output, -(float)maxWallCorrection, (float)maxWallCorrection
  ));

  previousError = error;
  pidTime = now;
  pidReady = true;
}

// ================= Setup =================
void setup() {
  digitalWrite(leftEnablePin, LOW);
  digitalWrite(rightEnablePin, LOW);
  pinMode(leftEnablePin, OUTPUT);
  pinMode(rightEnablePin, OUTPUT);

  pinMode(leftDirPin1, OUTPUT);
  pinMode(leftDirPin2, OUTPUT);
  pinMode(rightDirPin1, OUTPUT);
  pinMode(rightDirPin2, OUTPUT);

  drive(0, 0);

  pinMode(leftIrPin, INPUT_PULLUP);
  pinMode(centerIrPin, INPUT_PULLUP);
  pinMode(rightIrPin, INPUT_PULLUP);

  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);

  digitalWrite(frontTrigPin, LOW);
  digitalWrite(rightTrigPin, LOW);

  Serial.begin(115200);
  Serial.println("Starting in 5 seconds...");
  delay(5000);
}

// ================= Main loop =================
void loop() {
  bool L = digitalRead(leftIrPin) == LINE_SEEN;
  bool C = digitalRead(centerIrPin) == LINE_SEEN;
  bool R = digitalRead(rightIrPin) == LINE_SEEN;

  bool allTape = L && C && R;
  bool centered = !L && C && !R;

  updateDistances();
  unsigned long now = millis();

  // -------- Enter wall stage --------
  if (courseMode == LINE_MODE && !wallStageCompleted) {
    if (newFront) {
      frontNearCount =
        (!isnan(frontCm) && frontCm <= wallEnterCm)
        ? frontNearCount + 1 : 0;
    }

    if (newRight) {
      rightNearCount =
        (!isnan(rightCm) && rightCm <= wallEnterCm)
        ? rightNearCount + 1 : 0;
    }

    if (frontNearCount >= 2 || rightNearCount >= 2) {
      courseMode = WALL_MODE;
      wallStart = now;
      exitMarkerArmed = false;
      timingExit = false;
      turningFromWall = false;
      clearFrontCount = 0;
      resetPid();
    }
  }

  // -------- Maze exit marker --------
  if (courseMode == WALL_MODE) {
    // Ignore entry tape. Arm only after leaving it.
    if (!allTape && now - wallStart >= 1000) {
      exitMarkerArmed = true;
    }

    if (exitMarkerArmed && allTape) {
      if (!timingExit) {
        timingExit = true;
        exitStart = now;
      }

      if (now - exitStart >= 120) {
        courseMode = LINE_MODE;
        wallStageCompleted = true;
        turningFromWall = false;
        inGap = false;
        lastTapeSide = 0;
        resetPid();
      }
    } else {
      timingExit = false;
    }
  }

  int leftCommand = 0;
  int rightCommand = 0;
  const char* action = "STOP";

  // ================= Wall following =================
  if (courseMode == WALL_MODE) {
    bool frontValid =
      !isnan(frontCm) && now - frontTime < 250;

    bool rightValid =
      !isnan(rightCm) && now - rightTime < 250;

    if (!frontValid) {
      // Retry automatically; do not latch a permanent fault.
      clearFrontCount = 0;
      resetPid();
      action = "WALL - WAIT FRONT ECHO";

    } else {
      if (!turningFromWall && frontCm <= frontTurnCm) {
        turningFromWall = true;
        clearFrontCount = 0;
        resetPid();
      }

      if (turningFromWall) {
        if (newFront) {
          clearFrontCount = frontCm >= frontClearCm
            ? clearFrontCount + 1 : 0;
        }

        if (clearFrontCount >= 2) {
          turningFromWall = false;
          resetPid();
        }
      }

      if (turningFromWall) {
        leftCommand = -wallTurnSpeed;
        rightCommand = wallTurnSpeed;
        action = "WALL - TURN LEFT";

      } else if (!rightValid) {
        resetPid();
        action = "WALL - WAIT RIGHT ECHO";

      } else {
        if (newRight) updatePid(rightCm);

        leftCommand = wallSpeed + wallCorrection;
        rightCommand = wallSpeed - wallCorrection;
        action = "WALL - FOLLOW";
      }
    }

  // ================= Tape following =================
  } else {
    bool usable = centered || (L != R);

    if (usable) {
      inGap = false;

      if (centered) {
        leftCommand = lineSpeed;
        rightCommand = lineSpeed;
        action = "LINE - STRAIGHT";

      } else if (L && !R) {
        lastTapeSide = -1;
        leftCommand = C ? gentleInner : strongInner;
        rightCommand = C ? gentleOuter : strongOuter;
        action = C ? "LINE - SMALL LEFT" : "LINE - TIGHT LEFT";

      } else {
        lastTapeSide = 1;
        leftCommand = C ? gentleOuter : strongOuter;
        rightCommand = C ? gentleInner : strongInner;
        action = C ? "LINE - SMALL RIGHT" : "LINE - TIGHT RIGHT";
      }

    } else {
      if (!inGap) {
        inGap = true;
        gapStart = now;
      }

      if (now - gapStart < gapSearchMs) {
        if (L && R) {
          leftCommand = 45;
          rightCommand = 45;
          action = "LINE - WIDE TAPE";

        } else if (lastTapeSide == -1) {
          leftCommand = 0;
          rightCommand = 65;
          action = "LINE - SEARCH LEFT";

        } else if (lastTapeSide == 1) {
          leftCommand = 65;
          rightCommand = 0;
          action = "LINE - SEARCH RIGHT";

        } else {
          action = "LINE - NO DIRECTION";
        }
      } else {
        action = "LINE - LOST";
      }
    }
  }

  drive(leftCommand, rightCommand);

  // ================= Diagnostics =================
  static unsigned long lastLog = 0;

  if (now - lastLog >= 100) {
    lastLog = now;

    Serial.print("IR=");
    Serial.print(L);
    Serial.print(C);
    Serial.print(R);

    Serial.print(" | ");
    Serial.print(action);

    Serial.print(" | Front=");
    Serial.print(frontCm);
    Serial.print(" Right=");
    Serial.print(rightCm);

    Serial.print(" | Motor L=");
    Serial.print(leftCommand);
    Serial.print(" R=");
    Serial.println(rightCommand);
  }

  delay(5);
}