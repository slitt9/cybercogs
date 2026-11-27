/*
  Line follower + Arduino Cloud dashboard
  Cloud variables (created in the Thing, declared in thingProperties.h):
    bool   running;     // Switch widget: start / stop the robot
    String robotLog;    // Messenger widget: live status line
    bool   irLeft;      // LED widget
    bool   irCenter;    // LED widget
    bool   irRight;     // LED widget
*/
#include "thingProperties.h"

// ---------- Pins ----------
const int leftEnablePin = 10;
const int leftDirPin1 = A4;
const int leftDirPin2 = A3;

const int rightEnablePin = 9;
const int rightDirPin1 = A0;
const int rightDirPin2 = A1;

const int leftIrPin = A5;
const int centerIrPin = 4;
const int rightIrPin = A2;

// ---------- Settings ----------
const int LINE_SEEN = LOW;

const int straightSpeed = 60;
const int gentleInner = 5;
const int gentleOuter = 80;
const int strongInner = 5;
const int strongOuter = 100;

// Stage 1: both wheels backward.
const int reverseSpeed = 150;
const unsigned long reverseDurationMs = 900; // Set 600–1200.

// Stage 2: rotate with wheels in opposite directions.
const int rotateSpeed = 110;
const unsigned long rotateMinMs = 100;
const unsigned long rotateMaxMs = 1200;
const unsigned long centeredConfirmMs = 60;

// Line lost (000): back up along the path, then rotate toward the last side seen.
const int lostReverseSpeed = 70;
const unsigned long lostReverseMaxMs = 1000;
const int lostRotateSpeed = 90;
const unsigned long lostRotateMaxMs = 1500;

// ---------- Logging ----------
const unsigned long serialLogMs = 100;    // Serial Monitor (USB)
const unsigned long cloudSensorMs = 250;  // dashboard IR LEDs
const unsigned long cloudLogMs = 1000;    // dashboard Messenger log

// ---------- State ----------
enum Side { NONE, LEFT, RIGHT };
enum Mode { FOLLOW, BACK_UP, ROTATE, LINE_SEARCH, RECOVERY_FAILED };

Mode mode = FOLLOW;
Side lastSide = NONE;
Side thirdSensorSide = NONE;
Side turnSide = RIGHT;

bool havePrevious = false;
bool previousL = false;
bool previousC = false;
bool previousR = false;

unsigned long stageStart = 0;

bool timingCentered = false;
unsigned long centeredStart = 0;

bool recoveryArmed = true;
bool timingRearm = false;
unsigned long rearmStart = 0;

bool searchRotating = false;

// ---------- Motors ----------
void setMotor(int enablePin, int dirPin1, int dirPin2, int speed) {
  digitalWrite(dirPin1, speed < 0);
  digitalWrite(dirPin2, speed > 0);
  analogWrite(enablePin, constrain(abs(speed), 0, 255));
}

void drive(int left, int right) {
  setMotor(leftEnablePin, leftDirPin1, leftDirPin2, left);
  setMotor(rightEnablePin, rightDirPin1, rightDirPin2, right);
}

bool seesLine(int pin) {
  return digitalRead(pin) == LINE_SEEN;
}

// Fresh start every time the dashboard switch is turned on.
void resetLineState() {
  mode = FOLLOW;
  lastSide = NONE;
  thirdSensorSide = NONE;
  turnSide = RIGHT;
  havePrevious = false;
  timingCentered = false;
  recoveryArmed = true;
  timingRearm = false;
  searchRotating = false;
}

// Sends sensor states and a status line to Serial and the dashboard.
void logStatus(bool L, bool C, bool R, const char* action, int leftSpeed, int rightSpeed) {
  static unsigned long lastSerial = 0;
  static unsigned long lastSensors = 0;
  static unsigned long lastCloudLog = 0;
  unsigned long now = millis();

  if (now - lastSensors >= cloudSensorMs) {
    lastSensors = now;
    irLeft = L;
    irCenter = C;
    irRight = R;
  }

  if (now - lastSerial < serialLogMs && now - lastCloudLog < cloudLogMs) return;

  String line = "IR=" + String(L) + String(C) + String(R) +
                " | " + action +
                " | L=" + String(leftSpeed) + " R=" + String(rightSpeed);

  if (now - lastSerial >= serialLogMs) {
    lastSerial = now;
    Serial.println(line);
  }
  if (now - lastCloudLog >= cloudLogMs) {
    lastCloudLog = now;
    robotLog = line;
  }
}

// ---------- Setup ----------
void setup() {
  pinMode(leftEnablePin, OUTPUT);
  pinMode(leftDirPin1, OUTPUT);
  pinMode(leftDirPin2, OUTPUT);

  pinMode(rightEnablePin, OUTPUT);
  pinMode(rightDirPin1, OUTPUT);
  pinMode(rightDirPin2, OUTPUT);

  pinMode(leftIrPin, INPUT_PULLUP);
  pinMode(centerIrPin, INPUT_PULLUP);
  pinMode(rightIrPin, INPUT_PULLUP);

  drive(0, 0);

  Serial.begin(115200);
  delay(1500);

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  running = false;   // always start stopped; press the switch to go
}

// ---------- Loop ----------
void loop() {
  ArduinoCloud.update();

  bool L = seesLine(leftIrPin);
  bool C = seesLine(centerIrPin);
  bool R = seesLine(rightIrPin);

  // Stopped from the dashboard, or lost the cloud connection: stay still.
  if (!running || !ArduinoCloud.connected()) {
    drive(0, 0);
    logStatus(L, C, R, running ? "STOP - NO CLOUD CONNECTION" : "STOP - DASHBOARD", 0, 0);
    delay(5);
    return;
  }

  bool allThree = L && C && R;
  bool centered = !L && C && !R;
  unsigned long now = millis();

  // Record side-sensor arrivals during normal following.
  if (mode == FOLLOW) {
    if (havePrevious) {
      bool leftArrived = L && !previousL;
      bool rightArrived = R && !previousR;

      if (leftArrived && !rightArrived) {
        lastSide = LEFT;
      } else if (rightArrived && !leftArrived) {
        lastSide = RIGHT;
      }
    } else {
      if (L && !R) lastSide = LEFT;
      if (R && !L) lastSide = RIGHT;
    }

    if (allThree && recoveryArmed) {
      thirdSensorSide = NONE;

      // Identify exactly which side completed the 111 reading.
      if (havePrevious) {
        if (!previousL && previousC && previousR) {
          thirdSensorSide = LEFT;
        } else if (previousL && previousC && !previousR) {
          thirdSensorSide = RIGHT;
        }
      }

      Side referenceSide =
        thirdSensorSide != NONE ? thirdSensorSide : lastSide;

      // Opposite side. Unknown defaults to a right turn.
      turnSide = referenceSide == RIGHT ? LEFT : RIGHT;

      mode = BACK_UP;
      stageStart = now;
      recoveryArmed = false;
      timingRearm = false;
      timingCentered = false;
    }
  }

  // Line lost: start searching instead of stopping.
  if (mode == FOLLOW && !L && !C && !R) {
    mode = LINE_SEARCH;
    stageStart = now;
    searchRotating = false;
  }

  // Any sensor sees the line again: back to normal following.
  if (mode == LINE_SEARCH) {
    if (L || C || R) {
      mode = FOLLOW;
    } else if (!searchRotating && now - stageStart >= lostReverseMaxMs) {
      searchRotating = true;          // reversing didn't find it: rotate instead
      stageStart = now;
    } else if (searchRotating && now - stageStart >= lostRotateMaxMs) {
      mode = RECOVERY_FAILED;         // give up and stop
    }
  }

  previousL = L;
  previousC = C;
  previousR = R;
  havePrevious = true;

  // Complete the full straight-backward interval first.
  if (mode == BACK_UP && now - stageStart >= reverseDurationMs) {
    mode = ROTATE;
    stageStart = now;
    timingCentered = false;
  }

  // End rotation only after center-only detection is stable.
  if (mode == ROTATE) {
    unsigned long elapsed = now - stageStart;

    if (centered && elapsed >= rotateMinMs) {
      if (!timingCentered) {
        timingCentered = true;
        centeredStart = now;
      }
      if (now - centeredStart >= centeredConfirmMs) {
        mode = FOLLOW;
        timingCentered = false;
      }
    } else {
      timingCentered = false;
    }

    if (mode == ROTATE && elapsed >= rotateMaxMs) {
      mode = RECOVERY_FAILED;
    }
  }

  // Re-arm after 500 ms centered.
  if ((mode == FOLLOW || mode == RECOVERY_FAILED) && centered) {
    if (!timingRearm) {
      timingRearm = true;
      rearmStart = now;
    }
    if (now - rearmStart >= 500) {
      recoveryArmed = true;
      mode = FOLLOW;
    }
  } else {
    timingRearm = false;
  }

  int leftSpeed = 0;
  int rightSpeed = 0;
  const char* action = "STOP";

  if (mode == BACK_UP) {
    leftSpeed = -reverseSpeed;
    rightSpeed = -reverseSpeed;
    action = "REVERSE STRAIGHT";

  } else if (mode == ROTATE) {
    if (turnSide == LEFT) {
      leftSpeed = -rotateSpeed;
      rightSpeed = rotateSpeed;
      action = "ROTATE LEFT";
    } else {
      leftSpeed = rotateSpeed;
      rightSpeed = -rotateSpeed;
      action = "ROTATE RIGHT";
    }

  } else if (mode == LINE_SEARCH) {
    if (!searchRotating) {
      leftSpeed = -lostReverseSpeed;
      rightSpeed = -lostReverseSpeed;
      action = "LOST - REVERSING";
    } else if (lastSide == LEFT) {
      leftSpeed = -lostRotateSpeed;
      rightSpeed = lostRotateSpeed;
      action = "LOST - SEARCH LEFT";
    } else {
      leftSpeed = lostRotateSpeed;
      rightSpeed = -lostRotateSpeed;
      action = "LOST - SEARCH RIGHT";
    }

  } else if (mode == RECOVERY_FAILED) {
    action = "STOP - RECOVERY TIMEOUT";

  } else if (centered) {
    leftSpeed = straightSpeed;
    rightSpeed = straightSpeed;
    action = "STRAIGHT";

  } else if (L && !R) {
    leftSpeed = C ? gentleInner : strongInner;
    rightSpeed = C ? gentleOuter : strongOuter;
    action = C ? "GENTLE LEFT" : "STRONG LEFT";

  } else if (R && !L) {
    leftSpeed = C ? gentleOuter : strongOuter;
    rightSpeed = C ? gentleInner : strongInner;
    action = C ? "GENTLE RIGHT" : "STRONG RIGHT";

  } else if (!L && !C && !R) {
    action = "STOP - LINE LOST";

  } else {
    action = "STOP - AMBIGUOUS";
  }

  drive(leftSpeed, rightSpeed);
  logStatus(L, C, R, action, leftSpeed, rightSpeed);

  delay(5);
}

// ---------- Cloud callbacks ----------
// Called when the dashboard switch changes.
void onRunningChange() {
  if (running) {
    resetLineState();   // clears a stuck RECOVERY TIMEOUT, starts fresh
  } else {
    drive(0, 0);
  }
}

// The rest are generated by Arduino Cloud; the robot doesn't need them.
void onRobotLogChange() {}
void onIrLeftChange() {}
void onIrCenterChange() {}
void onIrRightChange() {}