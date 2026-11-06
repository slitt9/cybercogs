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

const int straightSpeed = 65;
const int gentleInner = 5;
const int gentleOuter = 75;
const int strongInner = 5;
const int strongOuter = 95;

// Stage 1: both wheels backward.
const int reverseSpeed = 75;
const unsigned long reverseDurationMs = 600; // Set 600–1200.

// Stage 2: rotate with wheels in opposite directions.
const int rotateSpeed = 90;
const unsigned long rotateMinMs = 100;
const unsigned long rotateMaxMs = 1400;
const unsigned long centeredConfirmMs = 60;

// ---------- State ----------
enum Side { NONE, LEFT, RIGHT };
enum Mode { FOLLOW, BACK_UP, ROTATE, RECOVERY_FAILED };

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

// ---------- Motors: your existing function ----------
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
  Serial.println("Starting in 5 seconds...");
  delay(5000);
}

// ---------- Loop ----------
void loop() {
  bool L = seesLine(leftIrPin);
  bool C = seesLine(centerIrPin);
  bool R = seesLine(rightIrPin);

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

  previousL = L;
  previousC = C;
  previousR = R;
  havePrevious = true;

  // Complete the full straight-backward interval first.
  if (mode == BACK_UP &&
      now - stageStart >= reverseDurationMs) {
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
  // Also allows restart after manually repositioning a failed robot.
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

  // ---------- Serial Monitor: 115200 baud ----------
  static unsigned long lastPrint = 0;

  if (now - lastPrint >= 100) {
    lastPrint = now;

    Serial.print("IR=");
    Serial.print(L);
    Serial.print(C);
    Serial.print(R);

    Serial.print(" | ");
    Serial.print(action);

    Serial.print(" | Third side=");
    Serial.print(
      thirdSensorSide == LEFT ? "LEFT" :
      thirdSensorSide == RIGHT ? "RIGHT" : "UNKNOWN"
    );

    Serial.print(" | left=");
    Serial.print(leftSpeed);
    Serial.print(" right=");
    Serial.println(rightSpeed);
  }

  delay(5);
}