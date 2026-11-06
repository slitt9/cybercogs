// ---------- Pins ----------
// Motors (shared with wall following)
const int leftEnablePin = 10, leftDirPin1 = A4, leftDirPin2 = A3;
const int rightEnablePin = 9, rightDirPin1 = A0, rightDirPin2 = A1;
// Line sensors (ultrasonic pins 5, 6, 7, 8 are left free)
const int leftIrPin = A5, centerIrPin = 4, rightIrPin = A2;

// ---------- Settings ----------
const int LINE_SEEN = LOW;      // most MH IR modules output LOW over white; flip to HIGH if yours is opposite
const int lineSpeed = 60;
const int lineInnerSpeed = 20;  // slow wheel when steering back toward the line
const int lineOuterSpeed = 70;  // fast wheel when steering back toward the line
const int searchSpeed = 60;     // pivot speed when the line is lost

// ---------- State ----------
enum Side { NONE, LEFT, RIGHT };
Side lastSeen = NONE;

// ================= Motors =================
void setMotor(int enablePin, int dirPin1, int dirPin2, int speed) {
  digitalWrite(dirPin1, speed < 0);
  digitalWrite(dirPin2, speed > 0);
  analogWrite(enablePin, constrain(abs(speed), 0, 255));
}

void drive(int left, int right) {
  setMotor(leftEnablePin, leftDirPin1, leftDirPin2, left);
  setMotor(rightEnablePin, rightDirPin1, rightDirPin2, right);
}

void driveStraight(int speed) {
  drive(speed, speed);
}

void steerLeft() {
  drive(lineInnerSpeed, lineOuterSpeed);
}

void steerRight() {
  drive(lineOuterSpeed, lineInnerSpeed);
}

// ================= Sensors =================
bool seesLine(int pin) {
  return digitalRead(pin) == LINE_SEEN;
}

// ================= Setup / loop =================
void setup() {
  pinMode(leftEnablePin, OUTPUT);  pinMode(leftDirPin1, OUTPUT);  pinMode(leftDirPin2, OUTPUT);
  pinMode(rightEnablePin, OUTPUT); pinMode(rightDirPin1, OUTPUT); pinMode(rightDirPin2, OUTPUT);
  pinMode(leftIrPin, INPUT_PULLUP);
  pinMode(centerIrPin, INPUT_PULLUP);
  pinMode(rightIrPin, INPUT_PULLUP);
  Serial.begin(115200);
  delay(5000);
}

void loop() {
  bool L = seesLine(leftIrPin);
  bool C = seesLine(centerIrPin);
  bool R = seesLine(rightIrPin);

const int straightSpeed = 65;
const int gentleInner   = 5;
const int gentleOuter   = 85;
const int strongInner   = 5;
const int strongOuter   = 110;

  int leftSpeed = 0;
  int rightSpeed = 0;
  const char* action = "STOP";

  if (C && !L && !R) {
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
    action = "STOP - AMBIGUOUS / JUNCTION";
  }

  drive(leftSpeed, rightSpeed);

  // Print 5 times per second without slowing sensor checks.
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint >= 200) {
    lastPrint = now;

    Serial.print("Time=");
    Serial.print(now);
    Serial.print("ms | L=");
    Serial.print(L);
    Serial.print(" C=");
    Serial.print(C);
    Serial.print(" R=");
    Serial.print(R);

    Serial.print(" | ");
    Serial.print(action);

    Serial.print(" | Motor PWM: left=");
    Serial.print(leftSpeed);
    Serial.print(" right=");
    Serial.println(rightSpeed);
  }

  delay(5);
}
