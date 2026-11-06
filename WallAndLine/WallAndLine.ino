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

  if (L && C && R) {
    driveStraight(lineSpeed);        // wide line or junction
  } else if (L) {
    steerLeft();
    lastSeen = LEFT;
  } else if (R) {
    steerRight();
    lastSeen = RIGHT;
  } else if (C) {
    driveStraight(lineSpeed);        // centered
  } else if (lastSeen == LEFT) {
    drive(0, searchSpeed);           // lost: pivot left to find it
  } else if (lastSeen == RIGHT) {
    drive(searchSpeed, 0);           // lost: pivot right to find it
  } else {
    driveStraight(lineSpeed);        // never seen yet: drive until we find it
  }

  Serial.print("IR=");
  Serial.print(L); Serial.print(C); Serial.println(R);

  delay(10);
}
