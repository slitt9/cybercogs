// ---------- Pins ----------
const int leftEnablePin = 10, leftDirPin1 = A4, leftDirPin2 = A3;
const int rightEnablePin = 9, rightDirPin1 = A0, rightDirPin2 = A1;
const int frontTrigPin = 5, frontEchoPin = 6;
const int sideTrigPin = 7, sideEchoPin = 8;

// ---------- Settings ----------
const int baseSpeed = 65;
const float setPoint = 10;
const float frontTurnCm = 32;
const int turnInnerSpeed = 0;
const int turnOuterSpeed = 60;
const int slowSpeed = 65;
const float frontSlowCm = 50;

// ---------- PID ----------
const float Kp = 0.9; 
const float Ki = 0;
const float Kd = 1.7;
const float integralLimit = 20;

float integral = 0;
float prevError = 0;
unsigned long lastTime = 0;
bool pidFirstRun = true;

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

void makeLeftTurn() {
  drive(turnInnerSpeed, turnOuterSpeed);
}

void followWall(float pid, int speed) {
  drive(speed + pid, speed - pid);
}

// ================= Sensors =================
float readCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long us = pulseIn(echoPin, HIGH, 6000UL);
  float cm = us * 0.0343 / 2;
  if (us == 0 || cm < 2) return NAN;
  return cm;
}

// ================= PID =================
float computePid(float distance) {
  unsigned long now = millis();
  float error = distance - setPoint;

  if (pidFirstRun) {
    prevError = error;
    lastTime = now;
    pidFirstRun = false;
  }

  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float derivative = 0;
  if (dt > 0) {
    integral = constrain(integral + error * dt, -integralLimit, integralLimit);
    derivative = (error - prevError) / dt;
  }
  prevError = error;

  return Kp * error + Ki * integral + Kd * derivative;
}

bool turningLeft = false;
int clearCount = 0;

// ================= Setup / loop =================
void setup() {
  pinMode(leftEnablePin, OUTPUT);  pinMode(leftDirPin1, OUTPUT);  pinMode(leftDirPin2, OUTPUT);
  pinMode(rightEnablePin, OUTPUT); pinMode(rightDirPin1, OUTPUT); pinMode(rightDirPin2, OUTPUT);
  pinMode(frontTrigPin, OUTPUT);   pinMode(frontEchoPin, INPUT);
  pinMode(sideTrigPin, OUTPUT);    pinMode(sideEchoPin, INPUT);
  Serial.begin(115200);
  delay(5000);
}

void loop() {
  float front = readCm(frontTrigPin, frontEchoPin);
  delay(30);
  float side = readCm(sideTrigPin, sideEchoPin);
  float pid = 0;
  bool frontNear = !isnan(front) && front < frontSlowCm;
  int speed = frontNear ? slowSpeed : baseSpeed;

  if (!isnan(front) && front < frontTurnCm) {
    turningLeft = true;
    clearCount = 0;
  } else if (turningLeft && ++clearCount >= 2) {
    turningLeft = false;
  }

  if (turningLeft) {
    makeLeftTurn();
    pidFirstRun = true;
  } else if (isnan(side)) {
    driveStraight(speed);
  } else {
    pid = computePid(side);
    followWall(pid, speed);
  }

  Serial.print("f=");    Serial.print(front, 1);
  Serial.print(" s=");   Serial.print(side, 1);
  Serial.print(" pid="); Serial.println(pid, 1);

  delay(20);
}