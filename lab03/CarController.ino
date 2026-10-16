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
// GLOBAL VARIABLES
// ==========================================

int speed = 255;
char movement = ' ';

// LED blinking
const unsigned long blinkInterval = 400;

unsigned long previousBlinkTime = 0;
bool blinkState = true;
char previousMovement = '\0';

// Encoder tracking
unsigned long leftPulseCount = 0;
unsigned long rightPulseCount = 0;

unsigned long previousLeftPulseCount = 0;
unsigned long previousRightPulseCount = 0;

int previousLeftEncoderState = HIGH;
int previousRightEncoderState = HIGH;

// Telemetry interval
const unsigned long telemetryInterval = 500;
unsigned long previousTelemetryTime = 0;


// ==========================================
// MOTOR CONTROL
// ==========================================

void setMotor(int enablePin, int input1, int input2, int motorPWM) {

  motorPWM = constrain(motorPWM, -255, 255);

  analogWrite(enablePin, 0);

  digitalWrite(input1, motorPWM < 0 ? HIGH : LOW);
  digitalWrite(input2, motorPWM > 0 ? HIGH : LOW);

  analogWrite(enablePin, abs(motorPWM));
}


void drive(int leftMotorSpeed, int rightMotorSpeed) {

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
// MOVEMENT CONTROL
// ==========================================

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

    case ' ':
      stop();
      break;

    default:
      stop();
      break;
  }
}


// ==========================================
// LED CONTROL
// ==========================================

void updateLEDs() {

  unsigned long currentTime = millis();

  // Reset blinking when movement changes
  if (movement != previousMovement) {
    previousMovement = movement;
    previousBlinkTime = currentTime;
    blinkState = true;
  }

  // Toggle blinking state
  if (currentTime - previousBlinkTime >= blinkInterval) {
    previousBlinkTime = currentTime;
    blinkState = !blinkState;
  }

  // Turn off all LEDs initially
  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
  digitalWrite(yellowLED, LOW);

  switch (movement) {

    case 'w':
      digitalWrite(yellowLED, HIGH);
      break;

    case 's':
      digitalWrite(yellowLED, blinkState ? HIGH : LOW);
      break;

    case 'a':
      digitalWrite(leftLED, blinkState ? HIGH : LOW);
      break;

    case 'd':
      digitalWrite(rightLED, blinkState ? HIGH : LOW);
      break;

    case 'q':
      digitalWrite(leftLED, blinkState ? HIGH : LOW);
      digitalWrite(yellowLED, HIGH);
      break;

    case 'e':
      digitalWrite(rightLED, blinkState ? HIGH : LOW);
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

  // Read current encoder states
  int leftEncoderState = digitalRead(leftEncoderPin);
  int rightEncoderState = digitalRead(rightEncoderPin);

  // Count each HIGH-to-LOW transition
  if (previousLeftEncoderState == HIGH &&
      leftEncoderState == LOW) {

    leftPulseCount++;
  }

  if (previousRightEncoderState == HIGH &&
      rightEncoderState == LOW) {

    rightPulseCount++;
  }

  // Save states for the next reading
  previousLeftEncoderState = leftEncoderState;
  previousRightEncoderState = rightEncoderState;
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

  // Pulses detected since previous report
  unsigned long leftPulses =
    leftPulseCount - previousLeftPulseCount;

  unsigned long rightPulses =
    rightPulseCount - previousRightPulseCount;

  // Calculate pulses per second
  float leftPPS =
    (leftPulses * 1000.0) / elapsedTime;

  float rightPPS =
    (rightPulses * 1000.0) / elapsedTime;

  // Print CSV telemetry
  Serial.print(currentTime);
  Serial.print(",");

  Serial.print(movement == ' ' ? "stop" : String(movement));
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

  // Update previous measurements
  previousLeftPulseCount = leftPulseCount;
  previousRightPulseCount = rightPulseCount;

  previousTelemetryTime = currentTime;
}


// ==========================================
// SETUP
// ==========================================

void setup() {

  // Motor pins
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorInput1, OUTPUT);
  pinMode(leftMotorInput2, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorInput1, OUTPUT);
  pinMode(rightMotorInput2, OUTPUT);

  // LED pins
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);

  // Encoder pins
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);

  // Read initial encoder states
  previousLeftEncoderState =
    digitalRead(leftEncoderPin);

  previousRightEncoderState =
    digitalRead(rightEncoderPin);

  // Initially turn LEDs off
  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
  digitalWrite(yellowLED, LOW);

  // Initially stop motors
  stop();

  // Start serial communication
  Serial.begin(9600);

  previousTelemetryTime = millis();

  // CSV header
  Serial.println(
    "t_ms,movement,pwm,left_count,right_count,left_pps,right_pps"
  );
}


// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  // Read encoders continuously
  updateEncoders();

  // Process serial commands
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
          break;

        case '+':
          speed = constrain(speed + 15, 0, 255);
          break;

        case '-':
          speed = constrain(speed - 15, 0, 255);
          break;

        default:
          movement = ' ';
          break;
      }

      applyMovement();
    }
  }

  // Update LED indicators
  updateLEDs();

  // Print encoder telemetry every 500 ms
  printTelemetry();
}