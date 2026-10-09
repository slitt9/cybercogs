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

const int leftLED = 2;     // Green LED - Left
const int rightLED = 3;    // Green LED - Right
const int yellowLED = 4;   // Yellow LED - Forward/Reverse


// ==========================================
// GLOBAL VARIABLES
// ==========================================

int speed = 255;
char movement = ' ';

// LED blink settings
const unsigned long blinkInterval = 400;

unsigned long previousBlinkTime = 0;
bool blinkState = true;
char previousMovement = '\0';


// ==========================================
// MOTOR CONTROL
// ==========================================

void setMotor(int enablePin, int input1, int input2, int motorPWM) {

  motorPWM = constrain(motorPWM, -255, 255);

  // Disable motor before changing direction
  analogWrite(enablePin, 0);

  // Set motor direction
  digitalWrite(input1, motorPWM < 0 ? HIGH : LOW);
  digitalWrite(input2, motorPWM > 0 ? HIGH : LOW);

  // Set motor speed using PWM
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

  // Toggle LED state every 400 milliseconds
  if (currentTime - previousBlinkTime >= blinkInterval) {
    previousBlinkTime = currentTime;
    blinkState = !blinkState;
  }

  // Turn all LEDs off first
  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
  digitalWrite(yellowLED, LOW);

  switch (movement) {

    case 'w':
      // Forward: yellow LED stays ON
      digitalWrite(yellowLED, HIGH);
      break;

    case 's':
      // Reverse: yellow LED blinks
      digitalWrite(yellowLED, blinkState ? HIGH : LOW);
      break;

    case 'a':
      // Turn left: left green LED blinks
      digitalWrite(leftLED, blinkState ? HIGH : LOW);
      break;

    case 'd':
      // Turn right: right green LED blinks
      digitalWrite(rightLED, blinkState ? HIGH : LOW);
      break;

    case 'q':
      // Spin left: left green blinks + yellow stays ON
      digitalWrite(leftLED, blinkState ? HIGH : LOW);
      digitalWrite(yellowLED, HIGH);
      break;

    case 'e':
      // Spin right: right green blinks + yellow stays ON
      digitalWrite(rightLED, blinkState ? HIGH : LOW);
      digitalWrite(yellowLED, HIGH);
      break;

    default:
      // Stop: all LEDs remain OFF
      break;
  }
}



// ==========================================
// SETUP
// ==========================================

void setup() {

  // Configure motor pins
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorInput1, OUTPUT);
  pinMode(leftMotorInput2, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorInput1, OUTPUT);
  pinMode(rightMotorInput2, OUTPUT);

  // Configure LED pins
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);

  // Start with all LEDs off
  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
  digitalWrite(yellowLED, LOW);

  // Stop motors initially
  stop();

  // Start Serial Monitor
  Serial.begin(9600);

  Serial.println("=== ROBOT CONTROLLER ===");
  Serial.println("w: Forward | s: Backward");
  Serial.println("a/d: Turn left/right");
  Serial.println("q/e: Spin left/right");
  Serial.println("Space: Stop");
  Serial.println("+/-: Increase/decrease speed");
}


// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  if (Serial.available() > 0) {

    char command = Serial.read();

    // Ignore line endings
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
          Serial.println("Unknown command: stopped.");
          break;
      }

      // Apply movement
      applyMovement();

      // Print status
      Serial.print("Movement: ");
      Serial.print(movement);
      Serial.print(" | Speed: ");
      Serial.println(speed);
    }
  }

  // Update LEDs continuously
  updateLEDs();
}