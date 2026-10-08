const int leftMotorEnablePin = 9;
const int leftMotorInput1 = A4;
const int leftMotorInput2 = A3;

const int rightMotorEnablePin = 10;
const int rightMotorInput1 = A0;
const int rightMotorInput2 = A1;


int speed = 255;
char movement = ' ';

void setMotor(int leftMotorEnablePinble, int input1, int input2, int motorPWM) {
  motorPWM = constrain(motorPWM, -255, 255);
  analogWrite(leftMotorEnablePinble, 0);

  digitalWrite(input1, motorPWM < 0 ? HIGH : LOW);
  digitalWrite(input2, motorPWM > 0 ? HIGH : LOW);

  analogWrite(leftMotorEnablePinble, abs(motorPWM));
}

void drive(int leftMotorSpeed, int rightMotorSpeed) {
  setMotor(leftMotorEnablePin, leftMotorInput1, leftMotorInput2, -leftMotorSpeed);
  setMotor(rightMotorEnablePin, rightMotorInput1, rightMotorInput2, -rightMotorSpeed);
}

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

void applyMovement() {
  switch (movement) {
    case 'w': forward(speed); break;
    case 's': backward(speed); break;
    case 'a': turnLeft(speed); break;
    case 'd': turnRight(speed); break;
    case 'q': spinLeft(speed); break;
    case 'e': spinRight(speed); break;
    case ' ': stop(); break;
    default: stop(); break;
  }
}

void setup() {
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorInput1, OUTPUT);
  pinMode(leftMotorInput2, OUTPUT);
  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorInput1, OUTPUT);
  pinMode(rightMotorInput2, OUTPUT);

  stop();
  Serial.begin(9600);

  Serial.println("f: forward | b: backward | s: stop");
  Serial.println("l/r: curve left/right | q/e: spin left/right");
  Serial.println("a: left motor | d: right motor");
  Serial.println("+/-: increase/decrease speed");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == '\n' || command == '\r') {
      return;
    }

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

    applyMovement();

    Serial.print("Movement: ");
    Serial.print(movement);
    Serial.print(" | Speed: ");
    Serial.println(speed);
  }
}