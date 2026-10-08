# Arduino Robot Controller

A simple three-wheeled robot controlled using an Arduino and a dual motor driver.

The robot uses **differential drive**, meaning it controls the speed and direction of its two powered wheels independently to move forward, backward, turn, and spin in place.

## Hardware

- Arduino Uno
- Dual motor driver (e.g., L298N)
- Two DC motors
- Two powered wheels
- One caster wheel
- USB cable
- External motor power supply (recommended)

### Pin Configuration

| Component | Arduino Pin | Motor Driver Pin |
|-----------|-------------|------------------|
| Left Motor Enable | D9 (PWM) | ENA |
| Left Motor Input 1 | A4 | IN1 |
| Left Motor Input 2 | A3 | IN2 |
| Right Motor Enable | D10 (PWM) | ENB |
| Right Motor Input 1 | A0 | IN3 |
| Right Motor Input 2 | A1 | IN4 |
| Ground | GND | GND |

This configuration assumes Motor A controls the left wheel and Motor B controls the right wheel.

## Controls

The robot is controlled through the Arduino IDE's Serial Monitor.

| Command | Action | Description |
|---------|--------|-------------|
| `w` | Forward | Both wheels move forward |
| `s` | Backward | Both wheels move backward |
| `a` | Turn Left | Left wheel moves slower than right |
| `d` | Turn Right | Right wheel moves slower than left |
| `q` | Spin Left | Left wheel backward, right wheel forward |
| `e` | Spin Right | Left wheel forward, right wheel backward |
| `Space` | Stop | Stops both motors |
| `+` | Increase Speed | Increases PWM by 15 |
| `-` | Decrease Speed | Decreases PWM by 15 |

**Note:** Commands must be entered and sent through the Serial Monitor. These are not live keyboard controls.

## Speed Control

Motor speed is controlled using **Pulse Width Modulation (PWM)**.

- Minimum PWM: `0`
- Maximum PWM: `255`
- Default PWM: `255`
- Speed adjustment: `15` per command

The PWM value determines the duty cycle of the signal sent to the motor driver.

The actual motor speed depends on factors such as voltage, friction, weight, and motor load.

### Turning

The robot turns by adjusting the relative speeds of its wheels.

**Turn Left:**
```cpp
drive(motorSpeed / 2, motorSpeed);
```

**Turn Right:**
```cpp
drive(motorSpeed, motorSpeed / 2);
```

### Spinning

The robot spins in place by rotating its wheels in opposite directions.

**Spin Left:**
```cpp
drive(-motorSpeed, motorSpeed);
```

**Spin Right:**
```cpp
drive(motorSpeed, -motorSpeed);
```

## Getting Started

1. Connect the Arduino to your computer via USB.
2. Open the project's `.ino` file in Arduino IDE.
3. Select the correct Arduino board and port.
4. Compile and upload the code.
5. Open the Serial Monitor.
6. Set the baud rate to `9600`.
7. Enter a command and press Enter (or Send).

To stop the robot, send a single space character.

## How It Works

### Motor Control

The `setMotor()` function controls an individual motor using three pins:

- **Enable pin:** Controls motor drive using PWM.
- **Input 1:** Controls motor direction.
- **Input 2:** Controls motor direction.

Positive and negative PWM values determine the motor's rotation direction.

The absolute value determines the PWM duty cycle.

### Differential Drive

The `drive()` function controls both motors independently.

```cpp
void drive(int leftMotorSpeed, int rightMotorSpeed) {
  setMotor(leftMotorEnablePin, leftMotorInput1, leftMotorInput2, -leftMotorSpeed);
  setMotor(rightMotorEnablePin, rightMotorInput1, rightMotorInput2, -rightMotorSpeed);
}
```

The negative signs compensate for the current motor wiring and orientation. If the robot moves in an unexpected direction, these signs may need adjustment.

## Troubleshooting

### Wheels do not move at low speeds

The motors may not receive enough power to overcome friction.

Try increasing the PWM setting or using an appropriately rated external motor power supply.

### Only one wheel rotates while turning

The slower wheel may not receive enough torque to start rotating.

Consider increasing its PWM by adjusting the turning ratio.

For example:

```cpp
void turnLeft(int motorSpeed) {
  drive(motorSpeed * 4 / 5, motorSpeed);
}
```

### Robot moves in the wrong direction

Check the motor wiring and direction signs in `drive()`.

### Motors work in the air but struggle on the ground

This can happen because of insufficient starting torque, limited motor power, or mechanical resistance.

Inspect the wheels and motor connections and consider using a separate motor power supply.

## Power Supply

The initial prototype uses the Arduino's 5V supply through USB.

However, this may not provide sufficient current for reliable motor operation.

An external motor power supply is recommended.

When using an external supply:

- Connect the supply to the motor driver's appropriate motor-power terminals.
- Connect the Arduino and motor driver's grounds together.
- Verify the motor driver's voltage and logic-power requirements.
- Do not connect an external battery's positive terminal directly to the Arduino's 5V pin.

## Project Structure

```text
RobotController/
├── RobotController.ino
├── README.md
└── circuit-diagram.png
```

## Future Improvements

- External battery-powered motor supply
- Improved turning calibration
- Minimum motor startup PWM
- Automatic stop on communication timeout
- Real-time keyboard controls
- Obstacle detection using sensors