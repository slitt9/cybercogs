# Arduino Robot Controller

A three-wheeled robot powered by an Arduino Uno and controlled through the Arduino Serial Monitor.

The robot uses a **differential drive system**, allowing it to move forward, backward, turn, and spin by independently controlling two DC motors.

It also features three LED indicators that visually communicate the robot's movement and direction.

## Features

- Forward and backward movement
- Gradual left and right turns
- 360° spinning in either direction
- Adjustable motor speed using PWM
- Serial Monitor controls (WASD + Q/E)
- LED direction indicators
- Non-blocking LED blinking using `millis()`

## Hardware Requirements

- Arduino Uno
- Dual motor driver (e.g., L298N)
- 2 DC gear motors
- 2 powered wheels
- 1 caster wheel
- 2 green LEDs
- 1 yellow LED
- 3 resistors (220Ω–330Ω each)
- Breadboard
- Jumper wires
- USB cable
- External motor power supply (recommended)

## Pin Configuration

### Motor Connections

| Component | Arduino Pin | Motor Driver |
|-----------|-------------|--------------|
| Left Motor Enable | D9 (PWM) | ENA |
| Left Motor Input 1 | A4 | IN1 |
| Left Motor Input 2 | A3 | IN2 |
| Right Motor Enable | D10 (PWM) | ENB |
| Right Motor Input 1 | A0 | IN3 |
| Right Motor Input 2 | A1 | IN4 |
| Ground | GND | GND |

This configuration assumes motor driver channel A controls the left wheel and channel B controls the right wheel.

### LED Connections

| LED | Arduino Pin | Purpose |
|-----|-------------|---------|
| Left Green | D2 | Left turn indicator |
| Right Green | D3 | Right turn indicator |
| Yellow | D4 | Forward/reverse/spin indicator |

Each LED uses its own current-limiting resistor.

### Breadboard Wiring

The following layout uses a standard breadboard with connected holes A–E on each numbered row.

| Connection | Left Green LED | Right Green LED | Yellow LED |
|-----------|----------------|-----------------|------------|
| Arduino Signal | D2 → B9 | D3 → B19 | D4 → B24 |
| Resistor | A9–A10 | A19–A20 | A24–A25 |
| LED Positive (+) | D10 | D20 | C25 |
| LED Negative (−) | D11 | D21 | C26 |
| Ground Jumper | B11 → GND rail | B21 → GND rail | B26 → GND rail |

**Important:**
- LED long leg = Positive (+)
- LED short leg = Negative (−)
- All LEDs share the negative ground rail.
- The ground rail must connect to Arduino GND.
- LEDs are powered through digital pins D2, D3, and D4.
- The LEDs do not require a direct connection to the breadboard's 5V rail.

## Robot Controls

The robot is controlled by sending characters through the Arduino Serial Monitor.

| Command | Action | Description |
|---------|--------|-------------|
| `w` | Forward | Both wheels move forward |
| `s` | Backward | Both wheels move backward |
| `a` | Turn Left | Left wheel moves slower |
| `d` | Turn Right | Right wheel moves slower |
| `q` | Spin Left | Left wheel backward, right wheel forward |
| `e` | Spin Right | Left wheel forward, right wheel backward |
| `Space` | Stop | Stops both motors |
| `+` | Increase Speed | Increases PWM by 15 |
| `-` | Decrease Speed | Decreases PWM by 15 |

**Note:** Commands must be entered and sent through the Serial Monitor. These are not real-time keyboard controls.

## LED Indicators

The robot features two green direction indicators and one yellow movement indicator.

### LED Behaviour

| Command | Movement | Left Green | Right Green | Yellow |
|---------|----------|------------|-------------|--------|
| `w` | Forward | OFF | OFF | Solid ON |
| `s` | Backward | OFF | OFF | Blinking |
| `a` | Turn Left | Blinking | OFF | OFF |
| `d` | Turn Right | OFF | Blinking | OFF |
| `q` | Spin Left | Blinking | OFF | Solid ON |
| `e` | Spin Right | OFF | Blinking | Solid ON |
| `Space` | Stop | OFF | OFF | OFF |

### Blink Timing

LED blinking is controlled using Arduino's `millis()` function.

```cpp
const unsigned long blinkInterval = 400;
```

The LED changes between ON and OFF every 400 milliseconds.

This produces:
- 400 ms ON
- 400 ms OFF
- 800 ms per complete blink cycle

The implementation is non-blocking, allowing the Arduino to continue processing commands while LEDs blink.

### Spin Indicators

When spinning, the robot activates two LEDs simultaneously.

**Spin Left (`q`):**
- Left green LED blinks.
- Yellow LED remains solid ON.

**Spin Right (`e`):**
- Right green LED blinks.
- Yellow LED remains solid ON.

## Motor Speed Control

Motor speed is controlled using Pulse Width Modulation (PWM).

| Setting | Value |
|---------|-------|
| Minimum PWM | 0 |
| Maximum PWM | 255 |
| Default PWM | 255 |
| Adjustment Step | 15 |

The PWM value controls the motor driver's duty cycle rather than specifying an exact rotation speed.

### Speed Adjustment

Sending `+` increases the speed setting:

```cpp
speed = constrain(speed + 15, 0, 255);
```

Sending `-` decreases the speed setting:

```cpp
speed = constrain(speed - 15, 0, 255);
```

The speed setting cannot exceed 255 or fall below 0.

## Differential Drive

The robot uses two independently powered wheels and one freely rotating caster wheel.

Direction changes are achieved by adjusting the relative speeds and rotation directions of the powered wheels.

### Forward

```cpp
drive(motorSpeed, motorSpeed);
```

Both wheels move forward at the same PWM setting.

### Backward

```cpp
drive(-motorSpeed, -motorSpeed);
```

Both wheels move backward.

### Turn Left

```cpp
drive(motorSpeed / 2, motorSpeed);
```

The left wheel receives half the PWM setting of the right wheel, causing the robot to curve left.

### Turn Right

```cpp
drive(motorSpeed, motorSpeed / 2);
```

The right wheel receives half the PWM setting of the left wheel, causing the robot to curve right.

### Spin Left

```cpp
drive(-motorSpeed, motorSpeed);
```

The left wheel rotates backward while the right wheel rotates forward.

### Spin Right

```cpp
drive(motorSpeed, -motorSpeed);
```

The left wheel rotates forward while the right wheel rotates backward.

## Software Architecture

The code is organized into reusable functions.

### Motor Control

`setMotor()`

Controls an individual motor's direction and PWM output.

`drive()`

Controls both motors independently.

### Movement Functions

- `forward()`
- `backward()`
- `turnLeft()`
- `turnRight()`
- `spinLeft()`
- `spinRight()`
- `stop()`

### Command Processing

`applyMovement()`

Converts Serial Monitor commands into motor movements.

### LED Control

`updateLEDs()`

Updates the LED indicators based on the currently selected movement.

Uses `millis()` to implement non-blocking blinking.

### Main Loop

`loop()`

Continuously:
1. Checks for serial input.
2. Processes movement and speed commands.
3. Updates motor control when commands arrive.
4. Updates LED indicators.

## Getting Started

### 1. Connect the hardware

Connect the motors, motor driver, and three LEDs according to the pin configuration and breadboard wiring tables.

Verify that all components share the appropriate ground connections.

### 2. Open Arduino IDE

Open the project's `.ino` file.

### 3. Connect the Arduino

Connect the Arduino Uno to your computer using USB.

### 4. Select the board

In Arduino IDE:
- Select **Arduino Uno**.
- Select the correct USB port.

### 5. Upload the code

Click **Upload** to compile and transfer the program to the Arduino.

### 6. Open Serial Monitor

Set the baud rate to:

```text
9600
```

### 7. Control the robot

Enter one of the supported commands and press Enter or Send.

For example:

```text
w
```

The robot moves forward and the yellow LED turns on.

Send:

```text
q
```

The robot spins left, the left green LED blinks, and the yellow LED stays on.

To stop the robot, send a single space character.

## Power Supply

The initial prototype uses a USB connection to power the Arduino and motor driver.

However, the Arduino's 5V supply may not provide sufficient current to reliably operate both motors under load.

An appropriately rated external motor power supply is recommended.

When using an external supply:
- Connect it to the motor driver's appropriate motor-power terminals.
- Connect the motor driver's ground to Arduino GND.
- Verify voltage limits and motor driver power configuration.
- Do not connect the external battery's positive terminal directly to Arduino 5V.

The LEDs can continue to be controlled and powered through Arduino digital pins D2, D3, and D4.

## Troubleshooting

### One wheel doesn't rotate during a turn

The slower wheel may not receive enough torque to overcome friction.

Possible solutions:
- Increase the PWM setting.
- Adjust the turning ratio.
- Use a suitable external motor power supply.

For example, a less aggressive left turn:

```cpp
void turnLeft(int motorSpeed) {
  drive(motorSpeed * 4 / 5, motorSpeed);
}
```

### Wheels rotate in the air but not on the ground

This may indicate insufficient starting torque or excessive mechanical resistance.

Check:
- Motor power supply
- Wheel alignment
- Gearbox resistance
- Robot weight
- Motor driver connections

### Robot moves in the wrong direction

Check the motor connections and the negative signs in the `drive()` function.

These signs are used to compensate for the robot's motor orientation and wiring.

### LED doesn't turn on

Verify:
- LED polarity
- Correct Arduino digital pin
- Resistor placement
- Ground connection
- Breadboard row connectivity

### LED doesn't blink

Verify that `updateLEDs()` is called continuously from `loop()`.

Also ensure that the correct movement command has been received.

### Commands don't work

Verify that:
- Arduino Serial Monitor is open.
- Baud rate is set to 9600.
- Correct board and port are selected.
- Commands are sent using Enter or Send.

## Project Structure

```text
RobotController/
├── RobotController.ino
├── README.md
└── circuit-diagram.png
```

## Future Improvements

- External battery-powered motor supply
- Motor speed calibration
- Improved turning accuracy
- Minimum startup PWM
- Automatic stop on communication timeout
- Real-time keyboard controls
- Obstacle detection
- Wireless robot control