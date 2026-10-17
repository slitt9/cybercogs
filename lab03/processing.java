package lab03;
import java.util.ArrayList;

import lab03.processing.serial.*;

// ==========================================
// SERIAL CONNECTION
// ==========================================

Serial port;

// -1 = automatically detect Arduino USB port
// Otherwise specify the port index
int SERIAL_PORT_INDEX = -1;

final int BAUD_RATE = 9600;

boolean connected = false;
String connectionMessage = "Not connected";


// ==========================================
// WHEEL CALIBRATION
// ==========================================

// Replace with your measured values.
//
// wheelDiameterCM:
// Diameter of the large drive wheel.
//
// pulsesPerRevolution:
// Encoder pulses detected during one complete
// revolution of the drive wheel.

float wheelDiameterCM = 6;
int pulsesPerRevolution = 20;

boolean isCalibrated() {
  return wheelDiameterCM > 0 &&
         pulsesPerRevolution > 0;
}


// ==========================================
// CSV LOGGING
// ==========================================

PrintWriter out;

String csvPath = "";

boolean recording = false;
boolean closed = false;

int rowsRecorded = 0;


// ==========================================
// ROBOT STATE
// ==========================================

// Currently held manual movement key
char activeCommand = ' ';

// Tracks whether autonomous mode was requested
boolean followMeActive = false;

// Values reported by Arduino
String movement = "stop";
String direction = "Stop";

int motorPWM = 255;

// Encoder counts
long leftCount = 0;
long rightCount = 0;

// Pulses per second
float leftPPS = 0;
float rightPPS = 0;

// Converted measurements
float leftRPM = Float.NaN;
float rightRPM = Float.NaN;

float leftSpeedCMS = Float.NaN;
float rightSpeedCMS = Float.NaN;

// Arduino timestamp
long arduinoTime = 0;


// ==========================================
// TELEMETRY DISPLAY
// ==========================================

ArrayList<String> recentRows =
  new ArrayList<String>();

final int MAX_ROWS = 12;


// ==========================================
// SETUP
// ==========================================

void setup() {

  size(1000, 750);

  surface.setTitle("Arduino Robot Dashboard");

  textFont(createFont("Arial", 16));

  println("================================");
  println("ARDUINO ROBOT DASHBOARD");
  println("================================");

  String[] ports = Serial.list();

  println("Available serial ports:");

  for (int i = 0; i < ports.length; i++) {
    println(i + ": " + ports[i]);
  }

  createLogFile();

  connectArduino(ports);
}


// ==========================================
// CREATE CSV FILE
// ==========================================

void createLogFile() {

  String timestamp =
    nf(year(), 4) +
    nf(month(), 2) +
    nf(day(), 2) + "_" +
    nf(hour(), 2) +
    nf(minute(), 2) +
    nf(second(), 2);

  String filename =
    "robot_telemetry_" + timestamp + ".csv";

  csvPath = sketchPath(filename);

  try {

    out = createWriter(csvPath);

    out.println(
      "t_ms,movement,direction,pwm," +
      "left_count,right_count," +
      "left_pps,right_pps," +
      "left_rpm,right_rpm," +
      "left_cm_s,right_cm_s"
    );

    out.flush();

    recording = true;

    println("CSV file created:");
    println(csvPath);

  } catch (Exception e) {

    recording = false;

    println("CSV creation failed:");
    println(e.getMessage());
  }
}


// ==========================================
// CONNECT TO ARDUINO
// ==========================================

void connectArduino(String[] ports) {

  if (ports.length == 0) {

    connectionMessage = "No serial ports found";
    return;
  }

  int selectedPort = SERIAL_PORT_INDEX;

  // Automatically identify USB serial port
  if (selectedPort == -1) {

    for (int i = 0; i < ports.length; i++) {

      String name = ports[i].toLowerCase();

      if (name.contains("usbmodem") ||
          name.contains("usbserial") ||
          name.contains("wchusbserial")) {

        selectedPort = i;
        break;
      }
    }
  }

  if (selectedPort < 0 ||
      selectedPort >= ports.length) {

    connectionMessage =
      "Arduino port not found. Select port manually.";

    println(connectionMessage);

    return;
  }

  try {

    String portName = ports[selectedPort];

    println("Connecting to: " + portName);

    port = new Serial(
      this,
      portName,
      BAUD_RATE
    );

    port.clear();
    port.bufferUntil('\n');

    connected = true;

    connectionMessage =
      "Connected: " + portName;

    println(connectionMessage);

  } catch (Exception e) {

    connected = false;

    connectionMessage =
      "Connection failed: " + e.getMessage();

    println(connectionMessage);
  }
}


// ==========================================
// SEND COMMAND TO ARDUINO
// ==========================================

void sendCommand(char command) {

  if (!connected || port == null) {
    return;
  }

  try {

    port.write(str(command));

  } catch (Exception e) {

    connected = false;

    followMeActive = false;
    activeCommand = ' ';

    connectionMessage =
      "Serial error: " + e.getMessage();
  }
}


// ==========================================
// MOVEMENT TRANSLATOR
// ==========================================

String translateMovement(String command) {

  switch (command) {

    case "w":
      return "Forward";

    case "s":
      return "Backward";

    case "a":
      return "Turn Left";

    case "d":
      return "Turn Right";

    case "q":
      return "Spin Left";

    case "e":
      return "Spin Right";

    // NEW: Autonomous Follow Me mode
    case "h":
      return "Follow Me";

    case "stop":
    case " ":
      return "Stop";

    default:
      return "Unknown";
  }
}


// ==========================================
// SPEED CONVERSIONS
// ==========================================

// Convert pulses per second into RPM
float calculateRPM(float pps) {

  if (!isCalibrated()) {
    return Float.NaN;
  }

  return (pps * 60.0) / pulsesPerRevolution;
}


// Convert pulses per second into cm/s
float calculateSpeed(float pps) {

  if (!isCalibrated()) {
    return Float.NaN;
  }

  float circumference =
    PI * wheelDiameterCM;

  float revolutionsPerSecond =
    pps / pulsesPerRevolution;

  return revolutionsPerSecond * circumference;
}


// ==========================================
// FORMAT MEASUREMENTS
// ==========================================

String formatMeasurement(float value) {

  if (Float.isNaN(value) ||
      Float.isInfinite(value)) {

    return "NA";
  }

  return nf(value, 0, 2);
}


// ==========================================
// KEYBOARD CONTROL
// ==========================================

void keyPressed() {

  char k = Character.toLowerCase(key);

  // --------------------------------------
  // Manual movement commands
  // --------------------------------------

  if (k == 'w' ||
      k == 's' ||
      k == 'a' ||
      k == 'd' ||
      k == 'q' ||
      k == 'e') {

    // Manual input overrides Follow Me
    followMeActive = false;

    if (activeCommand != k) {

      activeCommand = k;

      sendCommand(k);

      println("Manual command: " + k);
    }
  }

  // --------------------------------------
  // NEW: Follow Me autonomous mode
  // --------------------------------------

  else if (k == 'h') {

    // Follow Me stays active after releasing H
    // until STOP or a manual command is sent.

    if (!followMeActive) {

      activeCommand = ' ';

      followMeActive = true;

      sendCommand('h');

      println("================================");
      println("FOLLOW ME MODE ACTIVATED");
      println("Target distance: 25 cm");
      println("Press SPACE to stop.");
      println("================================");
    }
  }

  // --------------------------------------
  // Space = STOP
  // --------------------------------------

  else if (key == ' ') {

    activeCommand = ' ';

    followMeActive = false;

    sendCommand(' ');

    println("ROBOT STOPPED");
  }

  // --------------------------------------
  // Increase motor PWM
  // --------------------------------------

  else if (key == '+' || key == '=') {

    sendCommand('+');
  }

  // --------------------------------------
  // Decrease motor PWM
  // --------------------------------------

  else if (key == '-' || key == '_') {

    sendCommand('-');
  }
}


// ==========================================
// RELEASE MANUAL KEY = STOP
// ==========================================

void keyReleased() {

  char k = Character.toLowerCase(key);

  // Only stop on release of the active
  // MANUAL movement key.
  //
  // Releasing H must NOT stop Follow Me.

  if (k == activeCommand &&
      activeCommand != ' ') {

    activeCommand = ' ';

    sendCommand(' ');
  }
}


// ==========================================
// RECEIVE TELEMETRY
// ==========================================

void serialEvent(Serial p) {

  String line = p.readStringUntil('\n');

  if (line == null) {
    return;
  }

  line = trim(line);

  if (line.length() == 0) {
    return;
  }

  // Ignore Arduino CSV header
  if (line.startsWith("t_ms")) {
    return;
  }

  String[] parts = split(line, ',');

  // Arduino sends exactly 7 columns
  if (parts.length != 7) {

    println("Invalid telemetry row: " + line);

    return;
  }

  try {

    // --------------------------------------
    // Read Arduino telemetry
    // --------------------------------------

    arduinoTime =
      Long.parseLong(trim(parts[0]));

    movement = trim(parts[1]);

    motorPWM =
      Integer.parseInt(trim(parts[2]));

    leftCount =
      Long.parseLong(trim(parts[3]));

    rightCount =
      Long.parseLong(trim(parts[4]));

    leftPPS =
      Float.parseFloat(trim(parts[5]));

    rightPPS =
      Float.parseFloat(trim(parts[6]));


    // --------------------------------------
    // Translate movement
    // --------------------------------------

    direction = translateMovement(movement);

    // Sync dashboard autonomous indicator
    // with the mode reported by Arduino.
    followMeActive = movement.equals("h");


    // --------------------------------------
    // Calculate wheel measurements
    // --------------------------------------

    leftRPM = calculateRPM(leftPPS);
    rightRPM = calculateRPM(rightPPS);

    leftSpeedCMS = calculateSpeed(leftPPS);
    rightSpeedCMS = calculateSpeed(rightPPS);


    // --------------------------------------
    // Create enhanced CSV row
    // --------------------------------------

    String csvRow =
      arduinoTime + "," +
      movement + "," +
      direction + "," +
      motorPWM + "," +
      leftCount + "," +
      rightCount + "," +
      leftPPS + "," +
      rightPPS + "," +
      formatMeasurement(leftRPM) + "," +
      formatMeasurement(rightRPM) + "," +
      formatMeasurement(leftSpeedCMS) + "," +
      formatMeasurement(rightSpeedCMS);


    // --------------------------------------
    // Save CSV
    // --------------------------------------

    if (recording && out != null) {

      out.println(csvRow);
      out.flush();

      rowsRecorded++;
    }


    // --------------------------------------
    // Print telemetry to console
    // --------------------------------------

    println(csvRow);


    // --------------------------------------
    // Update recent telemetry
    // --------------------------------------

    recentRows.add(csvRow);

    if (recentRows.size() > MAX_ROWS) {
      recentRows.remove(0);
    }

  } catch (NumberFormatException e) {

    println("Invalid numeric telemetry: " + line);
  }
}


// ==========================================
// DRAW DASHBOARD
// ==========================================

void draw() {

  background(245);

  // Stop manual driving if dashboard
  // loses keyboard focus.
  if (!focused && activeCommand != ' ') {

    activeCommand = ' ';

    sendCommand(' ');
  }

  drawHeader();

  drawRobotStatus();

  drawEncoderData();

  drawControls();

  drawTelemetry();

  drawFooter();
}


// ==========================================
// HEADER
// ==========================================

void drawHeader() {

  fill(25);
  textSize(28);

  text("Arduino Robot Dashboard", 30, 45);

  textSize(13);

  if (connected) {

    fill(20, 140, 65);

  } else {

    fill(190, 40, 40);
  }

  text(connectionMessage, 30, 72);
}


// ==========================================
// ROBOT STATUS
// ==========================================

void drawRobotStatus() {

  fill(255);
  stroke(210);

  rect(25, 100, 950, 105, 10);

  fill(30);
  textSize(17);

  text("Robot Status", 45, 130);

  textSize(15);

  text("Direction: " + direction, 45, 165);

  // The Arduino's existing PWM field is
  // the stored manual speed setting.
  // It is NOT the live PID motor output.
  text("Speed setting: " + motorPWM, 340, 165);

  text(
    "Arduino time: " + arduinoTime + " ms",
    665,
    165
  );

  // Highlight autonomous mode
  if (followMeActive) {

    fill(20, 140, 65);

    textSize(13);

    text(
      "AUTO: FOLLOW ME - TARGET 25 cm",
      45,
      190
    );
  }
}


// ==========================================
// ENCODER DATA
// ==========================================

void drawEncoderData() {

  // --------------------------------------
  // LEFT WHEEL
  // --------------------------------------

  fill(255);
  stroke(210);

  rect(25, 225, 460, 150, 10);

  fill(30);
  textSize(18);

  text("LEFT WHEEL", 45, 255);

  textSize(27);

  text(
    nf(leftPPS, 0, 2) + " PPS",
    45,
    295
  );

  textSize(14);

  text(
    "Total pulses: " + leftCount,
    45,
    330
  );

  text(
    "RPM: " + formatMeasurement(leftRPM),
    270,
    290
  );

  text(
    "Speed: " +
    formatMeasurement(leftSpeedCMS) +
    " cm/s",
    270,
    320
  );


  // --------------------------------------
  // RIGHT WHEEL
  // --------------------------------------

  fill(255);
  stroke(210);

  rect(515, 225, 460, 150, 10);

  fill(30);
  textSize(18);

  text("RIGHT WHEEL", 535, 255);

  textSize(27);

  text(
    nf(rightPPS, 0, 2) + " PPS",
    535,
    295
  );

  textSize(14);

  text(
    "Total pulses: " + rightCount,
    535,
    330
  );

  text(
    "RPM: " + formatMeasurement(rightRPM),
    760,
    290
  );

  text(
    "Speed: " +
    formatMeasurement(rightSpeedCMS) +
    " cm/s",
    760,
    320
  );
}


// ==========================================
// KEYBOARD CONTROLS
// ==========================================

void drawControls() {

  fill(30);
  textSize(18);

  text("Keyboard Controls", 30, 415);

  // Q W E
  drawKey("Q", 'q', 60, 435);
  drawKey("W", 'w', 140, 435);
  drawKey("E", 'e', 220, 435);

  // A S D
  drawKey("A", 'a', 60, 495);
  drawKey("S", 's', 140, 495);
  drawKey("D", 'd', 220, 495);


  // --------------------------------------
  // FOLLOW ME BUTTON INDICATOR
  // --------------------------------------

  fill(
    followMeActive ?
    color(50, 155, 85) :
    color(230)
  );

  stroke(190);

  rect(330, 435, 180, 45, 8);

  fill(followMeActive ? 255 : 30);

  textAlign(CENTER, CENTER);
  textSize(15);

  text("H - FOLLOW ME", 420, 457);

  textAlign(LEFT, BASELINE);


  // --------------------------------------
  // STOP INDICATOR
  // --------------------------------------

  boolean stopActive =
    movement.equals("stop") &&
    !followMeActive;

  fill(
    stopActive ?
    color(220, 75, 75) :
    color(230)
  );

  stroke(190);

  rect(330, 495, 180, 45, 8);

  fill(stopActive ? 255 : 30);

  textSize(15);
  textAlign(CENTER, CENTER);

  text("SPACE - STOP", 420, 517);

  textAlign(LEFT, BASELINE);


  // --------------------------------------
  // INSTRUCTIONS
  // --------------------------------------

  fill(60);
  textSize(14);

  text(
    "H : Follow Me (25 cm)",
    565,
    455
  );

  text(
    "+ / - : Adjust manual motor speed",
    565,
    480
  );

  text(
    "SPACE : Stop / Exit Follow Me",
    565,
    505
  );

  text(
    "Hold WASD/QE for manual driving",
    565,
    530
  );
}


// ==========================================
// DRAW INDIVIDUAL KEY
// ==========================================

void drawKey(
  String label,
  char command,
  int x,
  int y
) {

  boolean active =
    activeCommand == command;

  fill(
    active ?
    color(70, 165, 100) :
    color(230)
  );

  stroke(190);

  rect(x, y, 65, 45, 8);

  fill(active ? 255 : 30);

  textSize(20);
  textAlign(CENTER, CENTER);

  text(label, x + 32, y + 22);

  textAlign(LEFT, BASELINE);
}


// ==========================================
// LIVE TELEMETRY
// ==========================================

void drawTelemetry() {

  fill(30);
  textSize(18);

  text("Live Telemetry", 30, 590);

  fill(255);
  stroke(210);

  rect(25, 605, 950, 65, 8);

  fill(40);
  textSize(12);

  if (recentRows.size() > 0) {

    String latest =
      recentRows.get(recentRows.size() - 1);

    text(
      latest,
      40,
      630
    );

  } else {

    text(
      "Waiting for telemetry...",
      40,
      630
    );
  }

  text(
    "Rows recorded: " + rowsRecorded,
    40,
    655
  );
}


// ==========================================
// FOOTER
// ==========================================

void drawFooter() {

  fill(40);
  textSize(11);

  text(
    "CSV: " + csvPath,
    25,
    705
  );

  // Save button
  fill(50, 130, 210);
  noStroke();

  rect(795, 690, 180, 35, 6);

  fill(255);

  textAlign(CENTER, CENTER);
  textSize(13);

  text(
    "STOP & SAVE",
    885,
    707
  );

  textAlign(LEFT, BASELINE);

  fill(100);
  textSize(11);

  if (!isCalibrated()) {

    text(
      "Wheel speed calibration required for RPM and cm/s.",
      25,
      735
    );
  }
}


// ==========================================
// MOUSE CONTROL
// ==========================================

void mousePressed() {

  // STOP & SAVE button
  if (mouseX >= 795 &&
      mouseX <= 975 &&
      mouseY >= 690 &&
      mouseY <= 725) {

    shutdown();

    exit();
  }
}


// ==========================================
// CLEAN SHUTDOWN
// ==========================================

void shutdown() {

  if (closed) {
    return;
  }

  closed = true;

  // Stop robot before closing connection
  sendCommand(' ');

  followMeActive = false;
  activeCommand = ' ';

  // Save CSV
  if (out != null) {

    out.flush();
    out.close();

    recording = false;

    println("================================");
    println("CSV SAVED SUCCESSFULLY");
    println("================================");

    println(csvPath);

    println("Total rows: " + rowsRecorded);
  }

  // Close serial port
  if (port != null) {
    port.stop();
  }

  connected = false;
}


// ==========================================
// PROCESSING CLEANUP
// ==========================================

void dispose() {

  shutdown();

  super.dispose();
}