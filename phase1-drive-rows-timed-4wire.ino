// Phase 1 — Robot vacuum rolling base (Arduino Uno R4 WiFi)
// NEAT ROWS, TIMED version (no encoders), VL53L1X single-point LiDAR
// 4-WIRE SENSOR variant — XSHUT and GPIO left unconnected.
//
// Wiring — sensor, 4 wires only:
//   VL53L1X:   VIN -> Arduino 5V, GND -> GND,
//              SDA -> A4, SCL -> A5
//              XSHUT and GPIO: LEAVE UNCONNECTED (see header note).
//              (CQRobot cable is color-coded; match by the labels
//              printed on the sensor board, not by wire color.)
//   TB6612FNG: STBY=8, AIN1=7, AIN2=4, PWMA=5 (left motor)
//              BIN1=12, BIN2=13, PWMB=6 (right motor)
//   Power:     4xAA box -> TB6612FNG VM/GND; Arduino from USB
//              while testing, battery only when wheels are down.

#include <Wire.h>
#include "Adafruit_VL53L1X.h"

// ---------- Tune these first ----------
const int BASE_SPEED = 150;   // 0-255. Start slow.
const int STOP_CM    = 20;    // U-turn when a wall is closer than this.
const int TURN_MS    = 700;   // Milliseconds of in-place turn for ~90 degrees.
                              // STARTING GUESS — tune it (see test step 4).
const int SHIFT_MS   = 500;   // Milliseconds of straight driving for the
                              // sideways shift, ~one robot width (~170 mm).
                              // STARTING GUESS — tune after TURN_MS.
const int BACKUP_MS  = 400;   // Reverse time before turning.
const int VEER_TRIM  = 0;     // Manual straight-line fix, -30..30.
                              // Robot veers LEFT  -> raise this (speeds up
                              // the left wheel / slows the right).
                              // Robot veers RIGHT -> lower it.
const long MAX_ROW_MS = 20000; // Safety: U-turn anyway after 20 s on one
                              // row (missed wall / huge open space).

// ---------- Pins ----------
const int STBY = 8;
const int AIN1 = 7, AIN2 = 4, PWMA = 5;    // Left motor
const int BIN1 = 12, BIN2 = 13, PWMB = 6;  // Right motor
// No XSHUT / IRQ pins in this variant — sensor XSHUT and GPIO are
// left unconnected. See the header note for why.

// ---------- Sensor ----------
// No-arg constructor: both pins default to -1, so begin() never
// drives XSHUT and dataReady() polls the sensor over I2C.
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

// Forward declaration — readDistanceCM() is defined below.
long readDistanceCM();

// ---------- Row state ----------
int rowNumber = 1;
bool uTurnLeft = true;        // Alternate U-turn direction each row.
unsigned long rowStartMs = 0; // When the current row started.

// ---------- Motor helpers ----------
void setMotor(int in1, int in2, int pwm, int speed) {
  // speed: -255 (full reverse) to +255 (full forward)
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);
  }
}

void driveStraight() {
  int leftSpeed  = constrain(BASE_SPEED + VEER_TRIM, 0, 255);
  int rightSpeed = constrain(BASE_SPEED - VEER_TRIM, 0, 255);
  setMotor(AIN1, AIN2, PWMA, leftSpeed);
  setMotor(BIN1, BIN2, PWMB, rightSpeed);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// Turn in place for TURN_MS milliseconds.
// left = true: left motor back, right motor forward.
void turnInPlace(bool left) {
  if (left) {
    setMotor(AIN1, AIN2, PWMA, -BASE_SPEED);
    setMotor(BIN1, BIN2, PWMB, BASE_SPEED);
  } else {
    setMotor(AIN1, AIN2, PWMA, BASE_SPEED);
    setMotor(BIN1, BIN2, PWMB, -BASE_SPEED);
  }
  delay(TURN_MS);
  stopMotors();
  delay(100);
}

// Drive straight for SHIFT_MS — the sideways step between rows.
// Stops early if something is right in front.
void shiftSideways() {
  unsigned long start = millis();
  while ((millis() - start) < (unsigned long)SHIFT_MS) {
    long cm = readDistanceCM();
    if (cm != -1 && cm < STOP_CM) break;
    driveStraight();
    delay(50);
  }
  stopMotors();
  delay(100);
}

// Full U-turn: back up, turn 90, shift one width, turn 90 again.
void doUTurn(bool left) {
  Serial.print("End of row ");
  Serial.print(rowNumber);
  Serial.print(" — U-turn ");
  Serial.println(left ? "left." : "right.");
  stopMotors();
  delay(100);
  setMotor(AIN1, AIN2, PWMA, -BASE_SPEED);
  setMotor(BIN1, BIN2, PWMB, -BASE_SPEED);
  delay(BACKUP_MS);
  stopMotors();
  delay(100);
  turnInPlace(left);
  shiftSideways();
  turnInPlace(left);
  rowNumber++;
  rowStartMs = millis();
  Serial.print("Starting row ");
  Serial.println(rowNumber);
}

// ---------- Sensor ----------
// Returns distance in cm, or -1 when there is no valid reading.
// Keeps the last good reading between ranging updates.
long lastGoodCM = -1;

long readDistanceCM() {
  if (vl53.dataReady()) {
    int16_t mm = vl53.distance();  // Millimetres, -1 = no valid data
    vl53.clearInterrupt();
    if (mm != -1) lastGoodCM = mm / 10;
  }
  return lastGoodCM;
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);  // Enable the motor driver

  if (!vl53.begin(0x29, &Wire)) {
    Serial.print("VL53L1X failed to start, status: ");
    Serial.println(vl53.vl_status);
    Serial.println("Check VIN (5V), GND, SDA (A4), SCL (A5). XSHUT/GPIO are unconnected in this variant — if it still fails, the board may need XSHUT pulled up: switch to the 6-wire timed sketch in phase1-drive-rows-timed/ and wire XSHUT to pin 9, GPIO to pin 10.");
    while (1) { delay(10); }  // Halt: driving blind is worse than not driving.
  }
  Serial.println("VL53L1X sensor OK!");
  vl53.setTimingBudget(50);
  if (!vl53.startRanging()) {
    Serial.print("Couldn't start ranging: ");
    Serial.println(vl53.vl_status);
    while (1) { delay(10); }
  }
  rowStartMs = millis();
  Serial.println("Phase 1 rolling base ready (VL53L1X, neat rows, TIMED/no encoders). Driving row 1...");
}

void loop() {
  long cm = readDistanceCM();
  Serial.print("row ");
  Serial.print(rowNumber);
  Serial.print(" | distance: ");
  Serial.print(cm);
  Serial.println(" cm");

  bool wallAhead = (cm != -1 && cm < STOP_CM);
  bool rowTooLong = (millis() - rowStartMs) > (unsigned long)MAX_ROW_MS;
  if (wallAhead || rowTooLong) {
    if (rowTooLong && !wallAhead) Serial.println("Row time limit hit — turning anyway.");
    doUTurn(uTurnLeft);
    uTurnLeft = !uTurnLeft;  // Next U-turn goes the other way
  } else {
    driveStraight();
  }
  delay(50);
}
