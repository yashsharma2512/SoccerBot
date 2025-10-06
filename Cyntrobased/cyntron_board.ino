/*
  Ported for Cytron MOTION 2350 Pro (RP2350)
  - Maps 4 omni motors (X pattern) to M1..M4 pins (GP8..GP15).
  - Uses PWM + DIR method (analogWrite on PWM pin, digitalWrite on DIR pin).
  - Keeps your MPU6050 and IR/analog sensor code intact.
  Notes:
  - Motion 2350 Pro pinout: GP8..GP15 are the motor control pins for M1..M4.
  - RP2350 GPIO erratum: when using inputs, consider external pull-downs <8kΩ. See datasheet.
  References: Cytron tutorial + datasheet + CytronMotorDriver repo.
  (If you want, I can later change the low-level control to use CytronMotorDriver objects.)
*/

#include "Wire.h"
#include <MPU6050_light.h>

int ir = 0;
MPU6050 mpu(Wire);

int sensorPin[4] = { A0, A1, A2, A3 };  // IR sensors (keep as analog pins)
float smooth[4] = { 0, 0, 0, 0 };       // smoothed value
float alpha = 0.1;

int lastPosition = 0;

// ---- MOTOR pin definitions mapped to MOTION 2350 Pro GP pins ----
// (GP8..GP15 used as PWM/dir pairs)
const int M1_PWM = 9;  // Front Left PWM (GP8)
const int M1_DIR = 8;  // Front Left DIR (GP9)

const int M2_PWM = 11;  // Front Right PWM (GP10)
const int M2_DIR = 10;  // Front Right DIR (GP11)

const int M3_PWM = 12;  // Rear Right PWM (GP12)
const int M3_DIR = 13;  // Rear Right DIR (GP13)

const int M4_PWM = 14;  // Rear Left PWM (GP14)
const int M4_DIR = 15;  // Rear Left DIR (GP15)

// PID / gyro target
float targetAngle = 0;
float Kp = 5.0;

// Convenience arrays for indexing
const int pwmPins[4] = { M1_PWM, M2_PWM, M3_PWM, M4_PWM };
const int dirPins[4] = { M1_DIR, M2_DIR, M3_DIR, M4_DIR };

void setup() {
  // Setup motor pins
  for (int i = 0; i < 4; i++) {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    // Ensure stopped
    analogWrite(pwmPins[i], 0);
    digitalWrite(dirPins[i], LOW);
  }

  // IR / sensors
  // pinMode(ir, INPUT); // note: consider external pull-down <8k if errata affects this pin

  for (int i = 0; i <= 3; i++)
    smooth[i] = analogRead(sensorPin[i]);

  Serial.begin(115200);
  Wire.setSDA(16);  // GP0
  Wire.setSCL(17);  // GP1
  Wire.begin();
  // Wire.setClock(100000);
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // halt if cannot connect

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done!\n");
}

// Low-level motor setter using PWM+DIR.
// speed range expected: -255 .. 255
void setMotor(int motorIndex, int speed) {
  speed = constrain(speed, -255, 255);
  int pwmPin = pwmPins[motorIndex];
  int dirPin = dirPins[motorIndex];

  if (speed == 0) {
    // brake/coast: set PWM to 0 and both dir low
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, LOW);
  } else if (speed > 0) {
    // forward
    digitalWrite(dirPin, LOW);  // choose polarity (LOW = forward here)
    analogWrite(pwmPin, abs(speed));
  } else {
    // reverse
    digitalWrite(dirPin, HIGH);  // set opposite polarity
    analogWrite(pwmPin, abs(speed));
  }
}

// generic motor driver wrapper to maintain your existing motor(IN1,IN2,EN,spd) pattern
void motor(int in1_dummy, int in2_dummy, int en_dummy, int spd, int motorIndex) {
  // existing code used (in1,in2,en) but we now map motorIndex 0..3 to M1..M4
  setMotor(motorIndex, spd);
}

// ================= Demo loop =================
void loop() {
  // int x = digitalRead(ir);
  // Serial.println(x);
  // if (digitalRead(ir) == LOW) {   // LEFT detects white
  //   omniDriveWithGyro(0,0,0);
  //   Serial.println("Left line detected → move right");
  //   delay(500);
  //   omniDrive(150, 0, 0);  // shift right
  //   delay(500);
  //   omniDriveWithGyro(0,0,0);
  // } else {
  int error = (1500 - ballPos()) / 10;
  float angle = (error / 150.0) * (135.0 * PI / 180.0);
  int vx = (int)(150 * cos(angle));
  int vy = (int)(150 * sin(angle));
  Serial.println(vx);
  Serial.print(',');
    Serial.println(vy);
  omniDriveWithGyro(vx, vy, 0);
  // omniDriveWithGyro(0, -100, 0);


  delay(1);
  // }
}


// NOTE: omniDrive & omniDriveWithGyro adapted to call setMotor with motorIndex 0..3
void omniDrive(int vx, int vy, int wz) {
  // int w1 = vx + vy - wz;  // Front Left  -> motor index 0
  // int w2 = vx - vy + wz;  // Front Right -> motor index 1
  // int w3 = vx + vy + wz;  // Rear Right  -> motor index 2
  // int w4 = vx - vy - wz;  // Rear Left   -> motor index 3
  int w1 = vy + vx - wz;  // Front Left
  int w2 = vy - vx + wz;  // Front Right
  int w3 = vy + vx + wz;  // Rear Right
  int w4 = vy - vx - wz;  // Rear Left
  w1 = constrain(w1, -255, 255);
  w2 = constrain(w2, -255, 255);
  w3 = constrain(w3, -255, 255);
  w4 = constrain(w4, -255, 255);

  // Send to motors (map to indices)
  setMotor(0, w1);
  setMotor(1, w2);
  setMotor(2, w3);
  setMotor(3, w4);
}

void omniDriveWithGyro(int vx, int vy, int vz) {
  mpu.update();
  float gyroAngle = mpu.getAngleZ();

  float error = targetAngle - gyroAngle;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  int wz = (int)(Kp * error);
  wz = constrain(wz, -100, 100);

  int w1 = vx + vy - wz;  // Front Left
  int w2 = vx - vy + wz;  // Front Right
  int w3 = vx + vy + wz;  // Rear Right
  int w4 = vx - vy - wz;  // Rear Left

  w1 = constrain(w1, -255, 255);
  w2 = constrain(w2, -255, 255);
  w3 = constrain(w3, -255, 255);
  w4 = constrain(w4, -255, 255);

  Serial.print(w1);
  Serial.print(",");
  Serial.print(w2);
  Serial.print(",");
  Serial.print(w3);
  Serial.print(",");
  Serial.println(w4);

  setMotor(0, w1);
  setMotor(1, w2);
  setMotor(2, w3);
  setMotor(3, w4);
}

int ballPos() {
  bool online = false;
  long weightedSum = 0;
  long total = 0;
  int position = 0;

  for (int i = 0; i <= 3; i++) {
    int raw = analogRead(sensorPin[i]);
    smooth[i] = alpha * raw + (1 - alpha) * smooth[i];
    if (smooth[i] > 80) {
      online = true;
      weightedSum += (long)smooth[i] * (i * 1000);
      total += smooth[i];
    }
  }

  if (total == 0) {
    if (lastPosition > 1500)
      return 3000;
    else
      return 0;
  }
  position = weightedSum / total;
  lastPosition = position;
  return position;
}
