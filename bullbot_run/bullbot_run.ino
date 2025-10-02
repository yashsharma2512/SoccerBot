#include "Wire.h"
#include <MPU6050_light.h>
int ir = 0;
MPU6050 mpu(Wire);

int sensorPin[4] = { A0, A1, A2, A3 };  // change if your IR sensor is on another pin
float smooth[4] = { 0, 0, 0, 0 };       // smoothed value
float alpha = 0.1;                      // smoothing factor (0.1 = smoother, 0.3 = faster)

int lastPosition = 0;
// Motor pins (adjust based on your wiring)
int IN1 = 3, IN2 = 2, ENA = 5;     // Motor 1
int IN3 = 4, IN4 = 7, ENB = 6;     // Motor 2
int IN5 = 9, IN6 = 8, ENC = 10;    // Motor 3
int IN7 = 13, IN8 = 12, END = 11;  // Motor 4

float targetAngle = 0;  // keep robot facing this direction
float Kp = 5.0;         // tune this gain for correction strength


void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  pinMode(END, OUTPUT);
  pinMode(ir, INPUT);
  for (int i = 0; i <= 3; i++)
    smooth[i] = analogRead(sensorPin[i]);
  Serial.begin(9600);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");
}

// Generic motor driver
void motor(int in1, int in2, int en, int spd) {
  if (spd > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (spd < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(en, abs(spd));
}


// ===== Demo loop =====
void loop() {
// int x = digitalRead(ir);
// Serial.println(x);
   if (digitalRead(ir) == HIGH) {   // LEFT detects white
     omniDriveWithGyro(0,0,0);
    Serial.println("Left line detected → move right");
    delay(500);   // pause
    omniDrive(150, 0, 0);  // shift right
    delay(500);   // move a little bit
    omniDriveWithGyro(0,0,0);
   }
  // // for (int i = 0; i < 2000; i++) {
  // //   omniDriveWithGyro(150, 0, 0);
  // //   delay(1);
  // // }
  // // omniDriveWithGyro(0, 0, 0);
  // // delay(1000);
  else
  {
  int error = (1500 - ballPos()) / 10;

  // float angle = (err / 150.0) * (PI / 2);
  float angle = (error / 150.0) * (135.0 * PI / 180.0);
  int x = 150 * cos(angle);
  int y = 150 * sin(angle);

  omniDriveWithGyro(x, y, 0);
  // Serial.print(x, DEC);
  // Serial.print(",");
  // Serial.println(y, DEC);
  delay(1);
  }
}



void omniDrive(int vx, int vy, int wz) {
  int w1 = vx + vy - wz;  // Front Left
  int w2 = vx - vy + wz;  // Front Right
  int w3 = vx + vy + wz;  // Rear Right
  int w4 = vx - vy - wz;  // Rear Left

  // // optional: normalize so values stay in PWM range
  // int maxVal = max(max(abs(w1), abs(w2)), max(abs(w3), abs(w4)));
  // if (maxVal > 255) {
  //   w1 = (w1 * 255) / maxVal;
  //   w2 = (w2 * 255) / maxVal;
  //   w3 = (w3 * 255) / maxVal;
  //   w4 = (w4 * 255) / maxVal;
  // }

  w1 = constrain(w1, -255, 255);
  w2 = constrain(w2, -255, 255);
  w3 = constrain(w3, -255, 255);
  w4 = constrain(w4, -255, 255);

  // Serial.print(w1, DEC);
  // Serial.print(",");
  // Serial.print(w2, DEC);
  // Serial.print(",");
  // Serial.print(w3, DEC);
  // Serial.print(",");
  // Serial.println(w4, DEC);

  // Send to motors
  motor(IN1, IN2, ENA, w1);  // Front Left
  motor(IN3, IN4, ENB, w2);  // Front Right
  motor(IN5, IN6, ENC, w3);  // Rear Right
  motor(IN7, IN8, END, w4);  // Rear Left
}


void omniDriveWithGyro(int vx, int vy, int vz) {
  mpu.update();
  float gyroAngle = mpu.getAngleZ();
  // Calculate heading error
  float error = targetAngle - gyroAngle;

  // Normalize error to -180..180
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  // Proportional control for rotation
  int wz = Kp * error;

  // Limit rotation speed
  if (wz > 100) wz = 100;
  if (wz < -100) wz = -100;

  // Use your existing omni kinematics
  int w1 = vx + vy - wz;  // Front Left
  int w2 = vx - vy + wz;  // Front Right
  int w3 = vx + vy + wz;  // Rear Right
  int w4 = vx - vy - wz;  // Rear Left

  w1 = constrain(w1, -255, 255);
  w2 = constrain(w2, -255, 255);
  w3 = constrain(w3, -255, 255);
  w4 = constrain(w4, -255, 255);

  Serial.print(w1, DEC);
  Serial.print(",");
  Serial.print(w2, DEC);
  Serial.print(",");
  Serial.print(w3, DEC);
  Serial.print(",");
  Serial.println(w4, DEC);

  // Send to motors
  motor(IN1, IN2, ENA, w1);
  motor(IN3, IN4, ENB, w2);
  motor(IN5, IN6, ENC, w3);
  motor(IN7, IN8, END, w4);
}


int ballPos() {
  bool online = false;
  long weightedSum = 0;
  long total = 0;
  int position = 0;

  // put your main code here, to run repeatedly:
  for (int i = 0; i <= 3; i++) {
    int raw = analogRead(sensorPin[i]);                 // read raw sensor value (0–1023)
    smooth[i] = alpha * raw + (1 - alpha) * smooth[i];  // EMA smoothing
    if (smooth[i] > 80) {
      online = true;
      weightedSum += (long)smooth[i] * (i * 1000);
      total += smooth[i];
    }
  }

  if (total == 0) {
    // No line detected
    if (lastPosition > 1500)
      return 3000;  // assume it exited right
    else
      return 0;  // exactly center fallback
  }
  position = weightedSum / total;
  // Optional smoothing (low-pass filter)
  //  position = (lastPosition * 2 + position * 8) / 10;
  lastPosition = position;
  return position;
}
