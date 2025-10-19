/*
  Full integration (cleaned & fixed):
  - Cytron MOTION 2350 Pro (RP2350)
  - Motors mapped to GP8..GP15 (M1..M4)
  - MPU6050 on Wire (SDA=GP16, SCL=GP17)
  - PCA9548A at 0x70 on same I2C bus
  - APDS-9960 sensors on PCA channels:
      Front -> channel 0
      Right -> channel 1
      Back  -> channel 2
  - Stop immediately when white is detected (to avoid leaving field)
  - NeoPixel status on GP23 (2 LEDs) and patches (9 LEDs) on GP7
*/

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ---------------- I2C / PCA / APDS registers ----------------
#define SDA_PIN 16
#define SCL_PIN 17
#define PCA9548A_ADDR 0x70
#define APDS9960_ADDR 0x39

#define APDS_REG_ENABLE  0x80
#define APDS_REG_ATIME   0x81
#define APDS_REG_CONTROL 0x8F
#define APDS_REG_CDATA_L 0x94
#define APDS_REG_AICLEAR 0xE7

// ---------------- NeoPixel config ----------------
const int STATUS_PIN = 23;   // status small strip (2 LEDs)
const int STATUS_N = 2;
const int PATCH_PIN  = 7;    // long strip (9 LEDs)
const int PATCH_N    = 9;

Adafruit_NeoPixel statusPixels(STATUS_N, STATUS_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel patches(PATCH_N, PATCH_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- Motors (Motion 2350 mapping) ----------------
const int M1_PWM = 9;   // front left pwm (GP8)
const int M1_DIR = 8;   // front left dir (GP9)
const int M2_PWM = 11;  // front right pwm (GP10)
const int M2_DIR = 10;  // front right dir (GP11)
const int M3_PWM = 12;  // rear right pwm (GP12)
const int M3_DIR = 13;  // rear right dir (GP13)
const int M4_PWM = 14;  // rear left pwm (GP14)
const int M4_DIR = 15;  // rear left dir (GP15)
const int pwmPins[4] = { M1_PWM, M2_PWM, M3_PWM, M4_PWM };
const int dirPins[4] = { M1_DIR, M2_DIR, M3_DIR, M4_DIR };

// ---------------- Analog ball sensors (kept) ----------------
int sensorPin[4] = { A0, A1, A2, A3 };
float smooth[4] = { 0, 0, 0, 0 };
float alpha = 0.1;
int lastPosition = 0;

// ---------------- MPU ----------------
MPU6050 mpu(Wire);
float targetAngle = 0;
float Kp = 5.0;

// ---------------- APDS / PCA helpers ----------------
uint8_t apdsChannels[3] = { 0, 1, 2 };       // front, right, back
uint16_t ambientBaseline[3] = { 300, 300, 300 };  // will be calibrated

void selectPCAChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(800); // small settle time
}

bool writeAPDSReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(APDS9960_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool initAPDSFast(uint8_t channel) {
  selectPCAChannel(channel);
  // Power ON + ALS enable (PON + AEN)
  if (!writeAPDSReg(APDS_REG_ENABLE, 0x03)) return false;
  // Short integration time to allow faster updates (~27.8 ms with 246)
  if (!writeAPDSReg(APDS_REG_ATIME, 246)) return false;  // (~28 ms)
  // Set color gain to max (AGAIN = 3 -> 64x)
  if (!writeAPDSReg(APDS_REG_CONTROL, 0x03)) return false;
  delay(10);
  return true;
}

bool readAPDSRaw(uint8_t channel, uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  selectPCAChannel(channel);

  Wire.beginTransmission(APDS9960_ADDR);
  Wire.write(APDS_REG_CDATA_L);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(APDS9960_ADDR, (uint8_t)8) != 8) return false;

  c = Wire.read() | (Wire.read() << 8);
  r = Wire.read() | (Wire.read() << 8);
  g = Wire.read() | (Wire.read() << 8);
  b = Wire.read() | (Wire.read() << 8);

  // clear ALS interrupt (AICLEAR)
  Wire.beginTransmission(APDS9960_ADDR);
  Wire.write(APDS_REG_AICLEAR);
  Wire.endTransmission();

  return true;
}

bool readAPDSAvg(uint8_t channel, uint16_t &r_out, uint16_t &g_out, uint16_t &b_out, uint16_t &c_out, int samples = 3, int pauseMs = 6) {
  uint32_t rsum = 0, gsum = 0, bsum = 0, csum = 0;
  uint16_t r,g,b,c;
  for (int i = 0; i < samples; i++) {
    if (!readAPDSRaw(channel, r, g, b, c)) return false;
    rsum += r; gsum += g; bsum += b; csum += c;
    delay(pauseMs);
  }
  r_out = rsum / samples;
  g_out = gsum / samples;
  b_out = bsum / samples;
  c_out = csum / samples;
  return true;
}

// Ambient calibration per sensor: sample a few times (on green mat) and store baseline
void calibrateAmbient(uint8_t channel, uint16_t &baseline, int samples = 10) {
  uint32_t sum = 0;
  uint16_t r,g,b,c;
  int collected = 0;
  for (int i = 0; i < samples; i++) {
    if (readAPDSRaw(channel, r, g, b, c)) {
      sum += c;
      collected++;
    } else {
      // if read fails, retry
      delay(20);
      i--;
      continue;
    }
    delay(30);
  }
  if (collected > 0) baseline = (uint16_t)(sum / collected);
  Serial.print("Calib ch"); Serial.print(channel); Serial.print(" baseline C="); Serial.println(baseline);
}

// Color decision: returns true if WHITE detected
bool isWhiteDetected(uint8_t channel) {
  uint16_t r,g,b,c;
  if (!readAPDSAvg(channel, r, g, b, c, 3, 6)) return false; // fail-safe: don't stop if no data

  Serial.print("ch"); Serial.print(channel);
  Serial.print(" C="); Serial.print(c);
  Serial.print(" R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.println(b);

  // dynamic threshold: white if clear is > baseline * factor AND absolute min
  const float factor = 1.6f;
  const uint16_t absMin = 500;  // minimum absolute C to consider white
  uint32_t dynThresh = max((uint32_t)(ambientBaseline[channel] * factor), (uint32_t)absMin);

  // Primary check
  if ((uint32_t)c > dynThresh) return true;

  // Fallback: require all RGB high and balanced -> white
  if (r > 900 && g > 900 && b > 900) return true;

  return false;
}

// ---------------- Motor functions ----------------
void setMotor(int motorIndex, int speed) {
  speed = constrain(speed, -255, 255);
  int pwmPin = pwmPins[motorIndex];
  int dirPin = dirPins[motorIndex];

  if (speed == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, LOW);
  } else if (speed > 0) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, abs(speed));
  } else {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, abs(speed));
  }
}

void omniDrive(int vx, int vy, int wz) {
  int w1 = vy + vx - wz;  // Front Left
  int w2 = vy - vx + wz;  // Front Right
  int w3 = vy + vx + wz;  // Rear Right
  int w4 = vy - vx - wz;  // Rear Left

  setMotor(0, constrain(w1, -255, 255));
  setMotor(1, constrain(w2, -255, 255));
  setMotor(2, constrain(w3, -255, 255));
  setMotor(3, constrain(w4, -255, 255));
}

void omniDriveWithGyro(int vx, int vy, int vz) {
  mpu.update();
  float gyroAngle = mpu.getAngleZ();

  float error = targetAngle - gyroAngle;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  int wz = (int)(Kp * error);
  wz = constrain(wz, -100, 100);

  int w1 = vx + vy - wz;
  int w2 = vx - vy + wz;
  int w3 = vx + vy + wz;
  int w4 = vx - vy - wz;

  setMotor(0, constrain(w1, -255, 255));
  setMotor(1, constrain(w2, -255, 255));
  setMotor(2, constrain(w3, -255, 255));
  setMotor(3, constrain(w4, -255, 255));
}

// ---------------- existing ballPos() (kept) ----------------
int ballPos() {
  bool online = false;
  long weightedSum = 0;
  long total = 0;
  int position = 0;

  for (int i = 0; i <= 3; i++) {
    int raw = analogRead(sensorPin[i]);
    smooth[i] = alpha * raw + (1 - alpha) * smooth[i];
    if (smooth[i] > 120) {
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

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  // NeoPixel init
  statusPixels.begin();
  patches.begin();
  statusPixels.clear();
  patches.clear();
  // turn patches white (main visible strip)
  for (int i = 0; i < PATCH_N; i++) {
    patches.setPixelColor(i, patches.Color(255,255,255));
  }
  patches.setBrightness(150);
  patches.show();
  statusPixels.show();

  delay(200);

  // motor pins
  for (int i = 0; i < 4; i++) {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    analogWrite(pwmPins[i], 0);
    digitalWrite(dirPins[i], LOW);
  }

  // I2C init
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Serial.println("I2C on 16/17 started");

  // MPU init
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while (status != 0) { delay(100); }
  Serial.println("Calc offsets...");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("MPU ready");

  // Init APDS sensors (fast)
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (initAPDSFast(ch)) {
      Serial.print("APDS ch"); Serial.print(ch); Serial.println(" init ok");
    } else {
      Serial.print("APDS ch"); Serial.print(ch); Serial.println(" init FAILED");
    }
  }

  // Small delay then calibrate ambient on mat (place robot on green mat for calibration)
  delay(200);
  Serial.println("Calibrating ambient (place sensors over green mat)...");
  for (uint8_t ch = 0; ch < 3; ch++) {
    calibrateAmbient(ch, ambientBaseline[ch], 10);
  }
  Serial.println("Calibration complete.");
}

// ---------------- Main loop ----------------
void loop() {
  // compute ball-follow velocities (same math you used)
  int error = (1500 - ballPos()) / 10;
  float angle = (error / 150.0) * (135.0 * PI / 180.0);
  int vx = (int)(150 * cos(angle));
  int vy = (int)(150 * sin(angle));

  // Read sensors quick and stop if white detected
  bool whiteDetected = false;
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (isWhiteDetected(ch)) {
      whiteDetected = true;
      Serial.print("WHITE detected on ch ");
      Serial.println(ch);
      break;
    }
  }

  if (whiteDetected) {
    // show status (statusPixels on)
    statusPixels.setPixelColor(0, statusPixels.Color(200,200,200));
    statusPixels.setPixelColor(1, statusPixels.Color(200,200,200));
    statusPixels.show();

    // immediate stop
    omniDrive(0, 0, 0);

    // hold until line is cleared (debounced)
    unsigned long stopT = millis();
    while (millis() - stopT < 2000) {
      // if any sensor still white, stay stopped
      bool stillWhite = false;
      for (uint8_t ch = 0; ch < 3; ch++) {
        if (isWhiteDetected(ch)) { stillWhite = true; break; }
      }
      if (!stillWhite) break;
      delay(50);
    }

    // clear status LEDs
    statusPixels.setPixelColor(0, statusPixels.Color(0,0,0));
    statusPixels.setPixelColor(1, statusPixels.Color(0,0,0));
    statusPixels.show();

    // skip motion this cycle
    return;
  }

  // Normal movement with gyro stabilization
  omniDriveWithGyro(vx, vy, 0);

  delay(10);
}
