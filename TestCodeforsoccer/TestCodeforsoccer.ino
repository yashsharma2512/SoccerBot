/*
  Full integration (updated):
  - Cytron MOTION 2350 Pro (RP2350)
  - Motors mapped to GP8..GP15 (M1..M4)
  - MPU6050 on Wire (SDA=GP16, SCL=GP17)
  - PCA9548A at 0x70 on same I2C bus
  - APDS-9960 sensors on PCA channels:
      Front -> channel 0
      Right -> channel 1
      Back  -> channel 2
  - Stop immediately when white is detected (to avoid leaving field)
  - Slow-down zone when approaching white
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

// ---------------- Analog ball sensors ----------------
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
  if (!writeAPDSReg(APDS_REG_ENABLE, 0x03)) return false; // PON + AEN
  if (!writeAPDSReg(APDS_REG_ATIME, 246)) return false;    // short integration ~28ms
  if (!writeAPDSReg(APDS_REG_CONTROL, 0x03)) return false; // max gain
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

  Wire.beginTransmission(APDS9960_ADDR);
  Wire.write(APDS_REG_AICLEAR);
  Wire.endTransmission();
  return true;
}

bool readAPDSAvg(uint8_t channel, uint16_t &r_out, uint16_t &g_out, uint16_t &b_out, uint16_t &c_out, int samples = 3, int pauseMs = 6) {
  uint32_t rsum = 0, gsum = 0, bsum = 0, csum = 0;
  uint16_t r,g,b,c;
  for (int i = 0; i < samples; i++) {
    if (!readAPDSRaw(channel, r,g,b,c)) return false;
    rsum += r; gsum += g; bsum += b; csum += c;
    delay(pauseMs);
  }
  r_out = rsum / samples;
  g_out = gsum / samples;
  b_out = bsum / samples;
  c_out = csum / samples;
  return true;
}

void calibrateAmbient(uint8_t channel, uint16_t &baseline, int samples = 10) {
  uint32_t sum = 0;
  uint16_t r,g,b,c;
  int collected = 0;
  for (int i = 0; i < samples; i++) {
    if (readAPDSRaw(channel, r,g,b,c)) { sum += c; collected++; } 
    else { delay(20); i--; continue; }
    delay(30);
  }
  if (collected > 0) baseline = (uint16_t)(sum / collected);
  Serial.print("Calib ch"); Serial.print(channel); Serial.print(" baseline C="); Serial.println(baseline);
}

// ---------------- Motor functions ----------------
void setMotor(int motorIndex, int speed) {
  speed = constrain(speed, -255, 255);
  int pwmPin = pwmPins[motorIndex];
  int dirPin = dirPins[motorIndex];
  if (speed==0) { analogWrite(pwmPin,0); digitalWrite(dirPin,LOW); }
  else if (speed>0) { digitalWrite(dirPin,LOW); analogWrite(pwmPin,abs(speed)); }
  else { digitalWrite(dirPin,HIGH); analogWrite(pwmPin,abs(speed)); }
}

void omniDrive(int vx, int vy, int wz) {
  int w1 = vy + vx - wz; 
  int w2 = vy - vx + wz; 
  int w3 = vy + vx + wz; 
  int w4 = vy - vx - wz;
  setMotor(0, constrain(w1,-255,255));
  setMotor(1, constrain(w2,-255,255));
  setMotor(2, constrain(w3,-255,255));
  setMotor(3, constrain(w4,-255,255));
}

void omniDriveWithGyro(int vx, int vy, int vz) {
  mpu.update();
  float gyroAngle = mpu.getAngleZ();
  float error = targetAngle - gyroAngle;
  if(error>180) error-=360;
  if(error<-180) error+=360;
  int wz = (int)(Kp*error); wz=constrain(wz,-100,100);

  int w1 = vx + vy - wz;
  int w2 = vx - vy + wz;
  int w3 = vx + vy + wz;
  int w4 = vx - vy - wz;
  setMotor(0, constrain(w1,-255,255));
  setMotor(1, constrain(w2,-255,255));
  setMotor(2, constrain(w3,-255,255));
  setMotor(3, constrain(w4,-255,255));
}

// ---------------- Ball sensor ----------------
int ballPos() {
  long weightedSum=0, total=0; int position=0;
  for(int i=0;i<=3;i++) {
    int raw=analogRead(sensorPin[i]);
    smooth[i]=alpha*raw+(1-alpha)*smooth[i];
    if(smooth[i]>120) { weightedSum+=(long)smooth[i]*(i*1000); total+=smooth[i]; }
  }
  if(total==0) { return (lastPosition>1500)?3000:0; }
  position = weightedSum/total;
  lastPosition=position;
  return position;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  statusPixels.begin(); patches.begin();
  statusPixels.clear(); patches.clear();
  for(int i=0;i<PATCH_N;i++) patches.setPixelColor(i, patches.Color(255,255,255));
  patches.setBrightness(150); patches.show(); statusPixels.show();
  delay(200);

  for(int i=0;i<4;i++) { pinMode(pwmPins[i],OUTPUT); pinMode(dirPins[i],OUTPUT); analogWrite(pwmPins[i],0); digitalWrite(dirPins[i],LOW); }

  Wire.setSDA(SDA_PIN); Wire.setSCL(SCL_PIN); Wire.begin();
  Serial.println("I2C started");

  byte status = mpu.begin(); Serial.print("MPU6050 status: "); Serial.println(status);
  while(status!=0){delay(100);} 
  delay(1000); mpu.calcOffsets(); Serial.println("MPU ready");

  for(uint8_t ch=0; ch<3; ch++) {
    if(initAPDSFast(ch)) Serial.print("APDS ch"+String(ch)+" init ok\n");
    else Serial.print("APDS ch"+String(ch)+" init FAILED\n");
  }

  delay(200);
  Serial.println("Calibrating ambient...");
  for(uint8_t ch=0; ch<3; ch++) calibrateAmbient(ch, ambientBaseline[ch],10);
  Serial.println("Calibration complete.");
}

// ---------------- Main loop ----------------
void loop() {
  int error=(1500-ballPos())/10;
  float angle=(error/150.0)*(135.0*PI/180.0);
  int vx=(int)(150*cos(angle));
  int vy=(int)(150*sin(angle));

  bool whiteDetected=false;
  float slowFactor = 1.0;

  // Check sensors
  for(uint8_t ch=0;ch<3;ch++){
    uint16_t r,g,b,c;
    if(readAPDSAvg(ch,r,g,b,c,3,6)){
      uint32_t dynThresh=max((uint32_t)(ambientBaseline[ch]*1.6f),500u);
      if(c>dynThresh) { whiteDetected=true; slowFactor=0.0; break; }
      else if(c>(dynThresh*0.8)) { // slow-down zone
        float prox = (float)c/dynThresh;
        if(prox>slowFactor) slowFactor = prox;
      }
    }
  }

  // Apply slow-down
  int vx_slow = vx * (1.0 - slowFactor);
  int vy_slow = vy * (1.0 - slowFactor);

  // Update NeoPixel warning for slow-down
  if(slowFactor>0.0) {
    int brightness = int(150 * slowFactor);
    statusPixels.fill(statusPixels.Color(brightness,0,0));
  } else {
    statusPixels.fill(statusPixels.Color(0,0,0));
  }
  statusPixels.show();

  if(whiteDetected){
    omniDrive(0,0,0);
    delay(500);
    return;
  }

  omniDriveWithGyro(vx_slow, vy_slow, 0);
  delay(10);
}
