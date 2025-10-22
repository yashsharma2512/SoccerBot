int sensorPin[3] = { A0, A1, A2 };  // change if your IR sensor is on another pin
float smooth[3] = { 0, 0, 0 };      // smoothed value
float alpha = 0.01;                  // smoothing factor (0.1 = smoother, 0.3 = faster)

void setup() {
  // put your setup code here, to run once:
    for (int i = 0; i <= 2; i++)
    smooth[i] = analogRead(sensorPin[i]);  // initialize with first reading
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i <= 2; i++) {
    int raw = analogRead(sensorPin[i]);                 // read raw sensor value (0–1023)
    smooth[i] = alpha * raw + (1 - alpha) * smooth[i];  // EMA smoothing
  }

  int sum = smooth[0] + smooth[1] + smooth[2];

  float angle = 0;
  if (sum > 0) {
    angle = ((smooth[0] * -45.0) + (smooth[1] * 0.0) + (smooth[2] * 45.0)) / sum;
  }

  Serial.print((int)smooth[0]);
  Serial.print(",");
  
  Serial.print((int)smooth[1]);
  Serial.print(",");
  
  Serial.print((int)smooth[2]);
  Serial.print(",");

  Serial.println((int)angle, DEC);
}
