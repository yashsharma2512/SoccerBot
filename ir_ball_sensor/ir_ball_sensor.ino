int sensorPin[6] = { A0, A1, A2, A3 };  // change if your IR sensor is on another pin
float smooth[6] = { 0, 0, 0, 0 };       // smoothed value
float alpha = 0.01;                     // smoothing factor (0.1 = smoother, 0.3 = faster)

int lastPosition = 0;

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i <= 3; i++)
    smooth[i] = analogRead(sensorPin[i]);  // initialize with first reading
  Serial.begin(115200);
}

void loop() {
  bool online = false;
  long weightedSum = 0;
  long total = 0;
  int position = 0;

  // put your main code here, to run repeatedly:
  for (int i = 0; i <= 3; i++) {
    int raw = analogRead(sensorPin[i]);                 // read raw sensor value (0–1023)
    smooth[i] = alpha * raw + (1 - alpha) * smooth[i];  // EMA smoothing
    if (smooth[i] > 80){
      online = true;
      weightedSum += (long)smooth[i] * (i * 1000);
      total += smooth[i];
    }
  }

  if (total == 0) {
    // No line detected
    if (lastPosition > 1500)
      position = 3000;  // assume it exited right
    else
      position = 0;  // exactly center fallback
  } else {
    position = weightedSum / total;

    // Optional smoothing (low-pass filter)
    position = (lastPosition * 5 + position * 5) / 10;

    lastPosition = position;
  }

  Serial.print(online, DEC);
  Serial.print(",");
  for (int i = 0; i <= 3; i++) {
    char buffer[5];                           // 4 digits + null terminator
    sprintf(buffer, "%04d", (int)smooth[i]);  // Format with leading zeros (0000 to 1000)
    Serial.print(buffer);
    Serial.print(",");
  }
  Serial.println((int)position, DEC);
}
