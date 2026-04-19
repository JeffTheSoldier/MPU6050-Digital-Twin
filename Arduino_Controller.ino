#include <Wire.h>

const int MPU_ADDR = 0x68; 
const float ALPHA = 0.98;   
const int CAL_SAMPLES = 500;

// PID Gains
float Kp = 3.50, Ki = 0.05, Kd = 2.00;

// Orientation & PID variables
float pitch = 0, roll = 0;
float pOff = 0, rOff = 0;
float int_p = 0, last_e_p = 0;
float int_r = 0, last_e_r = 0;
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission();

  // Calibration Phase
  float pSum = 0, rSum = 0;
  for(int i=0; i<CAL_SAMPLES; i++) {
    int16_t ax, ay, az;
    readRaw(ax, ay, az);
    pSum += atan2(ay, az) * 180 / PI;
    rSum += atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI;
    delay(2);
  }
  pOff = pSum / CAL_SAMPLES;
  rOff = rSum / CAL_SAMPLES;
  lastTime = micros();
}

void loop() {
  // Listen for Live Tuner Commands
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    if (type == 'P') Kp = val;
    if (type == 'I') Ki = val;
    if (type == 'D') Kd = val;
  }

  unsigned long cur = micros();
  float dt = (cur - lastTime) / 1000000.0;
  lastTime = cur;

  int16_t ax, ay, az, gx, gy, gz;
  getMPU(ax, ay, az, gx, gy, gz);

  // Sensor Fusion
  float accP = atan2(ay, az) * 180 / PI;
  float accR = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI;
  pitch = ALPHA * (pitch + (gx / 131.0) * dt) + (1.0 - ALPHA) * accP;
  roll  = ALPHA * (roll  + (gy / 131.0) * dt) + (1.0 - ALPHA) * accR;

  float curP = pitch - pOff;
  float curR = roll - rOff;

  // PID Calculations (Error = Target(0) - Current)
  float outP = computePID(0 - curP, int_p, last_e_p, dt);
  float outR = computePID(0 - curR, int_r, last_e_r, dt);

  // TELEMETRY: 7 Values
  Serial.print(curP); Serial.print(",");
  Serial.print(curR); Serial.print(",");
  Serial.print(outP); Serial.print(",");
  Serial.print(outR); Serial.print(",");
  Serial.print(Kp);   Serial.print(",");
  Serial.print(Ki);   Serial.print(",");
  Serial.println(Kd);

  while (micros() - cur < 10000); 
}

float computePID(float err, float &integ, float &lastE, float dt) {
  integ = constrain(integ + (err * dt), -100, 100); // Anti-Windup
  float deriv = (err - lastE) / dt;
  lastE = err;
  return (Kp * err) + (Ki * integ) + (Kd * deriv);
}

void getMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  ax = Wire.read()<<8|Wire.read(); ay = Wire.read()<<8|Wire.read(); az = Wire.read()<<8|Wire.read();
  Wire.read()<<8|Wire.read(); gx = Wire.read()<<8|Wire.read(); gy = Wire.read()<<8|Wire.read(); gz = Wire.read()<<8|Wire.read();
}

void readRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  ax = Wire.read()<<8|Wire.read(); ay = Wire.read()<<8|Wire.read(); az = Wire.read()<<8|Wire.read();
}
