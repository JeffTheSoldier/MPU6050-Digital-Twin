import processing.serial.*;

Serial myPort;
float pitch, roll, pidP, pidR, Kp, Ki, Kd;

void setup() {
  size(1200, 800, P3D);
  myPort = new Serial(this, "COM4", 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(10, 10, 15);
  lights();
  
  // Right-side HUD
  drawHUD();

  pushMatrix();
  translate(width/2, height/2, 0);
  rotateX(radians(-pitch));
  rotateZ(radians(roll));

  // Drone Frame
  stroke(150); strokeWeight(10);
  line(-150, 0, -150, 150, 0, 150); 
  line(150, 0, -150, -150, 0, 150);

  // --- MOTOR MIXING LOGIC (The Fix) ---
  // In an X-Frame, each motor combines P and R
  float thrust = 150; // Baseline power
  float mFR = thrust - pidP - pidR; // Front Right
  float mFL = thrust - pidP + pidR; // Front Left
  float mBR = thrust + pidP - pidR; // Back Right
  float mBL = thrust + pidP + pidR; // Back Left

  drawMotor( 120, -120, mFR, "FR");
  drawMotor(-120, -120, mFL, "FL");
  drawMotor( 120,  120, mBR, "BR");
  drawMotor(-120,  120, mBL, "BL");
  popMatrix();
}

void drawHUD() {
  hint(DISABLE_DEPTH_TEST);
  int xM = width - 350;
  fill(255); textSize(22); text("HIL X-FRAME TUNER", xM, 50);
  textSize(16);
  fill(0, 200, 255); text("Kp [Q/A]: " + nfc(Kp, 2), xM, 100);
  fill(0, 255, 150); text("Ki [W/S]: " + nfc(Ki, 3), xM, 130);
  fill(255, 150, 0); text("Kd [E/D]: " + nfc(Kd, 2), xM, 160);
  fill(200);
  text("Pitch: " + nfc(pitch, 2) + "°", xM, 210);
  text("Roll: " + nfc(roll, 2) + "°", xM, 240);
  hint(ENABLE_DEPTH_TEST);
}

void drawMotor(float x, float z, float pwr, String label) {
  pushMatrix();
  translate(x, -20, z);
  
  // Power Visualization: Red = High Thrust, Green = Low
  float r = map(pwr, 0, 300, 0, 255);
  float g = map(pwr, 0, 300, 255, 0);
  fill(r, g, 50);
  
  ellipse(0, 0, 80, 80);
  fill(255); textAlign(CENTER); text(label, 0, 5);
  popMatrix();
}

void keyPressed() {
  if (key == 'q') myPort.write("P" + (Kp + 0.1) + "\n");
  if (key == 'a') myPort.write("P" + (Kp - 0.1) + "\n");
  if (key == 'w') myPort.write("I" + (Ki + 0.005) + "\n");
  if (key == 's') myPort.write("I" + (Ki - 0.005) + "\n");
  if (key == 'e') myPort.write("D" + (Kd + 0.1) + "\n");
  if (key == 'd') myPort.write("D" + (Kd - 0.1) + "\n");
}

void serialEvent(Serial myPort) {
  String in = myPort.readStringUntil('\n');
  if (in != null) {
    float[] data = float(split(trim(in), ','));
    if (data.length == 7) {
      pitch = data[0]; roll = data[1];
      pidP = data[2]; pidR = data[3];
      Kp = data[4]; Ki = data[5]; Kd = data[6];
    }
  }
}
