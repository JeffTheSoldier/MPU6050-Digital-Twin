import processing.serial.*;

Serial myPort;
float pitch, roll, pidP, pidR, Kp, Ki, Kd;

void setup() {
  size(1200, 800, P3D);
  myPort = new Serial(this, "COM4", 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(15, 15, 22);
  lights();
  
  // HUD on Right Side
  int xM = width - 350;
  fill(255); textSize(22); text("HIL DUAL-AXIS STATION", xM, 50);
  textSize(16);
  fill(0, 200, 255); text("Kp [Q/A]: " + nfc(Kp, 2), xM, 100);
  fill(0, 255, 150); text("Ki [W/S]: " + nfc(Ki, 3), xM, 130);
  fill(255, 150, 0); text("Kd [E/D]: " + nfc(Kd, 2), xM, 160);
  fill(200);
  text("Pitch: " + nfc(pitch, 2) + "°", xM, 210);
  text("Roll: " + nfc(roll, 2) + "°", xM, 240);

  // Draw 3D Model
  pushMatrix();
  translate(width/2, height/2, 0);
  rotateX(radians(-pitch));
  rotateZ(radians(roll));

  // X-Frame Arms
  stroke(100); strokeWeight(10);
  line(-150, 0, -150, 150, 0, 150); line(150, 0, -150, -150, 0, 150);
  
  // Motor Mixing Visualization
  drawMotor(-120, -120, pidP + pidR, "FL"); // Front-Left
  drawMotor( 120, -120, pidP - pidR, "FR"); // Front-Right
  drawMotor(-120,  120, -pidP + pidR, "BL"); // Back-Left
  drawMotor( 120,  120, -pidP - pidR, "BR"); // Back-Right
  popMatrix();
}

void drawMotor(float x, float z, float pwr, String label) {
  pushMatrix();
  translate(x, -20, z);
  float intensity = map(pwr, -300, 300, 0, 255);
  fill(intensity, 255 - abs(intensity), 100);
  ellipse(0, 0, 70, 70);
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
