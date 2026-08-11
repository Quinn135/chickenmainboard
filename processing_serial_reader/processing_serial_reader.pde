import processing.serial.*;

Serial port;

void setup() {
  size(400, 200);

  // print every port processing can see - use this to find your nucleo's
  // st-link virtual com port if the guessed index below is wrong
  println(Serial.list());

  // guess: last port in the list is usually the most recently plugged in
  // device. change this index (or hardcode the exact name from the list
  // above, e.g. "/dev/ttyACM0" or "COM5") if it doesn't grab the right one
  String portName = Serial.list()[Serial.list().length - 1];

  port = new Serial(this, portName, 115200); // match Serial.begin() on the stm32
  port.bufferUntil('\n'); // fire serialEvent once per full line
}

void draw() {
  background(0);
  fill(255);
  text("check the console for incoming serial lines", 10, 20);
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;

  line = trim(line); // drop trailing \r / whitespace
  println(line);

  // your stm32 sketch prints "ax,ay,az,gx,gy,gz" - split it out if you
  // want the individual values instead of just the raw line
  String[] values = split(line, ',');
  if (values.length == 6) {
    float ax = float(values[0]);
    float ay = float(values[1]);
    float az = float(values[2]);
    float gx = float(values[3]);
    float gy = float(values[4]);
    float gz = float(values[5]);
    // do something with ax..gz here, or just leave the println above
  }
}
