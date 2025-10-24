#include <Servo.h>

// Define servo objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo grabberServo;

// Servo pin assignments (match servocontroller.py)
const int basePin = 2;
const int shoulderPin = 3;
const int elbowPin = 4;
const int grabberPin = 5;

// Buffer for incoming serial data
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);

  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  grabberServo.attach(grabberPin);

  // Set initial positions
  baseServo.write(60);
  shoulderServo.write(110);
  elbowServo.write(10);
  grabberServo.write(100);

  Serial.println("Ready");
}

void loop() {
  // Check if a complete command has been received
  if (stringComplete) {
    parseAndExecuteCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

// SerialEvent() runs automatically when new serial data arrives
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void parseAndExecuteCommand(String command) {
  // Expected format: B120,S100,E80,G130
  int b = getValue(command, 'B');
  int s = getValue(command, 'S');
  int e = getValue(command, 'E');
  int g = getValue(command, 'G');

  if (b >= 0) baseServo.write(b);
  if (s >= 0) shoulderServo.write(s);
  if (e >= 0) elbowServo.write(e);
  if (g >= 0) grabberServo.write(g);
}

int getValue(String data, char prefix) {
  int index = data.indexOf(prefix);
  if (index == -1) return -1;

  int endIndex = data.indexOf(',', index);
  if (endIndex == -1) endIndex = data.length();

  String valueStr = data.substring(index + 1, endIndex);
  return valueStr.toInt();
}
