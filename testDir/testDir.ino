#include <Servo.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

LIDARLite_v4LED myLidarLite;

// Constants and pins
const int ledPin = 3;
const int steeringPin = 12;
const int motorPin = 9;
const int lidarLiteAddress = 0x62;
int defaultSpeed = 58; // Standard speed value (very slow, for testing)

// Variables
int wheeldegree;
float filteredWheelDegree = 90; // Start at center (90 degrees)
const float alpha = 0.1; // Smoothing factor for the EMA, adjust between 0 and 1
float distance;

// Enum for different actions
enum class RobotAction : int {
  Alignmentmv = 1,
  AvoidObstacle,
  Stop,
  Advance
};

// Servo objects
Servo steeringServo;
Servo motorServo;

void setup() {
  pinMode(ledPin, OUTPUT);
  steeringServo.attach(steeringPin);
  motorServo.attach(motorPin);

  steeringServo.write(90); // Set the wheel to center position

  Serial.begin(115200);
  Wire.begin();

  // Configure the LIDARLite device
  myLidarLite.configure(0);
  Serial.println("Scanning I2C bus...");
  
  // Ensure LIDAR is connected
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at address 0x");
      Serial.println(address, HEX);
    }
  }

  motorServo.write(90);  // Set motor to neutral (stop)
  delay(5000); // Allow time for the ESC to recognize the neutral signal

  Serial.println("Setup complete. Waiting for commands...");
}

void loop() {
  // Read distance from LIDAR Lite if available
  if (myLidarLite.getBusyFlag() == 0) {
    myLidarLite.takeRange();
    distance = myLidarLite.readDistance();
  }

  // Print LIDAR distance
  Serial.print("Sensor distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check for incoming serial data
  if (Serial.available() > 0) {
    String value = Serial.readStringUntil('\n');
    int receivedVal = value.toInt(); // Raw value received from UART
    int actionCode = receivedVal / 100; // Extract action code
    int actionAmount = (receivedVal % 100) - 50; // Extract action amount (adjust to -50 to +50 range)

    Serial.print("Received value: ");
    Serial.println(receivedVal);
    Serial.print("Action: ");
    Serial.println(actionCode);
    Serial.print("Action amount: ");
    Serial.println(actionAmount);

    // Map the action to RobotAction enum
    RobotAction action = static_cast<RobotAction>(actionCode);

    // Perform the corresponding action
    performAction(action, actionAmount);
  }

  delay(100); // Add a small delay for stability
}

void performAction(RobotAction action, int value) {
  switch (action) {
    case RobotAction::Alignmentmv:
    case RobotAction::Advance:
      // Move motor and adjust steering based on action
      motorServo.write(120); // Move motor forward at low speed
      adjustSteering(value);
      break;

    case RobotAction::AvoidObstacle:
      performAvoidObstacle();
      break;

    case RobotAction::Stop:
      motorServo.write(90); // Stop motor
      Serial.println("Stopping the motor.");
      break;
  }
}

void adjustSteering(int value) {
  // Update wheel degree and apply smoothing
  wheeldegree = 90 - value; 
  filteredWheelDegree = (alpha * wheeldegree) + ((1 - alpha) * filteredWheelDegree);
  steeringServo.write(filteredWheelDegree);
  Serial.print("Steering to: ");
  Serial.println(filteredWheelDegree);
  delay(40); // Small delay to allow for servo response
}

void performAvoidObstacle() {
  motorServo.write(115); // Move forward at moderate speed
  steeringServo.write(115); // Turn right
  digitalWrite(ledPin, HIGH); // Turn on LED
  Serial.println("Avoiding obstacle...");

  delay(2000); // Keep the movement for 2 seconds

  steeringServo.write(55); // Turn left
  delay(9000); // Keep turning left for 9 seconds
  digitalWrite(ledPin, LOW); // Turn off LED
}
