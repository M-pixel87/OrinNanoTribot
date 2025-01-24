#include <Servo.h>

// Create Servo objects for controlling the servos
Servo steeringServo; // Servo that controls the angle of the wheel
Servo motorServo;    // Servo that controls the speed of the motor

int defaultSpeed = 58; // Standard speed value is very slow and made for testing anything under 90 is backwards 
const int ledPin = 3;    // Pin to which the LED is connected
int wheeldegree;
float filteredWheelDegree = 90; // Start with the center position (90 degrees)
const float alpha = 0.1; // Smoothing factor for the EMA, adjust between 0 and 1

// Enum for different actions
enum RobotAction {
  Alignmentmv = 1,
  AvoidObstacle = 2,
  Stop = 3,
  Advance = 4
};

// Function declaration for performing actions based on the enum
void performAction(RobotAction action, int value);

void setup() {

  pinMode(ledPin, OUTPUT);
  // Attach the servos to their respective pins
  steeringServo.attach(12);
  motorServo.attach(9);

  // Initialize the steering servo to the center position (90 degrees)
  steeringServo.write(90);

  // Start serial communication at 9600 baud rate
  Serial.begin(9600);

  // Set the ESC to the neutral state
  motorServo.write(90); // 90 degrees corresponds to a neutral signal
  delay(5000);          // Wait for 5 seconds to ensure the ESC recognizes the neutral signal

  Serial.println("Setup complete. Waiting for commands...");
}

void loop() {
  // Example usage: perform different actions based on the received signal
  if (Serial.available() > 0) {
    String value = Serial.readStringUntil('\n');
    int RecievedVal = value.toInt(); //raw received value from UART pin
    int calACTION = RecievedVal / 100; // Raw value calculated to find action found in transmission
    int calACTIONAMOUNT = RecievedVal % 100 - 50; // Raw value calculated to find action amount such as the degree to which to turn the wheel

    Serial.print("Received value: ");
    Serial.println(RecievedVal);
    Serial.print("Calculated action: ");
    Serial.println(calACTION);
    Serial.print("Calculated action amount: ");
    Serial.println(calACTIONAMOUNT);

    RobotAction action = static_cast<RobotAction>(calACTION);

    switch (action) {
      case Alignmentmv:
        Serial.println("Performing Alignmentmv action...");
        performAction(Alignmentmv, calACTIONAMOUNT);
        break;
      case AvoidObstacle:
        Serial.println("Performing AvoidObstacle action...");
        performAction(AvoidObstacle, calACTIONAMOUNT);
        break;
      case Stop:
        Serial.println("Performing Stop action...");
        performAction(Stop, calACTIONAMOUNT);
        break;
      case Advance:
        Serial.println("Performing Advance action...");
        performAction(Advance, calACTIONAMOUNT);
        break;
      default:
        Serial.println("Unknown action received.");
        break;
    }
  }
}

void performAction(RobotAction action, int value) {
  switch (action) {
    case Alignmentmv:
      motorServo.write(110); // forwards at slowish speed
      wheeldegree = 90 - value; // Assuming value is offset from center (90)
      filteredWheelDegree = (alpha * wheeldegree) + ((1 - alpha) * filteredWheelDegree); //alpha: The smoothing factor for the EMA. A value of 0.1 means the filter is fairly smooth. You can adjust this value between 0 and 1 to change the level of smoothing (closer to 0 means more smoothing).
      steeringServo.write(filteredWheelDegree);
      Serial.print("Aligning wheels to: ");
      Serial.println(filteredWheelDegree);
      delay(40);
      break;

    case AvoidObstacle:
      // Perform avoid obstacle action
      {
        //int pos = map(defaultSpeed, 0, 100, 10, 180); // Assuming 'value' represents a speed percentage
        motorServo.write(115);
        steeringServo.write(115);
        digitalWrite(ledPin, HIGH);  // Turn on the LED
        Serial.println("Avoiding obstacle...");
        delay(2000);
        motorServo.write(115);
        steeringServo.write(55);
        //motorServo.write(pos);
        delay(9000);
        digitalWrite(ledPin, LOW);   // Turn off the LED
      }
      break;

    case Stop:
      // Perform stop action
      motorServo.write(90); // Stop the motor
      Serial.println("Stopping the motor.");
      break;

    case Advance:
      // Moving forward by only following the color outline in the scope of a blue bucket
      {
      // Perform Alignmentmv action
      motorServo.write(120); // forwards at slowish speed
      wheeldegree = 90 - value; // Assuming value is offset from center (90)
      filteredWheelDegree = (alpha * wheeldegree) + ((1 - alpha) * filteredWheelDegree); //alpha: The smoothing factor for the EMA. A value of 0.1 means the filter is fairly smooth. You can adjust this value between 0 and 1 to change the level of smoothing (closer to 0 means more smoothing).
      steeringServo.write(filteredWheelDegree);
      Serial.print("Aligning wheels to: ");
      Serial.println(filteredWheelDegree);
      delay(40);
      }
      break;
  }
}
