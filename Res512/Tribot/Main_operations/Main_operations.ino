//this program takes in the error of the camera and uses that that to alter the pwm of the motors and straightens the bot
// Define pin numbers for motors
#define DIR1 4
#define PWM1 5
#define DIR2 7
#define PWM2 6

const int buttonPin = 2;  
int motor1 = 0;
int motor2 = 0;
unsigned long timer = 0;
bool motorsRunning = false;
int buttonState = HIGH;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up
  Serial.begin(9600);

  // Set pin modes
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);


}

void loop() {
  buttonState = digitalRead(buttonPin); // Read the state of the button
  Serial.print("Button State: ");
  Serial.println(buttonState); // Print the button state for debugging

  if (buttonState == LOW && !motorsRunning) { // LOW means the button is pressed
    motorsRunning = true;
    timer = millis(); // Start the timer
  }

  if (motorsRunning) {
    while (Serial.available() > 0) {
      // Read the incoming byte as a string
      String value = Serial.readStringUntil('\n');
      int intValue = value.toInt();

      // Set desired setpoint for motors
      if (intValue < 0) {
        motor1 = 30 ; 
        motor2 = 30 + abs(intValue); // Use abs to ensure subtraction is correct

      
      } else if (intValue > 0) {
        motor1 = 30 + intValue;
        motor2 = 30 ;
        
      } else { // When intValue is 0, set both motors to 10
        motor1 = 10;
        motor2 = 10;
      }

      // Print motor values for debugging
      Serial.print("Motor 1 value: ");
      Serial.println(motor1);
      Serial.print("Motor 2 value: ");
      Serial.println(motor2);

      // Move motor 1
      digitalWrite(DIR1, HIGH);
      analogWrite(PWM1, motor1);

      // Control motor 2
      digitalWrite(DIR2, HIGH);
      analogWrite(PWM2, motor2);
      delay(200); // Small delay to ensure motors respond
    }

    // Check if 5 seconds have passed
    if (millis() - timer >= 5000) {
      motorsRunning = false;

      // Stop the motors
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
    }
  }
}
