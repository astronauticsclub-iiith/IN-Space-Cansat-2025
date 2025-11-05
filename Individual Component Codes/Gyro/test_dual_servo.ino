#include <ESP32Servo.h>

// Create two servo objects
Servo myServo1;
Servo myServo2;

// Define the GPIO pins for the two servos
// You can use any PWM-capable GPIO pins.
const int servoPin1 = 18; // Servo 1 on GPIO 18
const int servoPin2 = 19; // Servo 2 on GPIO 19

void setup() {
  // Start the serial monitor (optional, for debugging)
  Serial.begin(115200);
  Serial.println("Two-servo one-time move test");

  // Attach the servo objects to their respective pins
  myServo1.attach(servoPin1);
  myServo2.attach(servoPin2);

  // --- Start of logic ---

  // 1. Set both servos to their initial position (0 degrees)
  Serial.println("Setting both servos to 0 degrees...");
  myServo1.write(0);
  myServo2.write(0);

  // 2. Wait for 2 seconds (2000 milliseconds)
  Serial.println("Waiting for 2 seconds...");
  delay(2000);

  // 3. Rotate both servos to 90 degrees
  Serial.println("Rotating both servos to 90 degrees...");
  myServo1.write(90);
  myServo2.write(90);
  
  Serial.println("Move complete. Nothing further will happen.");
  // --- End of logic ---
}

void loop() {
  // The loop is empty because the action only happens once.
  // Both servos will remain at 90 degrees.
}
