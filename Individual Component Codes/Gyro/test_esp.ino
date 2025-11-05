/*
  ESP32 Blink
  Turns the built-in LED on for one second, then off for one second, repeatedly.
  Most ESP32 boards have an on-board LED connected to GPIO pin 2.
*/

// The LED_BUILTIN variable is a constant that holds the pin number.
// For most ESP32 dev boards, this is pin 2.
#define LED_BUILTIN 2
const int ledPin = LED_BUILTIN; 

// The setup function runs once when you press reset or power the board
void setup() {
  // Initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
}

// The loop function runs over and over again forever
void loop() {
  digitalWrite(ledPin, HIGH);   // Turn the LED on by making the voltage HIGH
  delay(1000);                  // Wait for a second (1000 milliseconds)
  digitalWrite(ledPin, LOW);    // Turn the LED off by making the voltage LOW
  delay(1000);                  // Wait for a second
}
