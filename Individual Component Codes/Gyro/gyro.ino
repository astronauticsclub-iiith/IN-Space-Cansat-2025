#include <Wire.h>
#include "ICM_20948.h" // IMU Library
#include <math.h>     // Required for atan2 and sqrt

// I2C Address for the sensor
#define AD0_VAL 1

// I2C pins for IMU (ICM-20948)
#define I2C_SDA 21 
#define I2C_SCL 22 

ICM_20948_I2C myIMU; // Create an ICM_20948_I2C object

// --- Slew Rate Limiter Class ---
// This class limits the rate of change of a signal.
class SlewRateLimiter {
public:
  // Constructor: Initializes the limiter with a maximum rate of change.
  // maxRate: The maximum change in output units per second (e.g., PWM units/sec).
  SlewRateLimiter(double maxRate) {
    _maxRateOfChange = maxRate;
    _previousOutput = 0.0;
    _previousTime = micros(); // Use micros() for higher resolution
  }

  // Calculates the next rate-limited output value.
  // input: The raw, unlimited input value from the PID controller.
  // returns: The new output value, ramped if necessary.
  double calculate(double input) {
    unsigned long currentTime = micros();
    double elapsedTime = (double)(currentTime - _previousTime) / 1000000.0; // in seconds
    _previousTime = currentTime;

    // Calculate the maximum allowed change for this time interval
    double maxDelta = _maxRateOfChange * elapsedTime;

    // Calculate the desired change
    double delta = input - _previousOutput;

    // Constrain the change to the maximum allowed rate
    if (delta > maxDelta) {
      delta = maxDelta;
    } else if (delta < -maxDelta) {
      delta = -maxDelta;
    }

    // Apply the limited change to the previous output
    _previousOutput += delta;
    
    return _previousOutput;
  }
  
  // Resets the limiter's state to a specific value.
  void reset(double value) {
    _previousOutput = value;
    _previousTime = micros();
  }

private:
  double _maxRateOfChange;
  double _previousOutput;
  unsigned long _previousTime;
};

// --- Motor Dead-Zone Configuration ---
// This is the minimum speed threshold to overcome motor stiction.
#define DEAD_ZONE_THRESHOLD 38 // Approx 15% of 255

// --- Motor Pin Assignments ---
#define M1_IN1 34
#define M1_IN2 35
#define M1_PWM 32
#define M2_IN1 19
#define M2_IN2 18
#define M2_PWM 5
#define M3_IN1 2
#define M3_IN2 15
#define M3_PWM 4

const int M1_PWM_CHANNEL = 0, M2_PWM_CHANNEL = 1, M3_PWM_CHANNEL = 2;
const int PWM_FREQ = 5000, PWM_RESOLUTION = 8;

// --- PID Controller Variables ---
double Kp = 10.0, Ki = 0.1, Kd = 0.8; // You MUST re-tune these values!
double pitch_setpoint = 0.0, roll_setpoint = 0.0;
double pitch_error, roll_error, prev_pitch_error = 0, prev_roll_error = 0;
double pitch_integral = 0, roll_integral = 0;
double pitch_derivative, roll_derivative;
unsigned long lastTime = 0;
double elapsedTime;

// --- Slew Rate Limiter Configuration ---
#define MAX_RATE_OF_CHANGE 20.0 // PWM units per second. TUNE THIS VALUE!
SlewRateLimiter pitchLimiter(MAX_RATE_OF_CHANGE);
SlewRateLimiter rollLimiter(MAX_RATE_OF_CHANGE);

// --- NEW: Reversal Delay State Management ---
// This structure holds the state for the reversal delay logic for a single motor.
struct MotorControlState {
  int lastDirection = 0; // 0: stopped, 1: forward, -1: backward
  unsigned long reversalTimestamp = 0;
  bool delayActive = false;
};

MotorControlState pitchMotorState;
MotorControlState rollMotorState;

// Forward declaration of functions
void setMotorSpeed(int motor, int speed);
int getDelayedSpeed(int targetSpeed, MotorControlState &state);

// --- Dead-Zone Application Function ---
int applyDeadZone(int speed, int threshold) {
  if (speed == 0) {
    return 0;
  }
  if (abs(speed) < threshold) {
    return (speed > 0) ? threshold : -threshold;
  }
  return speed;
}

// --- MOTOR TEST FUNCTION (Unchanged) ---
void testMotors() {
  Serial.println("\n--- Starting Motor Test ---");
  const int testSpeed = 180;
  const int testDelay = 500;
  const int pauseDelay = 250;
  delay(1000);
  Serial.println("Testing Motor 1: Forward..."); setMotorSpeed(1, testSpeed); delay(testDelay);
  Serial.println("Testing Motor 1: Backward..."); setMotorSpeed(1, -testSpeed); delay(testDelay);
  setMotorSpeed(1, 0); delay(pauseDelay);
  Serial.println("Testing Motor 2: Forward..."); setMotorSpeed(2, testSpeed); delay(testDelay);
  Serial.println("Testing Motor 2: Backward..."); setMotorSpeed(2, -testSpeed); delay(testDelay);
  setMotorSpeed(2, 0); delay(pauseDelay);
  Serial.println("Testing Motor 3: Forward..."); setMotorSpeed(3, testSpeed); delay(testDelay);
  Serial.println("Testing Motor 3: Backward..."); setMotorSpeed(3, -testSpeed); delay(testDelay);
  setMotorSpeed(3, 0); delay(pauseDelay);
  Serial.println("--- Motor Test Complete ---\n");
  delay(1000);
}


void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.println("Attempting to connect to ICM20948...");
  bool initialized = false;
  while (!initialized) {
    myIMU.begin(Wire, AD0_VAL);
    if (myIMU.status!= ICM_20948_Stat_Ok) {
      Serial.print("!!! ICM20948 CONNECTION FAILED: ");
      Serial.println(myIMU.statusString());
      delay(500);
    } else {
      initialized = true;
    }
  }
  Serial.println("ICM20948 connection successful.");

  // --- Configure Motors & PWM ---
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);

  // --- NEW PWM SETUP for ESP32 Core v3.x ---
  ledcAttachChannel(M1_PWM, PWM_FREQ, PWM_RESOLUTION, M1_PWM_CHANNEL);
  ledcAttachChannel(M2_PWM, PWM_FREQ, PWM_RESOLUTION, M2_PWM_CHANNEL);
  ledcAttachChannel(M3_PWM, PWM_FREQ, PWM_RESOLUTION, M3_PWM_CHANNEL);

  lastTime = millis();
  Serial.println("Initialization complete. Starting control loop.");
}

void loop() {
  if (myIMU.dataReady()) {
    myIMU.getAGMT();

    // --- Time Calculation ---
    unsigned long currentTime = millis();
    elapsedTime = (double)(currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // --- Pitch & Roll Calculation (from Accelerometer data) ---
    float ax = myIMU.accX();
    float ay = myIMU.accY();
    float az = myIMU.accZ();
    float current_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    float current_roll  = atan2(ay, az) * 180.0 / M_PI;

    // --- PID Calculations (Raw Output) ---
    pitch_error = pitch_setpoint - current_pitch;
    roll_error = roll_setpoint - current_roll;
    pitch_integral += pitch_error * elapsedTime;
    roll_integral += roll_error * elapsedTime;
    pitch_derivative = (pitch_error - prev_pitch_error) / elapsedTime;
    roll_derivative = (roll_error - prev_roll_error) / elapsedTime;
    
    double raw_pitch_output = (Kp * pitch_error) + (Ki * pitch_integral) + (Kd * pitch_derivative);
    double raw_roll_output = (Kp * roll_error) + (Ki * roll_integral) + (Kd * roll_derivative);
    
    prev_pitch_error = pitch_error;
    prev_roll_error = roll_error;

    // --- Apply Slew Rate Limiting ---
    double limited_pitch_output = pitchLimiter.calculate(raw_pitch_output);
    double limited_roll_output = rollLimiter.calculate(raw_roll_output);

    // Constrain the output to the PWM range
    int motor_pitch_speed = constrain(limited_pitch_output, -255, 255);
    int motor_roll_speed = constrain(limited_roll_output, -255, 255);

    // --- NEW: Apply Directional Reversal Delay ---
    int delayed_pitch_speed = getDelayedSpeed(motor_pitch_speed, pitchMotorState);
    int delayed_roll_speed = getDelayedSpeed(motor_roll_speed, rollMotorState);

    // --- Apply Dead Zone / Minimum Speed ---
    int final_pitch_speed = applyDeadZone(delayed_pitch_speed, DEAD_ZONE_THRESHOLD);
    int final_roll_speed = applyDeadZone(delayed_roll_speed, DEAD_ZONE_THRESHOLD);

    // --- Motor Control ---
    setMotorSpeed(1, final_pitch_speed);
    setMotorSpeed(2, final_roll_speed);
    setMotorSpeed(3, 0); 
    
    // --- Serial Output ---
    Serial.print("Pitch: "); Serial.print(current_pitch, 2);
    Serial.print(" | Roll: "); Serial.print(current_roll, 2);
    Serial.print(" | P_Out: "); Serial.print(final_pitch_speed);
    Serial.print(" | R_Out: "); Serial.println(final_roll_speed);
  }
}

// --- NEW: Reversal Delay Logic ---
// This function enforces a delay when motor direction is reversed.
int getDelayedSpeed(int targetSpeed, MotorControlState &state) {
    // Determine the direction of the new command
    int newDirection = 0;
    if (targetSpeed > 0) newDirection = 1;
    else if (targetSpeed < 0) newDirection = -1;

    // Check if a reversal is being commanded.
    // A reversal occurs if we change from forward to backward (1 to -1), or vice-versa.
    if (newDirection != 0 && newDirection != state.lastDirection && state.lastDirection != 0) {
        // If a delay is not already active, start one.
        if (!state.delayActive) {
            state.delayActive = true;
            state.reversalTimestamp = millis();
        }
    }

    // If a delay is active, check if it should end.
    if (state.delayActive) {
        unsigned long timeSinceReversal = millis() - state.reversalTimestamp;
        // Conditions to end the delay: 2 seconds have passed AND the new command is strong enough.
        if (timeSinceReversal > 2000 && abs(targetSpeed) > DEAD_ZONE_THRESHOLD) {
            state.delayActive = false; // End the delay
        } else {
            // Delay is still active, command the motor to stop.
            state.lastDirection = 0; // We are actively holding it at zero
            return 0;
        }
    }

    // If we reach here, the delay is not active. The motor can move.
    state.lastDirection = newDirection;
    return targetSpeed;
}

// --- MOTOR SPEED FUNCTION (Updated for ESP32 Core v3.x) ---
void setMotorSpeed(int motor, int speed) {
  int pwmVal = abs(speed);
  if (pwmVal > 255) pwmVal = 255; 
  switch (motor) {
    case 1:
      if (speed > 0) { digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW); } else if (speed < 0) { digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH); } else { digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW); }
      ledcWrite(M1_PWM_CHANNEL, pwmVal); break;
    case 2:
      if (speed > 0) { digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW); } else if (speed < 0) { digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, HIGH); } else { digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, LOW); }
      ledcWrite(M2_PWM_CHANNEL, pwmVal); break;
    case 3:
      if (speed > 0) { digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW); } else if (speed < 0) { digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, HIGH); } else { digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, LOW); }
      ledcWrite(M3_PWM_CHANNEL, pwmVal); break;
  }
}
