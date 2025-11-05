#include <Wire.h>
#include "ICM_20948.h"     // SparkFun ICM-20948 IMU Library
#include <ESP32Servo.h>    // Library for controlling ESCs

// --- PIN DEFINITIONS ---
#define MOTOR1_PIN 4
#define MOTOR2_PIN 5

// --- SENSOR & MOTOR OBJECTS ---
ICM_20948_I2C myICM;
Servo motor1;
Servo motor2;

// --- PID CONTROL VARIABLES (FOR YAW RATE) ---
float kp = 1.4;
float ki = 0.15;
float kd = 0.15;

float yaw_rate_setpoint = 0.0;
float pid_output = 0;
float error = 0;
float last_error = 0;
float integral = 0;
float derivative = 0;

// --- IMU & FILTER VARIABLES ---
float gyro_z_offset = 0.0;
float gyro_z_filtered = 0.0;   // Filtered gyroscope Z-axis value
float alpha = 0.1;             // Low-pass filter constant (0.05–0.3 typical)
unsigned long last_time = 0;

// --- MOTOR CONTROL VARIABLES ---
int base_throttle = 1000;
int motor1_speed, motor2_speed;
const int MIN_THROTTLE = 1000;
const int MAX_THROTTLE = 1300;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("--- Yaw Rate Stabilization System ---");

  // --- IMU INITIALIZATION ---
  Wire.begin(21, 22);
  Wire.setClock(400000);

  myICM.begin(Wire, 0x69);
  // myICM.begin(Wire, 0x68);
  if (myICM.status != ICM_20948_Stat_Ok) {
    Serial.println("IMU initialization failed! Halting.");
    while (1) delay(10);
  }
  Serial.println("IMU Initialized.");

  calibrate_gyro();

  // --- MOTOR (ESC) INITIALIZATION & ARMING ---
  Serial.println("Initializing ESCs. Ensure they are powered on.");
  motor1.attach(MOTOR1_PIN, 1000, 2000);
  motor2.attach(MOTOR2_PIN, 1000, 2000);

  Serial.println("Arming ESCs: Sending min throttle...");
  motor1.writeMicroseconds(MIN_THROTTLE);
  motor2.writeMicroseconds(MIN_THROTTLE);
  delay(3000);
  Serial.println("ESCs Armed.");

  last_time = micros();
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT();

    unsigned long current_time = micros();
    float dt = (current_time - last_time) / 1000000.0;
    last_time = current_time;

    // --- 1. READ & FILTER GYRO Z ---
    float current_yaw_rate = myICM.gyrZ();
    current_yaw_rate -= gyro_z_offset;

    // Apply Low-Pass Filter
    gyro_z_filtered = alpha * current_yaw_rate + (1 - alpha) * gyro_z_filtered;

    // --- 2. PID CALCULATION ---
    error = yaw_rate_setpoint - gyro_z_filtered;
    integral += error * dt;
    derivative = (error - last_error) / dt;
    last_error = error;

    pid_output = (kp * error) + (ki * integral) + (kd * derivative);

    // --- 3. MOTOR CONTROL (DIFFERENTIAL THRUST) ---
    motor1_speed = base_throttle + pid_output;
    motor2_speed = base_throttle - pid_output;

    motor1_speed = constrain(motor1_speed, MIN_THROTTLE, MAX_THROTTLE);
    motor2_speed = constrain(motor2_speed, MIN_THROTTLE, MAX_THROTTLE);

    motor1.writeMicroseconds(motor1_speed);
    motor2.writeMicroseconds(motor2_speed);
    // motor1.writeMicroseconds(MAX_THROTTLE);
    // motor2.writeMicroseconds(MAX_THROTTLE);

    // --- 4. DEBUG OUTPUT ---
    print_debug();
  }
}

void calibrate_gyro() {
  Serial.println("Calibrating Gyroscope... Keep the device perfectly still.");
  float total_gyro_z = 0;
  int samples = 500;
  for (int i = 0; i < samples; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      total_gyro_z += myICM.gyrZ();
    }
    delay(5);
  }
  gyro_z_offset = (total_gyro_z / samples);
  Serial.print("Gyro Z-axis offset (dps): ");
  Serial.println(gyro_z_offset, 4);
}

void print_debug() {   
  int plot_min = -250;
  int plot_max = 250;

  Serial.print(plot_min);
  Serial.print(" ");
  Serial.print(plot_max);
  Serial.print(" ");

  Serial.print(yaw_rate_setpoint);
  Serial.print(" ");
  Serial.println(gyro_z_filtered, 2);
}
