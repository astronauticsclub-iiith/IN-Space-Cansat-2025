#include <Arduino.h>
#include <DShotRMT.h>

// Motor pin configuration - choose a suitable GPIO pin
#define MOTOR_PIN 18  // You can use pins like 18, 19, 21, 22, 23, etc.

// Create DShotRMT object for the motor
DShotRMT motor(MOTOR_PIN);

// FreeRTOS task handle
TaskHandle_t motorTaskHandle = NULL;

// Motor control variables
volatile uint16_t targetThrottle = 0;
volatile bool motorArmed = false;

// Function to send DShot commands regularly (must be called every ~2-10ms)
void motorControlTask(void *parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 2ms interval (500Hz)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (true) {
    // Send throttle value to ESC
    motor.send_dshot_value(targetThrottle);
    
    // If bidirectional mode is enabled, read telemetry
    uint32_t rpm = 0;  // IMPORTANT: uint32_t for DShot-rmt v4.3.0
    extended_telem_type_t telemType = TELEM_TYPE_ERPM;
    int error = motor.get_dshot_packet(&rpm, &telemType);
    
    if (error == 0) {
      // Successfully received telemetry
      // You can process RPM data here
    }
    
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== DShotRMT Bidirectional Motor Control ===");
  Serial.println("Initializing motor...");
  
  // IMPORTANT: Check your DShotRMT.h file for the correct bidirectional enum values
  // Look for: typedef enum bidirectional_mode_e { ... }
  // Common values might be: BIDIRECTION, WITH_BIDIRECTION, ENABLE_BIDIRECTION, etc.
  
  // Initialize the motor with:
  // - DSHOT600 protocol (you can use DSHOT150, DSHOT300, DSHOT600, or DSHOT1200)
  // - Bidirectional mode ENABLED 
  // - 14 magnets (adjust based on your motor, common values: 7, 12, 14)
  // Note: The begin() method signature expects: (dshot_mode_t, bidirectional_mode_t, uint16_t)
  // Available bidirectional modes: NO_BIDIRECTION or check DShotRMT.h for the enabled enum
  motor.begin(DSHOT600, ENABLE_BIDIRECTION, 14);  // Start with NO_BIDIRECTION to test basic operation first
  
  Serial.println("Motor initialized. Arming sequence starting...");
  
  // Arming sequence: Send zero throttle for 500ms
  // This is required by most ESCs (varies by firmware: BlueJay ~300ms, BLHeli_S ~1000ms)
  targetThrottle = 0;  // Zero throttle for arming
  
  unsigned long armStart = millis();
  while (millis() - armStart < 500) {
    motor.send_dshot_value(targetThrottle);
    delay(2);
  }
  
  motorArmed = true;
  Serial.println("Motor ARMED!");
  Serial.println("\n=== Commands ===");
  Serial.println("0-2047: Set throttle value");
  Serial.println("3d: Enable 3D mode (bidirectional)");
  Serial.println("beep: Make motor beep");
  Serial.println("reverse: Reverse direction");
  Serial.println("normal: Normal direction");
  Serial.println("stop: Stop motor");
  Serial.println("telem: Show telemetry success rate");
  Serial.println("================\n");
  
  // Create FreeRTOS task for continuous motor control
  // This ensures DShot packets are sent regularly
  xTaskCreatePinnedToCore(
    motorControlTask,      // Task function
    "MotorControl",        // Task name
    4096,                  // Stack size
    NULL,                  // Parameters
    1,                     // Priority
    &motorTaskHandle,      // Task handle
    1                      // Core (0 or 1)
  );
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("3d")) {
      // Enable 3D mode for bidirectional rotation
      Serial.println("Enabling 3D mode...");
      // Send 3D mode command multiple times
      for (int i = 0; i < 10; i++) {
        motor.send_dshot_value(DSHOT_CMD_3D_MODE_ON);
        delay(10);
      }
      Serial.println("3D mode enabled. Use throttle 1048 for center (stop)");
      Serial.println("Below 1048: Reverse, Above 1048: Forward");
      
    } else if (command.startsWith("beep")) {
      Serial.println("Beeping motor...");
      targetThrottle = 0;  // Stop motor first
      delay(100);
      motor.send_dshot_value(DSHOT_CMD_BEACON1);  // Correct command name
      delay(300);  // Wait for beep to complete
      
    } else if (command.startsWith("reverse")) {
      Serial.println("Setting reverse direction...");
      targetThrottle = 0;  // Stop motor first
      delay(100);
      // Send reverse command multiple times
      for (int i = 0; i < 10; i++) {
        motor.send_dshot_value(DSHOT_CMD_SPIN_DIRECTION_REVERSED);
        delay(10);
      }
      Serial.println("Direction set to REVERSE");
      
    } else if (command.startsWith("normal")) {
      Serial.println("Setting normal direction...");
      targetThrottle = 0;  // Stop motor first
      delay(100);
      // Send normal direction command multiple times
      for (int i = 0; i < 10; i++) {
        motor.send_dshot_value(DSHOT_CMD_SPIN_DIRECTION_NORMAL);
        delay(10);
      }
      Serial.println("Direction set to NORMAL");
      
    } else if (command.startsWith("stop")) {
      Serial.println("Stopping motor...");
      targetThrottle = 0;  // Zero throttle to stop
      
    } else if (command.startsWith("telem")) {
      float successRate = motor.get_telem_success_rate();
      Serial.print("Telemetry success rate: ");
      Serial.print(successRate);
      Serial.println("%");
      
    } else {
      // Try to parse as throttle value
      int throttle = command.toInt();
      if (throttle >= 0 && throttle <= DSHOT_THROTTLE_MAX) {
        targetThrottle = throttle;
        Serial.print("Throttle set to: ");
        Serial.println(throttle);
        
        if (throttle < DSHOT_THROTTLE_MIN && throttle > 0) {
          Serial.println("WARNING: Value below DSHOT_THROTTLE_MIN (48)");
          Serial.println("Values 1-47 are special commands!");
        }
      } else {
        Serial.println("Invalid command or throttle value out of range (0-2047)");
      }
    }
  }
  
  // Print telemetry data every second (optional)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    
    // Note: Frequent Serial.print can interfere with telemetry reception
    // Print less frequently for better telemetry performance
    static int printCounter = 0;
    if (printCounter++ % 5 == 0) {  // Print every 5 seconds
      Serial.print("Current throttle: ");
      Serial.print(targetThrottle);
      Serial.print(" | Telemetry rate: ");
      Serial.print(motor.get_telem_success_rate());
      Serial.println("%");
    }
  }
  
  delay(10);
}
