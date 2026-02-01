#include <Wire.h>
#include <VL53L0X.h>

// Define XSHUT pins
#define XSHUT1 7
#define XSHUT2 8

// Define I2C addresses for the two sensors
#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31

// Create sensor objects
VL53L0X sensor1;
VL53L0X sensor2;

void setup() {
  Serial.begin(115200);
  
  // Initialize XSHUT pins
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  
  // Reset both sensors by pulling XSHUT low
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(10);
  
  // Initialize I2C
  Wire.begin();  
  // Initialize Sensor 1
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  sensor1.setTimeout(500);
  if (!sensor1.init()) {
    Serial.println("Failed to detect Sensor 1!");
    while (1) {}
  }
  sensor1.setAddress(SENSOR1_ADDRESS);
  
  // Initialize Sensor 2
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  sensor2.setTimeout(500);
  if (!sensor2.init()) {
    Serial.println("Failed to detect Sensor 2!");
    while (1) {}
  }
  sensor2.setAddress(SENSOR2_ADDRESS);
  sensor1.setMeasurementTimingBudget(33000);
  sensor2.setMeasurementTimingBudget(33000);
  // Start continuous measurements
  sensor1.startContinuous();
  sensor2.startContinuous();
  
  Serial.println("VL53L0X sensors initialized!");
}

void loop() {
  // Read distances from both sensors
  uint16_t R1 = sensor1.readRangeContinuousMillimeters();
  uint16_t R2 = sensor2.readRangeContinuousMillimeters();
  Serial.print("R1: ");
  if (sensor1.timeoutOccurred()) {
    Serial.print("TIMEOUT");
  } else {
    Serial.print(R1);
    Serial.print(" mm");
  }
  
  Serial.print("  |  ");
  
  // Print Sensor 2 data
  Serial.print("R2: ");
  if (sensor2.timeoutOccurred()) {
    Serial.print("TIMEOUT");
  } else {
    Serial.print(R2);
    Serial.print(" mm");
  }
  
  Serial.println();
  
  delay(100);
}
