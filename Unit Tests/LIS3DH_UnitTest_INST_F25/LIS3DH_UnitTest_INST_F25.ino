// LIS3DH UNIT TEST - INSTRUCTOR VERSION
// FOR USE IN BLACK STUDENTS IN AEROSPACE CUBESAT DEMONSTRATION PROGRAM
// RELEASED: 08/25/2025

// Include necessary libraries
#include <Wire.h>                  // Enables I2C communication
#include <SPI.h>                   // Enables SPI communication
#include <Adafruit_Sensor.h>       // Base class for sensor objects (used by LIS3DH library)
#include <Adafruit_LIS3DH.h>       // Library for the LIS3DH accelerometer
#include "SdFat.h"                 // High-performance SD card library

// SD CARD SETUP
#define SD_CS_PIN 23               // Chip select pin for the Feather RP2040 Adalogger SD slot
SdFat SD;                          // Create an SdFat object to interface with the SD card
FsFile dataFile;                   // Create a file object for reading/writing
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);  // Configure SD card to use SPI1 bus

// LIS3DH SENSOR SETUP
Adafruit_LIS3DH lis = Adafruit_LIS3DH();  // Create LIS3DH object using default I2C address (0x18)

unsigned long startTime;           // Variable to track when logging starts

void setup() {
  Serial.begin(115200);            // Begin serial communication at 115200 baud
  while (!Serial);                 // Wait until Serial Monitor is open
  delay(100);                      // Small delay for USB and SD to stabilize

  Serial.println("Feather RP2040 - LIS3DH Unit Test");

  // Initialize SD Card
  Serial.print("Initializing SD card...");
  if (!SD.begin(config)) {         // Try to initialize the SD card with given SPI1 config
    Serial.println("Initialization failed!");
    while (1);                     // Halt program execution
  }
  Serial.println("SD card initialized.");

  // File Management
  if (SD.exists("LIS3DH_data.csv")) {   // Check if a data file already exists
    SD.remove("LIS3DH_data.csv");       // Delete it to start fresh
    Serial.println("Existing data file deleted.");
  }

  dataFile = SD.open("LIS3DH_data.csv", FILE_WRITE);  // Open new file for writing
  if (dataFile) {
    dataFile.println("Timestamp,X (m/s^2),Y (m/s^2),Z (m/s^2)");  // Write CSV header line
    dataFile.close();  // Close immediately to avoid keeping file open across loops
  } else {
    Serial.println("Error opening LIS3DH_data.csv");  // Print message if file couldn't be created
    while (1);  // Halt program execution
  }

  // Initialize I2C and LIS3DH 
  Wire.begin();                     // Start I2C communication on default SDA (20) and SCL (21) pins

  if (!lis.begin(0x18)) {           // Initialize LIS3DH sensor over I2C (default address is 0x18)
    Serial.println("Could not find a valid LIS3DH sensor, check wiring!");
    while (1);                      // Halt program execution if sensor not found
  }

  // Default LIS3DH configuration
  lis.setRange(LIS3DH_RANGE_2_G);       // Set measurement range to ±2g (options: 2G, 4G, 8G, 16G)
  lis.setDataRate(LIS3DH_DATARATE_50_HZ); // Set data rate to 50 Hz (50 samples per second internally)

  pinMode(PIN_LED, OUTPUT);         // Set built-in LED pin as output (D13)
  startTime = millis();             // Record the start time

  Serial.println("Data collection starting...");
}

void loop() {
  // Logic to Stop After 1 Minute
  if (millis() - startTime >= 60000) {     // Check if 60,000 ms (1 min) have passed
    Serial.println("1 minute passed. Stopping data logging.");
    digitalWrite(PIN_LED, LOW);           // Turn off LED
    while (1);                            // Halt the program
  }

if (sdAvailable()) {
  // Calculate the timestamp (in seconds) since the program started
  unsigned long timestamp = millis() / 1000;  // `millis()` returns milliseconds, so divide by 1000 to get seconds

  dataFile = SD.open("LIS3DH_data.csv", FILE_WRITE);  // Open new file for writing

  dataFile.print(timestamp);               // Write the timestamp
  dataFile.print(",");                     // CSV delimiter (comma)// Read sensor data (in SI units) into event structure

  // WHO_AM_I check for LIS3DH (I2C address 0x18 expected to return 0x33)
  uint8_t whoami = lis.getDeviceID();
  if (whoami == 0x33) {
    sensors_event_t event;
    lis.getEvent(&event);   // Get normalized acceleration data (m/s^2)

    dataFile.print(event.acceleration.x, 4);  // Write X-axis acceleration to 4 decimal places
    dataFile.print(",");
    dataFile.print(event.acceleration.y, 4);  // Write Y-axis acceleration
    dataFile.print(",");
    dataFile.println(event.acceleration.z, 4); // Write Z-axis acceleration
  } else {
    // If trying to connect to the sensor fails, print an error message and halt the program
    Serial.println("Failed to read LIS3DH Sensor!");
    while(1);
  }
} else {
    // If the file can't be accessed, print an error
    Serial.println("Error writing to LIS3DH_data.csv");
    while(1);
  }
  dataFile.close();                     // Close file to ensure data is written and prevent corruption
  digitalWrite(PIN_LED, HIGH);         // Turn on LED to show activity
  delay(1000);                         // Wait 1 second before next reading
}

bool sdAvailable() {
  // Check card-level error
  if (SD.card()->errorCode()) {
    return false;  // Card reports an error
  }

  return true;  // SD looks good
}