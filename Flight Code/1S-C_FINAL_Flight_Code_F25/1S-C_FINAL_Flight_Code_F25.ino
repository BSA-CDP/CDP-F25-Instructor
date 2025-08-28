// FINAL FLIGHT CODE TEST - INSTRUCTOR VERSION
// FOR USE IN BLACK STUDENTS IN AEROSPACE CUBESAT DEMONSTRATION PROGRAM
// RELEASED: 08/25/2025

// Include necessary libraries
#include <Wire.h>                  // Enables I2C communication
#include <SPI.h>                   // Enables SPI communication
#include <Adafruit_Sensor.h>       // Base class for sensor objects (used by BMP388 library)
#include <Adafruit_BMP3XX.h>       // Library for the BMP388 temperature/pressure sensor
#include "SdFat.h"                 // High-performance SD card library

#define SEALEVELPRESSURE_HPA (1013.25)  // Standard sea level pressure in hPa (used for altitude calc)

// SD CARD SETUP
#define SD_CS_PIN 23 // Chip select pin for the Feather RP2040 Adalogger SD slot
SdFat SD;                            // Create an SdFat object to interface with the SD card
FsFile dataFile;                     // Create a file object for reading/writing
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);  // Configure SD

Adafruit_BMP3XX bmp;                // Create an object for the BMP388 sensor
unsigned long startTime;           // Variable to track when logging starts

FsFile logFile;                    // Create a file object for reading/writing a log file

// RGB LED pin definitions
const int redPin = 9; // Define digital pin on Feather for Red (This can be any digital output pin)
const int greenPin = 10; // Define digital pin on Feather for Green (This can be any digital output pin)
const int bluePin = 11; // Define digital pin on Feather for Blue (This can be any digital output pin)

// Camera function definitions
#define TRIGGER_OUT_PIN 5  // Designate Feather trigger pin (This can be any digital output pin)
unsigned long lastTriggerTime = 0; // Variable to record last trigger time
const unsigned long triggerInterval = 10000;  // Variable to store trigger interval of 10 seconds

void setup() {
  // Delete the existing log file if it exists
  if (SD.exists("log.txt")) {
    SD.remove("log.txt");  // Remove existing file
  }

  logFile = SD.open("log.txt", FILE_WRITE); // Create Log file in write mode

  logFile.println("Flight Code Test Start");

  // initialize the digitals pin as an outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Turn the LED off initially
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // Set Camera Trigger pin to off initially
  pinMode(TRIGGER_OUT_PIN, OUTPUT); // Set trigger pin as output
  digitalWrite(TRIGGER_OUT_PIN, LOW);  // Start with off signal from trigger pin

  // Initialize SD Card
  logFile.print("Initializing SD card...");
  if (!SD.begin(config)) {      // Try to initialize the SD card with given config
    logFile.println("Initialization failed!");
    digitalWrite(redPin, HIGH);  // Turn on red LED for SD card failure
    while(1);                   // Halt program execution
  }
  logFile.println("SD card initialized.");

  // File Management
  if (SD.exists("Flight_Code_Test.csv")) {
    // File exists: append log marker
    dataFile = SD.open("Flight_Code_Test.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("# --- NEW LOG STARTED ---");
      dataFile.close();
      logFile.println("Existing data file found. Appending…");
    }
  } else {
    // File doesn't exist: create and write header
    dataFile = SD.open("Flight_Code_Test.csv", FILE_WRITE);
    if (dataFile) {
    dataFile.println("Timestamp,Temperature (C),Pressure (hPa),Altitude (m)");
    dataFile.close();
    } else {
      logFile.println("Error opening Flight_Code_Test.csv");  // Print message if file couldn't be created
      digitalWrite(redPin, HIGH); // Turn the red pin on 
      while (1);  // Halt program execution
    }
  }

  // Initialize I2C and BMP388 
  Wire.begin();                     // Start I2C communication on default SDA and SCL pins on RP2040)

  if (!bmp.begin_I2C()) {           // Initialize BMP388 sensor over I2C
    logFile.println("Could not find a valid BMP388 sensor, check wiring!");
    digitalWrite(redPin, HIGH);     // Turn on Red LED if BMP388 sensor could not be found
    while (1);                      // Halt program execution if sensor not found
  }

  // Default BMP388 configuration
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);  // Higher precision temp readings
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);     // Medium precision pressure
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);        // Filter out noise
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);                // Sensor outputs data at 100 Hz

  pinMode(PIN_LED, OUTPUT);         // Set built-in LED pin as output
  startTime = millis();             // Record the start time

  logFile.println("Data collection starting...");
}

void loop() {
  // Logic to Stop After 20 Minutes
  if (millis() - startTime >= 1200000) {     // Check if 1200000 ms (20 mins) have passed
    logFile.println("20 minutes passed. Stopping data logging.");
    digitalWrite(PIN_LED, LOW);  // turn the LED off (HIGH is the voltage level)
    digitalWrite(redPin, LOW);  // Turn off red LED
    digitalWrite(greenPin, LOW);  // Turn off green LED
    digitalWrite(bluePin, HIGH);   // Turn on blue LED
    while (1);                            // Halt the program
  }

 // Log data to the file if the SD file can be found
  if (sdAvailable()) {
    // Calculate the timestamp (in seconds) since the program started
    unsigned long timestamp = millis() / 1000;  // `millis()` returns milliseconds, so divide by 1000 to get seconds

    dataFile = SD.open("Flight_Code_Test.csv", FILE_WRITE); // Reopen file in prep to write to it

    dataFile.print(timestamp);               // Write the timestamp
    dataFile.print(",");                     // CSV delimiter (comma)

    // Take a reading from the BMP388 sensor
    if (bmp.performReading()) {
      // If reading can be  taken, write the sensor data to the SD card in CSV format
      dataFile.print(bmp.temperature);         // Write the temperature in Celsius
      dataFile.print(",");                     // CSV delimiter
      dataFile.print(bmp.pressure / 100.0);    // Write the pressure in hPa (Pa to hPa conversion)
      dataFile.print(",");                     // CSV delimiter
      dataFile.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));  // Write the calculated altitude in meters
      digitalWrite(redPin, LOW);    // Turn off red LED
      digitalWrite(greenPin, HIGH);   // Turn on green LED for successful logging
    } else {
      // If reading fails, print an error message and proceed
      logFile.println("Failed to read BMP388!");
      // If reading fails, print an error message and proceed
      logFile.println("Failed to read BMP388!");
      dataFile.println("-,-,-"); // Placeholder values for no sensor read
    }

  } else {
    // If the file can't be accessed, print an error
    logFile.println("Error writing to Flight_Code_Test.csv");
    digitalWrite(greenPin, LOW); // Turn off green LED
    digitalWrite(redPin, HIGH); // Turn on red LED
    // Start retry attempts
    int retryCount = 0; // Create integer to track retries 
    const int maxRetries = 10; // Set max retry number
    // Enter while loop for up to 10 retries
    while (retryCount < maxRetries) {
      // Try to reconnect to the SD card 
      if (SD.begin(config)) {
        break; // exits the while loop if successful
      } else {
      // Print to log if reconnect attempt unsuccessful
      logFile.print("Retrying SD card initialization... Attempt ");
      logFile.println(retryCount + 1); 
      retryCount++; //increase count by 1
      delay(500); // wait 0.5 seconds before trying again
      }
    }
    // Stop code once max retry attempts reach
    if (retryCount == maxRetries) {
      logFile.print("SD is lost...");
      while(1);
    }
    logFile.println("SD connection recovered..."); // Signify SD connection recovered
  }
  dataFile.close();                     // Close file to prevent corruption

  // Camera trigger pin logic
  unsigned long currentTime = millis(); // Get current time since program start

  if (currentTime - lastTriggerTime >= triggerInterval) {
    logFile.println("Triggering camera...");
    digitalWrite(TRIGGER_OUT_PIN, HIGH);  // Set trigger pin to on (High) to send signal to ESP32-CAM
    delay(100);                           // Hold on (HIGH) for 100 ms
    digitalWrite(TRIGGER_OUT_PIN, LOW);   // Return to off (LOW)

    lastTriggerTime = currentTime;        // Update the time of last trigger
  }

  digitalWrite(PIN_LED, HIGH);         // Turn on LED to show activity
  delay(1000);                         // Wait 1 second (in milliseconds) before next reading
}

bool sdAvailable() {
  // Check if SD card displays an error code
  if (SD.card()->errorCode()) {
    return false;  // Card reports an error
  }

  return true;  // SD looks good
}