// FLIGHT CODE TEST (2S-C) - INSTRUCTOR VERSION
// FOR USE IN BLACK STUDENTS IN AEROSPACE CUBESAT DEMONSTRATION PROGRAM
// RELEASED: 08/25/2025

// Include necessary libraries
#include <Wire.h>                  // Enables I2C communication
#include <SPI.h>                   // Enables SPI communication
#include <Adafruit_Sensor.h>       // Base class for sensor objects (used by BMP388 library)
#include <Adafruit_BMP3XX.h>       // Library for the BMP388 temperature/pressure sensor
#include <Adafruit_LIS3DH.h>       // Library for the LIS3DH accelerometer
#include "SdFat.h"                 // High-performance SD card library

#define SEALEVELPRESSURE_HPA (1013.25)  // Standard sea level pressure in hPa (used for altitude calc)

// SD CARD SETUP
#define SD_CS_PIN 23 // Chip select pin for the Feather RP2040 Adalogger SD slot
SdFat SD;                            // Create an SdFat object to interface with the SD card
FsFile dataFile;                     // Create a file object for reading/writing
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);  // Configure SD

Adafruit_BMP3XX bmp;                // Create an object for the BMP388 sensor
Adafruit_LIS3DH lis = Adafruit_LIS3DH();  // Create LIS3DH object using default I2C address (0x18)
unsigned long startTime;           // Variable to track when logging starts

// RGB LED pin definitions
const int redPin = 9; // Define digital pin on Feather for Red (This can be any digital output pin)
const int greenPin = 10; // Define digital pin on Feather for Green (This can be any digital output pin)
const int bluePin = 11; // Define digital pin on Feather for Blue (This can be any digital output pin)

void setup() {
  Serial.begin(115200);            // Begin serial communication at 115200 baud
  while (!Serial);                 // Wait until Serial Monitor is open
  delay(100);                      // Small delay for USB and SD to stabilize

  Serial.println("Flight Code Test");

    // initialize the digitals pin as an outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Turn the LED off initially
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // Initialize SD Card
  Serial.print("Initializing SD card...");
  if (!SD.begin(config)) {      // Try to initialize the SD card with given config
    Serial.println("Initialization failed!");
    digitalWrite(redPin, HIGH);  // Turn on red LED for SD card failure
    while(1);                   // Halt program execution
  }
  Serial.println("SD card initialized.");

  // File Management
  if (SD.exists("Flight_Code_Test.csv")) {
    // File exists: append log marker
    dataFile = SD.open("Flight_Code_Test.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("# --- NEW LOG STARTED ---");
      dataFile.close();
      Serial.println("Existing data file found. Appending…");
    }
  } else {
    // File doesn't exist: create and write header
    dataFile = SD.open("Flight_Code_Test.csv", FILE_WRITE);
    if (dataFile) {
    dataFile.println("Timestamp,Temperature (C),Pressure (hPa),Altitude (m),X (m/s^2),Y (m/s^2),Z (m/s^2)");  // Write CSV header
    dataFile.close();
    } else {
      Serial.println("Error opening Flight_Code_Test.csv");  // Print message if file couldn't be created
      digitalWrite(redPin, HIGH); // Turn the red pin on 
      while (1);  // Halt program execution
    }
  }

  // Initialize I2C 
  Wire.begin();                     // Start I2C communication on default SDA and SCL pins on RP2040)

  // BMP initialization
  if (!bmp.begin_I2C()) {           // Initialize BMP388 sensor over I2C
    Serial.println("Could not find a valid BMP388 sensor, check wiring!");
    digitalWrite(redPin, HIGH);     // Turn on Red LED if BMP388 sensor could not be found
    while (1);                      // Halt program execution if sensor not found
  }

  // Default BMP388 configuration
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);  // Higher precision temp readings
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);     // Medium precision pressure
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);        // Filter out noise
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);                // Sensor outputs data at 100 Hz

  // LIS3DH initialization
   if (!lis.begin(0x18)) {           // Initialize LIS3DH sensor over I2C (default address is 0x18)
    Serial.println("Could not find a valid LIS3DH sensor, check wiring!");
    digitalWrite(redPin, HIGH);     // Turn on Red LED if LIS3DH sensor could not be found
    while (1);                      // Halt program execution if sensor not found
  }

  // Default LIS3DH configuration
  lis.setRange(LIS3DH_RANGE_2_G);       // Set measurement range to ±2g (options: 2G, 4G, 8G, 16G)
  lis.setDataRate(LIS3DH_DATARATE_50_HZ); // Set data rate to 50 Hz (50 samples per second internally)

  pinMode(PIN_LED, OUTPUT);         // Set built-in LED pin as output
  startTime = millis();             // Record the start time

  Serial.println("Data collection starting...");
}

void loop() {
  // Logic to Stop After 20 Minutes
  if (millis() - startTime >= 1200000) {     // Check if 1200000 ms (20 mins) have passed
    Serial.println("20 minutes passed. Stopping data logging.");
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
      dataFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));  // Write the calculated altitude in meters
      dataFile.print(",");                     // CSV delimiter ahead of LIS3DH readings
    } else {
      // If reading fails, print an error message and proceed
      Serial.println("Failed to read BMP388!");
      dataFile.println("-,-,-"); // Placeholder values for no sensor read
    }

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
    } else{
      // If reading fails, print an error message and proceed
      Serial.println("Failed to read LIS3DH!");
      dataFile.println("-,-,-"); // Placeholder values for no sensor read
    }
    
  } else {
    // If the file can't be accessed, print an error
    Serial.println("Error writing to Flight_Code_Test.csv");
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
      // Print to serial if reconnect attempt unsuccessful
      Serial.print("Retrying SD card initialization... Attempt ");
      Serial.println(retryCount + 1); 
      retryCount++; //increase count by 1
      delay(500); // wait 0.5 seconds before trying again
      }
    }
    // Stop code once max retry attempts reach
    if (retryCount == maxRetries) {
      Serial.print("SD is lost...");
      while(1);
    }
    Serial.println("SD connection recovered..."); // Signify SD connection recovered
  }
  dataFile.close();                     // Close file to prevent corruption

  // LED Logic
  uint8_t whoami = lis.getDeviceID();
  if (bmp.performReading() && whoami == 0x33) {
    digitalWrite(redPin, LOW);    // Turn off red LED
    digitalWrite(greenPin, HIGH);   // Turn on green LED
  } else if (!bmp.performReading() || whoami != 0x33) {
    digitalWrite(redPin, HIGH);    // Turn on red LED to make yellow
    digitalWrite(greenPin, HIGH);   // Turn on green LED to make yellow
  } else {
    digitalWrite(redPin, HIGH);    // Turn on red LED to 
    digitalWrite(greenPin, LOW);   // Turn off green LED
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