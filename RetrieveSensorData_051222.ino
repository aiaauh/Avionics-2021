//This code was originally written by Kurt Rowland on 4/21/22 and based on example codes downloaded from Adafruit
//Modified on 5/9/22 by Kurt Rowland
//Modified by Pablo Rincon Cruz Starting 05/12/2022
//Boards:
//   Teensy 4.1

//   Pin 13 = SCL for all boards
//   Pin 12 = SDO for all boards
//   Pin 11 = SDA for all boards
//   Pin 10 = Chip Select for BMP390
//   Pin 9  = Chip Select for LIS331
//   Pin 8  = Chip Select for LSM6DSO32
//   Pin 0/1 = RX/TX for Ultimate GPS V3
//   BUILTIN_SDCARD = SD Card (FAT32 format)
//   Pin 25/24 = RX/TX for XBEE PRO

//All boards are connected to common 5V and GND rails.
//All boards can be powered via USB on Teensy or via USB on Buck/Boost board
//If powering through Buck/Boost board, power off: EN pin to GND, power on: EN pin to 5V or not connected


// Include all the required pre-installed libraries

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LIS331HH.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_GPS.h>
#include <SD.h>
#include <SoftwareSerial.h>


// Define bmp3xx SPI Pins
#define BMP_CS 10 // For SPI mode, we only need to define a CS pin
#define SEALEVELPRESSURE_HPA (1013.25)  //Note: We will need to adjust this to match pressure at launch site.
Adafruit_BMP3XX bmp;


// Define LIS331HH SPI Pins
#define LIS331HH_CS 9 // For SPI mode, we only need to define a CS pin
Adafruit_LIS331HH lis = Adafruit_LIS331HH();


// Define LSM6DSO32 SPI Pins
#define LSM_CS 8 // For SPI mode, we only need to define a CS pin
Adafruit_LSM6DSO32 dso32;


// Define GPS Pins
#define GPSSerial Serial1
#define GPSECHO  true


// Define SD SPI Pins
const int chipSelect = BUILTIN_SDCARD;
File dataFile;

// Define XBEE Pins
SoftwareSerial mySerial(25, 24); // RX, TX


void setup() {  // This is the setup block of our code.


  // Find Serial Monitor Communications
  Serial.begin(115200);
  while (!Serial)
   delay(10); // will pause until serial console opens
  Serial.println("Serial Monitor Communicating!");


  // Find BMP390
  Serial.println("Adafruit BMP388 / BMP390 test");
  if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    Serial.println("Failed to find BMP388 / BMP390 sensor, check wiring!");
  }
  else{
    Serial.println("Found BMP388 / BMP390 Sensor!");
  }


  // Find LIS331HH
  Serial.println("Adafruit LIS331HH test");
  if (!lis.begin_SPI(LIS331HH_CS)) {  // hardware SPI mode  
    Serial.println("Failed to find LIS331HH sensor, check wiring!");
  }
  else{
    Serial.println("Found LIS331HH Sensor!");
  }


  // Find LSM6DSO32
  Serial.println("Adafruit LSM6DSO32 test");
  if (!dso32.begin_SPI(LSM_CS)) {  // hardware SPI mode  
    Serial.println("Failed to find LSM6DSO32 sensor, check wiring!");
  }
  else{
    Serial.println("Found LSM6DSO32 Sensor!");
  }

  GPSSerial.begin(9600); //GPS Seroal Port
  mySerial.begin(38400); //XBEE Serial Port
  

  // Set BMP388 / BMP390 oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  
  // Set Ranges for LIS331HH
  lis.setRange(LIS331HH_RANGE_24_G);   // 6, 12, or 24 G
  Serial.print("Range set to: ");
  switch (lis.getRange()) {
    case LIS331HH_RANGE_6_G: Serial.println("6 g"); break;
    case LIS331HH_RANGE_12_G: Serial.println("12 g"); break;
    case LIS331HH_RANGE_24_G: Serial.println("24 g"); break;
  }
  // lis.setDataRate(LIS331_DATARATE_400_HZ); 
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS331_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS331_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS331_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS331_DATARATE_400_HZ: Serial.println("400 Hz"); break;
    case LIS331_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
    case LIS331_DATARATE_LOWPOWER_0_5_HZ: Serial.println("0.5 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_1_HZ: Serial.println("1 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_2_HZ: Serial.println("2 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_5_HZ: Serial.println("5 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_10_HZ: Serial.println("10 Hz Low Power"); break;
  }

  
  //Set Ranges for LSM6DSO32
  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);    // 4_G, 8_G, 16_G or 32_G
  Serial.print("Accelerometer range set to: +-32G\n");
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS );    // 125_DPS, 250_DPS, 500_DPS, 1000_DPS, 2000_DPS
  Serial.print("Gyro range set to: 125 degrees/s\n");
  dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);    // SHUTDOWN, 12_5_HZ, 26_HZ, 52_HZ, 104_HZ, 208_HZ, 416_HZ, 833_HZ, 1_66K_HZ, 3_33K_HZ, 6_66K_HZ
  Serial.print("Accelerometer data rate set to: 12.5 Hz\n");
  dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);    // SHUTDOWN, 12_5_HZ, 26_HZ, 52_HZ, 104_HZ, 208_HZ, 416_HZ, 833_HZ, 1_66K_HZ, 3_33K_HZ, 6_66K_HZ
  Serial.print("Gyro data rate set to: 12.5 Hz\n");
  
  // Create string and add the header for our data. This will be saved as the first line of data each power cycle.
  String header= "[Time (ms)]\t[Temperature(C)]\t[Temperature (F)]\t[Pressure (hPa)]\t[Pressure(PSI)]\t[Altitude (m)]\t[Altitude (ft)]\t";
  header += "[X(m/s^2)]\t[Y(m/s^2)]\t[Z(m/s^2)]\t[Temperature(C)]\t[Temperature (F)]\t[X Gyro(m/s^2)]\t[Y Gyro(m/s^2)]\t[Z Gyro(m/s^2)]\t";

  // Create text file, save header to first line, close text file
    Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  Serial.println(header);

  
}

void loop() {  // This is the main loop that runs ifinitly
  
  //Create a new line of code for each loop
//  Serial.println(""); 

  //Print Time since program started
  String rawData = millis();

  //BMP390 Data
  // Get a new normalized sensor event
  bmp.performReading();
  rawData += String("\t") + String(bmp.temperature);                         //Temperature (C)
  rawData += String("\t") + String(bmp.temperature *9/5+32);                 //Temperature (F)
  rawData += String("\t") + String(bmp.pressure / 100.0);                    //Pressure (hPa)
  rawData += String("\t") + String(bmp.pressure / 100.0 / 68.947572932);     //Pressure (PSI)
  rawData += String("\t") + String(bmp.readAltitude(SEALEVELPRESSURE_HPA));  //Approx. Altitude (m)
  rawData += String("\t") + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28084);  //Altitude (ft)
  

  //LIS331HH Data
  // Get a new normalized sensor event
  sensors_event_t event;
  lis.getEvent(&event);
  rawData += String("\t") + String(event.acceleration.x);  //X Acceleration (m/s^2)
  rawData += String("\t") + String(event.acceleration.y);  //Y Acceleration (m/s^2)
  rawData += String("\t") + String(event.acceleration.z);  //Z Acceleration (m/s^2)


  // LSM6DSO32 Data
  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);
  rawData += String("\t") + String(temp.temperature);         //Temperature (C)
  rawData += String("\t") + String(temp.temperature *9/5+32); //Temperature (F)
  rawData += String("\t") + String(accel.acceleration.x);  //X Acceleration (m/s^2)
  rawData += String("\t") + String(accel.acceleration.y);  //Y Acceleration (m/s^2)
  rawData += String("\t") + String(accel.acceleration.z);  //Z Acceleration (m/s^2)
  rawData += String("\t") + String(gyro.gyro.x);           //X Gyroscope    (rad/s)
  rawData += String("\t") + String(gyro.gyro.y);           //Y Gyroscope    (rad/s)
  rawData += String("\t") + String(gyro.gyro.z);           //Z Gyroscope    (rad/s)


  // Create text file, save rawData to first line, close text file
  dataFile = SD.open("ROCKET.txt", FILE_WRITE);  // Create/open the file
  if (dataFile) {
    dataFile.println(rawData);  // If the file is available, write to it
    dataFile.close();          // Close the file
    Serial.println(rawData);    // Print the header in Serial Monitor
  }  
  else {
    Serial.println("error opening ROCKET.txt");  // if the file isn't open, pop up an error
  } 

  //Transmit data over XBEE maybe? TESTING REQUIRED or COMMENT OUT
    if (Serial.available())
    mySerial.write(Serial.read());
  if (mySerial.available())
    Serial.write(mySerial.read());

}

//This is where we will put extra functions/modules to be called earlier in the code. This will help keep the code tidy/organized
