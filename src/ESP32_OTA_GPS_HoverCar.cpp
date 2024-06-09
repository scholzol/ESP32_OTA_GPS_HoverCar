/**
 * @file main.cpp
 * @author Olaf Scholz (olaf.scholz@online.de)
 * @brief 
 * @version 0.1
 * @date 2023-02-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

// #include section begin -------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <SPIFFS.h>
#include <FS.h>

#include "OTA.h"

#include "Version.h" // contains version information
#include "MySQL_1.h"
#include <StartServer.h>

#include <RF24.h>
#include <printf.h>
#include <SPI.h>
#include <Wire.h>
// #include section end ============================================================================================

// define global variables begin -----------------------------------------------------------------------------------

RF24 radio(4, 5); // CE, CSN
const uint64_t GPS_pipe = 0xB00B1E5000LL;
const uint64_t BME_pipe = 0xB00B1E5001LL;
uint8_t pipeNum;
byte code;

struct 
{
  float temperature;
  float humidity;
  float pressure;
} BME280_data;

#define BAUD_RATE 115200
#define TX2 17
#define RX2 16
#define SERIAL_MODE2 SERIAL_8N1

unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
char ssidesp32[13];

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

// define global variables end ======================================================================================

// helper functions begin --------------------------------------------------------------------------------------

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.println();
}

void getChipInfo() {
  Serial.println("\n\n================================");
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("with %d core\n", ESP.getChipCores());
  Serial.printf("Flash Chip Size : %d \n", ESP.getFlashChipSize());
  Serial.printf("Flash Chip Speed : %d \n", ESP.getFlashChipSpeed());
  

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  Serial.printf("\nFeatures included:\n %s\n %s\n %s\n %s\n %s\n",
      (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded flash" : "",
      (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "2.4GHz WiFi" : "",
      (chip_info.features & CHIP_FEATURE_BLE) ? "Bluetooth LE" : "",
      (chip_info.features & CHIP_FEATURE_BT) ? "Bluetooth Classic" : "",
      (chip_info.features & CHIP_FEATURE_IEEE802154) ? "IEEE 802.15.4" : "");

  Serial.println();
  // print Wifi data
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  // print Firmware data
  Serial.print("Version: ");
  Serial.println(SemanticVersion);
  Serial.print("short SHA: ");
  Serial.println(SHA_short);
}
// helper functions end ========================================================================================


// setup begin ------------------------------------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  Serial.println("Booting");

  Serial2.begin(BAUD_RATE, SERIAL_MODE2, RX2, TX2);

// Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  setupOTA();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  // read and print Chip, Wifi and Firmware data to Serial, has to be placed after setupOTA()
  getChipInfo(); 

  // start the Webserver
  startServer();

  // initialize radio nRF24L01
  bool ok = radio.begin();
  Serial.println(ok);
  delay(2000);
  radio.openReadingPipe(0, GPS_pipe);
  radio.openReadingPipe(1, BME_pipe);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.startListening();
  radio.printPrettyDetails();
  Serial.println("Start listening");

  // insert or update the board table for the ESP32 bords in the MySQL database
  updateBoardTable(ssidesp32);
}
// setup end =============================================================================================

// loop begin --------------------------------------------------------------------------------------------

void loop() {

  ArduinoOTA.handle();
  while (radio.available(&pipeNum)) {
    ArduinoOTA.handle();
    switch (pipeNum) {
      case 0:
        // code = '';
        radio.read(&code, sizeof(code));
        // Serial2.write(code);
        Serial.write(code);
        // Serial.print(" ");
        break;
      case 1:
        radio.read(&BME280_data, sizeof(BME280_data));

        Serial.print(" ");
        Serial.print("Temperature = ");
        Serial.print(BME280_data.temperature);
        Serial.println(" *C");
        
        // Convert temperature to Fahrenheit
        // Serial.print("Temperature = ");
        // Serial.print(1.8 * BME280_data.readTemperature() + 32);
        // Serial.println(" *F");
        
        Serial.print("Pressure = ");
        Serial.print(BME280_data.pressure);
        Serial.println(" hPa");

        // Serial.print("Approx. Altitude = ");
        // Serial.print(altitude);
        // Serial.println(" m");

        Serial.print("Humidity = ");
        Serial.print(BME280_data.humidity);
        Serial.println(" %");

        Serial.println();
        break;
      }
  }
}
// loop end =============================================================================================
