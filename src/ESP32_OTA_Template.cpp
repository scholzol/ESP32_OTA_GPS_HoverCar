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

// #include section end ============================================================================================

// define global variables begin -----------------------------------------------------------------------------------

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

  // insert or update the board table for the ESP32 bords in the MySQL database
  updateBoardTable(ssidesp32);
}
// setup end =============================================================================================

// loop begin --------------------------------------------------------------------------------------------

void loop() {

  ArduinoOTA.handle();
}
// loop end =============================================================================================
