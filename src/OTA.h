/**
 * @file OTA.h
 * @author Olaf Scholz (olaf.scholz@online.de)
 * @brief 
 * @version 0.1
 * @date 2023-02-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifdef ESP32
#include <WiFi.h>
#include <ESPmDNS.h>
#else
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#endif

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <credentials.h> // Wifi SSID and password

#if defined(ESP32_RTOS) && defined(ESP32)
void ota_handle( void * parameter ) {
  for (;;) {
    ArduinoOTA.handle();
    delay(3500);
  }
}
#endif
void handleWiFi() {
  int n;
  int i = 0;
  int i_strongest = -1;
  int32_t rssi_strongest = -100;

  n = WiFi.scanNetworks(); // WiFi.scanNetworks will return the number of networks found
  if (n == 0) {
    Serial.println("no networks found");
    } 
    else  {
      Serial.print(n);
      Serial.println(" networks found");
      for (int i = 0; i < n; ++i) {
        // Print SSID and RSSI for each network found
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print("BSSID: ");
        Serial.print(WiFi.BSSIDstr(i));
        Serial.print("  ");
        Serial.print(WiFi.RSSI(i));
        Serial.print("dBm, ");
        Serial.print(constrain(2 * (WiFi.RSSI(i) + 100), 0, 100));
        Serial.print("% ");
        Serial.print((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "open       " : "encrypted  ");
        Serial.println(WiFi.SSID(i));
        if ((WiFi.RSSI(i) > rssi_strongest) && (WiFi.SSID(i)==ssid1 || WiFi.SSID(i)==ssid2)) {
          rssi_strongest = WiFi.RSSI(i);
          i_strongest = i;
        delay(10);
        }
      } 
      Serial.println();
      WiFi.mode(WIFI_STA);
      WiFi.begin(WiFi.SSID(i_strongest).c_str(), password, 0, WiFi.BSSID(i_strongest));
      while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
        }
      Serial.print("BSSID: ");
      Serial.print(WiFi.BSSIDstr(i_strongest));
      Serial.print("  ");
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID(i_strongest));
      }
}

void setupOTA() {
  
  handleWiFi();

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232); // Use 8266 port if you are working in Sloeber IDE, it is fixed there and not adjustable

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
	//NOTE: make .detach() here for all functions called by Ticker.h library - not to interrupt transfer process in any way.
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("\nAuth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("\nBegin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("\nConnect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("\nReceive Failed");
    else if (error == OTA_END_ERROR) Serial.println("\nEnd Failed");
  });

  ArduinoOTA.begin();

  Serial.println("OTA Initialized");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

#if defined(ESP32_RTOS) && defined(ESP32)
  xTaskCreate(
    ota_handle,          /* Task function. */
    "OTA_HANDLE",        /* String with name of task. */
    10000,            /* Stack size in bytes. */
    NULL,             /* Parameter passed as input of the task */
    1,                /* Priority of the task. */
    NULL);            /* Task handle. */
#endif
}