#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <StartServer.h>
#include "Version.h" // contains version information

AsyncWebServer server(80);

const String formatBytes(size_t const& bytes) {
  return bytes < 1024 ? static_cast<String>(bytes) + " Byte" : bytes < 1048576 ? static_cast<String>(bytes / 1024.0) + " KB" : static_cast<String>(bytes / 1048576.0) + " MB";
}

void handleonce(AsyncWebServerRequest *request) {

    extern char ssidesp32[13];
    uint32_t chipId = 0;
    // String temp = R"({"SSID":")" + WiFi.SSID() + R"("})";
    for(int i=0; i<17; i=i+8) {
        chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    snprintf(ssidesp32,13,"ESP32-%06lX",chipId); // generate chipID string as SSID
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    JsonDocument json;
    json["SSID"] = WiFi.SSID();
    json["MAC"] = WiFi.macAddress();
    json["CHIP_ID"] = ssidesp32;
    json["CHIP_MODEL"] = ESP.getChipModel();
    json["BSSID"] = WiFi.BSSIDstr();
    json["FREE_HEAP"] = formatBytes(ESP.getFreeHeap());
    json["HEAP_SIZE"] = formatBytes(ESP.getHeapSize());
    json["LOCAL_IP"] = WiFi.localIP().toString();
    json["SKETCH_SIZE"] = formatBytes(ESP.getSketchSize());
    json["FREE_SKETCH_SPACE"] = formatBytes(ESP.getFreeSketchSpace());
    json["CHANNEL"] = WiFi.channel();
    json["RSSI"] = WiFi.RSSI();
    json["SKETCH"] = FullWorkingDirectory;
    json["SHA"] = SHA_short;
    json["FULLBUILD"] = FullBuildMetaData;
    json["VERSION"] = SemanticVersion;
    json["WORK_DIR"] = WorkingDirectory;
    json["COMPUTER_NAME"] = ComputerName;
    json["BRANCHNAME"] = BranchName;
    serializeJson(json, *response);
    request->send(response);     // Json als Objekt
}

void startServer() 
{
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  
  // Route for root / web page
  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/reboot.html");
    ESP.restart();
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });
  // Route to load the favicon file
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/favicon.ico", "image/x-icon");
    });
  server.on("/once", handleonce);

  server.begin();
}
