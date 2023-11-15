/* ========================================

Function Files
Related to Networking

======================================== */


#include "Networking.h"


// ===== User-Defined Object =====

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Define Firebase JSON objects
FirebaseJson json;

/*
// Create Software // Serial Object
SoftwareSerial espSS(ESP_RX, ESP_TX);
*/

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");


// Initialize WiFi Connection
void initWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Serial.print("Connecting to WiFi ..");

  while (WiFi.status() != WL_CONNECTED)
  {
    // Serial.print('.');
    // delay(1000);
    stateLED(1000, 1);
  }
  
  // Serial.println(WiFi.localIP());
  // Serial.println();
}

// Function that gets current epoch time
unsigned long getTime()
{
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}