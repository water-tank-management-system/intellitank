/* ========================================

Main Header Files
Capstone A9

======================================== */

#ifndef CapstoneA9_H
#define CapstoneA9_H

// ===== Import Necessary Library/Packages =====

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// ===== User-Defined Constant =====

// Define Data Communication State
#define ESP_TX 1
#define ESP_RX 3
#define RTDB_TX 200
#define RTDB_RX 202


// ===== User-Defined Variables =====

// ===== Variables to save database paths

extern String   uid,        // Variable to save USER UID
                parentPath; // Parent Node (to be updated in every loop)

// Database main path (to be updated in setup with the user UID)
extern String databasePath;

// Database child nodes
extern String tmpPath,
              lvlPath,
              flwPath,
              turPath,
              dirPath,
              empPath,
              autPath,
              clnPath,
              fllPath;

// Data Logging Variables
extern String   dataLogPath,
                tmpLog,
                lvlLog,
                flwLog,
                turLog,
                timePath;

// Variable to save current epoch time
extern int timestamp;

// ===== Variables Related to System Communication

// System Readings Data Variables
extern float  temperature,
              water_level,
              water_flow;
extern int turbidity;
extern int  automation,
            dirty_state,
            empty_state;

// User State Data Variables
extern int  auto_mode,
            cleaning_state_user,
            fill_state_user;

// Initial state
extern int state;

// Data Transfer Variables
extern String dataIn,
              temp,
              level,
              turb,
              flow,
              dirty,
              empty;

// Data Parsing Variables
extern uint8_t  indexOfA,
                indexOfB,
                indexOfC,
                indexOfD,
                indexOfE,
                indexOfF;

// Timer variables
extern unsigned long  sendDataPrevMillis,
                      timerDelay, // send new readings every minute
                      prev;


// ===== User-Defined Functions Declarations =====

// Create function to state the LED
void stateLED(const long interval, int period);


#endif