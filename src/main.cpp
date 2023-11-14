/* ========================================

MONITORING WEB-APPS
Firebase Project

======================================== */


#include <CapstoneA9.h>
#include "Networking.h"
#include "Data_Communication.h"

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"


// ===== Let's Go! =====

unsigned long interval = 1000;    // the time we need to wait
unsigned long previousMillis = 0; // millis() returns an unsigned long.

void setup()
{
  // Baudrate for ESP-PC Communication
  Serial.begin(9600);

  // Baudrate for ESP - Arduino Communication
  espSS.begin(9600);  

  // Initialize the timestamp
  timeClient.begin();

  // init Random seed for random number
  randomSeed(analogRead(0));

  // LED_BUILTIN initialize
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize the WiFi Connection
  initWiFi();

  // Initialize the Firebase 
  initFirebase();

  stateLED(500, 3);  
}

void loop()
{ 
  switch(state)    
  {
    case ESP_RX:
      while (Serial.available() > 0)
      {
        dataIn = Serial.readStringUntil('\n');
        parsingData();

        /*        
        // Serial.println("Data recieved");
        // Serial.println("temp = " + temp);
        // Serial.println("level = " + level);
        // Serial.println("turb = " + turb);
        // Serial.println("flow = " + flow);        
        // Serial.println("=====================");
        */

        state = RTDB_TX;
      }
      
      break;
    
    case RTDB_TX:
      // Send new readings to database
      temperature = temp.toFloat();
      water_level = level.toFloat();
      water_flow = flow.toFloat();
      turbidity = turb.toFloat();

      /*
      // temperature = random(10, 40) + ((float)random(0, 100) / 100.0);
      // water_level = random(0, 60) + ((float)random(0, 100) / 100.0);
      // water_flow  = random(0, 60) + ((float)random(0, 100) / 100.0);
      // turbidity = 5;
      */

      // Send values to database:
      sendFloat(tmpPath, temperature);
      sendFloat(lvlPath, water_level);
      sendFloat(flwPath, water_flow);
      sendInt(turPath, turbidity);

      unsigned long currentMillis = millis(); // grab current time
      if ((unsigned long)(currentMillis - previousMillis) >= interval)
      {
        sendDataLog();
        stateLED(500, 1);
        previousMillis = millis();
      }

      /*
      if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0))
      {
        sendDataPrevMillis = millis();

        temperature = temp.toFloat();
        water_level = level.toFloat();
        water_flow = flow.toFloat();
        turbidity = turb.toFloat();

        
        // temperature = random(10, 40) + ((float)random(0, 100) / 100.0);
        // water_level = random(0, 60) + ((float)random(0, 100) / 100.0);
        // water_flow  = random(0, 60) + ((float)random(0, 100) / 100.0);
        
        turbidity   = 5;

        // Send values to database:
        sendFloat(tmpPath, temperature);
        sendFloat(lvlPath, water_level);
        sendFloat(flwPath, water_flow);
        sendInt(turPath, turbidity);  
        sendDataLog();

        // stateLED(500, 2);
      }
      */
      
      state = RTDB_RX;
      break;
    
    case RTDB_RX:
      // Receive data from database
      sendDataPrevMillis = millis();

      // Receive values from database:
      auto_mode = receiveInt(autPath);
      cleaning_state_user = receiveInt(clnPath);
      fill_state_user = receiveInt(fllPath);

      /*
      if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0))
      {
        sendDataPrevMillis = millis();

        // Receive values from database:
        auto_mode = receiveInt(autPath);
        cleaning_state_user = receiveInt(clnPath);
        fill_state_user = receiveInt(fllPath);

        
        // Serial.println("=========================");
        // Serial.print(" automation\t: ");     Serial.println(auto_mode);
        // Serial.print(" cleaning-state\t: "); Serial.println(cleaning_state_user);
        // Serial.print(" fill-state\t: ");     Serial.println(fill_state_user);
        // Serial.println("=========================\n");
        

        // stateLED(500, 2);
      }      
      */

      state = ESP_TX;
      break;
    
    case ESP_TX:
      // Serial.println("Data transmitted: ");

      Serial.print(auto_mode); Serial.print("A");
      Serial.print(cleaning_state_user); Serial.print("B");
      Serial.print(fill_state_user); Serial.print("C");
      Serial.print("\n");

      /*
      // Serial.println(auto_mode);
      // Serial.println(cleaning_state_user);
      // Serial.println(fill_state_user);

      // Serial.println("=====================");
      // delay(1000);
      */

      state = ESP_RX;
      break;
  }
}

// Initialize Firebase Configuration
void initFirebase()
{
  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);

  // Getting the user UID might take a few seconds
  // Serial.println("Getting User UID");
  while ((auth.token.uid) == "")
  {    
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();

  // Serial.print("User UID: ");
  // Serial.println(uid);

  // Update database path
  databasePath = "/UsersData/" + uid;

  // Update database path for data logging
  dataLogPath = "/UsersData/" + uid + "/log";

  // Update database path for sensor readings
  tmpPath = databasePath + "/temperature";    // --> UsersData/<user_uid>/temperature
  turPath = databasePath + "/turbidity";      // --> UsersData/<user_uid>/turbidity
  lvlPath = databasePath + "/water-level";    // --> UsersData/<user_uid>/water-level
  flwPath = databasePath + "/water-flow";     // --> UsersData/<user_uid>/water-flow  
  autPath = databasePath + "/automation";     // --> UsersData/<user_uid>/automation
  clnPath = databasePath + "/cleaning-state"; // --> UsersData/<user_uid>/cleaning-state
  fllPath = databasePath + "/filling-state";  // --> UsersData/<user_uid>/filling-state

  sendInt(autPath, 0);
  sendInt(clnPath, 0);
  sendInt(fllPath, 0);
}