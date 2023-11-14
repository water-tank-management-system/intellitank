/* ========================================

Main Function Files
Capstone A9

======================================== */


#include <CapstoneA9.h>


// ===== User-Defined Variables =====

// ===== Variables to save database paths

String  uid,        // Variable to save USER UID
        parentPath; // Parent Node (to be updated in every loop)

// Database main path (to be updated in setup with the user UID)
String databasePath;

// Database child nodes
String  tmpPath,
        lvlPath,
        flwPath,
        turPath,        
        autPath,
        clnPath,
        fllPath;

// Data Logging Variables
String  dataLogPath,
        tmpLog = "/temperature",
        lvlLog = "/water-level",
        flwLog = "/water-flow",
        turLog = "/turbidity",
        timePath = "/timestamp";

// Variable to save current epoch time
int timestamp;

// ===== Variables Related to System Communication

// System Readings Data Variables
float temperature = 0,
      water_level = 0,
      water_flow = 0;
int   turbidity = 0;

// User State Data Variables
int auto_mode = 0,
    cleaning_state_user = 0,
    fill_state_user = 0;

// Initial state
int state = ESP_RX;

// Data Transfer Variables
String  dataIn,
        temp,
        level,
        turb,
        flow;        

// Data Parsing Variables
uint8_t indexOfA,
        indexOfB,
        indexOfC,
        indexOfD;        

// Timer variables
unsigned long sendDataPrevMillis = 0,
              timerDelay = 100, // send new readings every timerDelay
              prev = 0;


// ===== User-Defined Functions =====

// Create function to state the LED
void stateLED(const long interval, int period)
{
  for (int i = 0; i <= period; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(interval);
    digitalWrite(LED_BUILTIN, LOW);
    delay(interval);
  }
}
