/* ========================================

Function Files
Related to Data Communication

======================================== */


#include "Data_Communication.h"
#include "Networking.h"


// Write Float values to the database
void sendFloat(String path, float value)
{
  if (Firebase.RTDB.setFloat(&fbdo, path.c_str(), value))
  {
    /*
    // Serial.print("Writing value: ");
    // Serial.print(value);
    // Serial.print(" on the following path: ");
    // Serial.println(path);
    // Serial.println("PASSED");
    // Serial.println("PATH: " + fbdo.dataPath());
    // Serial.println("TYPE: " + fbdo.dataType());
    */
  }
  else
  {
    /*    
    // Serial.println("FAILED");
    // Serial.println("REASON: " + fbdo.errorReason());
    */
  }
}

// Write Int values to the database
void sendInt(String path, int value)
{
  if (Firebase.RTDB.setInt(&fbdo, path.c_str(), value))
  {
    /*
    // Serial.print("Writing value: ");
    // Serial.print(value);
    // Serial.print(" on the following path: ");
    // Serial.println(path);
    // Serial.println("PASSED");
    // Serial.println("PATH: " + fbdo.dataPath());
    // Serial.println("TYPE: " + fbdo.dataType());
    */
  }
  else
  {
    /*
    // Serial.println("FAILED");
    // Serial.println("REASON: " + fbdo.errorReason());
    */
  }
}

// Write Boolean values to the database
void sendBool(String path, bool value)
{
  if (Firebase.RTDB.setBool(&fbdo, path.c_str(), value))
  {
    /*
    // Serial.print("Writing value: ");
    // Serial.print(value);
    // Serial.print(" on the following path: ");
    // Serial.println(path);
    // Serial.println("PASSED");
    // Serial.println("PATH: " + fbdo.dataPath());
    // Serial.println("TYPE: " + fbdo.dataType());
    */
  }
  else
  {
    /*
    // Serial.println("FAILED");
    // Serial.println("REASON: " + fbdo.errorReason());
    */
  }
}

// Write Data Log in JSON to the database
void sendDataLog()
{
  // Get current timestamp
  timestamp = getTime();
  
  // Serial.print("time: ");
  // Serial.println(timestamp);

  parentPath = dataLogPath + "/" + String(timestamp);

  json.set(tmpLog.c_str(), String(temperature));
  json.set(flwLog.c_str(), String(water_flow));
  json.set(lvlLog.c_str(), String(water_level));
  json.set(turLog.c_str(), String(turbidity));
  json.set(timePath, String(timestamp));

  // Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
}

// Receive Boolean values from the database
bool receiveBool(String path)
{
  int val = 0;

  if (Firebase.RTDB.getBool(&fbdo, path.c_str()))
  {
    if (fbdo.dataType() == "bool")
    {
      val = fbdo.boolData();
    }
  }
  else
  {
    // Serial.println(fbdo.errorReason());
  }
  return val;
}

// Receive Int values from the database
bool receiveInt(String path)
{
  int val = 0;

  if (Firebase.RTDB.getInt(&fbdo, path.c_str()))
  {
    if (fbdo.dataType() == "int")
    {
      val = fbdo.intData();
    }
  }
  else
  {
    // Serial.println(fbdo.errorReason());
  }
  return val;
}

// Parse the Data from master
void parsingData()
{
  indexOfA = dataIn.indexOf("A");
  indexOfB = dataIn.indexOf("B");
  indexOfC = dataIn.indexOf("C");
  indexOfD = dataIn.indexOf("D");  

  temp = dataIn.substring(0, indexOfA);
  level = dataIn.substring(indexOfA + 1, indexOfB);
  turb = dataIn.substring(indexOfB + 1, indexOfC);
  flow = dataIn.substring(indexOfC + 1, indexOfD);  
}
