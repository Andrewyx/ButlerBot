#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <HardwareSerial.h>
#include <string>
#include <json/value.h>
#include <json/json.h>
#include <typeinfo>
#include "mymotor.h"

using namespace std;
char tempstr[256];

float deserializedFidData[6];
/*
deserializedFidData[0] = Aruco ID
deserializedFidData[1] = Distance from screen center to marker center
deserializedFidData[2] = Angle of the aforementioned distance to the xy plane
deserializedFidData[3] = Boolean value that is true when an offcentered marker appears on screen
deserializedFidData[4] = Rotational Vector of the Fiducial relative to the camera. (Only the roll component in Euler Angles)
deserializedFidData[5] = Boolean value that is true when the Fiducial is within the bounding box (ie: initiate turning)
*/

HardwareSerial SerialPort(2);  //if using UART2, open this 2nd port for RX2 and TX2

//This region contains the necessary serial communication functions for the ESP32
#pragma region UARTSerial

void parseData(char* data){
    Json::Reader reader;
    Json::Value cleanedData;
  
    bool parseSuccess = reader.parse(data, cleanedData, false);
    
    if(parseSuccess){
      Json::Value idValue = cleanedData["id"];
      Json::Value distanceValue = cleanedData["distance"];
      Json::Value angleValue = cleanedData["angle"];
      Json::Value isOffCourse = cleanedData["isoffcourse"];
      Json::Value RVec = cleanedData["RVec"];
      Json::Value Reorientate = cleanedData["Reorientate"];
      deserializedFidData[0] = idValue.asInt();
      deserializedFidData[1] = distanceValue.asFloat();
      deserializedFidData[2] = angleValue.asFloat();
      deserializedFidData[3] = isOffCourse.asInt();
      deserializedFidData[4] = RVec.asFloat();
      deserializedFidData[5] = Reorientate.asInt();
      //Serial.printf("ID: [%d] Distance: [%d] AngleInRAD: [%d]\n", idValue.asInt(), 
      //        distanceValue.asFloat(), angleValue.asFloat());
      
      Serial.print("ID: ");
      Serial.print(deserializedFidData[0]);
      Serial.print(" Distance: ");
      Serial.print(deserializedFidData[1]);
      Serial.print(" Angle: ");
      Serial.print(deserializedFidData[2]);
      Serial.print(" IsOffCourse: ");
      Serial.print(deserializedFidData[3]);
      Serial.print(" RVec: ");
      Serial.print(deserializedFidData[4]);
      Serial.print(" isReorientating :");
      Serial.println(deserializedFidData[5]);
    }
} 
/*TODO: 
PGRM motor controls
PGRM Deserialzaton funcs
PGRM is_Ahead funcs
PRGM orientation funcs
*/

void init_UART(){
  int BaudRate = 9600;
  SerialPort.begin (BaudRate, SERIAL_8N1, 16, 17);
  Serial.begin(9600);
  //begins serial coms and enables UART on RX2 and TX2 pins for ESP32
}

void serialUARTCommunication(){
  if (SerialPort.available())
  {
    //Reads Json data and stores it within an Arduino String
    //This is then converted to a const C-String and copied into a temp char ptr
    //Data is then parsed into a usable array as the deserializedFidData
    String serialData = SerialPort.readStringUntil('\n');
    strcpy(tempstr, serialData.c_str());
    parseData(tempstr);
  }
}

#pragma endregion UARTSerial

//This region contains functions that calculate devating angles and issues compensating motor functions
#pragma region DriveFunctions

void fixOrientation(float RVecData, const float deviationInDeg){
  /*
  Takes in the rotational vector data as Euler Angles along with a constant deviation angle
  This function will turn the robot while the fiducial is directly above the robot and 
  thus, inside the bounding box
  */
  if(RVecData >  deviationInDeg)
  {
    turnRightSlowly();
  }
  else if (RVecData < -deviationInDeg)
  {
    turnLeftSlowly();
  }
  else{
    goForward();
  }
}

void is_Ahead(float angle){
  /*
  Takes in radians as a float and issues motor commands while the center of the tracking 
  fiducial is outside of the bounding box
  */
  int deviationVal = 8;
  int detectionArc = 120;
  float angleDEG = angle*RAD_TO_DEG;
  if(angleDEG >= 90 - deviationVal && angleDEG <= 90 + deviationVal){
    //drive forward
    goForward();
    Serial.println("goin ahead");
  }
  else if(angleDEG > 90 && angleDEG < (90 + detectionArc/2)){
    //turn left
    turnLeft();
  }
  else if(angleDEG < 90 && angleDEG > (90 - detectionArc/2)){
    //turn right
    turnRight();
  }
}

#pragma endregion DriveFunctions

void setup(){
  init_UART();
}
void loop(){
  if (int(deserializedFidData[3]) == 0)
  {
    goForward();
  }
  else if (int(deserializedFidData[5]) == 1)
  {
    fixOrientation(deserializedFidData[4], 10);
  }
  
  else if (int(deserializedFidData[3]) == 0 && int(deserializedFidData[5]) == 0)
  {
    is_Ahead(deserializedFidData[2]);
    //off course and not reorientating  
  }
  
  serialUARTCommunication();
}