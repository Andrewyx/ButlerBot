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
float deserializedFidData[3];
HardwareSerial SerialPort(2);  //if using UART2

void parseData(char* data){
    Json::Reader reader;
    Json::Value cleanedData;
  
    bool parseSuccess = reader.parse(data, cleanedData, false);
    
    if(parseSuccess){
      Json::Value idValue = cleanedData["id"];
      Json::Value distanceValue = cleanedData["distance"];
      Json::Value angleValue = cleanedData["angle"];
      deserializedFidData[0] = idValue.asInt();
      deserializedFidData[1] = distanceValue.asFloat();
      deserializedFidData[2] = angleValue.asFloat();
      //Serial.printf("ID: [%d] Distance: [%d] AngleInRAD: [%d]\n", idValue.asInt(), 
      //        distanceValue.asFloat(), angleValue.asFloat());
      
      Serial.print("ID: ");
      Serial.print(deserializedFidData[0]);
      Serial.print(" Distance: ");
      Serial.print(deserializedFidData[1]);
      Serial.print(" Angle: ");
      Serial.println(deserializedFidData[2]);

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
}

void serialUARTCommunication(){
  if (SerialPort.available())
  {
    String serialData = SerialPort.readStringUntil('\n');
    Serial.println(serialData);
    strcpy(tempstr, serialData.c_str());
    parseData(tempstr);
  }
}
void is_Ahead(float angle){
  int deviationVal = 8;
  int detectionArc = 120;
  float angleDEG = angle*RAD_TO_DEG;
  if(angleDEG >= 90 - deviationVal && angleDEG <= 90 + deviationVal){
    //drive forward
    goForward();
    Serial.println("goin ahead");
  }
  else if(angleDEG > 90 && angleDEG < 90 + detectionArc/2){
    //turn left
    turnLeft();
  }
  else if(angleDEG < 90 && angleDEG > 90 - detectionArc/2){
    //turn right
    turnRight();
  }


}

void setup(){
  init_UART();
}
void loop(){
  serialUARTCommunication();
}