#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include "mymotor.h"

//These are the ESP32 pins for the Soccer Bot Motor
//make sure to jump EN pins OR THIS WILL NOT WORK

//IN4 goes to D4
const int MotorA1 = 4;
//IN1 to D19
const int MotorB1 = 19;

//IN3 to D18
const int MotorA2 = 18;
//IN2 to D23
const int MotorB2 = 23;

bool isMoving = false;
int leftMotor, rightMotor;
int isForward, isTurn;
Servo leftServo, rightServo;

//DO NOT mess with the code below unless you are working on autonomous butler bots
#pragma region ButlerBot

int DegreeInRadian(double x){
  return x * (3.1415/180);;
}

void initServoLib(){
  // 16 servo objects can be created on the ESP32
  // Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33   
  // Allow allocation of all timers
	leftServo.attach(MotorA1, 1000, 2000);
  rightServo.attach(MotorB1, 1000, 2000);  
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	leftServo.setPeriodHertz(50);
  rightServo.setPeriodHertz(50);


}

void runButlerMotorsWithFids(float id, float distance, float angle){
  
}

void goForward()
{
  leftServo.write(70);
  rightServo.write(70); 
}
void turnLeft()
{
  leftServo.write(65);
  rightServo.write(70); 
}
void turnRight()
{
  leftServo.write(70);
  rightServo.write(65); 
}
void halt()
{
  leftServo.write(90);
  rightServo.write(90); 
}

void turnRightSlowly()
{
  leftServo.write(100);
  rightServo.write(80);
  Serial.println("turnin right");
}

void turnLeftSlowly()
{
  leftServo.write(80);
  rightServo.write(100);
  Serial.println("turnin left");
}

void runButlerMotor(){
  isForward = (int)(rawIntData[2] * sin(rawIntData[3]*PI/180));
  isTurn = (int)(rawIntData[2] * cos(rawIntData[3]*PI/180));
  
  /*
  Serial.print("Cos: ");
  Serial.print(cos(rawIntData[3]*PI/180));
  Serial.print(" Speed: ");
  Serial.print(rawIntData[2]);
  Serial.print(" Angle: ");
  Serial.print(rawIntData[3]);  
  Serial.print(" ForwardVal: ");
  Serial.print(isForward);
  Serial.print(" TurnVal: ");
  Serial.print(isTurn);
  */
    
  if (rawIntData[2] == 0){
    leftServo.write(90);
    rightServo.write(90);
  }

  else {
      if(isForward > 0){
        leftMotor = isForward + isTurn;
        rightMotor = isForward - isTurn;
      }
      else{
        leftMotor = isForward - isTurn;
        rightMotor = isForward + isTurn;
      }

      leftMotor = constrain(leftMotor, -100, 100);
      rightMotor = constrain(rightMotor, -100, 100);
      leftMotor = map(leftMotor, -100, 100, 60, 130);
      rightMotor = map(rightMotor, -100, 100, 60, 130);
      leftServo.write(leftMotor);
      rightServo.write(rightMotor);
  }
  
}

#pragma endregion ButlerBot

//Below is where you want to stay to make Soccer Bots
#pragma region SoccerBot

void initL298N(){
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);

  digitalWrite(MotorA1, LOW);
  digitalWrite(MotorA2, LOW);
  digitalWrite(MotorB1, LOW);
  digitalWrite(MotorB2, LOW);  
}

void calcMotor(){
  isForward = (int)(rawIntData[2] * sin(rawIntData[3]*PI/180));
  isTurn = (int)(rawIntData[2] * cos(rawIntData[3]*PI/180));

  /*

  //this code is to display website readings in the serial to debug

  Serial.print("Cos: ");
  Serial.print(cos(rawIntData[3]*PI/180));
  Serial.print(" Speed: ");
  Serial.print(rawIntData[2]);
  Serial.print(" Angle: ");
  Serial.print(rawIntData[3]);  
  Serial.print(" ForwardVal: ");
  Serial.print(isForward);
  Serial.print(" TurnVal: ");
  Serial.println(isTurn);
  */
    
  if(rawIntData[2] == 0){
    A1PWM = 0;
    A2PWM = 0;
    B1PWM = 0;
    B2PWM = 0;
    isMoving = false;
  }

  else{
    isMoving = true;
  }

  if (isMoving){

    if (rawIntData[3] <= 15 || rawIntData[3] >= 345){
      leftMotor = abs(isForward) + abs(isTurn);
      rightMotor = abs(isForward) + abs(isTurn);
      leftMotor = constrain(leftMotor, 0, 100);
      rightMotor = constrain(rightMotor, 0, 100);
      A2PWM = map(leftMotor, 0, 100, 0, 255);
      A1PWM = 0;
      B2PWM = 0; 
      B1PWM = map(rightMotor, 0, 100, 0, 255);
              
    }
    else if (rawIntData[3] >= 165 && rawIntData[3] <= 195){
      leftMotor = abs(isForward) + abs(isTurn);
      rightMotor = abs(isForward) + abs(isTurn);
      leftMotor = constrain(leftMotor, 0, 100);
      rightMotor = constrain(rightMotor, 0, 100);
      A2PWM = 0;
      A1PWM = map(leftMotor, 0, 100, 0, 255);
      B2PWM = map(rightMotor, 0, 100, 0, 255);
      B1PWM = 0;         
    }    

    else if(isForward < 0 && isTurn > 0){
      leftMotor = abs(isForward) + abs(isTurn);
      rightMotor = abs(isForward);
      leftMotor = constrain(leftMotor, 0, 100);
      rightMotor = constrain(rightMotor, 0, 100);
      A1PWM = map(leftMotor, 0, 100, 0, 255);
      A2PWM = 0;
      B1PWM = map(rightMotor, 0, 100, 0, 255);
      B2PWM = 0;      
    }
    else if(isForward < 0 && isTurn < 0){
      leftMotor = abs(isForward);
      rightMotor = abs(isForward) + abs(isTurn);
      leftMotor = constrain(leftMotor, 0, 100);
      rightMotor = constrain(rightMotor, 0, 100);
      A1PWM = map(leftMotor, 0, 100, 0, 255);
      A2PWM = 0;
      B1PWM = map(rightMotor, 0, 100, 0, 255);
      B2PWM = 0;        
    }
    else if(isForward > 0 && isTurn > 0){
      leftMotor = abs(isForward) + abs(isTurn);
      rightMotor = abs(isForward);
      leftMotor = constrain(leftMotor, 0, 100);
      rightMotor = constrain(rightMotor, 0, 100);      
      A1PWM = 0;
      A2PWM = map(leftMotor, 0, 100, 0, 255);
      B1PWM = 0;
      B2PWM = map(rightMotor, 0, 100, 0, 255);      
    }
    else if(isForward > 0 && isTurn < 0){
      leftMotor = abs(isForward);
      rightMotor = abs(isForward) + abs(isTurn);
      leftMotor = constrain(leftMotor, 0, 100);
      rightMotor = constrain(rightMotor, 0, 100);
      A1PWM = 0;
      A2PWM = map(leftMotor, 0, 100, 0, 255);
      B1PWM = 0;        
      B2PWM = map(rightMotor, 0, 100, 0, 255);
    }
  }
}

void printMotorValues(){
  Serial.print(A1PWM);
  Serial.print(A2PWM);
  Serial.print(B1PWM);
  Serial.println(B2PWM);
}

void runMotor(){
    calcMotor();
  //printMotorValues();
  analogWrite(MotorA1, A1PWM);
  analogWrite(MotorA2, A2PWM);
  analogWrite(MotorB1, B1PWM);
  analogWrite(MotorB2, B2PWM);  
} 

#pragma endregion SoccerBot