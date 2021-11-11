#include <Servo.h> 
#include <Wire.h>

// Motor Specificatios
int minSpeedInRPM=1450;
int maxSpeedInRPM=1550;
int leftWheelPin = 13;
int rightWheelPin = 12;
signed char centerPosition ;
signed char leftSideOfCenter = -60;
signed char rightSideOfCenter = 60;
signed char positiveRefValue = 10;
signed char negativeRefValue = 10;

// I2C Protocol Specifications
int baudRate = 9600;

Servo servoLeftHandSide;
Servo servoRightHandSide;

void setup() 
{ 
  Serial.begin(baudRate);
  Wire.begin(0x8);                
  Wire.onReceive(receiveEvent);
  servoLeftHandSide.attach(leftWheelPin);
  servoRightHandSide.attach(rightWheelPin);
} 

void loop() 
{
  if(centerPosition > negativeRefValue && centerPosition < positiveRefValue){
    servoRightHandSide.writeMicroseconds(maxSpeedInRPM);
    servoLeftHandSide.writeMicroseconds(minSpeedInRPM);
  } else if(centerPosition > positiveRefValue){
    if ( centerPosition > rightSideOfCenter ){
      centerPosition =rightSideOfCenter;
    }
    float feedBackValue = centerPosition;
    float x =(minSpeedInRPM -feedBackValue);
    servoRightHandSide.writeMicroseconds(minSpeedInRPM);
    servoLeftHandSide.writeMicroseconds(x);
   } else if( centerPosition < negativeRefValue){
     if ( centerPosition < leftSideOfCenter ){
      centerPosition = leftSideOfCenter;
    }
    float position = (-1*centerPosition);
    float adjustedSpeed =(maxSpeedInRPM + position);
    servoRightHandSide.writeMicroseconds(maxSpeedInRPM);
    servoLeftHandSide.writeMicroseconds(adjustedSpeed);
   }
}

/* Get executed whenever data is received from master
  -This function is registered as an event
*/
void receiveEvent(int howMany) 
{
  while (Wire.available()) 
  { 
    Serial.println (Wire.read());
  }
}
