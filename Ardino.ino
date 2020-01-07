#include <Servo.h> 
#include <Wire.h>

Servo servoLeft;
Servo servoRight;
float BaseSpeed=1450;
float BaseSpeed1=1550;


signed char c;
signed char cn=-60;
signed char cp=60;



void setup() 
{ 
  Serial.begin(9600); 
  Wire.begin(0x8);                
  Wire.onReceive(receiveEvent);
  servoLeft.attach(13);
  servoRight.attach(12);
} 

void loop() 
{
 
  if(c > -10 && c <10) 
   {
    servoRight.writeMicroseconds(BaseSpeed1);
    servoLeft.writeMicroseconds(BaseSpeed);
   }
   else if(c > 10) 
   {
    if ( c > cp ){c =cp;}
    float y = c;
    float x =(BaseSpeed-y);
    servoRight.writeMicroseconds(BaseSpeed);
    servoLeft.writeMicroseconds(x);
   }
    else if( c < -10 ) 
   {
    if ( c < cn ){c =cn;}
    float p= (-1*c);
    
    float x1 =(BaseSpeed1+p);  
    servoRight.writeMicroseconds(BaseSpeed1);
    servoLeft.writeMicroseconds(x1);
   }
   
   
   
}


void receiveEvent(int howMany) 
{
  while (Wire.available()) 
  { 
  
    c = Wire.read(); 
    Serial.println (c);
    
  }
}
