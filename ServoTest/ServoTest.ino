#include <Servo.h> 
 
int armPin = 22;
int tiltPin = 24;
 
Servo servoArm;
Servo servoTilt;  
 
int angle = 0;   // servo position in degrees 
 
void setup() 
{ 
  servoArm.attach(armPin);
  servoTilt.attach(tiltPin); 
} 
 
 
void loop() 
{ 
  servoArm.write(160);//up position
  delay(1500);
  servoArm.detach();
  servoTilt.write(80);
  delay(1500);
  servoTilt.write(100);
  delay(1500);
  servoArm.attach(armPin);
  servoArm.write(20);//down position
  delay(1500); 
} 
