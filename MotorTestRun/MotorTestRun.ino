/******************************************************************************
TestRun.ino
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.

Resources:
TB6612 SparkFun Library

Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457
******************************************************************************/

// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include <SparkFun_TB6612.h>

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define PWMA 13
#define AIN2 12
#define AIN1 11
#define STBY 10
#define BIN1 9
#define BIN2 8
#define PWMB 7

#define PWMC 6
#define CIN2 5
#define CIN1 4
// #define STBY 10
#define DIN1 3
#define DIN2 A5
#define PWMD 2

#define BAD -1

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define ROTATE_L 5
#define ROTATE_R 6

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = -1;
const int offsetB = 1;
const int offsetC = -1;
const int offsetD = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
Motor motor3 = Motor(CIN1, CIN2, PWMC, offsetC, STBY);
Motor motor4 = Motor(DIN1, DIN2, PWMD, offsetD, STBY);

int direction = FORWARD;
int varSpeed = 50;

void brake() {
  motor1.brake();
  motor2.brake();
  motor3.brake();
  motor4.brake();
}

void setup()
{
 //Nothing here
}


void loop()
{
   //Use of the forward function, which takes as arguements two motors
   //and optionally a speed.  If a negative number is used for speed
   //it will go backwards
   switch (direction) { 
      case FORWARD:
        forward(motor1, motor2, 255);
        back(motor3, motor4, 255);
        direction = BACKWARD;
      break;
      
      case BACKWARD:
        forward(motor3, motor4, 255);
        back(motor1, motor2, 255);
        direction = FORWARD;
      break;
      
      case LEFT:
        forward(motor2, motor3, varSpeed);
        back(motor1, motor4, varSpeed);
        direction = RIGHT;
      break;
      
      case RIGHT:
        forward(motor1, motor4, varSpeed);
        back(motor2, motor3, varSpeed);
        if (varSpeed == 250) {
           varSpeed = 50;
        }
        else {
           varSpeed += 50;
        }
        direction = LEFT;
      break;
      
      case ROTATE_L:
        forward(motor1, motor2, 255);
        forward(motor3, motor4, 255);
        direction = ROTATE_R;
      break;
      
      case ROTATE_R:
        back(motor1, motor2, 255);
        back(motor3, motor4, 255);
        direction = FORWARD;
      break;

      case BAD:
        forward(motor2, motor4, 255);
        back(motor1, motor3, 255);
        
      break;
      
   }
   
   delay(5000);
   brake();
}
