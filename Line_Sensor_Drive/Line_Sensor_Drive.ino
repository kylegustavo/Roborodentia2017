
#define CBUFFER_SIZE 100
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


//state machine values
#define BAD -1

#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define ROTATE_L 5
#define ROTATE_R 6

#define APPROACH_RINGS 1
#define PICKUP_RINGS 2
#define APPROACH_FLAG 3
#define LEAVE_FLAG 4
#define APPROACH_DROP 5
#define DROP 6
#define LEAVE_DROP 7

#define CHANGE_STATE 1

#define DONE 10

#include <SparkFun_TB6612.h>
#include <Servo.h>

#include "Wire.h"
#include "sensorbar.h"

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS_FRONT = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar SensorBarFront(SX1509_ADDRESS_FRONT);

CircularBuffer positionHistory(CBUFFER_SIZE);

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

int state = APPROACH_RINGS;

int armPin = 22;
int tiltPin = 24;
 
Servo servoArm;
Servo servoTilt;

void brake() {
  motor1.brake();
  motor2.brake();
  motor3.brake();
  motor4.brake();
}

void forward(int spdL, int spdR) {
  forward(motor3, motor4, spdR);
  back(motor1, motor2, spdL);
}
void backward(int spdL, int spdR) {
  forward(motor1, motor2, spdL);
  back(motor3, motor4, spdR);
}
void left(int spd) {
  forward(motor2, motor3, spd);
  back(motor1, motor4, spd);
}
void right(int spd) {
  forward(motor1, motor4, spd);
  back(motor2, motor3, spd);  
}
void rotate_l(int spd) {
  forward(motor1, motor2, spd);
  forward(motor3, motor4, spd);
}
void rotate_r() {
  back(motor1, motor2, 255);
  back(motor3, motor4, 255);
}

int forwardUntilChange() {
  uint8_t rawValue = SensorBarFront.getRaw();
  //using middle 4 line sensors for now
  if((rawValue & 0x3C) == 0x3C) {
    //all white, go full speed
    forward(255, 255);
  }
  else if((rawValue & 0x3C) == 0x00) {
    //black horizontal line seen, stop and transition
    brake();
    return CHANGE_STATE;
  }
  else if(!(rawValue & 1 << 4)) {
    forward(215, 255);
  }
  else if(!(rawValue & 1 << 3)) {
    forward(255, 215);
  }
  else if(!(rawValue & 1 << 5)) {
    forward(195, 255);
  }
  else if(!(rawValue & 1 << 2)) {
    forward(255, 195);
  }
  return 0;
}

void turnRight() {
  right(200); 
  delay(1500);
  brake();
   
}

void setup() {
  servoArm.attach(armPin);
  servoTilt.attach(tiltPin); 
  servoArm.write(20);
  delay(200);
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  // put your setup code here, to run once:

  /* setup line sensor arrays */
  //For this demo, the IR will only be turned on during reads.
  SensorBarFront.setBarStrobe();
  //Other option: Command to run all the time
  //mySensorBar.clearBarStrobe();

  //Default dark on light
  SensorBarFront.clearInvertBits();
  //Other option: light line on dark
  //mySensorBar.setInvertBits();
  
  //Don't forget to call .begin() to get the bar ready.  This configures HW.
  uint8_t returnStatus = SensorBarFront.begin();
  if(returnStatus)
  {
    Serial.println("sx1509 IC communication OK");
  }
  else
  {
    Serial.println("sx1509 IC communication FAILED!");
    while(1);
  }
}

void loop() {
  
  switch(state) {
    case APPROACH_RINGS:
      if(forwardUntilChange() == CHANGE_STATE) {
        state = PICKUP_RINGS;
      }
      break;

    case PICKUP_RINGS:
      servoArm.write(160);
      /*add stepper motor or left/right movement */
      delay(1000);
      servoArm.detach();
      state = APPROACH_FLAG;
      break;

    case APPROACH_FLAG: //drive backwards
      break;

    case LEAVE_FLAG:
      if(forwardUntilChange() == CHANGE_STATE) {
        state = APPROACH_DROP;
      }
      turnRight();
      break;

    case APPROACH_DROP:
      if(forwardUntilChange() == CHANGE_STATE) {
        state = DROP;
      }
      break;

    case DROP:
      servoTilt.write(80);
      delay(1500);
      servoTilt.write(100);
      delay(1500);
      state = LEAVE_DROP;
      break;

    case LEAVE_DROP:
      state = APPROACH_RINGS;
      break;

    default:
      break;
  }
  
}
