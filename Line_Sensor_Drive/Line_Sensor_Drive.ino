
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
#define PICKUP_LR_PEGS 22
#define STEPPER 23
#define MECANUM 24
//#define STEPPER_PICKUP_LR_PEGS 23
#define APPROACH_FLAG 3
#define LEAVE_FLAG 4
#define SPECIAL_LEAVE_RINGS 11
#define APPROACH_DROP 5
#define APPROACH_BUMP 6
#define DROP 7
#define LEAVE_DROP_TO_BUMP 8
#define LEAVE_BUMP 9

#define FLIP 0
#define RESET_FLIP 1

#define CHANGE_STATE 1

#define DONE 10

#define FLIP_ONCE 0
#define FLIP_ALWAYS 1

#define ONE_ROTATION 1600

#include <SparkFun_TB6612.h>
#include <Servo.h>
#include <DualMC33926MotorShield.h>
#include <AccelStepper.h>

#include "Wire.h"
#include "sensorbar.h"

const uint8_t SX1509_ADDRESS_FRONT = 0x3E;  // SX1509 I2C address (00)
const uint8_t SX1509_ADDRESS_BACK = 0x3F;  // SX1509 I2C address (01)

SensorBar SensorBarFront(SX1509_ADDRESS_FRONT);
SensorBar SensorBarBack(SX1509_ADDRESS_BACK);

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
Motor *motor1;// = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor *motor2;// = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
DualMC33926MotorShield md; //use for Motor 3 and 4
AccelStepper stepper(AccelStepper::DRIVER, 34, 32);
//Motor motor3 = Motor(CIN1, CIN2, PWMC, offsetC, STBY);
//Motor motor4 = Motor(DIN1, DIN2, PWMD, offsetD, STBY);

int state = APPROACH_RINGS;
int flagFlipped = 0;
int mode = FLIP_ALWAYS;
int pos = ONE_ROTATION * 1.1;
int sleep;
int8_t value;

const int armPin = 31;
const int armPinL = 22;
const int tiltPin = 24;
const int buttonPin = 26;
const int flipperPin = 28;
const int modePin = 30;
const int stepperDir = 32;
const int stepperStep = 34;
const int stepperSleep = 36;
 
Servo servoArm;
Servo servoArmL;
Servo servoTilt;
Servo servoFlip;

void brake() {
  motor1->brake();
  motor2->brake();
  md.setM1Speed(0);
  md.setM2Speed(0);
}

void forward(int spdL, int spdR) {
  double spdM = 0 - 1.5686 * spdL;
  forward(*motor1, *motor2, spdR);
  md.setM1Speed((int)spdM);
  md.setM2Speed((int)spdM);
  //back(motor3, motor4, spdL);
}
void backward(int spdL, int spdR) {
  //forward(motor3, motor4, spdL);
  double spdM = 1.5686 * spdL;
  md.setM1Speed((int)spdM);
  md.setM2Speed((int)spdM);
  back(*motor1, *motor2, spdR);
}
void left(int spd) {
  double spdM = 1.5686 * spd;
  //forward(motor2, motor3, spd);
  //back(motor1, motor4, spd);
  left(*motor1, *motor2, spd);
  md.setM1Speed((int)spdM);
  md.setM2Speed(0 - (int)spdM);
}
void right(int spd) {
  double spdM = 1.5686 * spd;
  //forward(motor1, motor4, spd);
  //back(motor2, motor3, spd);
  right(*motor1, *motor2, spd); 
  md.setM1Speed(0 - (int)spdM);
  md.setM2Speed((int)spdM); 
}
void rotate_l(int spd) {
  double spdM = 1.5686 * spd;
  forward(*motor1, *motor2, spd);
  md.setM1Speed((int)spdM);
  md.setM2Speed((int)spdM);
  //forward(motor3, motor4, spd);
}
void rotate_r(int spd) {
  double spdM = 0 - 1.5686 * spd;
  back(*motor1, *motor2, spd);
  md.setM1Speed((int)spdM);
  md.setM2Speed((int)spdM);
  //back(motor3, motor4, spd);
}

int forwardUntilChange() {
  uint8_t rawValue = SensorBarFront.getRaw();
  //using middle 4 line sensors for now
  if((rawValue & 0x3C) == 0x00) {
    //all white, go full speed
    forward(255, 255);
  }
  else if((rawValue & 0x3C) == 0x3C) {
    //black horizontal line seen, stop and transition
    brake();
    return CHANGE_STATE;
  }
  else if(rawValue & 1 << 4) {
    forward(185, 255);
  }
  else if(rawValue & 1 << 3) {
    forward(255, 185);
  }
  else if(rawValue & 1 << 5) {
    forward(150, 255);
  }
  else if(rawValue & 1 << 2) {
    forward(255, 150);
  }
  return 0;
}

int backwardUntilChange() {
  uint8_t rawValue = SensorBarBack.getRaw();
  //using middle 4 line sensors for now
  if((rawValue & 0x3C) == 0x00) {
    //all white, go full speed
    backward(255, 255);
  }
  else if((rawValue & 0x3C) == 0x3C) {
    //black horizontal line seen, stop and transition
    brake();
    return CHANGE_STATE;
  }
  else if(rawValue & 1 << 3) {
    backward(185, 255);
  }
  else if(rawValue & 1 << 4) {
    backward(255, 185);
  }
  else if(rawValue & 1 << 2) {
    backward(150, 255);
  }
  else if(rawValue & 1 << 5) {
    backward(255, 150);
  }
  return 0;
}

int backwardUntilButton() {
  uint8_t rawValue = SensorBarBack.getRaw();
  //using middle 4 line sensors for now
  if(digitalRead(buttonPin) == LOW) {
    //button pressed, transition
    brake();
    return CHANGE_STATE;
  }
  else if((rawValue & 0x3C) == 0x00) {
    //all white, go full speed
    backward(255, 255);
  }
  else if(rawValue & 1 << 3) {
    backward(185, 255);
  }
  else if(rawValue & 1 << 4) {
    backward(255, 185);
  }
  else if(rawValue & 1 << 2) {
    backward(150, 255);
  }
  else if(rawValue & 1 << 5) {
    backward(255, 150);
  }
  return 0;
}

void turnRight(int isForward) {
  if (isForward) {
    forward(255,255);
  }
  else {
    backward(255,255);
  }
  if(isForward) {
    delay(500);
  }
  else {
    delay(700);
  }
  rotate_r(200);
  if(isForward) { 
    delay(1500);
  }
  else {
    delay(850);
  }
  while(!(SensorBarFront.getRaw() & 1 << 5)) {
    rotate_r(255);
  }
  brake();
  delay(100);
}

void turnLeft() {
  backward(255,255);
  delay(700);
  rotate_l(255);
  delay(1000);
  while(!(SensorBarFront.getRaw() & 1 << 4)) {
    rotate_l(255);
  }
  delay(300);
  brake();
  delay(100);
}

void flip(int val) {

  if (val == FLIP) {
    servoFlip.write(160);
  } else {
    servoFlip.write(20);
  }
}

void lr_pickup(int style) {
    if (style == STEPPER) { //using stepper motor
       pos = ONE_ROTATION * 1.1;
       stepper.moveTo(pos);
       while(stepper.distanceToGo() != 0) {
          stepper.run();
       }
       pos = -pos;
    }
    else { //using mecanum drive
      left(255); //left according to watcher
      delay(1000);
    }
    servoTilt.write(105);
    servoArm.attach(armPin);
    servoArmL.attach(armPinL);
    delay(600);
    brake();
    servoArmL.write(160);
    delay(500);
    forward(255,255);
    delay(500);
    brake();
    servoArmL.write(20);
    delay(500);
    servoTilt.write(110);
    delay(500);
    backward(255,255);
    delay(500);
    brake();
    servoTilt.write(120);

    stepper.moveTo(pos);
    if (style == STEPPER) {
       while(stepper.distanceToGo() != 0) {
          stepper.run();
       }
       pos = 0;
       
    }
    else {
       right(255); //right according to watcher
       delay(1000);
    }
    servoTilt.write(105);
    brake();
    servoArm.write(20);
    delay(500);
    forward(255,255);
    delay(500);
    brake();
    servoArm.write(160);
    delay(400);
    servoTilt.write(110);
    delay(500);
    backward(255,255);
    delay(500);
    brake();
    if (style == STEPPER) {
       stepper.moveTo(pos);
       while(stepper.distanceToGo() != 0) {
          stepper.run();
       }
    }
    else {
       left(255); //left according to watcher
       delay(1000);
    }
    brake();
}

void setup() {
  servoArm.attach(armPin);
  servoArm.attach(armPin);
  servoTilt.attach(tiltPin);
  servoFlip.attach(flipperPin);
  brake();
  servoFlip.write(20);
  md.init();
  pinMode(buttonPin, INPUT_PULLUP); 
  pinMode(modePin, INPUT_PULLUP); 

  if(digitalRead(modePin) == LOW) {
    mode = FLIP_ONCE;
  }
  
  servoArm.write(20); //start in down position
  servoArmL.write(160); //start in down position
  servoTilt.write(104); //start with tilt up
  brake();
  delay(200);
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  // put your setup code here, to run once:

  /* setup line sensor arrays */
  //For this demo, the IR will only be turned on during reads.
  SensorBarFront.setBarStrobe();
  SensorBarBack.setBarStrobe();
  //Other option: Command to run all the time
  //mySensorBar.clearBarStrobe();

  //Default dark on light
  SensorBarFront.clearInvertBits();
  SensorBarBack.clearInvertBits();
  //Other option: light line on dark
  //mySensorBar.setInvertBits();
  stepper.setMaxSpeed(8000);
  stepper.setAcceleration(2000);
  
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
  motor1 = new Motor(AIN1, AIN2, PWMA, offsetA, STBY);
  motor2 = new Motor(BIN1, BIN2, PWMB, offsetB, STBY);
}

void loop() {
  
  switch(state) {
    case APPROACH_RINGS:
      if(forwardUntilChange() == CHANGE_STATE) {
        state = PICKUP_RINGS;
      }
      Serial.println("Approach Rings");
      break;

    case PICKUP_RINGS:
      forward(255,255);
      delay(500);
      brake();
      delay(200);
      servoArm.attach(armPin);
      servoArmL.attach(armPinL);
      servoArm.write(160);
      /*add stepper motor or left/right movement */
      delay(1000);
      servoArm.detach();
      servoTilt.write(110);
      delay(700);
      backward(255,255);
      delay(500);
      brake();
      servoArm.detach();
      servoTilt.write(120);
      state = PICKUP_LR_PEGS;
      break;

    case PICKUP_LR_PEGS:
      //use mecanum drive to pickup from left and right pegs
      lr_pickup(STEPPER);
      servoArm.detach();
      servoArmL.detach();
      servoTilt.write(150);
      if(flagFlipped && mode == FLIP_ONCE) {
        state = SPECIAL_LEAVE_RINGS;
      }
      else {
        state = APPROACH_FLAG;
      }
      break;

    case APPROACH_FLAG: //drive backwards
      if(backwardUntilButton() == CHANGE_STATE) {
        state = LEAVE_FLAG;
        flip(FLIP);
        delay(500);
        flagFlipped = 1;
      }
      break;

    case LEAVE_FLAG:
      if(forwardUntilChange() == CHANGE_STATE) {
        state = APPROACH_BUMP;
        flip(RESET_FLIP);
        turnRight(1);
      }
      break;

    case SPECIAL_LEAVE_RINGS: //special state to skip flag
      if(backwardUntilChange() == CHANGE_STATE) {
        state = APPROACH_BUMP;
        turnRight(0);
      } 
      break;

    case APPROACH_BUMP:
      if(forwardUntilChange() == CHANGE_STATE) {
        state = APPROACH_DROP;
        servoTilt.write(120);
        forward(255, 255);
        delay(1000);
      }
      break;

    case APPROACH_DROP:
      if(forwardUntilChange() == CHANGE_STATE) {
        state = DROP;
      }
      break;

    case DROP:
      forward(255,255);
      delay(1200);
      brake();
      delay(50);
      backward(50,50);
      delay(150);
      brake();
      value = SensorBarFront.getPosition();
      pos = value * -9.32;
      Serial.print("Val: ");
      Serial.println(value);
      Serial.print("Pos: ");
      Serial.println(pos);
      if(abs(value) > 30) {
        stepper.moveTo(pos);
        while(stepper.distanceToGo() != 0) {
            stepper.run();
        }
      }
      for (int i = 0; i <= 0; i++) {
        servoTilt.write(80);
        delay(1000);
        servoTilt.write(102);
        delay(1000);
      }
      pos = 0;
      stepper.moveTo(pos);
      while(stepper.distanceToGo() != 0) {
          stepper.run();
      }
      backward(255,255);
      delay(1000);
      brake();
      state = LEAVE_DROP_TO_BUMP;
      break;

    case LEAVE_DROP_TO_BUMP:
      if(backwardUntilChange() == CHANGE_STATE) {
        state = LEAVE_BUMP;
        backward(255,255);
        delay(1000);
      }

      break;

    case LEAVE_BUMP:
      if(backwardUntilChange() == CHANGE_STATE) {
        state = APPROACH_RINGS;
        turnLeft();
        servoArm.attach(armPin);
        servoArm.write(20);
        servoArmL.attach(armPinL);
        servoArmL.write(160);
        servoTilt.write(104);
        delay(200);
      }
      break;

    default:
      break;
  }
  
}
