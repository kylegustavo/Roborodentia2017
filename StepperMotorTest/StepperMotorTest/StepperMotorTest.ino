#include <AccelStepper.h>

#define ONE_ROTATION 1600

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

int pos = ONE_ROTATION * 1.1;
int sleep;

void setup()
{  
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);

  pinMode(10, INPUT_PULLUP);
  pinMode(11, OUTPUT);
  
  sleep = true;
  digitalWrite(11, HIGH);
}

void loop()
{
  if (!digitalRead(10)) {
    if (sleep) {
      sleep = false;
      digitalWrite(11, HIGH);
    }
    if (stepper.distanceToGo() == 0)
    {
      delay(500);
      pos = -pos;
      stepper.moveTo(pos);
    }
    stepper.run();
  }
  else {
    if (!sleep) {
      sleep = true;
      digitalWrite(11, LOW);
    }
  }
}
