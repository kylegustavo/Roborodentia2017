#include <AccelStepper.h>

#define ONE_ROTATION 1600

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 34, 32);

int pos = ONE_ROTATION * .75;
int sleep;

void setup()
{  
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);

  pinMode(30, INPUT_PULLUP);
  pinMode(36, OUTPUT);
  
  sleep = true;
  digitalWrite(36, HIGH);
}

void loop()
{
  if (!digitalRead(30)) {
    if (sleep) {
      sleep = false;
      digitalWrite(36, HIGH);
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
      digitalWrite(36, LOW);
    }
  }
}
