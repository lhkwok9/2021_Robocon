#include <Servo.h>

const int servoPort[] = {11, 10, 8, 9};
const int numServo = 4;

Servo servo[numServo];

void initServo()
{
  for (int i=0; i<numServo; i++)
  {
    servo[i].attach(servoPort[i]);
    servo[i].write(155);
  }
}

void setServoAngle(int servoNum, int newAngle)
{
  if (servoNum >= 1 && servoNum <= numServo)
    servo[servoNum-1].write(newAngle);
}
