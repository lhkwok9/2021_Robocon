#include "PID.h"

#define numMotor 6
#define maxMotorOP 3400
const int motorEncoderMapping[numMotor] = {1, 2, 3, 4, 5, 6};
const int motorSign[numMotor] = {1, 1, 1, 1, 1, 1};

const double velCountPeriods = (double)loopPeriod / 1000.0;
const int velCountFactor = 1000 / loopPeriod;

// Position  PID gains
const int defaultpKp = 9000;
const int defaultpKi = 0;
const int defaultpKd = 300000;
// Velocity PID gains
const int defaultvKp = 300;
const int defaultvKi = 6500;
const int defaultvKd = 0;

struct motor
{
  int ioA, ioB, pwm;
  int encA, encB;
  volatile long pos, lastPos;
  volatile int vel;
  int inputVel, targetVel;
  int velInc, velDec;
  PID posPid = PID(defaultpKp, defaultpKi, defaultpKd, -maxMotorOP, maxMotorOP, loopPeriod/**2*/);
  PID velPid = PID(defaultvKp, defaultvKi, defaultvKd, -maxMotorOP, maxMotorOP, loopPeriod/**2*/);
}motor[numMotor];

void initMotor()
{
  motor[0].ioA = 51; motor[0].ioB = 52; motor[0].pwm = 7; motor[0].encA = 34; motor[0].encB = 33;
  motor[1].ioA = 42; motor[1].ioB = 41; motor[1].pwm = 6; motor[1].encA = 36; motor[1].encB = 35;
  motor[2].ioA = 43; motor[2].ioB = 44; motor[2].pwm = 5; motor[2].encA = 38; motor[2].encB = 37;
  motor[3].ioA = 45; motor[3].ioB = 46; motor[3].pwm = 4; motor[3].encA = 40; motor[3].encB = 39;
  motor[4].ioA = 47; motor[4].ioB = 48; motor[4].pwm = 3; motor[4].encA = 26; motor[4].encB = 25;
  motor[5].ioA = 49; motor[5].ioB = 50; motor[5].pwm = 2; motor[5].encA = 28; motor[5].encB = 27;

  for (int i=0; i<numMotor; i++)
  {
    // Motor driver
    pinMode(motor[i].ioA, OUTPUT);
    pinMode(motor[i].ioB, OUTPUT);
    pinMode(motor[i].pwm, OUTPUT);
    digitalWrite(motor[i].ioA, HIGH);
    digitalWrite(motor[i].ioB, HIGH);
    // Change PWM resolution to 12bit (0-4095)
    analogWriteResolution(12);
    analogWrite(motor[i].pwm, 0);

    //Encoder
    pinMode(motor[i].encA, INPUT_PULLUP);
    pinMode(motor[i].encB, INPUT_PULLUP);
    motor[i].pos = motor[i].lastPos = motor[i].vel = 0;
    motor[i].inputVel = motor[i].targetVel = 0;
    motor[i].velInc = 5;
    motor[i].velDec = 10;

    // PID
    motor[i].posPid.SetMode(AUTOMATIC);
    motor[i].velPid.SetMode(AUTOMATIC);
  }
  
  attachInterrupt(motor[0].encA, doEncoder1, RISING);
  attachInterrupt(motor[1].encA, doEncoder2, RISING);
  attachInterrupt(motor[2].encA, doEncoder3, RISING);
  attachInterrupt(motor[3].encA, doEncoder4, RISING);
  attachInterrupt(motor[4].encA, doEncoder5, RISING);
  attachInterrupt(motor[5].encA, doEncoder6, RISING);
}

void runMotor(int motorNum, int value)
{
  if (motorNum >= 1 && motorNum <= numMotor)
  {
    value = constrain(value, -maxMotorOP, maxMotorOP) * motorSign[motorNum-1];
    if (value >= 0)
    {
      digitalWrite(motor[motorNum-1].ioA, HIGH);
      digitalWrite(motor[motorNum-1].ioB, LOW);
      analogWrite(motor[motorNum-1].pwm, value);
    }
    else
    {
      digitalWrite(motor[motorNum-1].ioA, LOW);
      digitalWrite(motor[motorNum-1].ioB, HIGH);
      analogWrite(motor[motorNum-1].pwm, -value);
    }
  }
}

void stopMotor(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
  {
    digitalWrite(motor[motorNum-1].ioA, HIGH);
    digitalWrite(motor[motorNum-1].ioB, HIGH);
    analogWrite(motor[motorNum-1].pwm, 0);
  }
}

void brakeMotor(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
  {
    digitalWrite(motor[motorNum-1].ioA, LOW);
    digitalWrite(motor[motorNum-1].ioB, LOW);
    analogWrite(motor[motorNum-1].pwm, 4095);
  }
}

void goTargetMotorPos(int motorNum, long value)
{
  if (motorNum >= 1 && motorNum <= numMotor)
  {
    motor[motorNum-1].posPid.SetSetpoint(value);
    motor[motorNum-1].posPid.SetInput(motor[motorEncoderMapping[motorNum-1]-1].pos);
    motor[motorNum-1].posPid.Compute();
    runMotor(motorNum, motor[motorNum-1].posPid.GetOutput());
  }
}

void goTargetMotorVel(int motorNum, int value)
{
  if (motorNum >= 1 && motorNum <= numMotor)
  {
    motor[motorNum-1].velPid.SetSetpoint(value);
    motor[motorNum-1].velPid.SetInput(motor[motorEncoderMapping[motorNum-1]-1].vel);
    motor[motorNum-1].velPid.Compute();
    runMotor(motorNum, motor[motorNum-1].velPid.GetOutput());
  }
}

long getMotorPos(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
    return motor[motorNum-1].pos;
  else
    return 0;
}

int getMotorVel(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
    return motor[motorNum-1].vel;
  else
    return 0;
}

void calMotorVel()
{
  for (int i=0; i<numMotor; i++)
  {
    motor[i].vel = (motor[i].pos - motor[i].lastPos) * velCountFactor;
    motor[i].lastPos = motor[i].pos;
  }
}

void resetAllMotors()
{
  for (int i=0; i<numMotor; i++)
  {
    motor[i].pos = motor[i].lastPos = motor[i].vel = 0;
    motor[i].inputVel = motor[i].targetVel = 0;
    motor[i].posPid.Initialize();
  }
}

void resetMotor(int motorNum)
{  
  if (motorNum >= 1 && motorNum <= numMotor)
  {
    motor[motorNum-1].pos = motor[motorNum-1].lastPos = motor[motorNum-1].vel = 0;
    motor[motorNum-1].inputVel = motor[motorNum-1].targetVel = 0;
    motor[motorNum-1].posPid.Initialize();
  }
}


void doEncoder1()
{
  if (!!(PIOC->PIO_PDSR & (1<<2)) == !!(PIOC->PIO_PDSR & (1<<1)))
    motor[0].pos++;
  else
    motor[0].pos--;
}

void doEncoder2()
{
  if (!!(PIOC->PIO_PDSR & (1<<4)) == !!(PIOC->PIO_PDSR & (1<<3)))
    motor[1].pos++;
  else
    motor[1].pos--;  
}

void doEncoder3()
{
  if (!!(PIOC->PIO_PDSR & (1<<6)) == !!(PIOC->PIO_PDSR & (1<<5)))
    motor[2].pos++;
  else
    motor[2].pos--;
}

void doEncoder4()
{
  if (!!(PIOC->PIO_PDSR & (1<<8)) == !!(PIOC->PIO_PDSR & (1<<7)))
    motor[3].pos++;
  else
    motor[3].pos--;
}

void doEncoder5()
{
  if (!!(PIOD->PIO_PDSR & (1<<1)) == !!(PIOD->PIO_PDSR & (1<<0)))
    motor[4].pos++;
  else
    motor[4].pos--;
}

void doEncoder6()
{
  if (!!(PIOD->PIO_PDSR & (1<<3)) == !!(PIOD->PIO_PDSR & (1<<2)))
    motor[5].pos++;
  else
    motor[5].pos--;
}

int getMotorVelPTerm(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
    return motor[motorNum-1].velPid.GetPTerm();
  else
    return 0;
}

int getMotorVelITerm(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
    return motor[motorNum-1].velPid.GetITerm();
  else
    return 0;
}

int getMotorVelDTerm(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
    return motor[motorNum-1].velPid.GetDTerm();
  else
    return 0;
}

int getMotorPosPTerm(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
    return motor[motorNum-1].posPid.GetPTerm();
  else
    return 0;
}

int getMotorPosITerm(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
    return motor[motorNum-1].posPid.GetITerm();
  else
    return 0;
}

int getMotorPosDTerm(int motorNum)
{
  if (motorNum >= 1 && motorNum <= numMotor)
    return motor[motorNum-1].posPid.GetDTerm();
  else
    return 0;
}
