#include "PID.h"
// Chassis physical dimension in mm
//const int motorLocation[4] = {1, 5, 2, 3}; /*fl, fr, rl, rr*/
const int motorLocation[4] = {3, 2, 5, 1}; /*fl, fr, rl, rr*/
const int chassisR = sqrt(400*400+400*400); /*mm*/
const int chassisDia = chassisR * 2;
const int wheelDia = 150; /*mm*/
const int gearRatio = 12; //New motor
const int countPerRev = gearRatio*500;
const int mmPerKiloCount = (PI*wheelDia/countPerRev) * 1000; /*mm/kilocount*/

const double motorVelFactorX = 0.70711;
const double motorVelFactorY = 0.70711;
const double motorVelFactorT = 0.25/chassisDia/2;

long desiredPos[4] = {0};
int flVel, frVel, rlVel, rrVel;
int flPos, frPos, rlPos, rrPos;


void goTargetChassisVel(int xVel, int yVel, int tVel)
{
  calMotorVel();
  tVel/=100;
  flVel = ( xVel + yVel - 83*tVel) * 1000 / mmPerKiloCount;
  frVel = ( xVel - yVel - 83*tVel) * 1000 / mmPerKiloCount;
  rlVel = (-xVel + yVel - 83*tVel) * 1000 / mmPerKiloCount;
  rrVel = (-xVel - yVel - 83*tVel) * 1000 / mmPerKiloCount;

  goTargetMotorVel(motorLocation[0], flVel);
  goTargetMotorVel(motorLocation[1], frVel);
  goTargetMotorVel(motorLocation[2], rlVel);
  goTargetMotorVel(motorLocation[3], rrVel);
}

//void motorVel2ChassisVel(int flVel, int frVel, int rlVel, int rrVel, int *xVel, int* yVel, int* tVel)
//{
//  *xVel = (flVel + frVel - rlVel - rrVel) * mmPerKiloCount / 1000 * motorVelFactorX;
//  *yVel = (flVel - frVel + rlVel - rrVel) * mmPerKiloCount / 1000 * motorVelFactorY;
//  *tVel = (flVel + frVel + rlVel + rrVel) * mmPerKiloCount / 1000 * motorVelFactorT;
//}

void chassis2MotorKinematics(int x, int y, int t, int* fl, int* fr, int* rl, int* rr)
{
  *fl = ( x + y - 1.4142 * chassisR * t) * 1000 / mmPerKiloCount;
  *fr = ( x - y - 1.4142 * chassisR * t) * 1000 / mmPerKiloCount;
  *rl = (-x + y - 1.4142 * chassisR * t) * 1000 / mmPerKiloCount;
  *rr = (-x - y - 1.4142 * chassisR * t) * 1000 / mmPerKiloCount;
}


void goTargetChassisPos(int xPos, int yPos, int tPos)
{
  flPos = ( xPos + yPos - 1.4142 * chassisR * tPos) * 1000 / mmPerKiloCount;
  frPos = ( xPos - yPos - 1.4142 * chassisR * tPos) * 1000 / mmPerKiloCount;
  rlPos = (-xPos + yPos - 1.4142 * chassisR * tPos) * 1000 / mmPerKiloCount;
  rrPos = (-xPos - yPos - 1.4142 * chassisR * tPos) * 1000 / mmPerKiloCount;
  
  goTargetMotorPos(motorLocation[0], flPos);
  goTargetMotorPos(motorLocation[1], frPos);
  goTargetMotorPos(motorLocation[2], rlPos);
  goTargetMotorPos(motorLocation[3], rrPos);
}

void getChassisPos(int * xPos, int * yPos, int * tPos)
{
  //int xPos; int yPos; int tPos;
  int flPos;int frPos;int rlPos; int rrPos;
  flPos = getMotorPos(motorLocation[0]);
  frPos = getMotorPos(motorLocation[1]);
  rlPos = getMotorPos(motorLocation[2]);
  rrPos = getMotorPos(motorLocation[3]);
  *xPos = (flPos + frPos - rlPos - rrPos) * mmPerKiloCount / 1000 / 3 * motorVelFactorX;
  *yPos = (flPos - frPos + rlPos - rrPos) * mmPerKiloCount / 1000 / 3 * motorVelFactorY;
  *tPos = (flPos + frPos + rlPos + rrPos) * mmPerKiloCount / 1000 / 3 * motorVelFactorT;
  Serial.print(*xPos);
  Serial.print( " ");
  Serial.print(*yPos);
  Serial.print( " "); 
  Serial.print(*tPos);
  Serial.println( " ");
}

void resetChassisPos()
{
  for (int i=0; i<4; i++)
    resetMotor(motorLocation[i]);
}

/************************Require Gyro*****************************/
const int turnMotorOP = 2000;
//Turning PID
const int defaulttKp = 25000;
const int defaulttKi = 0;
const int defaulttKd = 360000;//350000;

PID turnPid = PID(defaulttKp, defaulttKi, defaulttKd, -2000, 2000, loopPeriod);

void initChassisTPID()
{
  turnPid.SetMode(AUTOMATIC);
}

int getChassisTvelGyro(int tune_rate, int gyro_error_threshold)
{
  
  turnPid.SetSetpoint(0);
  turnPid.SetInput(-roundf(getGyroError(0))*tune_rate);
  turnPid.Compute();
//  Serial.println(turnPid.GetOutput());
  return turnPid.GetOutput();

}

void goTargetGlobalChassisVel(int xVel, int yVel, int tVel){
  int localX, localY,localT;
  double angle = getGyroError(0)*M_PI/180;
  localT = tVel;
  double Cangle = cos(angle), Sangle = sin(angle);
  localX = xVel*Cangle+yVel*Sangle;
  localY = yVel*Cangle-xVel*Sangle;
  goTargetChassisVel(-localX, -localY, localT);
}
