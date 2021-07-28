#define PS4Controller ESP32   // RASPI OR ESP32 OR Comment whole line for VEX
#include "Constant.h"

const int loopPeriod = 5; // Update rate
unsigned long nowTime;
unsigned long lastTime;
/////// Manual ////////
#ifndef PS4Controller
const int controllerIONum = 12;
#else
const int controllerIONum = 16;
#endif
int controllerIO[controllerIONum] = {0};
int lastControllerIO[controllerIONum] = {0};
int controllerIOToggle[controllerIONum] = {0};
unsigned long buttonPressTime[controllerIONum] = {0};
int controllerADC[4] = {0, 0, 0, 0};
const int controllerThreshold = 15;
const int controllerReturnThreshold = 30;
int targetXvel = 0;
int targetYvel = 0;
int targetTvel = 0;
//const int numLP = 20;
const int numLP = 40;
int inputXvel[numLP] = {0};
int inputYvel[numLP] = {0};
int inputTvel[numLP] = {0};
bool safety = true;
////////////////////////
//////// Auto //////////
const int numSequence = 11;
int sequencPoint[numSequence] = {1, 2, 3, 2, 4, 2, 5, 2, 6, 2, 7}; //Point sequence
int pointState = 0;

int targetXpos = 0;
int targetYpos = 0;
int targetTpos = 0;
bool manualMode = true;
bool brakemode = false;
bool redField = true;
bool autoPathStart = false;
unsigned long autoPathStartTime = 0;
unsigned long stayStartTime = 0;
int currentPosition = 1;
int targetPosition = 0;
int runningEdge = 0;
unsigned long runningTime = 0;
#define RESET 1
#define RUNNING 2
#define STOP 3
#define STAY 4
int PATH_STATE = RESET;
////////////////////////

int state = 0;

int servoStatus = 0;
//int arrowDownDone = 1;
int arrowPackDone = 1, shootDone = 1, shootSeqDone = 1;
int shootState = 0;

const int pressure_low = 420;
const int pressure_mid = 1500;
const int pressure_high = 2000;
int pressure = pressure_low;

const int DOWNANGLE_UP = 157;
const int DOWNANGLE_DOWN = 55;
unsigned long arrowDownTime, arrowPackTime, shootTime, shootSeqTime;

void setup()
{
  initPneumatic();
  initMotor();
  //  initChassisTPID();
  //  initTFT();
  initCom();
  //  initLineSensor();
  //  initIOADC();
  initServo();
  //  initGyro();
  initIOADC();
  initLaser(6, 1802, 4074, 1000, 4762);
  initDac();
  //  loadPoint(redField);
  //  if (getIO(1) == 0)
  //  {
  //    loadPoint(redField);
  //    Serial.println("Red");
  //  }
  //  else
  //  {
  //    loadPoint(!redField);
  //    Serial.println("Blue");
  //  }
  //  initPath();
  //  currentPosition = sequencPoint[pointState];
  Serial.begin(115200);
  //setPneu(1, LOW);
  setPneu(7, LOW);
  setPneu(2, HIGH);
  setPneu(5, HIGH);   // For Dist Sensor
  setPneu(3, LOW);
  setDac(0, pressure);
  lastTime = millis();
}

void loop()
{
  getController();
  safety = getSafety();
  getLaserSensorDistance(6);
  //  getLineSensor();
  //  getComData();
  //  getGyro();
  //  debugGyro();
  //  getChassisPos(&targetXpos, &targetYpos, &targetTpos);

  nowTime = millis();
  if (nowTime - lastTime >= loopPeriod)
  {
    lastTime = nowTime;
    if (safety)
      //    if(1)
    {
      // printDebug();
      blinkLED();
      getToggle();
      //      Serial.println(getLaserSensorDistance(6));
      //      Serial.println(tuningLaser(6));
      //      calMotorVel();
      //      goTargetMotorVel(1, 24000);
      //      runMotor(1, 4000);
      //      Serial.println(getMotorVel(1));

      if (controllerIOToggle[CIRCLE] == 1) {
        /*  DAC       Dist
            340       1750
            835       3172
           1195       4102
           1351       4556
        */
        pressure = int(float(getLaserSensorDistance(6) - 850 ) / 2.76);
        setDac(0, pressure);
        Serial.println(pressure);
      }

      //  L7 shoot near
      if (controllerIOToggle[L7] == 1) {
        pressure = pressure_low;
        setDac(0, pressure);
      }
      //  U7 shoot middle
      if (controllerIOToggle[U7] == 1) {
        pressure = pressure_mid;
        setDac(0, pressure);
      }
      //  R7 shoot far
      if (controllerIOToggle[R7] == 1) {
        pressure = pressure_high;
        setDac(0, pressure);
      }
      //  L1 fine tune +10
      if (controllerIOToggle[L1] == 1) {
        pressure += 15;
        setDac(0, pressure);
      }
      //  L2 fine tune -10
      if (controllerIOToggle[L2] == 1) {
        pressure -= 15;
        setDac(0, pressure);
      }

      //  D7 shoot
      if (controllerIOToggle[D7] == 1) {
        shootTime = buttonPressTime[D7];
        shootDone = 0;
        //setPneu(1, HIGH);   //  up
        setPneu(7, HIGH);
        Serial.print(pressure); Serial.print("   "); Serial.println(getLaserSensorDistance(6));
      }
      if (millis() - shootTime > 1000 && shootDone == 0) {
        //setPneu(1, LOW);      //  down
        setPneu(7, LOW);
        shootDone = 1;
      }

      //  D8 shoot
      if (controllerIOToggle[D8] == 1) {
        if (shootState % 5 == 0) {
          shootSeqTime = buttonPressTime[D8];
          shootSeqDone = 0;
          //setPneu(1, HIGH);   //  up
          setPneu(7, HIGH);
          Serial.print(pressure); Serial.print("   "); Serial.println(getLaserSensorDistance(6));
          shootState++;
        }


      }
      if (millis() - shootSeqTime > 1000 && shootState % 5 == 1) {
        //setPneu(1, LOW);           //  down shooter
        setPneu(7, LOW);
        shootState++;
      }
      if (millis() - shootSeqTime > 1600 && shootState % 5 == 2) {
        setServoAngle(4, DOWNANGLE_DOWN);      //  down arrow
        if (shootState % 10 == 2) {
          pressure = pressure_high;
          setDac(0, pressure);
        } else if (shootState % 10 == 7) {
          pressure = pressure_low;
          setDac(0, pressure);
        }
        shootState++;
      }
      if (millis() - shootSeqTime > 2400 && shootState % 5 == 3) {
        setServoAngle(4, DOWNANGLE_UP);      //  up arrow
        setPneu(3, HIGH);
        shootState++;
      }
      if (millis() - shootSeqTime > 2700 && shootState % 5 == 4) {
        setPneu(3, LOW);
        shootState++;
      }


      //  down arrow
      //      if (controllerIOToggle[D8] == 1){
      //          setPneu(1, LOW);      //  shooter down
      //          setServoAngle(4, DOWNANGLE_DOWN);
      //        arrowDownTime = buttonPressTime[D8];
      //        arrowDownDone = 0;
      //        if (servoStatus % 2 == 0) {
      //          //  down
      //          setServoAngle(4, DOWNANGLE_DOWN);
      //        } else {
      //          //  up
      //          setServoAngle(4, DOWNANGLE_UP);
      //        }
      //        servoStatus++;
      //      }
      //    if (millis() - arrowDownTime > 100 && arrowDownDone == 0){
      //        setServoAngle(4, 40);
      //        arrowDownDone = 1;
      //    }

      //  pack arrow
      if (controllerIOToggle[L8] == 1) {
        arrowPackTime = buttonPressTime[L8];
        arrowPackDone = 0;
        setPneu(3, HIGH);     //  in
        setServoAngle(4, DOWNANGLE_UP);
      }
      if (millis() - arrowPackTime > 300 && arrowPackDone == 0) {
        setPneu(3, LOW);      //  out
        arrowPackDone = 1;
      }

      //  hold pun
      if (controllerIOToggle[U8] == 1) {
        setPneu(4, !getPneu(4));
      }
      //  spare
      if (controllerIOToggle[R8] == 1) {
        //setPneu(5, !getPneu(5));
        setPneu(6, !getPneu(6));
      }

      //      if (controllerIOToggle[R7] == 1){
      //
      //        if(state % 4 == 0){
      //          setPneu(4, !getPneu(4));
      //        }else if(state % 4 == 1){
      //          setPneu(4, !getPneu(4));
      //        }else if(state % 4 == 2){
      //          setPneu(5, !getPneu(5));
      //        }else if(state % 4 == 3){
      //          setPneu(5, !getPneu(5));
      //        }
      //
      //        state++;
      //
      //    }

      //      if (controllerIOToggle[D8] == 1 && brakemode == false) {
      //        brakemode = true;
      //        // Serial.println(brakemode);
      //        brakeMotor(1);
      //        brakeMotor(2);
      //        brakeMotor(3);
      //        brakeMotor(4);
      //        brakeMotor(5);
      //        brakeMotor(6);
      //      } else if ((controllerIOToggle[D8] == 1 || controllerIOToggle[L1] == 1) && brakemode == true) {
      //        brakemode = false;
      //        setCurrentGyroPos();
      //      } else;

      //      if (controllerIOToggle[U7] == 1 && autoPathStart == false)
      //      {
      //        //Next state
      //        pointState++;
      //        if (pointState >= numSequence)
      //        {
      //          pointState = numSequence - 1;
      //        }
      //        else
      //        {
      //          PATH_STATE = RESET;
      //          manualMode = false;
      //          targetPosition = sequencPoint[pointState];
      //        }
      //      }

      //      if (controllerIOToggle[D7] == 1 && autoPathStart == false)
      //      {
      //        //Next state
      //        pointState--;
      //        if (pointState < 0)
      //        {
      //          pointState = 0;
      //        }
      //        else
      //        {
      //          PATH_STATE = RESET;
      //          manualMode = false;
      //          targetPosition = sequencPoint[pointState];
      //        }
      //      }


      //    if (controllerIOToggle[U7] == 1 && currentPosition == 2 && autoPathStart == false)
      //    {
      //      PATH_STATE = RESET;
      //      manualMode = false;
      //      targetPosition = 1;
      //    }

      //      //Danger stop
      //      if ((controllerIOToggle[U7] == 1 || controllerIO[D7] == 1)  && autoPathStart == true)
      //      {
      //        manualMode = true;
      //        autoPathStart = false;
      //        resetChassisPos();
      //        targetPosition = 0;
      //      }

      if (manualMode == true &&  brakemode == false)
      {
        /////////////////////////////
        // Chassis velocity control by controller
        /////////////////////////////
        targetYvel = controllerADC[ADC3];
        targetXvel = controllerADC[ADC4];
        targetTvel = controllerADC[ADC1];

        if (abs(targetXvel) < controllerThreshold)
          targetXvel = 0;
        if (abs(targetYvel) < controllerThreshold)
          targetYvel = 0;
        if (abs(targetTvel) < controllerThreshold)
          targetTvel = 0;

        // Mapping the range of joy stick, avoid sunddenly jumping from 0 to controllerThreshold
        targetXvel = targetXvel != 0 ? (targetXvel > 0 ? map(targetXvel, controllerThreshold, 99, 0, 99) : map(targetXvel, -99, controllerThreshold, -99, 0)) : 0;
        targetYvel = targetYvel != 0 ? (targetYvel > 0 ? map(targetYvel, controllerThreshold, 99, 0, 99) : map(targetYvel, -99, controllerThreshold, -99, 0)) : 0;
        targetTvel = targetTvel != 0 ? (targetTvel > 0 ? map(targetTvel, controllerThreshold, 99, 0, 99) : map(targetTvel, -99, controllerThreshold, -99, 0)) : 0;

        //        float velDivider = (controllerIO[R1] == 0) ? 2.4  : .8; // R1 pressed: Slow mode/ Fast mode

        float velDivider = 1;
        //  accelerate x6
        if (controllerIO[R1] == 1) {
          velDivider = .6;
          //  decelerate /3
        } else if (controllerIO[R2] == 1) {
          velDivider = 4.4;
        } else {
          //  normal speed
          velDivider = 1;
        }

        targetXvel *= 20 / velDivider;
        targetYvel *= 20 / velDivider;
        targetTvel *= 20 / velDivider;


        for (int i = numLP - 1; i > 0; i--)
        {
          inputXvel[i] = inputXvel[i - 1];
          inputYvel[i] = inputYvel[i - 1];
          inputTvel[i] = inputTvel[i - 1];
        }
        inputXvel[0] = targetXvel;
        inputYvel[0] = targetYvel;
        inputTvel[0] = targetTvel;
        targetXvel = targetYvel = targetTvel = 0;

        for (int i = 0; i < numLP; i++)
        {
          targetXvel += inputXvel[i];
          targetYvel += inputYvel[i];
          targetTvel += inputTvel[i];
        }
        targetXvel /= numLP;
        targetYvel /= numLP;
        targetTvel /= numLP;

        //        if (targetTvel != 0) {
        //          setCurrentGyroPos();
        //        }
        //        else targetTvel = getChassisTvelGyro(5, 50);

        goTargetChassisVel(targetXvel, targetYvel, targetTvel);

        // End of velocity control
        /////////////////////////////
      }
      else  if (brakemode == false) // Auto mode
      {
        if (abs(controllerADC[ADC3]) > controllerReturnThreshold || abs(controllerADC[ADC4]) > controllerReturnThreshold || abs(controllerADC[ADC1]) > controllerReturnThreshold)
        {
          manualMode = true;
          autoPathStart = false;
          resetChassisPos();
          targetPosition = 0;
        }
        if (PATH_STATE == RESET)
        {
          resetChassisPos();
          setCurrentGyroPos();
          resetPathTargetCal();
          if (currentPosition == targetPosition)
          {
            PATH_STATE = STOP;
          }
          else if (targetPosition > getNumPt() || targetPosition < 1)
          {
            //End state
            PATH_STATE = STOP;
            pointState = 0;
            currentPosition = sequencPoint[pointState];
            targetPosition = 0;
          }
          else
          {
            PATH_STATE = RUNNING;
            autoPathStart = true;
            //Next state
            runningEdge = point2edge(currentPosition, targetPosition);
            currentPosition = targetPosition;
          }
          autoPathStartTime = millis();
        }

        else if (PATH_STATE == RUNNING)
        {
          runningTime = millis() - autoPathStartTime;
          if (runningTime > (getEdgeFinishTime(runningEdge) + getEdgeToleranceTime(runningEdge)))
          {
            resetChassisPos();
            PATH_STATE = STAY;
            stayStartTime = millis();
            autoPathStart = false;
          }
          else
          {
            //Mode 1, most ideal path, recommend in slow motion
            //          getPathTargetPos(runningEdge, runningTime, &targetXpos, &targetYpos, &targetTpos);
            //          goTargetChassisPos(targetXpos, targetYpos, targetTpos);

            //Mode 2, require gyro, global position sense, recommend in both accurate and speed required motion, need fine tune when arrive, PATH_STATE = STAY
            //          getPathTargetGlobalPos(runningEdge, runningTime, &targetXpos, &targetYpos, &targetTpos);
            //          goTargetChassisPos(targetXpos, targetYpos, targetTpos);

            //Mode 3, require gyro, global velocity sense, recommend in speed path without accurate destination
            getPathTargetVel(runningEdge, runningTime, &targetXvel, &targetYvel, &targetTvel);
            goTargetGlobalChassisVel(targetXvel, targetYvel, targetTvel);
          }
        }

        else if (PATH_STATE == STOP)
        {
          goTargetChassisVel(0, 0, 0);
        }

        else if (PATH_STATE == STAY)
        {
          goTargetChassisVel(0, 0, getChassisTvelGyro(0, 100));
          if (millis() - stayStartTime > 1000)
            PATH_STATE = STOP;
        }
      } else;
    }

    else if (!safety)
    {
      goTargetChassisVel(0, 0, 0);
    }
  } // End of control loop
}
