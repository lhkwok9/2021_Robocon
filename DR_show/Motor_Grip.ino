const int motorGripMapping = 1;
const int initCountThreshold = 100;
const int numRecord = 10;
const int upperlimit = 21000;
long encoderValue[numRecord] = {0};
long previousEncoderCount = 0;
long currentEncoderCount = 0;
int motorGripState = 0;

unsigned long startStayTime = 0;
int previousEncoderValueStay = 0;
int encoderValueStay = 0;   // 0 or 1

void initMotorGrip()
{
  stopMotor(motorGripMapping);
  motorGripState = 0;
}

void rotate(long value)
{
  value = constrain(value, 0, upperlimit);
  goTargetMotorPos(motorGripMapping, value);
}

long getActualMotorGrip()
{
  return getMotorPos(motorGripMapping);
}

void stopMotorGrip()
{
  stopMotor(motorGripMapping);
}

void brakeMotorGrip()
{
  brakeMotor(motorGripMapping);
}

void resetMotorGrip()
{
  resetMotor(motorGripMapping);
}

void debugMotorGrip()
{
  Serial.print("Motor grip count: ");
  Serial.print(getMotorPos(motorGripMapping));
  Serial.println();
  Serial.print("Motor grip state: ");
  Serial.print(getMotorPos(motorGripState));
  Serial.println();
}

void getMotorGripState() 
{
  if (getActualMotorGrip() > upperlimit/2)
  {
    motorGripState = 1;
  }
  else
  {
    motorGripState = 0;
  }
}

int getMotorGripPos() // 0 = init sides, 1 = opposite sides
{
  return motorGripState;
}

void resetMotorGripStayTime()
{
  startStayTime = millis();
}

int isMotorGripStayFor(unsigned long period)
{
  encoderValueStay = isMotorGripStay();

  //Check not 0
  if (previousEncoderValueStay == encoderValueStay && encoderValueStay == 0)
  {
    startStayTime = millis();
    return 0;
  }

  //Start Count
  if (previousEncoderValueStay != encoderValueStay && encoderValueStay == 1)
  {
    startStayTime = millis();
  }

  //To the time limit
  if (encoderValueStay == 1 && millis() - startStayTime >= period)
  {
    startStayTime = millis();
    return 1;
  }
  previousEncoderValueStay = encoderValueStay;
  return 0;
}

int isMotorGripStay()
{
  previousEncoderCount = getMotorPos(motorGripMapping);
  for (int i=numRecord-1; i>0; i--)
  {
      encoderValue[i] = encoderValue[i-1];
  }
  encoderValue[0] = getMotorPos(motorGripMapping);
  currentEncoderCount = 0;
  for (int i=0; i<numRecord; i++)
  {
      currentEncoderCount += encoderValue[i];
  }
  currentEncoderCount /= numRecord;
  if (previousEncoderCount == currentEncoderCount)
  {
    encoderValueStay = 1;
  }
  else
  {
    encoderValueStay = 0;
  }
  return encoderValueStay;
}
