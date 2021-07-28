const int IOport[] = {A4, A2, A1, A5, A3, A0};
const int ADCport[] = {A4, A2, A1, A5, A3, A0};
const int numIO = 6;
const int numADC = 6;
const int ledPin = 13;
int ledPeriod = 0;
int ledCounter = 0;
bool ledFlag = true;

void initIOADC()
{
  for (int i=0; i<numADC; i++)
  {
    pinMode(ADCport[i], INPUT);
  }
//  for (int i=0; i<numIO; i++)
//  {
//    pinMode(IOport[i], INPUT);
//    digitalWrite(IOport[i], HIGH);
//  }
  pinMode(ledPin, OUTPUT);
  changeLedPeriod(300);
}

boolean getIO(int IOnum)
{
  if (IOnum >= 1 && IOnum <= numIO)
    return digitalRead(IOport[IOnum - 1]);
  else
    return false;
}

void setIO(int IOnum, int IOstatus)
{
  if (IOnum >= 1 && IOnum <= numIO)
    digitalWrite(IOport[IOnum - 1], IOstatus);
}

void blinkLED()
{
  ledCounter++;
  if (ledCounter == ledPeriod)
  {
    ledCounter = 0;
    ledFlag = !ledFlag;
  }
  if (ledFlag)
    digitalWrite(ledPin, HIGH);
  else
    digitalWrite(ledPin, LOW);
}

void changeLedPeriod(int period)
{
  ledPeriod = period / loopPeriod / 2;
  ledCounter = 0;
}

int getADC(int ADCnum)
{
  if (ADCnum >=1 && ADCnum <= numADC)
    return analogRead(ADCport[ADCnum-1]);
  else
    return 0;
}
