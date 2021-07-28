const int enPin = 18;
const int junctionPin = 19;
int linePos = 255;

void initLineSensor()
{
  Serial2.begin(115200);
  pinMode(enPin, OUTPUT);
  pinMode(junctionPin, INPUT);
  // Enable UART output of the line sensor
  enableLineSensor(false);
}

void getLineSensor()
{
  if (Serial2.available() > 0)
  {
    linePos = Serial2.read();
  }
}

int getLinePos()
{
  return linePos;
}

void resetLineSensor()
{
  linePos = 255;
}

void enableLineSensor(bool enable)
{
  if (enable)
  {
    digitalWrite(enPin, LOW);
  }
  else
  {
    digitalWrite(enPin, HIGH);
    resetLineSensor();
  }
}

bool junctionDetected()
{
  return digitalRead(junctionPin);
}
