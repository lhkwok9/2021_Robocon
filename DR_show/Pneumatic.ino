const int pneuPort[] = {53, A11, A10, A9, A8, A7, A6};
const int numPneu = 7;

void initPneumatic()
{
  for (int i=0; i<numPneu; i++)
  {
    pinMode(pneuPort[i], OUTPUT);
    digitalWrite(pneuPort[i], LOW);
  }
}

void setPneu(int pneuNum, int pneuStatus)
{
  if (pneuNum >= 1 && pneuNum <= numPneu)
    digitalWrite(pneuPort[pneuNum-1], pneuStatus);
}

int getPneu(int pneuNum)
{
  if (pneuNum >= 1 && pneuNum <= numPneu)
    return digitalRead(pneuPort[pneuNum-1]);
}

int getPneuNum()
{
  return numPneu;
}


void resetPneu()
{
  for (int i=0; i<numPneu; i++)
  {
    digitalWrite(pneuPort[i], LOW);
  }
}
