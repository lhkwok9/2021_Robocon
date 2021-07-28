const int LSReadingCount = 200; //200 should be enough for now
const int laserNum = 1;

int laserRef[laserNum][4] = {0};
int laserMoveAverage[laserNum][LSReadingCount + 2] = {0};
int laserMoveCount[laserNum] = {0};

void initLaser(int ADCnum, int Lread, int Hread, int Lrange, int Hrange){
  laserRef[ADCnum-1][0] = Lread; //lower range reading
  laserRef[ADCnum-1][1] = Hread; //higher range reading
  laserRef[ADCnum-1][2] = Lrange; //lower range dist
  laserRef[ADCnum-1][3] = Hrange; //higher range dist
}

int tuningLaser(int ADCnum) {
  analogReadResolution(12); // default 10-bit, use according to the board resolution: Zero, Due, MKR family and Nano 33 (BLE and IoT): can be up to 12-bit, Portenta H7 has a 16 bit ADC
  laserMoveAverage[ADCnum - 1][LSReadingCount] -= laserMoveAverage[ADCnum - 1][laserMoveCount[ADCnum - 1]];
  laserMoveAverage[ADCnum - 1][laserMoveCount[ADCnum-1]] = getADC(ADCnum);
  laserMoveAverage[ADCnum - 1][LSReadingCount] += laserMoveAverage[ADCnum - 1][laserMoveCount[ADCnum - 1]];
  laserMoveAverage[ADCnum - 1][LSReadingCount + 1] = laserMoveAverage[ADCnum - 1][LSReadingCount] / LSReadingCount;
  laserMoveCount[ADCnum - 1] ++;
  if (laserMoveCount[ADCnum - 1] > (LSReadingCount - 1))
    laserMoveCount[ADCnum - 1] = 0;
  if(laserMoveAverage[ADCnum - 1][LSReadingCount + 1] < 0||laserMoveAverage[ADCnum - 1][LSReadingCount + 1] > 4096) // depends on the resolution
  return -1;
  else return laserMoveAverage[ADCnum - 1][LSReadingCount + 1];  //for tuning
}

int getLaserSensorDistance(int ADCnum) {
  analogReadResolution(12); // default 10-bit, use according to the board resolution: Zero, Due, MKR family and Nano 33 (BLE and IoT): can be up to 12-bit, Portenta H7 has a 16 bit ADC
  laserMoveAverage[ADCnum - 1][LSReadingCount] -= laserMoveAverage[ADCnum - 1][laserMoveCount[ADCnum - 1]];
  laserMoveAverage[ADCnum - 1][laserMoveCount[ADCnum-1]] = getADC(ADCnum);
  laserMoveAverage[ADCnum - 1][LSReadingCount] += laserMoveAverage[ADCnum - 1][laserMoveCount[ADCnum - 1]];
  laserMoveAverage[ADCnum - 1][LSReadingCount + 1] = laserMoveAverage[ADCnum - 1][LSReadingCount] / LSReadingCount;
  laserMoveCount[ADCnum - 1] ++;
  if (laserMoveCount[ADCnum - 1] > (LSReadingCount - 1))
    laserMoveCount[ADCnum - 1] = 0;
  if(laserMoveAverage[ADCnum - 1][LSReadingCount + 1] < 0||laserMoveAverage[ADCnum - 1][LSReadingCount + 1] > 4096) // depends on the resolution
  return -1;
  else return map(laserMoveAverage[ADCnum - 1][LSReadingCount + 1], laserRef[ADCnum-1][0],laserRef[ADCnum-1][1],laserRef[ADCnum-1][2],laserRef[ADCnum-1][3]);
}

/*
//the following would be done when i am really bored and have nothing to do
bool getLaserSensorNear(int ADCnum){
   
}

bool getLaserSensorFar{

  return
}

bool getLaserSensorBetween{
  
}
*/
