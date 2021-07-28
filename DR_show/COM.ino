int comOPcounter = 0;
char tmp = '\0';
int sign = 1;
byte buttonValue[2];
//const bool pi_ps4 = false;
bool conSafety = true;
int startConnectCount = 0;
int seq[4] = {1, -1, -1, 1};


void initCom()
{
  Serial.begin(115200);   // Bluetooth
  Serial3.begin(115200);  // Controller
#if PS4Controller == RASPI
  SerialUSB.begin(115200);
#endif
}

void getComData()
{
  if (Serial.available() >= 5)
  {
  }
}

void getController()
{
#ifdef PS4Controller
#if PS4Controller==RASPI
  //  Serial.println("in raspi");
  if (SerialUSB.available() >= 7)
  {
    if (SerialUSB.read() == '#')
    {
      conSafety = true;
      buttonValue[0] = SerialUSB.read();
      //      Serial.print("ButtonValue0: "); Serial.println(buttonValue[0]);
      for (int i = 0; i < 8; i++)
      {
        controllerIO[i] = bitRead(buttonValue[0], i);
      }
      buttonValue[1] = SerialUSB.read();
      //      Serial.print("ButtonValue1: "); Serial.println(buttonValue[1]);
      for (int i = 0; i < 8; i++)
      {
        controllerIO[i + 8] = bitRead(buttonValue[1], i);
      }
      for (int i = 0; i < 4; i++)
      {
        byte temp = SerialUSB.read();
        if ( i == 1 || i == 2 || i == 3) // UP DOWN are inverted
          controllerADC[i] = -constrain(map(temp, 0, 255, -99, 99), -99, 99);
        else
          controllerADC[i] = constrain(map(temp, 0, 255, -99, 99), -99, 99);
      }
    }
    else if (SerialUSB.read() == '@')
    {
      // Safety mode
      // The controller is disconnected
      Serial.println("In Safety Mode");
      conSafety = false;
      startConnectCount = 0;
      for (int j = 0; j < 4; j++) controllerADC[j] = 0;
      for (int i = 0; i < 6; i++) Serial3.read();
    }
    SerialUSB.flush();
  }

#elif PS4Controller==ESP32
  //  Serial.println("in esp32");
  if (Serial3.available() >= 7)
  {
    if (Serial3.read() == '#')
    {
      conSafety = true;
      if (startConnectCount <= 1500)
      {
        startConnectCount += 1;
        for (int i = 0; i < 6; i++) Serial3.read();
      }
      else
      {
        buttonValue[0] = Serial3.read();
        //      Serial.print("ButtonValue0: "); Serial.println(buttonValue[0]);
        for (int i = 0; i < 8; i++)
        {
          controllerIO[i] = bitRead(buttonValue[0], i);
        }
        buttonValue[1] = Serial3.read();
        //      Serial.print("ButtonValue1: "); Serial.println(buttonValue[1]);
        for (int i = 0; i < 8; i++)
        {
          controllerIO[i + 8] = bitRead(buttonValue[1], i);
        }
        for (int i = 0; i < 4; i++)
        {
          byte temp = Serial3.read();
          if (i == 2)
            controllerADC[i] = -constrain(map(temp, 0, 255, -99, 99), -99, 99);
          else
            controllerADC[i] = constrain(map(temp, 0, 255, -99, 99), -99, 99);
        }
      }
    }
    else if (Serial3.read() == '@')
    {
      // Safety mode
      // The controller is disconnected
      Serial.println("In Safety Mode");
      conSafety = false;
      startConnectCount = 0;
      for (int j = 0; j < 4; j++) controllerADC[j] = 0;
      for (int i = 0; i < 6; i++) Serial3.read();
    }
    //    Serial3.flush();
  }
#endif
#endif
#ifndef PS4Controller
  //    Serial.println("in vex");
  if (Serial3.available() >= 26)
  {
    if (Serial3.read() == '#')
    {
      for (int i = 0; i < 12; i++)
      {
        controllerIO[i] = int(Serial3.read() - 48);
      }
      for (int i = 3; i >= 0; i--)
      {
        controllerADC[i] = 0;
        tmp = Serial3.read();
        if (tmp == '-')
          sign = -1;
        else
          sign = 1;
        controllerADC[i] += int(Serial3.read() - 48) * 10;
        controllerADC[i] += int(Serial3.read() - 48);
        controllerADC[i] *= seq[i] * sign;
      }
      tmp = Serial3.read();
    }
  }
#endif
}

void getToggle()
{
  // Filter Toggle
  for (int i = 0; i < controllerIONum; i++)
  {
    if (controllerIO[i] != lastControllerIO[i] && controllerIO[i] == 1 && millis() - buttonPressTime[i] > 400)  //250
    {
      buttonPressTime[i] = millis();
      controllerIOToggle[i] = 1;
    }
    else controllerIOToggle[i] = 0;
    lastControllerIO[i] = controllerIO[i];
    //            Serial.print(controllerIOToggle[i]);Serial.print(" ");
  }
}

bool getSafety()
{
  return conSafety;
}

// Print debug message to PC com
void printDebug()
{
  comOPcounter++;
  if (comOPcounter % 10 == 0)
  {
    comOPcounter = 0;

    ///////////////
    // Controller
    ///////////////
        Serial.print(controllerADC[0]);
        Serial.print(" ");
        Serial.print(controllerADC[1]);
        Serial.print(" ");
        Serial.print(controllerADC[2]);
        Serial.print(" ");
        Serial.print(controllerADC[3]);
        Serial.print(" ");
        Serial.println();

//          Serial.print(String(0) + ": ");
//          Serial.print(controllerIO[0]);
//          Serial.print(" ");
//          Serial.print(String(11) + ": ");
//          Serial.print(controllerIO[11]);
//          Serial.print(" ");
//          Serial.println();

//        for (int i = 0; i < controllerIONum; i++) {
//          Serial.print(String(i) + ": ");
//          Serial.print(controllerIO[i]);
//          Serial.print(" ");
//    
//        }
//        Serial.println();
          Serial.print(targetXvel);
          Serial.print(" ");
          Serial.print(targetYvel);
          Serial.print(" ");
          Serial.print(targetTvel);
          Serial.print(" ");
          Serial.println();

    ///////////////
    // Velocity
    ///////////////
    //      Serial.print(getMotorVel(1));
    //      Serial.print(" ");
    //      Serial.print(getMotorVel(2));
    //      Serial.print(" ");
    //      Serial.print(getMotorVel(3));
    //      Serial.print(" ");
    //      Serial.print(getMotorVel(4));
    //      Serial.print(" ");

    //      calMotorVel();
          Serial.print(getMotorVel(1));
          Serial.print(" ");
          Serial.print(getMotorVel(2));
          Serial.print(" ");
          Serial.print(getMotorVel(3));
          Serial.print(" ");
          Serial.print(getMotorVel(5));
          Serial.print(" ");
          Serial.println();


    ///////////////
    // Position
    ///////////////
          Serial.print(getMotorPos(1));
          Serial.print(" ");
          Serial.print(getMotorPos(2));
          Serial.print(" ");
          Serial.print(getMotorPos(3));
          Serial.print(" ");
          Serial.print(getMotorPos(4));
          Serial.print(" ");
          Serial.print(getMotorPos(5));
          Serial.print(" ");
          Serial.print(getMotorPos(6));
          Serial.println();

    ///////////////
    // PID term
    ///////////////
    //    Serial.print(getMotorVelPTerm(2));
    //    Serial.print(" ");
    //    Serial.print(getMotorVelITerm(2));
    //    Serial.print(" ");
    //    Serial.print(getMotorVelDTerm(3));
    //    Serial.print(" ");

    ///////////////
    // Line position
    ///////////////
    //      Serial.print(getLinePos());


    ///////////////
    //      Serial.println();
  }
}
