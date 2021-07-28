//#include <TFT.h>  // Arduino LCD library
//#include <SPI.h>
//
//const int RS = 22;
//const int RST = 23;
//const int CS = 24;
//char TFToutput[20];
//int displayCounter = 0;
//
//TFT TFTscreen = TFT(CS, RS, RST);
//
//void initTFT()
//{
//  memset(TFToutput, 0, sizeof(TFToutput));
//  TFTscreen.begin();
//  TFTscreen.background(0, 0, 0);
//  TFTscreen.setTextSize(2);
//  TFTscreen.stroke(255, 255, 255);
//  TFTscreen.text("     CUHK", 0, 0);
//  TFTscreen.text(" Robocon 2019", 0, 20);
//  TFTscreen.text("Mode: Manual", 0, 40);
//  TFTscreen.text("Field: Blue", 0, 60);
//  TFTscreen.text("Speed: Stable",0,80);
//  TFTscreen.text("Path: 0", 0, 100);
//
//}
//
//void resetTFT()
//{
//   memset(TFToutput, 0, sizeof(TFToutput));
//   TFTscreen.background(0, 0, 0);
//   TFTscreen.setTextSize(2);
//   TFTscreen.stroke(255, 255, 255);
//   TFTscreen.text("     CUHK", 0, 0);
//   TFTscreen.text(" Robocon 2019", 0, 20);
//   TFTscreen.text("Mode: Manual", 0, 40);
//   TFTscreen.text("Field: Blue", 0, 60);
//   TFTscreen.text("Speed: Stable",0,80);
//   TFTscreen.text("Path: 0", 0, 100);
//}
//
//void initTFTViewMode()
//{
//    memset(TFToutput, 0, sizeof(TFToutput));
//    TFTscreen.background(0, 0, 0);
//    TFTscreen.setTextSize(2);
//    TFTscreen.stroke(255, 255, 255);
//    TFTscreen.text("     CUHK", 0, 0);
//    TFTscreen.text(" Robocon 2019", 0, 20);
//    TFTscreen.text("X: 0", 0, 40);
//    TFTscreen.text("Y: 0", 0, 60);
//    TFTscreen.text("T: 0", 0, 80);
//}
//
//void updateTFTVal()
//{
//  updateTFTXVal();
//  updateTFTYVal();
//  updateTFTTVal();
//}
//
//void updateTFTXVal()
//{
//  TFTscreen.stroke(0, 0, 0);
//  memset(TFToutput, 0, sizeof(TFToutput));
//  sprintf(TFToutput, "X: %c%c%c%c", char(218), char(218), char(218), char(218));
//  TFTscreen.text(TFToutput, 0, 40);
//  memset(TFToutput, 0, sizeof(TFToutput));
//  sprintf(TFToutput, "X: %d", displayX);
//  TFTscreen.stroke(255, 255, 255);
//  TFTscreen.text(TFToutput, 0, 40);
//}
//
//void updateTFTYVal()
//{
//   TFTscreen.stroke(0, 0, 0);
//  memset(TFToutput, 0, sizeof(TFToutput));
//  sprintf(TFToutput, "Y: %c%c%c%c", char(218), char(218), char(218), char(218));
//  TFTscreen.text(TFToutput, 0, 60);
//  memset(TFToutput, 0, sizeof(TFToutput));
//  sprintf(TFToutput, "Y: %d", displayY);
//  TFTscreen.stroke(255, 255, 255);
//  TFTscreen.text(TFToutput, 0, 60);
//}
//
//void updateTFTTVal()
//{
//   TFTscreen.stroke(0, 0, 0);
//  memset(TFToutput, 0, sizeof(TFToutput));
//  sprintf(TFToutput, "T: %c%c%c%c", char(218), char(218), char(218), char(218));
//  TFTscreen.text(TFToutput, 0, 80);
//  memset(TFToutput, 0, sizeof(TFToutput));
//  sprintf(TFToutput, "T: %d", displayT);
//  TFTscreen.stroke(255, 255, 255);
//  TFTscreen.text(TFToutput, 0, 80);
//}
//
//
//void updateTFTMode()
//{
//  if (manualMode)
//  {    
//    TFTscreen.stroke(0, 0, 0);
//    memset(TFToutput, 0, sizeof(TFToutput));
//    sprintf(TFToutput, "Mode: %c%c%c%c", char(218), char(218), char(218), char(218));
//    TFTscreen.text(TFToutput, 0, 40);
//    TFTscreen.stroke(255, 255, 255);
//    TFTscreen.text("Mode: Manual", 0, 40);
//  }
//  else
//  {
//    TFTscreen.stroke(0, 0, 0);
//    memset(TFToutput, 0, sizeof(TFToutput));
//    sprintf(TFToutput, "Mode: %c%c%c%c%c%c", char(218), char(218), char(218), char(218), char(218), char(218));
//    TFTscreen.text(TFToutput, 0, 40);
//    TFTscreen.stroke(255, 255, 255);
//    TFTscreen.text("Mode: Auto", 0, 40);
//  }
//}
//
//void updateTFTField()
//{
//  if (redField)
//  {
//    TFTscreen.stroke(0, 0, 0);
//    memset(TFToutput, 0, sizeof(TFToutput));
//    sprintf(TFToutput, "Field: %c%c%c%c", char(218), char(218), char(218), char(218));
//    TFTscreen.text(TFToutput, 0, 60);
//    TFTscreen.stroke(255, 255, 255);
//    TFTscreen.text("Field: Red", 0, 60);
//  }
//  else
//  {
//    TFTscreen.stroke(0, 0, 0);
//    memset(TFToutput, 0, sizeof(TFToutput));
//    sprintf(TFToutput, "Field: %c%c%c", char(218), char(218), char(218));
//    TFTscreen.text(TFToutput, 0, 60);
//    TFTscreen.stroke(255, 255, 255);
//    TFTscreen.text("Field: Blue", 0, 60);
//  }
//}
//
//void updateTFTSpeed()
//{
//  if (stableMode)
//  {
//    TFTscreen.stroke(0, 0, 0);
//    memset(TFToutput, 0, sizeof(TFToutput));
//    sprintf(TFToutput, "Speed: %c%c%c%c", char(218), char(218), char(218), char(218));
//    TFTscreen.text(TFToutput, 0, 80);
//    TFTscreen.stroke(255, 255, 255);
//    TFTscreen.text("Speed: Stable", 0, 80);
//  }
//  else
//  {
//     TFTscreen.stroke(0, 0, 0);
//    memset(TFToutput, 0, sizeof(TFToutput));
//    sprintf(TFToutput, "Speed: %c%c%c%c%c%c", char(218), char(218), char(218), char(218), char(218), char(218));
//    TFTscreen.text(TFToutput, 0, 80);
//    TFTscreen.stroke(255, 255, 255);
//    TFTscreen.text("Speed: Fast", 0, 80);
//  }
//}
//
//void updateTFTpathNum()
//{  
//  TFTscreen.stroke(0, 0, 0);
//  memset(TFToutput, 0, sizeof(TFToutput));
//  sprintf(TFToutput, "Path: %c%c", char(218), char(218));
//  TFTscreen.text(TFToutput, 0, 100);
//  memset(TFToutput, 0, sizeof(TFToutput));
//  sprintf(TFToutput, "Path: %d", pathCounter);
//  TFTscreen.stroke(255, 255, 255);
//  TFTscreen.text(TFToutput, 0, 100);
//}
