//Point defination
#define X 0
#define Y 1
#define T 2

//Point 2 point defination
#define INIT_X 0
#define INIT_Y 1
#define END_X  2
#define END_Y  3
#define TOR    4

//Control pts defination
#define X1 0
#define Y1 1
#define X2 2
#define Y2 3
#define SampleTime 4

//Vel profile defination
#define D 0
#define Phi 1
#define T1 2
#define T2 3
#define T3 4
#define A 5
#define B 6
#define C 7
#define VelMode 8

const int numPt = 7;
const int numEdge = numPt * (numPt - 1);

const float defaultSampleTime = 1000;
const float defaultTor = 500;

typedef float point_t[3];

point_t *point = NULL;
int field = 0;  //Red field = 1, Blue field = -1, unset = 0

float control_pts[numEdge][5]; //    {X1(mm),       Y1(mm),      X2(mm), Y2(mm), sampleTime(ms)}
float pt2pt[numEdge][5];       //    {init_X(mm),       init_Y(mm),      end_X(mm), end_Y(mm), Tor(ms)}
float maxV[numEdge];
float acc[numEdge];
float deacc[numEdge];
float accTime[numEdge];
float deaccTime[numEdge];
float minPathTime[numEdge];
float minPathDist[numEdge];
float velProfile[numEdge][9]; // [path][{D(mm), Phi(rad), T1(ms), T2(ms), T3(ms), A(mm), B(mm), C(mm)}, mode]

const int correctionTableSize = defaultSampleTime/loopPeriod;
float correctionTable[numEdge][correctionTableSize];

point_t point_red[numPt] =
{
 //    {X(mm),       Y(mm),      T(rad)}
  /*1*/{ 11000,         0,          0},     //Start zone
  /*2*/{ 0,             0,          0},           //Receive zone
  //{4026, -4778, 0  },
  /*3*/{1806, -6192, 0 },
  /*4*/{3986, -6492, 0 },  //x-500
  /*5*/{5992, -6492, 0 },
  /*6*/{9055, -7692, 0},
  /*7*/{10728, -7692, 0},


};

point_t point_blue[numPt] =
{
 //    {X(mm),       Y(mm),      T(rad)}
  /*1*/{ -10000,         0,          0},     //Start zone
  /*2*/{ 0,             0,          0},           //Receive zone
  //{4026, -4778, 0  },
  /*3*/{-2246, -6492, 0 },
  /*4*/{-4066, -6492, 0 },
  /*5*/{-5952, -6492, 0 },
  /*6*/{-8025, -6092, 0},
  /*7*/{-10038, -6092, 0},

};


//////////////////////Local Function/////////////////////
double find2ptDistance(int x1, int y1, int x2, int y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
/////////////////////////////////////////////////////////
//////////////////////GET FUNCTION///////////////////////
int getNumPt()
{
  return numPt;
}

int getNumEdge()
{
  return numEdge;
}


int getEdgeFinishTime(int edge)
{
  if (edge >= 0 && edge < numEdge)
  {
    return velProfile[edge][T3];
  }
  else
    return 0;
}

int getEdgeToleranceTime(int edge)
{
  if (edge >= 0 && edge < numEdge)
  {
    return (int)pt2pt[edge][TOR];
  }
  else
    return 0;
}
/////////////////////////////////////////////////////////
/***************************Point2Edge, Edge2Point function******************/
//Mark edge for an index
int point2edge(int startPoint, int endPoint)
{
  if ((startPoint >= 1 && startPoint <= numPt) && (endPoint >= 1 && endPoint <= numPt))
  {
    int index = 0;
    for (int i = 0; i < numPt; i++)
    {
      for (int j = 0; j < numPt; j++)
      {
        if (i == j) continue;
        if ((startPoint - 1 == i) && (endPoint - 1 == j))
        {
          return index;
        }
        index++;
      }
    }
  }
  return -1;
}

int edge2point(int index, int point_type)  //point_type = 1 (start point), point_type = 2 (end point)
{
  if ((index >= 0 && index <= numEdge) && (point_type == 1 || point_type == 2))
  {
    for (int i = 0; i < numPt; i++)
    {
      for (int j = 0; j < numPt; j++)
      {
        if (i == j) continue;
        if (index == 0)
        {
          if (point_type == 1) {
            return i + 1;
          }
          else if (point_type == 2) {
            return j + 1;
          }
        }
        index --;
      }
    }
  }
  return -1;
}
/****************************************************************************/
/***********************Correction table*************************************/
void initCorrectionTable(int edge)
{
  double distance = 0;
  int init_x; int init_y; int end_x; int end_y;
  int x1; int y1; int x2; int y2;

  init_x = pt2pt[edge][INIT_X];
  init_y = pt2pt[edge][INIT_Y];
  end_x = pt2pt[edge][END_X];
  end_y = pt2pt[edge][END_Y];

  x1 = control_pts[edge][X1];
  y1 = control_pts[edge][Y1];
  x2 = control_pts[edge][X2];
  y2 = control_pts[edge][Y2];

  int x; int y; int previous_x; int previous_y;
  x = init_x;
  y = init_y;
  previous_x = x;
  previous_y = y;

  float u = 0;

  for (int i = 0;i<correctionTableSize;i++)
  {
    u = (float)i/(correctionTableSize-1);

    x = init_x * (1 - u) * (1 - u) * (1 - u) + 3 * x1 * u * (1 - u) * (1 - u) + 3 * x2 * u * u * (1 - u) + end_x * u * u * u;
    y = init_y * (1 - u) * (1 - u) * (1 - u) + 3 * y1 * u * (1 - u) * (1 - u) + 3 * y2 * u * u * (1 - u) + end_y * u * u * u;
    
    distance += find2ptDistance(x, y, previous_x, previous_y);

    correctionTable[edge][i] = distance;
    
    previous_x = x;
    previous_y = y;
  }

  for (int i = 0;i<correctionTableSize;i++)
  {
    correctionTable[edge][i]/=distance;
  }
}

float correction(int edge, float u)
{
  int index = 0;

  for (int i = correctionTableSize-1 ;i>=0;i--)
  {
    if(correctionTable[edge][i] <= u)
    {
      index = i;
      break;
    }
  }

  if (index == correctionTableSize-1)
  {
    return 1;
  }
  else
  {
    return ((u - correctionTable[edge][index]) / (correctionTable[edge][index+1] - correctionTable[edge][index]) + index) / (correctionTableSize - 1); 
  }
}

/****************************************************************************/
double find_edge_distance(int edge)
{
  double distance = 0;
  int init_x; int init_y; int end_x; int end_y;
  int x1; int y1; int x2; int y2;

  init_x = pt2pt[edge][INIT_X];
  init_y = pt2pt[edge][INIT_Y];
  end_x = pt2pt[edge][END_X];
  end_y = pt2pt[edge][END_Y];

  x1 = control_pts[edge][X1];
  y1 = control_pts[edge][Y1];
  x2 = control_pts[edge][X2];
  y2 = control_pts[edge][Y2];

  float u = 0;
  float sample_time = control_pts[edge][SampleTime];

  int x; int y; int previous_x; int previous_y;
  x = init_x;
  y = init_y;
  previous_x = x;
  previous_y = y;

  while (x != end_x || y != end_y)
  {
    if (u >= 1) u = 1;
    x = init_x * (1 - u) * (1 - u) * (1 - u) + 3 * x1 * u * (1 - u) * (1 - u) + 3 * x2 * u * u * (1 - u) + end_x * u * u * u;
    y = init_y * (1 - u) * (1 - u) * (1 - u) + 3 * y1 * u * (1 - u) * (1 - u) + 3 * y2 * u * u * (1 - u) + end_y * u * u * u;
    u += (float)1 / (sample_time / loopPeriod); //5 for the case

    distance += find2ptDistance(x, y, previous_x, previous_y);

    previous_x = x;
    previous_y = y;
  }

  return distance;
}


void loadPoint(bool redField)
{
  if (redField)
  {
    point = point_red;
    field = 1;
  }
  else
  {
    point = point_blue;
    field = -1;
  }
}

void initPath()
{
  if (point == NULL)
  {
    Serial.println("Please run loadPoint() - point uninitialized");
    return;
  }
  if (field == 0)
  {
    Serial.println("Please run loadPoint() - field uninitialized");
    return;
  }
  for (int i = 0; i < numEdge; i++)
  {
    //Speed initialization
    maxV[i]        = 2800.0 / 1000.0;
    acc[i]         = 2800.0 / 1000000.0;             // mm/ms^2
    deacc[i]       = 2800.0 / 1000000.0;    //2200         // mm/ms^2

    //Control point initialization, default to be all to be straight line
    pt2pt[i][INIT_X] = point[edge2point(i, 1) - 1][X];
    pt2pt[i][INIT_Y] = point[edge2point(i, 1) - 1][Y];
    pt2pt[i][END_X]  = point[edge2point(i, 2) - 1][X];
    pt2pt[i][END_Y]  = point[edge2point(i, 2) - 1][Y];
    pt2pt[i][TOR] = defaultTor;

    control_pts[i][X1] = pt2pt[i][INIT_X];
    control_pts[i][Y1] = pt2pt[i][INIT_Y];
    control_pts[i][X2] = pt2pt[i][END_X];
    control_pts[i][Y2] = pt2pt[i][END_Y];
    control_pts[i][SampleTime] = defaultSampleTime; //ms

    //Set any special case here....
    //Red field
    if (field == 1)     //field red
    {
      maxV[point2edge(1,2)] = 2800.0/1000.0;
      acc[point2edge(1,2)] = 2800.0/1000000.0;
      deacc[point2edge(1,2)] = 2800.0 / 1000000.0;
  
     
    control_pts[point2edge(2, 3)][X1] = 3078;
    control_pts[point2edge(2, 3)][Y1] = -3212;
    control_pts[point2edge(2, 3)][X2] = 5518;
    control_pts[point2edge(2, 3)][Y2] = -5000;


  
    control_pts[point2edge(3, 2)][X2] = 3078;
    control_pts[point2edge(3, 2)][Y2] = -3212;
    control_pts[point2edge(3, 2)][X1] = 5018;
    control_pts[point2edge(3, 2)][Y1] = -3512;


   
    control_pts[point2edge(2, 4)][X1] = 3078;
    control_pts[point2edge(2, 4)][Y1] = -2912;
    control_pts[point2edge(2, 4)][X2] = 4078; //5518;
    control_pts[point2edge(2, 4)][Y2] = -3912; //-5000;


    control_pts[point2edge(4, 2)][X2] = 3078;
    control_pts[point2edge(4, 2)][Y2] = -2912;
    control_pts[point2edge(4, 2)][X1] = 4078; //5518;
    control_pts[point2edge(4, 2)][Y1] = -3912; //-5000;





    //    control_pts[point2edge(2, 5)][X1] = 3078;
    //    control_pts[point2edge(2, 5)][Y1] = -2912;
    //    control_pts[point2edge(2, 5)][X2] = 5252; //5518;
    //    control_pts[point2edge(2, 5)][Y2] = -5000; //-5000;




    control_pts[point2edge(2, 6)][X1] = 2778;
    control_pts[point2edge(2, 6)][Y1] = -3300;
    control_pts[point2edge(2, 6)][X2] = 2078;
    control_pts[point2edge(2, 6)][Y2] = -4212;


 
    control_pts[point2edge(6, 2)][X2] = 2778;
    control_pts[point2edge(6, 2)][Y2] = -3300;
    control_pts[point2edge(6, 2)][X1] = 2078;
    control_pts[point2edge(6, 2)][Y1] = -4212;

    
    control_pts[point2edge(2, 7)][X1] = 7500;
    control_pts[point2edge(2, 7)][Y1] = -1512;
    control_pts[point2edge(2, 7)][X2] = 5878;
    control_pts[point2edge(2, 7)][Y2] = -4212;

    
    control_pts[point2edge(7, 2)][X2] = 7500;
    control_pts[point2edge(7, 2)][Y2] = -1512;
    control_pts[point2edge(7, 2)][X1] = 5878;
    control_pts[point2edge(7, 2)][Y1] = -4212;
    }
    //Blue field
    else if (field == -1) //Blue field
    {

//      maxV[point2edge(1,2)] = 2800.0/1000.0;
//      acc[point2edge(1,2)] = 2800.0/1000000.0;
//      deacc[point2edge(1,2)] = 600.0 / 1000000.0;

      
        maxV[point2edge(2, 3)] = 800.0 / 1000.0;
    //acc[point2edge(1,2)] = 1200.0/1000000.0;
    control_pts[point2edge(2, 3)][X1] = -3078;
    control_pts[point2edge(2, 3)][Y1] = -2912;
    control_pts[point2edge(2, 3)][X2] = -5018; //5518;
    control_pts[point2edge(2, 3)][Y2] = -3512; //-5000;


    maxV[point2edge(3, 2)] = 800.0 / 1000.0;
    control_pts[point2edge(3, 2)][X2] = -3078;
    control_pts[point2edge(3, 2)][Y2] = -2912;
    control_pts[point2edge(3, 2)][X1] = -5018;
    control_pts[point2edge(3, 2)][Y1] = -3512;


    maxV[point2edge(2, 4)] = 800.0 / 1000.0;
    control_pts[point2edge(2, 4)][X1] = -3078;
    control_pts[point2edge(2, 4)][Y1] = -2912;
    control_pts[point2edge(2, 4)][X2] = -4078; //5518;
    control_pts[point2edge(2, 4)][Y2] = -3912; //-5000;


    maxV[point2edge(4, 2)] = 800.0 / 1000.0;
    control_pts[point2edge(4, 2)][X2] = -3078;
    control_pts[point2edge(4, 2)][Y2] = -2912;
    control_pts[point2edge(4, 2)][X1] = -4078; //5518;
    control_pts[point2edge(4, 2)][Y1] = -3912; //-5000;




    maxV[point2edge(2, 5)] = 800.0 / 1000.0;
    maxV[point2edge(5, 2)] = 800.0 / 1000.0;
    //    control_pts[point2edge(2, 5)][X1] = 3078;
    //    control_pts[point2edge(2, 5)][Y1] = -2912;
    //    control_pts[point2edge(2, 5)][X2] = 5252; //5518;
    //    control_pts[point2edge(2, 5)][Y2] = -5000; //-5000;



    maxV[point2edge(2, 6)] = 800.0 / 1000.0;
    control_pts[point2edge(2, 6)][X1] = -2778;
    control_pts[point2edge(2, 6)][Y1] = -3300;
    control_pts[point2edge(2, 6)][X2] = -2078;
    control_pts[point2edge(2, 6)][Y2] = -4212;


    maxV[point2edge(6, 2)] = 800.0 / 1000.0;
    control_pts[point2edge(6, 2)][X2] = -2778;
    control_pts[point2edge(6, 2)][Y2] = -3300;
    control_pts[point2edge(6, 2)][X1] = -2078;
    control_pts[point2edge(6, 2)][Y1] = -4212;

    maxV[point2edge(2, 7)] = 800.0 / 1000.0;
    control_pts[point2edge(2, 7)][X1] = -7500;
    control_pts[point2edge(2, 7)][Y1] = -1512;
    control_pts[point2edge(2, 7)][X2] = -5878;
    control_pts[point2edge(2, 7)][Y2] = -4212;

    maxV[point2edge(7, 2)] = 800.0 / 1000.0;
    control_pts[point2edge(7, 2)][X2] = -7500;
    control_pts[point2edge(7, 2)][Y2] = -1512;
    control_pts[point2edge(7, 2)][X1] = -5878;
    control_pts[point2edge(7, 2)][Y1] = -4212;

    
    
   
      
    }
    //Set any special case here.... 
    accTime[i]     = maxV[i] / acc[i];               // ms
    deaccTime[i]   = maxV[i] / deacc[i];             // ms
    minPathTime[i] = accTime[i] + deaccTime[i];      // ms
    minPathDist[i] = minPathTime[i] * maxV[i] / 2.0; // mm
  }

  //Relative shift value to 0
  for (int i = 0; i < numEdge; i++)
  {
    pt2pt[i][END_X] = pt2pt[i][END_X] - pt2pt[i][INIT_X];
    control_pts[i][X1] = control_pts[i][X1] - pt2pt[i][INIT_X];
    control_pts[i][X2] = control_pts[i][X2] - pt2pt[i][INIT_X];
    pt2pt[i][INIT_X] = pt2pt[i][INIT_X] - pt2pt[i][INIT_X];

    pt2pt[i][END_Y] = pt2pt[i][END_Y] - pt2pt[i][INIT_Y];
    control_pts[i][Y1] = control_pts[i][Y1] - pt2pt[i][INIT_Y];
    control_pts[i][Y2] = control_pts[i][Y2] - pt2pt[i][INIT_Y];
    pt2pt[i][INIT_Y] = pt2pt[i][INIT_Y] - pt2pt[i][INIT_Y];
  }

  for (int i = 0; i < numEdge; i++)
  {
    //Find total distance
    velProfile[i][D] = find_edge_distance(i);
    //Correction table
    initCorrectionTable(i);
    
    if (minPathDist[i] > velProfile[i][D]) {
      Serial.print("***********************************");
      Serial.print(edge2point(i, 1)); Serial.print("-" ); Serial.print(edge2point(i, 2)); 
      Serial.println(" Dangerous!*****************************************");
    }
    velProfile[i][Phi] = -1; //velProfile[i][Phi] = atan2(path[i][Y], path[i][X]);
    velProfile[i][T1] = accTime[i];
    velProfile[i][T2] = (velProfile[i][D] - minPathDist[i]) / maxV[i] + velProfile[i][T1];
    velProfile[i][T3] = velProfile[i][T2] + deaccTime[i];
    velProfile[i][A] = velProfile[i][T1] * maxV[i] / 2.0;
    velProfile[i][B] = (velProfile[i][T2] - velProfile[i][T1]) * maxV[i];
    velProfile[i][C] = (velProfile[i][T3] - velProfile[i][T2]) * maxV[i] / 2.0;
    // Serial.println("Vel profile okay");
    //    cout << "From "<<edge2point(i,1)<< " to "<<edge2point(i,2)<<endl;
    //    cout << edge2point(i,1) << ": X - "<<pt2pt[i][INIT_X] << " Y - "<<pt2pt[i][INIT_Y]<<endl;
    //    cout << edge2point(i,2) << ": X - "<<pt2pt[i][END_X] << " Y - "<<pt2pt[i][END_Y]<<endl;
    //    cout << "Control points: X1 - "<<control_pts[i][X1]<< " Y1 - "<<control_pts[i][Y1] << " X2 - "<<control_pts[i][X2]<<" Y2 - "<<control_pts[i][Y2]<<endl;
    //    cout << "D: " << velProfile[i][D] << "\t Mode: " << velProfile[i][VelMode] << endl;
    //    cout << "T1: " << velProfile[i][T1] << "\t T2: " << velProfile[i][T2] << "\t T3: " << velProfile[i][T3] << "\t Total time: " << velProfile[i][T3] << endl;
    //    cout << "A: " << velProfile[i][A] << "\t B: " << velProfile[i][B] << "\t C: " << velProfile[i][C] << "\t Total distance: " << velProfile[i][A] + velProfile[i][B] + velProfile[i][C] << endl << endl;
//    Serial.print("From "); Serial.print(edge2point(i, 1)); Serial.print(" to "); Serial.print(edge2point(i, 2)); Serial.println();
//    Serial.print( edge2point(i, 1)); Serial.print(": X - "); Serial.print(pt2pt[i][INIT_X]); Serial.print(" Y - "); Serial.print(pt2pt[i][INIT_Y]); Serial.println();
//    Serial.print( edge2point(i, 2)); Serial.print(": X - "); Serial.print(pt2pt[i][END_X]); Serial.print(" Y - "); Serial.print(pt2pt[i][END_Y]); Serial.println();
//    Serial.print("Control points: X1 - "); Serial.print(control_pts[i][X1]); Serial.print(" Y1 - "); Serial.print(control_pts[i][Y1]); Serial.print(" X2 - "); Serial.print(control_pts[i][X2]); Serial.print(" Y2 - "); Serial.print(control_pts[i][Y2]); Serial.println();
//    Serial.print("D: "); Serial.print(velProfile[i][D]); Serial.print("\t Mode: "); Serial.print(velProfile[i][VelMode]); Serial.println();
//    Serial.print("T1: "); Serial.print(velProfile[i][T1]); Serial.print("\t T2: "); Serial.print(velProfile[i][T2]); Serial.print("\t T3: "); Serial.print(velProfile[i][T3]); Serial.println();
    //
  }
}


void getPathTargetPos(int edge, int t, int* x, int* y, int* theta)
{

  double u = 0;
  if (edge >= 0 && edge < numEdge)
  {
    if (t <= velProfile[edge][T1])
    {
      //cout << "Acc - ";
      u = (0.5 * acc[edge] * t * t) / velProfile[edge][D];
    }
    else if (t > velProfile[edge][T1] && t <= velProfile[edge][T2])
    {
      //cout << "MaxV - ";
      u = (velProfile[edge][A] + (t - velProfile[edge][T1]) * maxV[edge]) / velProfile[edge][D];
    }
    else if (t < velProfile[edge][T3])
    {
      //cout << "Deacc - ";
      u = (velProfile[edge][A] + velProfile[edge][B]
           + ((t - velProfile[edge][T2]) * maxV[edge] - 0.5 * deacc[edge] * (t - velProfile[edge][T2]) * (t - velProfile[edge][T2]))) / velProfile[edge][D];
    }
    else
    {
      u = 1;
    }

    u = correction(edge,u);
    
    //cout << "init x: " << init_x << "\t init y: " << init_y << endl;
    *x = pt2pt[edge][INIT_X] * (1 - u) * (1 - u) * (1 - u) + 3 * control_pts[edge][X1] * u * (1 - u) * (1 - u) + 3 * control_pts[edge][X2] * u * u * (1 - u) + pt2pt[edge][END_X] * u * u * u;
    *y = pt2pt[edge][INIT_Y] * (1 - u) * (1 - u) * (1 - u) + 3 * control_pts[edge][Y1] * u * (1 - u) * (1 - u) + 3 * control_pts[edge][Y2] * u * u * (1 - u) + pt2pt[edge][END_Y] * u * u * u;
    *theta = 0;
  }
}


/********************************************Velocity, GYRO REQUIRE***********************************************/
float previous_x; float previous_y; float previous_t;
float current_x; float current_y; float current_t;
double accumulate_x; double accumulate_y; double accumulate_t;
float change_x; float change_y; float change_t;

void resetPathTargetCal()
{
  previous_x = 0;
  previous_y = 0;
  previous_t = 0;
  accumulate_x = 0; 
  accumulate_y = 0;
  accumulate_t = 0;
}

void getPathTargetVel(int edge, int t, int* x, int* y, int* theta)
{

  double u = 0;
  if (edge >= 0 && edge < numEdge)
  {
    if (t <= velProfile[edge][T1])
    {
      //cout << "Acc - ";
      u = (0.5 * acc[edge] * t * t) / velProfile[edge][D];
    }
    else if (t > velProfile[edge][T1] && t <= velProfile[edge][T2])
    {
      //cout << "MaxV - ";
      u = (velProfile[edge][A] + (t - velProfile[edge][T1]) * maxV[edge]) / velProfile[edge][D];
    }
    else if (t < velProfile[edge][T3])
    {
      //cout << "Deacc - ";
      u = (velProfile[edge][A] + velProfile[edge][B]
           + ((t - velProfile[edge][T2]) * maxV[edge] - 0.5 * deacc[edge] * (t - velProfile[edge][T2]) * (t - velProfile[edge][T2]))) / velProfile[edge][D];
    }
    else
    {
      u = 1;
    }

    u = correction(edge,u);
    
    //cout << "init x: " << init_x << "\t init y: " << init_y << endl;
    current_x = pt2pt[edge][INIT_X] * (1 - u) * (1 - u) * (1 - u) + 3 * control_pts[edge][X1] * u * (1 - u) * (1 - u) + 3 * control_pts[edge][X2] * u * u * (1 - u) + pt2pt[edge][END_X] * u * u * u;
    current_y = pt2pt[edge][INIT_Y] * (1 - u) * (1 - u) * (1 - u) + 3 * control_pts[edge][Y1] * u * (1 - u) * (1 - u) + 3 * control_pts[edge][Y2] * u * u * (1 - u) + pt2pt[edge][END_Y] * u * u * u;
    
    *x = (current_x - previous_x) * 1000 / loopPeriod;
    *y = (current_y - previous_y) * 1000 / loopPeriod;
    *theta = getChassisTvelGyro(0,50);
    
    previous_x = current_x;
    previous_y = current_y;
  }
}


//Remark: Somehow T not work here
void getPathTargetGlobalPos(int edge, int t, int* x, int* y, int* theta)
{

  double u = 0;
  if (edge >= 0 && edge < numEdge)
  {
    if (t <= velProfile[edge][T1])
    {
      //cout << "Acc - ";
      u = (0.5 * acc[edge] * t * t) / velProfile[edge][D];
    }
    else if (t > velProfile[edge][T1] && t <= velProfile[edge][T2])
    {
      //cout << "MaxV - ";
      u = (velProfile[edge][A] + (t - velProfile[edge][T1]) * maxV[edge]) / velProfile[edge][D];
    }
    else if (t < velProfile[edge][T3])
    {
      //cout << "Deacc - ";
      u = (velProfile[edge][A] + velProfile[edge][B]
           + ((t - velProfile[edge][T2]) * maxV[edge] - 0.5 * deacc[edge] * (t - velProfile[edge][T2]) * (t - velProfile[edge][T2]))) / velProfile[edge][D];
    }
    else
    {
      u = 1;
    }

    u = correction(edge,u);
    
    //cout << "init x: " << init_x << "\t init y: " << init_y << endl;
    current_x = pt2pt[edge][INIT_X] * (1 - u) * (1 - u) * (1 - u) + 3 * control_pts[edge][X1] * u * (1 - u) * (1 - u) + 3 * control_pts[edge][X2] * u * u * (1 - u) + pt2pt[edge][END_X] * u * u * u;
    current_y = pt2pt[edge][INIT_Y] * (1 - u) * (1 - u) * (1 - u) + 3 * control_pts[edge][Y1] * u * (1 - u) * (1 - u) + 3 * control_pts[edge][Y2] * u * u * (1 - u) + pt2pt[edge][END_Y] * u * u * u;
    current_t = -getGyroError(0);
    
    change_x = current_x - previous_x;
    change_y = current_y - previous_y;
    change_t = current_t - previous_t;

    double angle = getGyroError(0)*M_PI/180;
    double Cangle = cos(angle), Sangle = sin(angle);

    accumulate_x += change_x*Cangle+change_y*Sangle;;
    accumulate_y += change_y*Cangle-change_x*Sangle;
    accumulate_t += change_t/5;

    *x = accumulate_x;
    *y = accumulate_y;
    *theta = 0;
    
    previous_x = current_x;
    previous_y = current_y;
    previous_t = current_t;
  }
}
