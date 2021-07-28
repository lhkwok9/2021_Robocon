
float ypr[3] = {-1, -1, -1};
float targetYpr[3] = {-1, -1, -1};

int cnt;

void initGyro()
{
	Serial2.begin(115200);  //In case not yet declare

  Serial.println("Start init gyro");
  while(cnt == 0)
  {
    getGyro();
  }
  setCurrentGyroPos();
  Serial.println("End init");
}

void getGyro()  //Need to do in each loop
{
  receiveIMUPacket();
}

void setCurrentGyroPos()
{
  for (int i = 0;i<3;i++)
  {
    targetYpr[i] = ypr[i];
  }
}

float angle;
float getGyroError(int num)   // 0 = turning , 1 = ? , 2 = ?
{
  if (num >= 0 && num < 3)
  {
    angle = ypr[num] - targetYpr[num];
    if(angle < -180) angle += 360;
    else if (angle > 180) angle -= 360;
    return angle;
  }
  else
  {
    return 0;
  }
}

float getGyroError(int num, int targetAngle) // 0 = turning , 1 = ? , 2 = ?
{
  if (num >= 0 && num < 3)
  {
    angle = ( ypr[num] - targetYpr[num]) - targetAngle;
    if(angle < -180) angle += 360;
    else if (angle > 180) angle -= 360;
    return angle;
  }
  else
  {
    return 0;
  }
}


void debugGyro()
{
//  Serial.print("Target Value: ");
//  for (int i = 0;i < 3; i++)
//  {
//    Serial.print(targetYpr[i]);
//    Serial.print(' ');
//  }
//  Serial.println();
//
//  Serial.print("Current Value: ");
//  for (int i = 0;i < 3; i++)
//  {
//    Serial.print(ypr[i]);
//    Serial.print(' ');
//  }
//  Serial.println();

//    Serial.print("Error: ");
//    for (int i = 0;i<3;i++)
//    { 
//      Serial.print(getGyroError(i));
//      Serial.print(" ");
//    }
//    Serial.println();
}


// ==================Backend code==================
unsigned int packet_byte_count = 0;
bool flag_receiving_packet = false;
char packet[27 + 5];
int distance = -1;
bool flag_out_of_range = false;
int buf[20] = {0};
// drop the first entry
void pop()
{
	if(cnt > 0){
		for(unsigned int i = 0; i < cnt - 1; i++){
			buf[i] = buf[i + 1];
		}
		cnt--;
	}
}

void push(int x)
{
	// if the array is full, drop the first entry
	if(cnt >= sizeof(buf) / sizeof(int)){
		pop();
	}

	// push the value to the array
	buf[cnt++] = x;
}

long get_sum()
{
	long sum = 0;

	for(unsigned int i = 0; i < cnt; i++){
		sum += buf[i];
	}

	return sum;
}

float get_moving_average(int x)
{
	if(x == -1){
		// out of range
		// drop the first entry
		pop();
	}else{
		// push the value to the array
		push(x);
	}

	if(cnt <= 0){
		return -1;
	}else{

		return (float)get_sum() / (float)cnt;
	}
	
}

/*float get_moving_average(int x)
{
	static bool flag = false;
	static int buf[20];
	long sum = 0;

	if(flag == false){
		flag = true;
		for(unsigned int i = 0; i < sizeof(buf) / sizeof(int); i++){
			buf[i] = x;
		}
	}else{
		for(unsigned int i = 0; i < sizeof(buf) / sizeof(int) - 1; i++){
			buf[i] = buf[i + 1];
		}
		buf[sizeof(buf) / sizeof(int) - 1] = x;
	}

	for(unsigned int i = 0; i < sizeof(buf) / sizeof(int); i++){
		sum += buf[i];
	}

	return (float)sum / (sizeof(buf) / sizeof(int));
}*/

void decodeIMUPacket()
{
	float yaw = atof(&packet[1]);
	float pitch = atof(&packet[9]);
	float roll = atof(&packet[17]);
	int dist = atoi(&packet[25]);

	ypr[0] = yaw;
	ypr[1] = pitch;
	ypr[2] = roll;

	distance = get_moving_average(dist);

	if(distance == -1){
		flag_out_of_range = true;
	}else{
		flag_out_of_range = false;
	}
}

bool receiveIMUPacket()
{
	char ch;

	while(Serial2.available()){
		ch = Serial2.read();
		// Serial.print(ch);

		if(ch == '['){
			// reset the whole process
			flag_receiving_packet = true;
			packet_byte_count = 0;
			packet[packet_byte_count++] = ch;

		}else if(flag_receiving_packet == true){
			packet[packet_byte_count++] = ch;

			if(packet_byte_count == sizeof(packet) - 1){
				// reset the whole process
				flag_receiving_packet = false;
				
				if(ch == ']'){
					// decode packet only if ']' token is received
					decodeIMUPacket();
					return true;
				}else{
					// do nothing
					// if the ']' token is not received,
					// drop the packet
				}
			}
		}
	}
	return false;
}
