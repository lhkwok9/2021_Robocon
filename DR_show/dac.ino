void initDac()
{
  analogWriteResolution(12);
//  analogWrite(DAC0, LeftDacValue[1]);
//  analogWrite(DAC1, RightDacValue[1]);
  analogWrite(DAC0, 0);
  analogWrite(DAC1, 0);
}

void setDac(int i, int value) 
{
  if(i==0){
    analogWrite(DAC0, value);   //position1(front): 240, 790   position1(back): 330, 950 
  }else{
    analogWrite(DAC1, value);
    Serial.println(value);
  }
}
//1.22 pa
//1.22 pa
//set  pa 1 - 6
//4095: 5.14 pa
//3095: 4.34 pa
//2095: 3.52 pa
//1095: 2.72 pa
//95: 1.91
//10: 1.83 pa
//1: 1.83 pa

//Alwin
//2040: 3.07-3.08 pa
//1700: 2.75



//2050: 1.66, 2.49  --> 1.5
//3000: 2.17, 3.26    --> 1.506
//3100: 2.223,3.31    --> 1.489
//3250: 2.303, 3.43  -->1.489
//3300: 2.33, 3.47    -->1.489
//3900:   2.65, 3.41  -->1.28


//7.4V
//1000: 1.093, 1.641 -->1.5

//4095: 2.78   4.11  --> 1.478
