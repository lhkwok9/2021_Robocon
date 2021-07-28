#define RASPI 1
#define ESP32 2

#ifndef PS4Controller

#define U7 4
#define D7 9
#define L7 5
#define R7 8
#define U8 6
#define D8 11
#define L8 10
#define R8 7
#define L1 3
#define L2 0
#define R1 2
#define R2 1

#define ADC1 2
#define ADC2 3
#define ADC3 1
#define ADC4 0

// Redefine for ps4 controller
#define TRIANGLE U8
#define CIRCLE R8
#define CROSS D8
#define SQUARE L8
#define LEFT L7
#define DOWN D7
#define RIGHT R7
#define UP U7

#else

#define R3 13
#define L3 12
#define OPTIONS 15
#define SHARE 14
#define R2 7
#define L2 6
#define R1 5
#define L1 4
#define TRIANGLE 2
#define CIRCLE 1
#define CROSS 0
#define SQUARE 3
#define LEFT 8
#define DOWN 11
#define RIGHT 9
#define UP 10

#define ADC1 2
#define ADC2 3
#define ADC3 1
#define ADC4 0

// Redefine for vex controller
#define U8 TRIANGLE 
#define R8 CIRCLE
#define D8 CROSS
#define L8 SQUARE
#define L7 LEFT 
#define D7 DOWN
#define R7 RIGHT
#define U7 UP

#endif  
