/**********************************************************************************************
* Arduino PID Library - Version 1.0.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
* Modified by Lau Ka Chun, Mechanical and Automation Engineering, The Chinese University of Hong Kong
* Last modification: 18 May 2013
* This Library is licensed under a GPLv3 License
**********************************************************************************************/

#ifndef PID_h
#define PID_h
#define LIBRARY_VERSION	1.0.1

class PID
{
public:
	//Constants used in some of the functions below
	#define AUTOMATIC	1
	#define MANUAL		0

	// Constructor. Initial tuning parameters, output limits and sampling time
	PID();
	PID(int Kp, int Ki, int Kd, int MinOut, int MaxOut, unsigned int Period);   

	// Performs the PID calculation.  it should be called every time loop() cycles. ON/OFF and
	// calculation frequency can be set using SetMode SetSampleTime respectively
	bool Compute();
	
	// While most users will set the tunings once in the constructor, this function gives the user the option
	// of changing tunings during runtime for Adaptive control
	void SetTunings(int Kp, int Ki, int Kd);

	// Sets the frequency, in Milliseconds, with which the PID calculation is performed.
	void SetSampleTime(unsigned int NewSampleTime);
	
	// Clamps the output to a specific range.
	void SetOutputLimits(int Min, int Max); 

	// Sets PID to either Manual (0) or Auto (non-0)
	void SetMode(int Mode);

	// Setup setpoint	
	void SetSetpoint(long Setpoint);
	
	// Setup input
	void SetInput(long);
	
	// Get output	
	int GetOutput();
	
	//Display functions ****************************************************************
	int GetKp();
	int GetKi();
	int GetKd();
	int GetPTerm();
	int GetITerm();
	int GetDTerm();
  int GetMyInput();
  int GetLastInput();
	int GetMode();

	void Initialize();

private:
	int kp, ki, kd;

	long mySetpoint;
	long myInput;				// Input, Output, and Setpoint variables
	long lastInput;
	int myOutput;
	int error;
  int lastError;

	unsigned long now, lastTime;
	unsigned int sampleTime;

	bool inAuto;
	int outMin, outMax;
	int pTerm, iTerm, dTerm;
};
#endif
