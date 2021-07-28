/**********************************************************************************************
* Arduino PID Library - Version 1.0.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
* Modified by Lau Ka Chun, Mechanical and Automation Engineering, The Chinese University of Hong Kong
* Last modification: 18 May 2013
* This Library is licensed under a GPLv3 License
**********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID.h"

PID::PID(int Kp, int Ki, int Kd, int MinOut, int MaxOut, unsigned int Period)
{
	inAuto = false;
	iTerm = 0;
  lastInput = lastError = 0;

	PID::SetOutputLimits(MinOut, MaxOut);

	sampleTime = Period;

	PID::SetTunings(Kp, Ki, Kd);

	lastTime = millis() - sampleTime;				
}

/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   pid Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/ 
bool PID::Compute()
{
	if(!inAuto)
		return false;
	now = millis();
	if(now - lastTime >= sampleTime)
	{
		/*Compute all the working error variables*/
		error = mySetpoint - myInput;
   
    // P term
    pTerm = kp * error;

    // I term
		iTerm += (ki * error / 1000000);
		if(iTerm > outMax)
			iTerm = outMax;
		else if(iTerm < outMin)
			iTerm = outMin;

		// D term
		dTerm = -kd * (myInput - lastInput);

		//Compute PID Output
		myOutput = (pTerm + dTerm) / 1000 + iTerm;

		if(myOutput > outMax)
			myOutput = outMax;
		else if(myOutput < outMin)
			myOutput = outMin;

		/*Remember some variables for next time*/
		lastInput = myInput;
    lastError = error;
		lastTime = now;
		return true;
	}
	else
		return false;
}


/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted. 
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/ 
void PID::SetTunings(int Kp, int Ki, int Kd)
{
	if (Kp < 0 || Ki < 0 || Kd < 0)
		return;

	kp = Kp;
	ki = Ki * sampleTime;
	kd = Kd / sampleTime;
}

/* SetSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed	
******************************************************************************/
void PID::SetSampleTime(unsigned int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		ki = ki * NewSampleTime / sampleTime;
		kd = kd / NewSampleTime * sampleTime;
		sampleTime = (unsigned int)NewSampleTime;
	}
}

/* SetOutputLimits(...)****************************************************
*     This function will be used far more often than SetInputLimits.  while
*  the input to the controller will generally be in the 0-1023 range (which is
*  the default already,)  the output will be a little different.  maybe they'll
*  be doing a time window and will need 0-8000 or something.  or maybe they'll
*  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
*  here.
**************************************************************************/
void PID::SetOutputLimits(int Min, int Max)
{
	if(Min >= Max)
		return;

	outMin = Min;
	outMax = Max;

	if(inAuto)
	{
		if(myOutput > outMax)
			myOutput = outMax;
		else if(myOutput < outMin)
			myOutput = outMin;

		if(iTerm > outMax)
			iTerm = outMax;
		else if(iTerm < outMin)
			iTerm = outMin;
	}
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/ 
void PID::SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if(newAuto == !inAuto)
	{
		/*we just went from manual to auto*/
		PID::Initialize();
	}
	inAuto = newAuto;
}

void PID::SetSetpoint(long Setpoint)
{
	mySetpoint = Setpoint;
}

void PID::SetInput(long Input)
{
	myInput = Input;
}

int PID::GetOutput()
{
	return myOutput;
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/ 
void PID::Initialize()
{
	iTerm = myOutput;
	if(iTerm > outMax)
		iTerm = outMax;
	else if(iTerm < outMin)
		iTerm = outMin;
   
  lastInput = myInput = 0;
}

/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the PID.  they're here for display 
* purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
int PID::GetKp(){ return  kp; }
int PID::GetKi(){ return  ki; }
int PID::GetKd(){ return  kd; }
int PID::GetPTerm() { return pTerm; }
int PID::GetITerm() { return iTerm; }
int PID::GetDTerm() { return dTerm; }
int PID::GetMyInput() { return myInput; }
int PID::GetLastInput() { return lastInput; }
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL; }
