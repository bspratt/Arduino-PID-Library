/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * Adapted for ROBOT-C by Brian Pratt <brianstephenspratt@gmail.com>
 * Assumes that we're controlling RPM using a quad encoder
 * N.B. could be further simplified but left a lot alone for ease of tracking
 * developments in the Arduino code
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void PID(int InputPort, int *OutputPorts, int nOutputPorts, float Setpoint,
        float Kp, float Ki, float Kd, int ControllerDirection)
{

    myOutputPorts = OutputPorts;
    myOutputPortCount = nOutputPorts;
    myInputPort = InputPort;
    mySetpoint = &_setpoint;
    *mySetpoint = Setpoint;
	  inAuto = true;

	  SetOutputLimits(0, 127);				//default output limit corresponds to
												//the vex motor command limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    SetControllerDirection(ControllerDirection);
    SetTunings(Kp, Ki, Kd);

    lastTime = millis()-SampleTime;

    // start the PID task
    StartTask( pidController );
}

  // Arduino-style lock check
	unsigned long millis()
	{
		return nPgmTime;
	}

/*
  Motor control task.  Ideally run at high priority.
*/
task pidController()
{
	int lastOutput = 0;
	while(true)
	{
	  // Calculate error and calculate new output level
		Compute();
		// Set the motors
		int output = (int)*myOutput;
		if (output != lastOutput)
		{
			lastOutput = output;
			for (int i=myOutputPortCount; i-- > 0;)
			{
				motor[myOutputPorts[i]] = output;
			}
		}
		// Let other tasks have a go
		endTimeSlice();
	}
}

void ChangeSetpoint(int delta)
{
	*mySetpoint += delta;
}

void SetSetpoint(int setpoint)
{
	*mySetpoint = setpoint;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      		// Read the encoder
	    float input = (float)SensorValue[myInputPort] / (float)timeChange;
	    if(controllerDirection ==REVERSE)
	    	input = -input;
	    *myInput = input;
      float error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      float dInput = (input - lastInput);

      /*Compute PID Output*/
      float output = kp * error + ITerm- kd * dInput;

	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
      // reset encoder count
      SensorValue[myInputPort] = 0;
      lastError = error;
	  return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void SetTunings(float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   float SampleTimeInSec = ((float)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

   /* handle this at encoder read instead
  if(controllerDirection == REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   */
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
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
void SetOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto != inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void SetControllerDirection(int Direction)
{
	/* handle this at encoder read
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   */
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float GetKp(){ return  dispKp; }
float GetKi(){ return  dispKi;}
float GetKd(){ return  dispKd;}
int GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int GetDirection(){ return controllerDirection;}
