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

#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.1.1

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1

  #define ARRAYSIZE(a) (sizeof(a)/sizeof(a[0]))

  //commonly used functions **************************************************************************
   void PID(int InputPort, int *OutputPorts, int nOutputPorts, float Setpoint,        // * constructor.  links the PID to the Input, Output, and
        float Kp, float Ki, float Kd, int ControllerDirection);     //   Setpoint.  Initial tuning parameters are also set here

    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(float Min, float Max); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



  //available but not commonly used functions ********************************************************
    void SetTunings(float Kp, float Ki,       // * While most users will set the tunings once in the
                    float Kd);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
	void SetControllerDirection(int Direction);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int NewSampleTime);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100



  //Display functions ****************************************************************
	float GetKp();						  // These functions query the pid for interal values.
	float GetKi();						  //  they were created mainly for the pid front-end,
	float GetKd();						  // where it's important to know what is actually
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

	void Initialize();

	float dispKp;				// * we'll hold on to the tuning parameters in user-entered
	float dispKi;				//   format for display purposes
	float dispKd;				//

	float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;

    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the
    float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
	unsigned long lastTime;
	float ITerm, lastInput;

	unsigned long SampleTime;
	float outMin, outMax;
	bool inAuto;

	///
	/// Stuff added for ROBOT-C/VEX use
	void ChangeSetpoint(int delta);
	void SetSetpoint(int setpoint);
	float _setpoint;
	int *myOutputPorts;
	int myOutputPortCount;
	int myInputPort;
	unsigned long millis();
	task pidController();
	float lastError; // Of possible use in setting up different tunings depending on how big error is (see Arduino example)

#endif
