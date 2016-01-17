/* proof of concept exerciser for ROBOTC port of Arduino PID library */

#include <PID_v1.cpp> // ROBOTC port of Arduino PID library

#include <MyPIDSetup.cpp> // Tweak the contents of the include file to suit your hardware

task main()
{

	MyPIDSetup(); // Configure PID and start it running (see MyPIDSetup.cpp)

	// use joystick to modify the requested speed
	while( true )
	{
		// bump setupoint up or down while joystick is held
		int bump = 10; // No idea if this is a meaningful value

		if (vexRT[ Ch2 ]>10)
			ChangeSetpoint(bump);
		else if (vexRT[ Ch2 ]<10)
			ChangeSetpoint(-bump);
		wait1Msec(50);
	}

}
