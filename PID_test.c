/* proof of concept exerciser for ROBOT-C port of Arduino PID library */

///
/// Motor Control setup.  Consider moving this function into
/// an include file for easy use in development and competition code.
///
#include <PID_v1.cpp> // ROBOT-C port of Arduino PID library

void MyPIDSetup()
{
    int EncoderPort = 0;  // Encoder sensor port
    int MotorPorts[] = { 1, 2, 3, 4, 5 }; // Motor drive ports - we assume that +127 give max positive RPM
    int ControllerDirection = DIRECT; // set to REVERSE if your encoder is reversed relative to output direction

    float Setpoint = 500; // normal desired speed, in encoder ticks per millisecond

    // "Conservative" tuning parameters per the Arduino adaptive tuning example (aggressive: Kp=4, Ki=0.2, Kd=1;)
    float Kp = 1;
    float Ki = .05;
    float Kd = .25;

    // Begin the motor control background task
    PID(EncoderPort, MotorPorts, ARRAYSIZE(MotorPorts), Setpoint, Kp, Ki, Kd, ControllerDirection);
}
///
/// End of Motor Control setup.  Consider moving this function into
/// an include file for easy use in development and competition code.
///


task main()
{

    MyPIDSetup();

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
