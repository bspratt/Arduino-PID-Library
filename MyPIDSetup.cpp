///
/// Motor Control setup.  This is normally the only file you'd want to change, to suit your hardware setup.
/// Note that we are NOT using pragmas and the motor+sensor setup dialog due to difficulties with include files.
///
void MyPIDSetup()
{
    //
    // Stuff you almost certainly need to change
    //
    tSensors EncoderPort = dgtl1;  // Encoder sensor port
    tMotor MotorPorts[] = { port1, port2, port3, port4 }; // Motor drive ports
    bool MotorReversed[] = {false, true, false, true}; // Motor direction sense

    float Setpoint = 500; // normal desired speed, in encoder ticks per millisecond

    // "Conservative" tuning parameters per the Arduino adaptive tuning example (example's "aggressive": Kp=4, Ki=0.2, Kd=1;)
    float Kp = 1;
    float Ki = .05;
    float Kd = .25;

    //
    // Stuff you probably don't need to change, but might
    //
    int ControllerDirection = DIRECT; // see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/

    // Begin the motor control background task
    PID(EncoderPort, MotorPorts, MotorReversed, ARRAYSIZE(MotorPorts), Setpoint, Kp, Ki, Kd, ControllerDirection);
}
