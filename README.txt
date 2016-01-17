THIS IS NOT THE OFFICIAL CODE!!! THIS IS A PORT TO ROBOTC FOR VEX CORTEX - SEE BELOW

***************************************************************
* Arduino PID Library - Version 1.1.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under a GPLv3 License
***************************************************************

 - For an ultra-detailed explanation of why the code is the way it is, please visit: 
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary


 * Adapted for VEX ROBOTC by Brian Pratt <brianstephenspratt@gmail.com>
 * Assumes that we're controlling rotation speed using a quad encoder
 * N.B. could be further simplified but left a lot alone for ease of 
 * tracking any developments in the Arduino code, especially the use
 * of doubles which makes ROBOTC issue a raft of warnings.