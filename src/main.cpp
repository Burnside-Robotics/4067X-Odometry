/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// lEncoder             encoder       A, B            
// rEncoder             encoder       C, D            
// backEncoder          encoder       E, F            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
controller Controller1;


motor lDrive1(PORT1, ratio18_1);
motor lDrive2(PORT2, ratio18_1);
motor rDrive1(PORT3, ratio18_1);
motor rDrive2(PORT4, ratio18_1);



const float encoderWheelCircumfrence = 0;
//Distance from Tracking Centre to Left and Right Encoder Wheels
const float encoderTrackRadius = 10;  

//Distance from Tracking Centre to Rear Encoder Wheel
const float rearEncoderLength = 10;



int prevXCoord = 0;
int prevYCoord = 0;
int prevRotation = 0;

int xCoord = 0;
int yCoord = 0;
int currentRotation = 0;

int previousLEncoder = 0;
int previousREncoder = 0;
int previousBackEncoder = 0;


void CalculatePosition()
{
  float lDistChange = (lEncoder.rotation(deg) - previousLEncoder) / 360 * encoderWheelCircumfrence;
  float rDistChange = (rEncoder.rotation(deg) - previousREncoder) / 360 * encoderWheelCircumfrence;
  float backDistChange = (backEncoder.rotation(deg) - previousBackEncoder) / 360 * encoderWheelCircumfrence;
  float totallDist = lEncoder.rotation(deg) / 360 * encoderWheelCircumfrence;
  float totalrDist = rEncoder.rotation(deg) / 360 * encoderWheelCircumfrence;
  currentRotation = (totallDist - totalrDist) / (encoderTrackRadius * 2);
  float angleChange = currentRotation - prevRotation;
  float localXOffset = 0;
  float localYOffset = 0;
  if(angleChange == 0)
  {
    localXOffset = backDistChange;
    localYOffset = rDistChange;
  }
  else
  {
    localXOffset = 2 * sin(currentRotation/2) * ((backDistChange / angleChange) + rearEncoderLength);
    localYOffset = 2 * sin(currentRotation/2) * ((rDistChange / angleChange) + encoderTrackRadius);
  }
  float averageOrientation = prevRotation + angleChange / 2;
  float polarRadius = sqrt(localXOffset * localXOffset + localYOffset * localYOffset);
  float polarTheta = atan(localYOffset / localXOffset);
  polarTheta -= averageOrientation;
  xCoord = polarRadius * cos(polarTheta);
  yCoord = polarRadius * sin(polarTheta);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

void autonomous(void) {

}


void usercontrol(void) {
  while (1) {
    CalculatePosition();
    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
