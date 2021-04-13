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
motor rDrive1(PORT3, ratio18_1, true);
motor rDrive2(PORT4, ratio18_1, true);

inertial inertialSensor(PORT10);


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

float MOTOR_ACCEL_LIMIT = 10;


int lastl1Speed = 0;
int lastl2Speed = 0;
int lastr1Speed = 0;
int lastr2Speed = 0;

float toRadians(float input)
{
  return input / (180 / 3.1415926535);
}
float toDegrees(float input)
{
  return input * (180 / 3.1415926535);
}
// Adjusts Input by robot heading
void AdjustToRotation(int& forwardAxis, int& lateralAxis)
{
  int polarRadius = sqrt(lateralAxis * lateralAxis + forwardAxis * forwardAxis);
  float theta = toDegrees((atan(forwardAxis / lateralAxis)));
  int head = inertialSensor.heading();
  if(lateralAxis < 0)
    theta += 180;
  else if(forwardAxis < 0)
    theta += 360;

  if(head > 180)
    head -= 360;
  theta += head;
  forwardAxis = polarRadius * sin(toRadians(theta));
  lateralAxis = polarRadius * cos(toRadians(theta)); 
}
void DriveWheels(int forwardAxis, int lateralAxis, int rotalAxis)
{
  AdjustToRotation(forwardAxis, lateralAxis);
  int l1Speed = forwardAxis + lateralAxis + rotalAxis;
  int l2Speed = forwardAxis - lateralAxis + rotalAxis;
  int r1Speed = forwardAxis - lateralAxis - rotalAxis;
  int r2Speed = forwardAxis + lateralAxis - rotalAxis;

  if ((l1Speed - lastl1Speed) > MOTOR_ACCEL_LIMIT)
      l1Speed = lastl1Speed + MOTOR_ACCEL_LIMIT;
  if ((l1Speed - lastl1Speed) < -MOTOR_ACCEL_LIMIT)
      l1Speed = lastl1Speed - MOTOR_ACCEL_LIMIT;
  if ((l2Speed - lastl2Speed) > MOTOR_ACCEL_LIMIT)
      l2Speed = lastl2Speed + MOTOR_ACCEL_LIMIT;
  if ((l2Speed - lastl2Speed) < -MOTOR_ACCEL_LIMIT)
      l2Speed = lastl2Speed - MOTOR_ACCEL_LIMIT;

  if ((r1Speed - lastr1Speed) > MOTOR_ACCEL_LIMIT)
      r1Speed = lastr1Speed + MOTOR_ACCEL_LIMIT;
  if ((r1Speed - lastr1Speed) < -MOTOR_ACCEL_LIMIT)
      r1Speed = lastr1Speed - MOTOR_ACCEL_LIMIT;
  if ((r2Speed - lastr2Speed) > MOTOR_ACCEL_LIMIT)
      r2Speed = lastr2Speed + MOTOR_ACCEL_LIMIT;
  if ((r2Speed - lastr2Speed) < -MOTOR_ACCEL_LIMIT)
      r2Speed = lastr2Speed - MOTOR_ACCEL_LIMIT;

  lastl1Speed = l1Speed;
  lastl2Speed = l2Speed;
  lastr1Speed = r1Speed;
  lastr2Speed = r2Speed;

  if (l1Speed == 0)
      lDrive1.stop(brakeType::brake);
  else
      lDrive1.spin(directionType::fwd, l1Speed, velocityUnits::pct);
  if (l2Speed == 0)
      lDrive2.stop(brakeType::brake);
  else
      lDrive2.spin(directionType::fwd, l2Speed, velocityUnits::pct);
  if (r1Speed == 0)
      rDrive1.stop(brakeType::brake);
  else
      rDrive1.spin(directionType::fwd, r1Speed, velocityUnits::pct);
  if (r2Speed == 0)
      rDrive2.stop(brakeType::brake);
  else
      rDrive2.spin(directionType::fwd, r2Speed, velocityUnits::pct);
}
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

void HandleDriveInput()
{
  int forwardAxis = Controller1.Axis3.position();
  int lateralAxis = Controller1.Axis4.position();
  int rotalAxis = Controller1.Axis1.position();

  if(Controller1.ButtonUp.pressing())
  {
    forwardAxis += 100;
  }
  else if(Controller1.ButtonDown.pressing())
  {
    forwardAxis -= 100;
  }
  else if(Controller1.ButtonRight.pressing())
  {
    lateralAxis += 100;
  }
  else if(Controller1.ButtonLeft.pressing())
  {
    lateralAxis -= 100;
  }
  DriveWheels(forwardAxis, lateralAxis, rotalAxis);
}
void usercontrol(void) {
  while (1) {
    HandleDriveInput();
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
