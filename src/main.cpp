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
#include "Auton.cpp"

using namespace vex;

// A global instance of competition
competition Competition;
controller Controller1;


motor lDrive1(PORT1, ratio18_1);
motor lDrive2(PORT2, ratio18_1);
motor rDrive1(PORT3, ratio18_1, true);
motor rDrive2(PORT4, ratio18_1, true);

inertial inertialSensor(PORT10);

float MOTOR_ACCEL_LIMIT = 10;

int lastl1Speed = 0;
int lastl2Speed = 0;
int lastr1Speed = 0;
int lastr2Speed = 0;



// Adjusts Input by robot heading such that the robot will always drive forward relative to its starting position unless reset
void AdjustToRotation(float& forwardAxis, float& lateralAxis)
{
  //Convert Forward and Lateral Axis into Polar Coordinates
  float polarRadius = 0;
  float theta = 0;
  toPolar(lateralAxis, forwardAxis, polarRadius, theta);

  //Adjust Heading, 270 becomes -90, this is needed for the math to work
  int head = inertialSensor.heading();
  if(head > 180)
    head -= 360;

  //Adjust for Negative Values
  if(lateralAxis < 0)
    theta += 180;
  else if(forwardAxis < 0)
    theta += 360;

  //Rotate Polar Coordinates of input by the rotation of the robot
  theta += head;

  //Convert Polar Coordinates back into Cartesian
  forwardAxis = 0;
  lateralAxis = 0; 
  toCartesian(polarRadius, theta, forwardAxis, lateralAxis);
}

void DriveWheels(int forwardAxis, int lateralAxis, int rotalAxis)
{
  //Adjust Input Values
  AdjustToRotation(forwardAxis, lateralAxis);

  //Convert Input Values into Speeds for all 4 Motors
  int l1Speed = forwardAxis + lateralAxis + rotalAxis;
  int l2Speed = forwardAxis - lateralAxis + rotalAxis;
  int r1Speed = forwardAxis - lateralAxis - rotalAxis;
  int r2Speed = forwardAxis + lateralAxis - rotalAxis;

  //Clamp Speeds to Acceleration Limits
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
  
  //Set Last Speeds to current
  lastl1Speed = l1Speed;
  lastl2Speed = l2Speed;
  lastr1Speed = r1Speed;
  lastr2Speed = r2Speed;

  //Drive or Brake Wheels
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



void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

void autonomous(void) {

}

void HandleDriveInput()
{
  //Calculate Axis from Controller
  int forwardAxis = Controller1.Axis3.position();
  int lateralAxis = Controller1.Axis4.position();
  int rotalAxis = Controller1.Axis1.position();

  //Adjust for Arrow Key Input
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
  
  //Send input to Wheels
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


/*Paste bin
  if(print){
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Input Polar Radius: "); 
    Brain.Screen.print(polarRadius);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Input Theta: ");
    Brain.Screen.print(theta);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Robot Heading: ");
    Brain.Screen.print(head);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Forward Axis: ");
    Brain.Screen.print(forwardAxis);  
    Brain.Screen.print(" Lateral Axis: ");
    Brain.Screen.print(lateralAxis); 
  }
    if(print){
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("New Input Theta: ");
    Brain.Screen.print(theta);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("New Forward Axis: ");
    Brain.Screen.print(forwardAxis);  
    Brain.Screen.print(" New Lateral Axis: ");
    Brain.Screen.print(lateralAxis); 
  }  
 */