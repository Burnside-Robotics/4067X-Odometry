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


const float encoderWheelCircumfrence = 21.9440246847;

//Distance from Tracking Centre to Left and Right Encoder Wheels
const float encoderTrackRadius = 30;  

//Distance from Tracking Centre to Rear Encoder Wheel
const float rearEncoderLength = 10;




//Current Coordinates and Rotation
int xCoord = 0;
int yCoord = 0;
int currentRotation = 0;

//Previous Rotatation Values
int previousLEncoder = 0;
int previousREncoder = 0;
int previousBackEncoder = 0;
int prevRotation = 0; // Rotation of whole robot relative to starting position


float MOTOR_ACCEL_LIMIT = 10;

int lastl1Speed = 0;
int lastl2Speed = 0;
int lastr1Speed = 0;
int lastr2Speed = 0;

//Convert Degrees to Radians and Vice Versa
float toRadians(float input)
{
  return input / (180 / 3.1415926535);
}
float toDegrees(float input)
{
  return input * (180 / 3.1415926535);
}
void toCartesian(int radius, int theta, int& posX, int& posY)
{
  posY = radius * sin(toRadians(theta));
  posX = radius * cos(toRadians(theta)); 
}
void CircleWithLine(int posX, int posY, int radius, int theta)
{
  Brain.Screen.clearScreen();
  Brain.Screen.drawCircle(posX, posY, radius);
  int lineX = 0;
  int lineY = 0;
  theta -= theta * 2;
  toCartesian(radius, theta, lineX, lineY);
  Brain.Screen.drawLine(posX, posY, lineX + posX, lineY + posY);
}
// Adjusts Input by robot heading such that the robot will always drive forward relative to its starting position unless reset
void AdjustToRotation(int& forwardAxis, int& lateralAxis)
{
  //Convert Forward and Lateral Axis into Polar Coordinates
  int polarRadius = sqrt(lateralAxis * lateralAxis + forwardAxis * forwardAxis);
  float theta = toDegrees((atan(forwardAxis / lateralAxis)));

  //Adjust Heading, 270 becomes -90, this is needed for the math to work
  int head = inertialSensor.heading();
  if(head > 180)
    head -= 360;
  CircleWithLine(100,100, 40, head);
  //Adjust for Negative Values
  if(lateralAxis < 0)
    theta += 180;
  else if(forwardAxis < 0)
    theta += 360;

  //Rotate Polar Coordinates of input by the rotation of the robot
  theta += head;

  //Convert Polar Coordinates back into Cartesian
  forwardAxis = polarRadius * sin(toRadians(theta));
  lateralAxis = polarRadius * cos(toRadians(theta)); 
}

void DriveWheels(int forwardAxis, int lateralAxis, int rotalAxis)
{
  //Adjust Input Values
  //AdjustToRotation(forwardAxis, lateralAxis);

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


//Calculate XYCoordinates of Robot
void CalculatePosition()
{
  //Calculate how far all encoders ahve traveled since last call
  float lDistChange = (lEncoder.rotation(deg) - previousLEncoder) / 360 * encoderWheelCircumfrence;
  float rDistChange = (rEncoder.rotation(deg) - previousREncoder) / 360 * encoderWheelCircumfrence;
  float backDistChange = (backEncoder.rotation(deg) - previousBackEncoder) / 360 * encoderWheelCircumfrence;

  //Calculate Total Distance Left and Right Encoders have Traveled in Total
  float totallDist = lEncoder.rotation(deg) / 360 * encoderWheelCircumfrence;
  float totalrDist = rEncoder.rotation(deg) / 360 * encoderWheelCircumfrence;

  //Calculate the Current Orientation of the Robot
  currentRotation = (totallDist - totalrDist) / (encoderTrackRadius * 2);

  //Calculate Change in rotation of the robot since last call
  float angleChange = currentRotation - prevRotation;

  float localXOffset = 0;
  float localYOffset = 0;

  //If the Robot has not rotated, the X and Y offset since last call are exactly equal to the distance the encoders have Traveled
  if(angleChange == 0)
  {
    localXOffset = backDistChange;
    localYOffset = rDistChange;
  }
  //If the robot has rotated, calculate how much the robot has offset since the last call with trig
  else
  {
    localXOffset = 2 * sin(currentRotation/2) * ((backDistChange / angleChange) + rearEncoderLength);
    localYOffset = 2 * sin(currentRotation/2) * ((rDistChange / angleChange) + encoderTrackRadius);
  }

  //Calculate The Average Oreintation of the change in angle since the last call, and the last angle
  float averageOrientation = prevRotation + angleChange / 2;

  //Convert the Offset Coordinates into Polar
  float polarRadius = sqrt(localXOffset * localXOffset + localYOffset * localYOffset);
  float polarTheta = atan(localYOffset / localXOffset);
  polarTheta -= averageOrientation;

  //Convert Offset Coordinates from Polar to Cartesian and Store as global Coordinates
  xCoord = polarRadius * cos(polarTheta);
  yCoord = polarRadius * sin(polarTheta);

  //Store Previous Values
  prevRotation = currentRotation;
  previousLEncoder = lEncoder.rotation(degrees);
  previousREncoder = rEncoder.rotation(degrees);
  previousBackEncoder = backEncoder.rotation(degrees);
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
void PrintOdomData()
{

}
void usercontrol(void) {
  while (1) {
    //HandleDriveInput();
    CalculatePosition();
    PrintOdomData();
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

    Brain.Screen.setCursor(9, 1);
  Brain.Screen.print("Current Rotation: ");
  Brain.Screen.print(currentRotation);
  Brain.Screen.print(" Dist Subtraction: ");
  Brain.Screen.print((totallDist - totalrDist));
  Brain.Screen.setCursor(10, 1);
  Brain.Screen.print(" Track Diameter ");
  Brain.Screen.print((encoderTrackRadius * 2));


  
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(8, 1);  
  Brain.Screen.print("L Encoder Total: ");
  Brain.Screen.print(totallDist);
  Brain.Screen.print(" R Encoder Total: ");
  Brain.Screen.print(totalrDist);
  

 */