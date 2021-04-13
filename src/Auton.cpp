#include "vex.h"

using namespace vex;

///In Inches
const float encoderWheelCircumfrence = 12.5663706144;

//Distance from Tracking Centre to Left and Right Encoder Wheels
const float encoderTrackRadius = 10;  

//Distance from Tracking Centre to Rear Encoder Wheel
const float rearEncoderLength = 10;

//Current Coordinates and Rotation
float xCoord = 0;
float yCoord = 0;
float currentRotation = 0;

//Previous Rotatation Values
int previousLEncoder = 0;
int previousREncoder = 0;
int previousBackEncoder = 0;
int prevRotation = 0; // Rotation of whole robot relative to starting position


//Convert Degrees to Radians and Vice Versa
float toRadians(float input)
{
  return input / (180 / 3.1415926535);
}
float toDegrees(float input)
{
  return input * (180 / 3.1415926535);
}

void toPolar(float lateral, float forward, float& radius, float& theta)
{
  radius = sqrt(lateral * lateral + forward * forward);
  theta = toDegrees((atan(forward / lateral)));
}
void toCartesian(float radius, float theta, float& forward, float& lateral)
{
  forward = radius * sin(toRadians(theta));
  lateral = radius * cos(toRadians(theta)); 
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
  toPolar(localXOffset, localYOffset, polarRadius, polarTheta);
  polarTheta -= averageOrientation;

  //Convert Offset Coordinates from Polar to Cartesian and Store as global Coordinates
  xCoord = polarRadius * cos(polarTheta);
  yCoord = polarRadius * sin(polarTheta);
  toCartesian(polarRadius, polarTheta, yCoord, xCoord);

  //Store Previous Values
  prevRotation = currentRotation;
  previousLEncoder = lEncoder.rotation(degrees);
  previousREncoder = rEncoder.rotation(degrees);
  previousBackEncoder = backEncoder.rotation(degrees);
}