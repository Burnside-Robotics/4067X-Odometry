   /*----------------------------------------------------------------------------*/
  /*                                                                            */
  /*    Module:       main.cpp                                                  */
  /*    Author:       Alex Cutforth                                             */
  /*    Created:      Thu Sep 26 2019                                           */
  /*    Description:  X-Drive code for 4067X                                    */
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


  const float encoderWheelCircumfrence = 8.63937979737;

  //Distance from Tracking Centre to Left and Right Encoder Wheels
  const float encoderTrackRadius = 7.244094;  

  //Distance from Tracking Centre to Rear Encoder Wheel
  const float rearEncoderLength = 6.29921;




  //Current Coordinates and Rotation
  float xCoord = 0;
  float yCoord = 0;
  float currentRotation = 0;

  //Previous Rotatation Values
  float previousLEncoder = 0;
  float previousREncoder = 0;
  float previousBackEncoder = 0;
  float prevRotation = 0; // Rotation of whole robot relative to starting position

  float totallDist = 0;
  float totalrDist = 0;

  float lDistChange = 0;
  float rDistChange = 0;
  float backDistChange = 0;

  float angleChange = currentRotation - prevRotation;

  float localXOffset = 0;
  float localYOffset = 0;

  float averageOrientation = 0;

  float offsetRadius = 0;
  float offsetTheta = 0;

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
  void toCartesian(float radius, float theta, float& posX, float& posY)
  {
    posY = radius * sin(toRadians(theta));
    posX = radius * cos(toRadians(theta)); 
  }
  void toPolar(float posX, float posY, float& radius, float &theta)
  {
    radius = sqrt(posX * posX + posY * posY);
    theta = atan(posY / posX);
  }
  void CircleWithLine(int posX, int posY, int radius, int theta)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.drawCircle(posX, posY, radius);
    float lineX = 0;
    float lineY = 0;
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
    int head = toDegrees(currentRotation);
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


  //Calculate XYCoordinates of Robot
  void CalculatePosition()
  {
    //Calculate how far all encoders ahve traveled since last call
    lDistChange = (lEncoder.rotation(deg) - previousLEncoder) / 360 * encoderWheelCircumfrence;
    rDistChange = (rEncoder.rotation(deg) - previousREncoder) / 360 * encoderWheelCircumfrence;
    backDistChange = (backEncoder.rotation(deg) - previousBackEncoder) / 360 * encoderWheelCircumfrence;

    //Calculate Total Distance Left and Right Encoders have Traveled in Total
    totallDist = lEncoder.rotation(deg) / 360 * encoderWheelCircumfrence;
    totalrDist = rEncoder.rotation(deg) / 360 * encoderWheelCircumfrence;

    //Calculate the Current Orientation of the Robot
    currentRotation = (totallDist - totalrDist) / (encoderTrackRadius * 2);
    //Calculate Change in rotation of the robot since last call
    angleChange = currentRotation - prevRotation;

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
    averageOrientation = prevRotation + angleChange / 2;

    //Convert the Offset Coordinates into Polar
    offsetRadius = 0;
    offsetTheta = 0;
    toPolar(localXOffset, localYOffset, offsetRadius, offsetTheta);
    offsetTheta -= averageOrientation;
  
    //Convert Offset Coordinates from Polar to Cartesian and Store as global Coordinates
    xCoord += (offsetRadius * (float)cos(offsetTheta));
    yCoord += (offsetRadius * (float)sin(offsetTheta));

    //Store Previous Values
    prevRotation = currentRotation;
    previousLEncoder = lEncoder.rotation(degrees);
    previousREncoder = rEncoder.rotation(degrees);
    previousBackEncoder = backEncoder.rotation(degrees);
  }
  void ResetHeading()
  {
    lEncoder.resetRotation();
    rEncoder.resetRotation();
  }
  void pre_auton(void) {
    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();
    Controller1.ButtonB.pressed(ResetHeading);
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
    CircleWithLine(40,40, 40, toDegrees(currentRotation));
    int firstLine = 5;
    Brain.Screen.setCursor(firstLine, 1);
    Brain.Screen.print("Current Position: ");
    Brain.Screen.print(xCoord);
    Brain.Screen.print(", ");
    Brain.Screen.print(yCoord);
    Brain.Screen.setCursor(firstLine + 1, 1);
    Brain.Screen.print("Previous Encoders: L");
    Brain.Screen.print(previousLEncoder);
    Brain.Screen.print(" R");
    Brain.Screen.print(previousREncoder);
    Brain.Screen.print(" B");
    Brain.Screen.print(previousBackEncoder);
    Brain.Screen.setCursor(firstLine + 2, 1);
    Brain.Screen.print("Previous Rotation: ");
    Brain.Screen.print(prevRotation * (180 / 3.1415926));

    Brain.Screen.setCursor(firstLine + 3, 1);
    Brain.Screen.print("Total Encoders: L");
    Brain.Screen.print(totallDist);
    Brain.Screen.print(" R");
    Brain.Screen.print(totalrDist);
    Brain.Screen.setCursor(firstLine + 4, 1);
    Brain.Screen.print("");
    Brain.Screen.setCursor(firstLine+5,1);
    Brain.Screen.print("Angle Change: ");
    Brain.Screen.print(angleChange * (180 / 3.1415926));
    Brain.Screen.print(" Local Offset: X");
    Brain.Screen.print(localXOffset);
    Brain.Screen.print(" Y");
    Brain.Screen.print(localYOffset);
    Brain.Screen.setCursor(firstLine+6,1);
    Brain.Screen.print("Average Orientation: ");
    Brain.Screen.print(averageOrientation * (180 / 3.1415926));
    Brain.Screen.setCursor(firstLine + 7, 1);
    Brain.Screen.print("Offset Radius: ");
    Brain.Screen.print(offsetRadius);
    Brain.Screen.print(" Offset Theta: ");
    Brain.Screen.print(offsetTheta);
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



    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(8, 1);  
    Brain.Screen.print("L Encoder Total: ");
    Brain.Screen.print(totallDist);
    Brain.Screen.print(" R Encoder Total: ");
    Brain.Screen.print(totalrDist);
    

  */