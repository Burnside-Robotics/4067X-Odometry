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
#include "auton-util.cpp"

using namespace vex;

// A global instance of competition
competition Competition;
controller Controller1;


motor lDrive1(PORT1, ratio18_1);
motor lDrive2(PORT2, ratio18_1);
motor rDrive1(PORT3, ratio18_1);
motor rDrive2(PORT4, ratio18_1);


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

void autonomous(void) {

}


void usercontrol(void) {
  while (1) {

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
