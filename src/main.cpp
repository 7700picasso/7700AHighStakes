/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      11/2/2024, 12:45:57 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

brain Brain; 

motor intake = motor(PORT10, ratio6_1, false); 
motor hook = motor(PORT3, ratio6_1, true);
motor LF = motor(PORT6, ratio6_1, false);
motor LB = motor(PORT2, ratio6_1, false);
motor RF = motor(PORT19, ratio6_1, false);
motor RB = motor(PORT16, ratio6_1, false);

controller Controller1; 

digital_out clamp = digital_out (Brain.ThreeWirePort.H ); 

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
void driveRobot(float rspeed, float lspeed, int wt) { 

    LF.spin(forward, lspeed, pct ); 
    LB.spin(forward, lspeed, pct );
    RF.spin(forward, rspeed, pct);
    RB.spin(forward, rspeed, pct);
    wait(wt, msec); 

}
/*---------------------------------------------------------------------------*/


void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    

    int rspeed = Controller1.Axis2.position(pct); 
    int lspeed = Controller1.Axis3.position(pct);
    
    driveRobot(rspeed, lspeed, 10);


    if (Controller1.ButtonL1.pressing()){ 
      clamp.set(true); 

    }
    if (Controller1.ButtonL2.pressing()){ 
      clamp.set(false); 
    }

    if (Controller1.ButtonR1.pressing()){ 
      intake.spin(fwd, 90, pct); 
      hook.spin(fwd,90, pct ); 
    }
    
      else if (Controller1.ButtonR2.pressing()){ 
      intake.spin(reverse, 90, pct); 
      hook.spin(reverse,90, pct ); }

      else {
        intake.stop();
        hook.stop();
      }



    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
