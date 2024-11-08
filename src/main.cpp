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
motor LF = motor(PORT5, ratio6_1, true);
motor LB = motor(PORT2, ratio6_1, true);
motor RF = motor(PORT19, ratio6_1, false);
motor RB = motor(PORT21, ratio6_1, false);

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

void driveBrake() { 
  LF.stop(brake); 
  LB.stop(brake); 
  RF.stop(brake); 
  RB.stop(brake); 
}



double YOFFSET = 20; //offset for the display
//Writes a line for the diagnostics of a motor on the Brain
void MotorDisplay(double y, double curr, double temp)
{
	Brain.Screen.setFillColor(transparent);
	Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
	
	if (curr < 1){
		Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	} else if(curr >= 1 && curr  <= 2.5) {
		Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	} else {
		Brain.Screen.setFillColor(red);
		Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	}

	Brain.Screen.setFillColor(transparent);
	Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);
	
	if (temp < 45){
		Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
	} else if(temp <= 50 && temp  >= 45){
	
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
		
	} else {
		Brain.Screen.setFillColor(red);
		Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
    
		
	}
}


//Displays information on the brain
void Display()
{
	double leftFrontCurr = LF.current(amp);
	double leftFrontTemp = LF.temperature(celsius);
	double leftBackCurr = LB.current(amp);
	double leftBackTemp = LB.temperature(celsius);
	double rightFrontCurr = RF.current(amp);
	double rightFrontTemp = RF.temperature(celsius);
	double rightBackCurr = RB.current(amp);
	double rightBackTemp = RB.temperature(celsius);
  double intakeTemp = intake.temperature(celsius); 
  double intakeCurr = intake.current(amp); 
  double hookTemp = hook.temperature(celsius); 
  double hookCurr = hook.current(amp); 
  
	if (LF.installed()){
		MotorDisplay(1, leftFrontCurr, leftFrontTemp);
    Brain.Screen.setFillColor(transparent);
		Brain.Screen.printAt(300, YOFFSET + 1, "LeftFront");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 1, "LeftFront Problem");
	}
	
	
	if (LB.installed()){
    
		MotorDisplay(31, leftBackCurr, leftBackTemp);
    Brain.Screen.setFillColor(transparent);
		Brain.Screen.printAt(300, YOFFSET + 31, "LeftBack");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 31, "LeftBack Problem");
	}


	if (RF.installed()) {
   
		MotorDisplay(61, rightFrontCurr, rightFrontTemp);
     Brain.Screen.setFillColor(transparent);
		Brain.Screen.printAt(300, YOFFSET + 61, "RightFront");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 61, "RightFront Problem");
	}
	
	
	if (RB.installed()) {
    
		MotorDisplay(91, rightBackCurr, rightBackTemp);
    Brain.Screen.setFillColor(transparent);
		Brain.Screen.printAt(300, YOFFSET + 91, "RightBack");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 91, "RightBack Problem");
	}
  if (intake.installed()) {
  
		MotorDisplay(121, intakeCurr, intakeTemp);
    Brain.Screen.setFillColor(transparent);
		Brain.Screen.printAt(300, YOFFSET + 121, "intake");
	} else {
		Brain.Screen.printAt(5, YOFFSET +121, "Intake Problem");
	}
  if (hook.installed()) {
    
		MotorDisplay(151, hookCurr, hookTemp);
    Brain.Screen.setFillColor(transparent);
		Brain.Screen.printAt(300, YOFFSET + 151, "hook");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 151, "Hook Problem");
	}
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
  // driveRobot(20, 20, 2000); 
  // driveBrake(); 

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
    
    Display(); 

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