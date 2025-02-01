/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      12/27/2024, 3:45:56 PM                                    */
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
inertial Gyro = inertial (PORT7);


controller Controller1; 

digital_out clamp = digital_out (Brain.ThreeWirePort.H ); 


// define your global instances of motors and other devices here

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
void gyroTurn(float target)
{
		float heading=0.0; //initialize a variable for heading
		float accuracy=2.0; //how accurate to make the turn in degrees
		float error=target-heading;
		float kp=.2;//3//2.55
		float speed=kp*error;
		Gyro.setRotation(0.0, degrees);  //reset Gyro to zero degrees
		
		while(fabs(error)>accuracy)
		{
			speed=kp*error;
			driveRobot(-speed, speed, 10); //turn right at speed
			heading=Gyro.rotation();  //measure the heading of the robot
			error=target-heading;  //calculate error
      Brain.Screen.printAt(100,100, "%f", heading);
		}
			driveBrake();  //stope the drive
}

void inchDriveP(float target){
  float x=0;
  float error=target;
  float kp=3;
  float speed =kp*error;
  float accuracy=2.8;
  float Dia = 3.25; //inches
  float Gr = 0.6; //(36Teeth / 60Teeth)
LF.setPosition(0.0, rev);

while(fabs(error)>accuracy){
driveRobot(speed,speed,10);
x=LF.position(rev)*M_PI*Dia*Gr; //pie = 3.14   Diameter=3.25   Gr=GearRatio=0.6
error=target-x;
speed=kp*error;
}

driveBrake();
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//comment 2
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
  //red positive corner
  //angled toward mobile goal
  wait(1000, msec);
	inchDriveP(-3.5);
	clamp.set(true); //grabbed clamp
  intake.spin(reverse, 90, pct); 
  hook.spin(reverse,90, pct ); //start intake and score preload
  gyroTurn(-45);
  inchDriveP(8); //score one ring
  gyroTurn(-160);
  inchDriveP(-18); //turn around to put goal in corner
  clamp.set(false);
  inchDriveP(5);   // these are meant to
  inchDriveP(-5);  // shake the mobile goal off
  inchDriveP(24);
  gyroTurn(160);
  inchDriveP(-37);// drives to other goal




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
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

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
