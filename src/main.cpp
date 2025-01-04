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

motor intake = motor(PORT10, ratio6_1, true);  
motor hook = motor(PORT3, ratio6_1, true);
motor LF = motor(PORT5, ratio6_1, true);
motor LB = motor(PORT2, ratio6_1, true);
motor RF = motor(PORT19, ratio6_1, false);
motor RB = motor(PORT21, ratio6_1, false);
inertial Gyro = inertial (PORT7);


controller Controller1; 

digital_out clamp = digital_out (Brain.ThreeWirePort.H ); 
digital_out sweep= digital_out (Brain.ThreeWirePort.A);

int AutonSelected = 0;
int AutonMin = 0;
int AutonMax = 4;
bool Clamp_count;
bool Sweep_count;

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

// Gyro Codes 
void gyroPrint()
{
	float heading = Gyro.rotation(deg);
	Brain.Screen.printAt(1, 60, "heading  =  %.2f. degrees", heading);
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

void drawGUI() {
	// Draws 2 buttons to be used for selecting auto
	Brain.Screen.clearScreen();
	Brain.Screen.printAt(1, 40, "Select Auton then Press Go");
	Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonSelected);
	Brain.Screen.setFillColor(red);
	Brain.Screen.drawRectangle(20, 50, 100, 100);
	Brain.Screen.drawCircle(300, 75, 25);
	Brain.Screen.printAt(25, 75, "Select");
	Brain.Screen.setFillColor(green);
	Brain.Screen.drawRectangle(170, 50, 100, 100);
	Brain.Screen.printAt(175, 75, "GO");
	Brain.Screen.setFillColor(black);
}

void selectAuton() {

		bool selectingAuton = true;
		
		int x = Brain.Screen.xPosition(); // get the x position of last touch of the screen
		int y = Brain.Screen.yPosition(); // get the y position of last touch of the screen
		

		// check to see if buttons were pressed
		if (x >= 20 && x <= 120 && y >= 50 && y <= 150){ // select button pressed
				AutonSelected++;
				if (AutonSelected > AutonMax){
						AutonSelected = AutonMin; // rollover
				}
				Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonSelected);
		}
		
		
		if (x >= 170 && x <= 270 && y >= 50 && y <= 150) {
				selectingAuton = false; // GO button pressed
				Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           ", AutonSelected);
		}
		
		if (!selectingAuton) {
				Brain.Screen.setFillColor(green);
				Brain.Screen.drawCircle(300, 75, 25);
		} else {
				Brain.Screen.setFillColor(red);
				Brain.Screen.drawCircle(300, 75, 25);
		}
		
		wait(10, msec); // slow it down
		Brain.Screen.setFillColor(black);
}

/*---------------------------------------------------------------------------*/


void pre_auton(void) {

  drawGUI();
	Brain.Screen.pressed(selectAuton);

	while (true){
	if (AutonSelected == 0){ 
		Brain.Screen.printAt(10, 10, "BLUE Negative"); 
	
	}
	else if (AutonSelected== 1 ){
		Brain.Screen.printAt(10,10,"BlUE Positive");

	
	}
	else if (AutonSelected== 2 ){
		Brain.Screen.printAt(10,10,"RED Positive");


	}
	else if (AutonSelected== 3 ){
		Brain.Screen.printAt(10,10,"RED Negative");


	}
	else if (AutonSelected==4){
		Brain.Screen.printAt(10,10,"SKILLS AUTON");
	}

 while (Gyro.isCalibrating()){ 
  wait(100, msec);
 }
}
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
	wait(1000, msec);
	// inchDriveP(-26.5);
	// gyroTurn(38);
	// inchDriveP(12.5);
	// clamp.set(true);
	// wait(500,msec);
	// hook.spin(reverse,90, pct );
	// gyroTurn(128);
	// inchDriveP(19);

	switch (AutonSelected) {
			case 0:
			// code 0  blue - corner
				wait(1000, msec);
				inchDriveP(-26.5);
				gyroTurn(-38); 
				inchDriveP(-12.5);
				clamp.set(true);  //clamped on mobile goal
				wait(500,msec);
				hook.spin(reverse,90, pct ); //scored preload
				gyroTurn(128);
				inchDriveP(19);
				
				//go forward to touch the ladder
				break;
			
			case 1:
				// code 1  blue + corner
				wait(1000, msec);
				inchDriveP(-26.5);
				gyroTurn(38);
				inchDriveP(-12.5);
				clamp.set(true);
				wait(500,msec);
				hook.spin(reverse,90, pct );
				gyroTurn(-128);
				inchDriveP(19);
					break; 
			case 2:
				//code 2 red + corner
				wait(1000, msec);
				inchDriveP(-26.5);
				gyroTurn(-38); 
				inchDriveP(-12.5);
				clamp.set(true);  //clamped on mobile goal
				wait(500,msec);
				hook.spin(reverse,90, pct ); //scored preload
				gyroTurn(128);
				inchDriveP(19);
				
				//go forward to touch the ladder
				break;
		
					
			case 3:
				//code 3 red- corner
				wait(1000, msec);
				inchDriveP(-26.5);
				gyroTurn(38);
				inchDriveP(-12.5);
				clamp.set(true);
				wait(500,msec);
				hook.spin(reverse,90, pct );
				gyroTurn(-128);
				inchDriveP(19);
				break;
			case 4:
				//SKILLS AUTON
				wait(1000, msec);
				inchDriveP(-10.5);
				clamp.set(true); //grabbed clamp
				intake.spin(reverse, 100, pct); 
				hook.spin(reverse,93, pct ); //start intake and score preload
				gyroTurn(-45);
				inchDriveP(14); //score one ring
				gyroTurn(-160);
				inchDriveP(-22); //turn around to put goal in corner
				wait(3000,msec);
				clamp.set(false);
				inchDriveP(9);   // these are meant to
				inchDriveP(-9);  // shake the mobile goal off
				inchDriveP(25);
				gyroTurn(150);
				inchDriveP(-49);// drives to other goal
				clamp.set(true); //grabbed clamp
				gyroTurn(155);
				inchDriveP(30);
				gyroTurn(180);
				inchDriveP(-18);
				clamp.set(false);
				inchDriveP(9);   // these are meant to
				inchDriveP(-9);  // shake the mobile goal off
				inchDriveP(12);



	

				break;

			}

// ..........................................................................
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

//comment 
void usercontrol(void) {
  // User control code here, inside the loop
  bool Clamp_count=false;
  bool Sweep_count=false;

  Brain.Screen.clearScreen();
  while (1) {
    
    Display(); 
	
    int rspeed = Controller1.Axis2.position(pct); 
    int lspeed = Controller1.Axis3.position(pct);
    
    driveRobot(rspeed, lspeed, 10);


    if (Controller1.ButtonL1.pressing()){
		if(!(Clamp_count)){
			Clamp_count=true;
		}else if(Clamp_count){
			Clamp_count=false;
		}while(Controller1.ButtonL1.pressing()){
			wait(1,msec);
		}
	} 
	if (Clamp_count){
		clamp.set(true);
	}else if(!(Clamp_count)){
		clamp.set(false);
	}



	
	
	if (Controller1.ButtonL2.pressing()){
		if(!(Sweep_count)){
			Sweep_count=true;
		}else if(Sweep_count){
			Sweep_count=false;
		}while(Controller1.ButtonL2.pressing()){
			wait(1,msec);
		}
	} 

	if (Sweep_count){
		sweep.set(true);
	}else if(!(Sweep_count)){
		sweep.set(false);
	}
      

    if (Controller1.ButtonR1.pressing()){ 
      intake.spin(fwd, 70, pct); 
      hook.spin(fwd,40, pct ); 
    }
    
    else if (Controller1.ButtonR2.pressing()){ 
    intake.spin(reverse, 70, pct); 
	hook.spin(reverse,70, pct ); }

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
