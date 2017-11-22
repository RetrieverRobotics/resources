/////////////////////////////////////////////////////////////////////
//
// PID Template for Retriever Robotics
//
/////////////////////////////////////////////////////////////////////


// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

typedef struct PIDStruct{
	bool debug; // true will print out debug info in debugStream
	float target; // target value to aim for
	float previousError;
	float integral;
	float output;
	int 	input;

	float Kp; // P tuning constant
	float Ki; // I tuning constant
	float Kd; // D tuning constant

	float integralLimit; // maximum value the integral term will reach
	float integralActiveZone; // if the error is in this zone teh integral will be active

	int 	loopTime;

}PIDStruct;

// example PID variables
PIDStruct driveLPID;
PIDStruct driveRPID;
PIDStruct armPID;
#define MAX_PID_VARS 3 // update this number to however many PID variables you have

// array storing pointers to the PID variables for ease of access in PID functions
//example
PIDStruct *PIDVars[MAX_PID_VARS] = {&driveLPID, &driveRPID, &armPID};

//---------------------------------------------------------------
// user functions
//---------------------------------------------------------------

void initPIDVars(){
	//set everything to 0 initially
	for(int i; i < MAX_PID_VARS; i++){
		PIDVars[i]->debug = false;
		PIDVars[i]->target = 0;
		PIDVars[i]->previousError = 0;
		PIDVars[i]->integral = 0;
		PIDVars[i]->output = 0;
		PIDVars[i]->input = 0;
		PIDVars[i]->Kp = 0;
		PIDVars[i]->Ki = 0;
		PIDVars[i]->Kd = 0;
		PIDVars[i]->integralLimit = 0;
		PIDVars[i]->integralActiveZone = 0;
		PIDVars[i]->loopTime = 0;
	}

	//---------------------------------------------------------------------------------------
	//
	// These are your tuning constants, add another section for each PID variable you have.
	//
	//---------------------------------------------------------------------------------------
	// Google PID tuning for more information on how to tune

	//examples:
	driveLPID.Kp = 0.36; 		// P
	driveLPID.Ki = 0.00040; // I
	driveLPID.Kd = 8; 			// D
	driveLPID.integralLimit = 50; // max power the integral will wind up to
	driveLPID.integralActiveZone = 127/driveLPID.Kp;
	driveLPID.loopTime = 50; // ms
	driveLPID.debug = false; // set to true to see the output of this PID variable's terms for tuning.

	driveRPID = driveLPID;
	driveRPID.debug = false;

	armPID.Kp = 0.1; 			// P
	armPID.Ki = 0.000060; // I
	armPID.Kd = 7; 				// D
	armPID.integralLimit = 50; // max power the integral will wind up to
	armPID.integralActiveZone = 127/armPID.Kp;
	armPID.loopTime = 50; // ms
	armPID.debug = true; // set to true to see the output of this PID variable's terms for tuning.
}

// The heavy lifter for PID tasks
void updatePIDVar(PIDStruct *PIDVar){
	float proportional = 0;
	float derivative = 0;
	float error = 0;

	error = PIDVar->target - PIDVar->input;

	proportional = PIDVar->Kp * error; // P
	if(fabs(error) <= PIDVar->integralActiveZone){ // Active zone for I
		PIDVar->integral += PIDVar->Ki * error * PIDVar->loopTime; // I
		if(fabs(PIDVar->integral) >= PIDVar->integralLimit) // limit I
			PIDVar->integral = sgn(PIDVar->integral) * PIDVar->integralLimit;
	}
	else
		PIDVar->integral = 0;
	derivative = PIDVar->Kd * (error - PIDVar->previousError) / PIDVar->loopTime; // D

	PIDVar->output = round(proportional + PIDVar->integral + derivative);
	PIDVar->previousError = error;

	if(PIDVar->debug){
		writeDebugStream("%f\t%f\t%f\n", proportional, PIDVar->integral, derivative);
	}

}


// example PID tasks ////////////////////////////////////////////////
//task drivePIDTask(){
//	while(true){
//		driveLPID.input = nMotorEncoder[driveLeft];
//		driveRPID.input = nMotorEncoder[driveRight];

//		updatePIDVar(&driveLPID);
//		updatePIDVar(&driveRPID);

//		motor[driveLeft] = driveLPID.output;
//		motor[driveRight] = driveRPID.output;

//		wait1Msec(driveLPID.loopTime);
//	}
//}

//task armPIDTask(){
//	while(true){
//		armPID.input = SensorValue[armPot];
//		updatePIDVar(&armPID);
//		motor[arm] = armPID.output;

//		wait1Msec(armPID.loopTime);
//	}
//}
////////////////////////////////////////////////////////////////////



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
	clearDebugStream();
	initPIDVars();

  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

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

task autonomous()
{
	//example
	//startTask(armPID);
	//armPID.target = 2000; // value in potentiometer ticks to go to

  // ..........................................................................
  // Insert user code here.
  // ..........................................................................

  // Remove this function call once you have "real" code.
  AutonomousCodePlaceholderForTesting();
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

task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // Remove this function call once you have "real" code.
    UserControlCodePlaceholderForTesting();
  }
}
