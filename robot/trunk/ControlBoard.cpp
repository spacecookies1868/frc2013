#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotPorts2012.h"
#include "Debugging.h"

#include <math.h>

#define NEW_CONTROL_BOARD 1

ControlBoard::ControlBoard(RobotModel* myRobot){
	robot = myRobot;

	leftJoy  = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy = new Joystick(RIGHT_JOY_USB_PORT);
	operatorJoy = new Joystick(OPERATOR_JOY_USB_PORT);

	tankdrive = false;
	wheelSpeedSet.left = 0;
	wheelSpeedSet.right = 0;

	flywheelSpeedAdjustVal = 0;

	shootingDesired = false;

	gearShiftDesired = false;
	lowGearDesired = false;
	lowGearDesiredLast = false;

	reverseDriveDesired = false;

	shooterSwitchDesired = false;

	firingPistonDesired = false;

	climbUpDesired = false;
	climbDownDesired = false;
	dumperDesired = false;

#ifdef NEW_CONTROL_BOARD

	climbUpButton = new ButtonReader(operatorJoy, 3);

	climbDownButton = new ButtonReader(operatorJoy, 4);

	shooterButton = new ButtonReader(operatorJoy, 1); //same as shoot fender

	shooterSwitchButton = new ButtonReader(operatorJoy, 6);

	firingPistonButton = new ButtonReader(operatorJoy, 5); //same as ball advance button

	shootingAnglePistonButton = new ButtonReader(operatorJoy, 7);//same as roller reverse

	rapidFireButton = new ButtonReader(operatorJoy, 11); // same as shootPosKeyNearButton 
	
	dumperButton = new ButtonReader(operatorJoy, 8); //find new button?
	
#else

	climbUpButton = new ButtonReader(operatorJoy, 4);

	climbDownButton = new ButtonReader(operatorJoy, 3);

	shooterButton = new ButtonReader(operatorJoy, 6); //same as shoot fender

	shooterSwitchButton = new ButtonReader(operatorJoy, 7);

	firingPistonButton = new ButtonReader(operatorJoy, 8); //same as ball advance button

	shootingAnglePistonButton = new ButtonReader(operatorJoy, 9);//same as roller reverse

	rapidFireButton = new ButtonReader(operatorJoy, 11); // same as shootPosKeyNearButton 
	
#endif
	
	gearShiftButton = new ButtonReader(rightJoy, 3); 
	
	reverseDriveButton = new ButtonReader(rightJoy, 8);

	quickTurnButton = new ButtonReader(rightJoy, 1);
	
	climbPistonButton = new ButtonReader(leftJoy, 3);


#ifdef TEST_SHOOTER
#ifdef NEW_CONTROL_BOARD
	testShooterButton = new ButtonReader(operatorJoy, 2); //same as roller
#else
	testShooterButton = new ButtonReader(operatorJoy, 5);
#endif
	testShooterPower = 0.5;
	testShooterPowerFinal = 0.0;
	testShooterDesired = false;
#endif

#ifdef TEST_PNEUMATICS
	testPneumaticsButton = new ButtonReader(operatorJoy, 1); //far key button
	testPneumaticsDesired = false;
#endif

}

/*
 * takes the Y axis values of the right and left joysticks and sets the motorSpeed for the left and right
 * motors to the corresponding raw joystick value
 */
MotorSpeedSet ControlBoard::CalculateTankDriveMotorValues(double leftJoyY, double rightJoyY) {

	if(reverseDriveDesired) {
		leftJoyY = -leftJoyY;
	}

	// Note: Reversing one side (the left side) is handled in RobotModel::SetWheelSpeed, so for now,
	// we're going to work as if both sides have the same sign.

	wheelSpeedSet.left = -leftJoyY;
	wheelSpeedSet.right = -rightJoyY;
	return wheelSpeedSet;
}

/**
 * Takes the raw inputs from the joysticks (the Y axis on one joystick and the X axis on the other)
 * and calculates the values to be fed to the left and right motors
 * 
 * @param speedJoyY the value from the Y axis of the "throttle" or "speed" Joystick (left joystick)
 * @param dirJoyX the value from the X axis of the "wheel" or "direction" Joystick (right joystick)
 */
MotorSpeedSet ControlBoard::CalculateArcadeDriveMotorValues(double speedJoyY, double dirJoyX) {
	//printf("In Arcade code now \n");
	dirJoyX = -dirJoyX;
	if(reverseDriveDesired) {
		speedJoyY = -speedJoyY;
	}
	//isQuickTurn = (rightJoy->GetButton(Joystick::kTriggerButton));
	bool isHighGear = !(robot->IsLowGear()); 
	if (isHighGear)
	{
		// printf("IN HIGH GEAR\n");
	}

	double wheel = dirJoyX;

	if(fabs(wheel) < 0.1) //If user does not want to steer, stop and cancel acceleration
		wheel = 0.0;


	static double old_wheel = 0.0;
	double neg_inertia = wheel - old_wheel;
	old_wheel = wheel;

	double left_pwm, right_pwm, overPower;
	float sensitivity = 1.7;

	float angular_power;
	float linear_power;

	static int i = 0;
	i++;

#define M_PI 3.1415926535

	// 2012 version - added wheel nonlinearity - to affect steering feel at low speeds
	double wheelNonLinearity;
	if (isHighGear) {
		wheelNonLinearity = 0.7; // used to be csvReader->TURN_NONLIN_HIGH
		// Apply a sin function that's scaled to make it feel better.
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
	} else {
		wheelNonLinearity = 0.4; // used to be csvReader->TURN_NONLIN_LOW
		// Apply a sin function that's scaled to make it feel better.
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
	}

	static double neg_inertia_accumulator = 0.0;
	double neg_inertia_scalar;
	if (isHighGear) {
		// printf("High ");
		neg_inertia_scalar = 20.0; // used to be csvReader->NEG_INTERTIA_HIGH
		sensitivity = 1.22; // used to be csvReader->SENSE_HIGH
	} else {
		// printf("Low ");
		if (wheel * neg_inertia > 0) {
			neg_inertia_scalar = 10.0; // used to be csvReader->NEG_INERTIA_LOW_MORE
		} else {
			if (fabs(wheel) > 0.65) {
				neg_inertia_scalar = 10.0;// used to be csvReader->NEG_INTERTIA_LOW_LESS_EXT
			} else {
				neg_inertia_scalar = 5.0; // used to be csvReader->NEG_INTERTIA_LOW_LESS
			}
		}
		sensitivity = 1.24; // used to be csvReader->SENSE_LOW

		if (fabs(speedJoyY) > 0.1) { // used to be csvReader->SENSE_CUTTOFF
			sensitivity = 1 - (1 - sensitivity) / fabs(speedJoyY);
		}
	}
	double neg_inertia_power=neg_inertia * neg_inertia_scalar;
	neg_inertia_accumulator+=neg_inertia_power;
	// printf("neg_inertia_power=%f accumulator=%f\n", neg_inertia_power, neg_inertia_accumulator);

	//printf("wheel: %f neg_inertia_accumulator: %f ", wheel, neg_inertia_accumulator);
	if (isHighGear) {
		wheel = wheel + neg_inertia_accumulator;	
	}
	else { 
		wheel = wheel;
	}
	if(neg_inertia_accumulator > 1)
		neg_inertia_accumulator -= 1;
	else if (neg_inertia_accumulator < -1)
		neg_inertia_accumulator += 1;
	else
		neg_inertia_accumulator = 0;

	//printf("new wheel: %f\n", wheel);
	linear_power = speedJoyY;

	if (isQuickTurn) {
		overPower = 1.0;
		if (isHighGear) {
			sensitivity = 1.0;
		} else {
			sensitivity = 1.0;
		}
		angular_power = wheel;
	} else {
		overPower = 0.0;
		angular_power = fabs(speedJoyY) * wheel * sensitivity;
	}

	right_pwm = left_pwm = linear_power;
	left_pwm += angular_power;
	right_pwm -= angular_power;

	if (left_pwm > 1.0) {
		right_pwm -= overPower*(left_pwm - 1.0);
		left_pwm = 1.0;
	} else if (right_pwm > 1.0) {
		left_pwm -= overPower*(right_pwm - 1.0);
		right_pwm = 1.0;
	} else if (left_pwm < -1.0) {
		right_pwm += overPower*(-1.0 - left_pwm);
		left_pwm = -1.0;
	} else if (right_pwm < -1.0) {
		left_pwm += overPower*(-1.0 - right_pwm);
		right_pwm = -1.0;
	}

	// if (i % 50 == 0) {
	// 	printf("l: %f r: %f t: %f w: %f ax: %f\n", left_pwm, right_pwm, throttle, wheel, axis);
	// }

	// Note: Reversing one side (the left side) is handled in RobotModel::SetWheelSpeed, so for now,
	// we're going to work as if both sides have the same sign.
	wheelSpeedSet.left = left_pwm ;
	wheelSpeedSet.right = right_pwm ;

	return wheelSpeedSet;
}

/**
 * Reads and inteprets human inputs
 */
void ControlBoard::ReadControls() {
	//read shooter joy values here
	//desiredShooterSpeed = operatorJoy->GetRawAxis(3) ; // read z wheel for shooter speed edit: will not use joystick

	if (tankdrive) {
		wheelSpeedSet = 
				CalculateTankDriveMotorValues(rightJoy->GetRawAxis(2), leftJoy->GetRawAxis(2));
		// Left Joystick Y and Right Joystick Y
	} 
	else { // Arcade Drive
		wheelSpeedSet = 
				CalculateArcadeDriveMotorValues(leftJoy->GetRawAxis(2), rightJoy->GetRawAxis(1));
		// Assuming speed controlled by left joystick and direction controlled by right.
		// May be the wrong assumption -- should be made settable.
	}


	if (shooterButton->WasJustPressed()) {
		if (robot->IsFlywheelOn()) {
			shootingDesired = false; 
		}
		else {
			shootingDesired = true;
		}
	}

	if (firingPistonButton->WasJustPressed()) {
		firingPistonDesired = true;
	}
	else {
		firingPistonDesired = false;
	}

	if (rapidFireButton->WasJustPressed()){
		rapidFireDesired = true;
	}
	else {
		rapidFireDesired = false;
	}

	if (shooterSwitchButton->IsDown()){
		shooterSwitchDesired = true;
	}
	else {
		shooterSwitchDesired = false;
	}

	if (shootingAnglePistonButton->WasJustPressed()){
		angleSwitchDesired = true;
	}
	else {
		angleSwitchDesired = false;
	}

	if (climbUpButton->IsDown()) {
		climbUpDesired = true;
	}
	else {
		climbUpDesired = false;
	}

	if (climbDownButton->IsDown()){
		climbDownDesired = true;
	}

	else {
		climbDownDesired = false;
	}

	if (climbUpDesired){
		climbDownDesired = false;
	}

	if (climbDownDesired) {
		climbUpDesired = false;
	}

	if (climbPistonButton->IsDown()){
		climbPistonExtendedDesired = true;
	}
	else {
		climbPistonExtendedDesired = false;
	}
	if (dumperButton->WasJustPressed()){
		dumperDesired = true;
	}
	else {
		dumperDesired = false;
	}
#ifdef TEST_PNEUMATICS
	if (testPneumaticsButton->WasJustPressed()){
		testPneumaticsDesired = true;
	}
	else {
		testPneumaticsDesired = false;
	}
#endif

	lowGearDesiredLast = lowGearDesired;
	lowGearDesired = gearShiftButton->IsDown();
	gearShiftDesired = (lowGearDesiredLast != lowGearDesired);

	if ( reverseDriveButton->WasJustPressed() ) reverseDriveDesired = !reverseDriveDesired;

	isQuickTurn = quickTurnButton->IsDown();

	flywheelSpeedAdjustVal = -(operatorJoy->GetRawAxis(4));
#ifdef TEST_SHOOTER	
	//testShooterPower = operatorJoy->GetRawAxis(3);
	testShooterPowerFinal = testShooterPower + ((operatorJoy->GetRawAxis(3))/5.0);
	robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "FW Test: %d", (int)(testShooterPowerFinal));
	//DO_PERIODIC(20, printf("Flywheel testShooterPowerFinal: %f \n", testShooterPowerFinal));

	if (testShooterButton->WasJustPressed()){
		if (robot->IsFlywheelOn()) {
			testShooterDesired = false;
		}
		else {
			testShooterDesired = true;
		}
	}
#endif
}



ControlBoard::~ControlBoard()
{
}

