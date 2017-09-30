#ifndef CONTROLBOARD_H_
#define CONTROLBOARD_H_

#include "WPILib.h"
#include "ButtonReader.h"
#include "RemoteControl.h"
#include "RobotModel.h"

//#define TEST_SHOOTER 1
//#define TEST_PNEUMATICS 1

class ControlBoard : public RemoteController
{
public:
	ControlBoard(RobotModel* myRobot);
	virtual ~ControlBoard();

	virtual double GetLeftWheelDesiredSpeed() { return wheelSpeedSet.left; };

	virtual double GetRightWheelDesiredSpeed(){ return wheelSpeedSet.right; };
	
	virtual bool GearShiftDesired() { return gearShiftDesired; };
	
	virtual bool LowGearDesired() { return lowGearDesired; };
	
	virtual void ReadControls();
		
	virtual bool ShootingDesired() { return shootingDesired; };
	
	virtual bool ShooterSwitchDesired() { return shooterSwitchDesired; };
	
	virtual bool FiringPistonDesired() { return firingPistonDesired; };
	
	virtual bool AngleSwitchDesired() { return angleSwitchDesired; };	
	
	virtual bool RapidFireDesired() { return rapidFireDesired; };
	
	virtual bool ClimbUpDesired() { return climbUpDesired; };
	
	virtual bool ClimbDownDesired() { return climbDownDesired; };
	
	virtual bool ClimbPistonExtendedDesired() { return climbPistonExtendedDesired; };
	
	virtual double GetFlywheelSpeedAdjustMultiplier() {
			return (1.0 + (flywheelSpeedAdjustVal / 5.0));
	};
	
	virtual bool DumperDesired() { return dumperDesired; };
	
	
#ifdef TEST_SHOOTER
	virtual double TestShooterPowerDesired() { return testShooterPower; };
	virtual bool TestShooterDesired() { return testShooterDesired; };
#endif
	
#ifdef TEST_PNEUMATICS
	virtual bool TestPneumaticsDesired(){ return testPneumaticsDesired; };
#endif


private:
	bool tankdrive;
	Joystick* leftJoy;
	Joystick* rightJoy;
	Joystick* operatorJoy;
	// Joystick* ISSJoy;
	RobotModel* robot;
	
	ButtonReader* gearShiftButton;
	ButtonReader* reverseDriveButton;
	
	ButtonReader* climbUpButton;
	ButtonReader* climbDownButton;
	ButtonReader* climbPistonButton;
	ButtonReader* dumperButton;
	
	ButtonReader* quickTurnButton;
	ButtonReader* shooterButton;
	ButtonReader* shooterSwitchButton;
	
	ButtonReader* firingPistonButton;
	ButtonReader* shootingAnglePistonButton;
	
	ButtonReader* rapidFireButton;

#ifdef TEST_SHOOTER
	ButtonReader* testShooterButton;
	double testShooterPower;
	double testShooterPowerFinal;
	bool testShooterDesired;
#endif
	
#ifdef CALIBRATE_FLYWHEEL
	ButtonReader* positiveFlywheelButton;
	bool positiveFlywheelDesired;
#endif
	
#ifdef TEST_PNEUMATICS
	ButtonReader* testPneumaticsButton;
	bool testPneumaticsDesired;
#endif
	
	double desiredLeftSpeed;
	double desiredRightSpeed;

	bool dumperDesired;
	bool isQuickTurn;
	bool gearShiftDesired;
	bool lowGearDesired;
	bool lowGearDesiredLast;
	bool reverseDriveDesired;
	
	bool firingPistonDesired; 
	bool shootingDesired;
	bool shooterSwitchDesired;
	bool angleSwitchDesired;
	bool rapidFireDesired;
	
	bool climbUpDesired;
	bool climbDownDesired;
	bool climbPistonExtendedDesired;
		
	double flywheelSpeedAdjustVal;
	
	virtual MotorSpeedSet CalculateTankDriveMotorValues(double joy1, double joy2);

	virtual MotorSpeedSet CalculateArcadeDriveMotorValues(double joy1, double joy2);
};

#endif /*CONTROLBOARD_H_*/
