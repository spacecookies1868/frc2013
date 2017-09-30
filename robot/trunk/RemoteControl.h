#ifndef REMOTECONTROLLER_H_
#define REMOTECONTROLLER_H_

typedef struct {
	double left;
	double right;
} MotorSpeedSet;

//#define TEST_SHOOTER 1
//#define CALIBRATE_FLYWHEEL 1
//#define TEST_PNEUMATICS 1

class RemoteController {
	
public:
	
    //What the User wants
	/**
	 * Returns the desired speed of the left motor.
	 *
	 * @return the desired speed of the left motor
	 */
	//virtual MotorSpeedSet GetDesiredMotorSpeedSet() = 0;
	virtual double GetLeftWheelDesiredSpeed() = 0;
    
    /**
	 * Returns the desired speed of the right motor.
	 *
	 * @return the desired speed of the right motor
	 */
	virtual double GetRightWheelDesiredSpeed() = 0;
		
	virtual bool ShootingDesired() = 0;
	
	virtual double GetFlywheelSpeedAdjustMultiplier() = 0;

	virtual bool FiringPistonDesired() = 0; 
	
	virtual bool AngleSwitchDesired() = 0;
	
	virtual bool ShooterSwitchDesired() = 0;

	
	virtual bool DumperDesired()= 0;
	
	//virtual int GetDesiredGear() = 0;
	
	virtual bool GearShiftDesired() = 0;
	
	virtual bool LowGearDesired() = 0;
	
	virtual bool RapidFireDesired() = 0;
	
	virtual bool ClimbUpDesired() = 0;
	
	virtual bool ClimbDownDesired() = 0;
	
	virtual bool ClimbPistonExtendedDesired() = 0;
	

#ifdef CALIBRATE_FLYWHEEL
	virtual bool PositiveFlywheelDesired() = 0;
#endif
	
#ifdef TEST_PNEUMATICS
	virtual bool TestPneumaticsDesired() = 0;
#endif
	
#ifdef TEST_SHOOTER
	virtual double TestShooterPowerDesired() = 0;
	virtual bool TestShooterDesired() = 0;
#endif
	
    /**
	 * Updates the state of the controller
	 */
	virtual void ReadControls() = 0;
	
	virtual ~RemoteController() {}
	
protected:
	MotorSpeedSet wheelSpeedSet;
	
};

#endif
