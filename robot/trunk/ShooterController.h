#ifndef SHOOTERCONTROLLER_H_
#define SHOOTERCONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"

#define IP_FILTER_SIZE 5
#define OP_FILTER_SIZE 3

class ShooterController
{
public:
	ShooterController(RobotModel*, RemoteController*);
	void Update(double currTimeSec, double deltaTimeSec);
	virtual ~ShooterController();
	void SetShooterSpeed(double speed);
	double GetFlywheelSpeed();
	double GetFlywheelCounterRate();
	double FlywheelPID(double m_DesiredFlywheelSpeed, double m_CurrentFlywheelSpeed, double *diffFlywheelError); // Speed in encoder ticks per second
	void RequestFlywheel(bool on);
	void RequestFiringPiston();
	bool IsAutonomousFiringPistonDone() {return m_bAutonomousFiringPistonDone; };
	void RequestAutonomousShooterAngleChange(bool lowAngle);
	bool IsAutonomousAngleChangeDone() { return !m_bAutonomousShooterAngleChangeDesired; } ;
	void Reset();
	double InputFilter(double value);
	double OutputFilter(double value);
	void ResetFilters();
	
	void RefreshIni();
	
	enum ShooterState {
			//states
			kReset, kInitialize, kIdle, kExtendFiringPiston, kWaitForFiringPiston, kRetractFiringPiston, kWaitBetweenRapidFire
			
		};
	
private:
	RobotModel *robot;
	RemoteController *humanControl;
	bool m_bRequestShooter;
	bool m_bFlywheelDesiredOn;
	bool m_bHumanRequestsFlywheelOff;
	bool m_bAutonomousFlywheelDesired;
	bool m_bTestShooterDesired;
	
	bool m_bShooterSwitchDesired;
	bool m_bFiringPistonDesired;
	double m_initialFiringTime, m_initialRapidTime;
	
	bool m_bAutonomousFiringPistonDesired;
	bool m_bAutonomousFiringPistonDone;
	
	bool m_bShooterAngleChangeDesired;
	bool m_bRapidFireDesired;
	
	bool m_bAutonomousShooterAngleChangeDesired;
	bool m_bAutonomousLowAngleDesired;
	
	double m_FlywheelSpeedAdjustMultiplier;
	double m_FlyWheelSpeed;
	
	double m_CurrentFlywheelSpeed;
	double m_GivenFlywheelSpeed;
	double m_DiffFlywheelSpeed;
	
	double m_FlywheelMotorVal, m_Flywheel_sum_error;
	double m_DesiredFlywheelSpeed;
	
	double m_FlywheelPID_P, m_FlywheelPID_I, m_FlywheelPID_D;
		
	double m_curr_time, m_last_time, m_last_value ;
	int rapidCount;
	
	bool mustReset;
	
	float m_ifilter[IP_FILTER_SIZE], m_ofilter[OP_FILTER_SIZE];
	int m_ifilterindex, m_ofilterindex ;

	
#ifdef TEST_PNEUMATICS
	bool m_bTestPneumaticsDesired;
#endif
	
	uint32_t m_stateVal;
	uint32_t nextState;
};

#endif /*SHOOTERCONTROLLER_H_*/

