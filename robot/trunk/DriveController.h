#ifndef DRIVECONTROLLER_H_
#define DRIVECONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"
#include "Debugging.h"



class DriveController{
public:
	DriveController(RobotModel*, RemoteController*);
	void Update(double currTimeSec, double deltaTimeSec);
	void RequestAutoDrive(double distance, double speed);
	void RequestAutoPivot(double speed, double target_angle);
	double DrivePID(double goalDist, double distTraveled, double *diffErr);
	double PivotPID(double goalAngle, double currAngle, double *diffAngleError);
	bool AutoDriveDone();
	bool AutoPivotDone();
	void Reset();
	void RefreshIni();
	virtual ~DriveController();
	
	enum DriveState {
			//states
			kReset, kInitialize, kTeleopDrive, 
			kAutoDriveStart, kAutoDrive, kAutoDriveDone,
			kAutoPivotStart, kAutoPivot, kAutoPivotDone
		
		};

private:
	RobotModel *robot;
	RemoteController *humanControl;
	
	uint32_t m_stateVal;
	uint32_t nextState;
	
		bool autoDriveRequested;
		bool autoDriveDone;
		double autoGoalDist;
		double autoSpeed;
		double leftWheelEncoderDistance;
		double rightWheelEncoderDistance;
		double autoDistTraveled;
		double currSpeedFactor;
		double currSpeed;
		double autoDiffDriveError;
		double autoInitialGyroAngle;
		double DrivePID_P, DrivePID_I, DrivePID_D, DrivePID_Gyro_P, DrivePID_Encoder_P;
		double PivotPID_P, PivotPID_I, PivotPID_D;
		double Drive_sum_error, Pivot_sum_error;
		float currAngle;
		double pivotSpeedFactor;
		bool autoPivotRequested;
		bool autoPivotDone;
		double autoPivotGoal;
		double autoPivotSpeed;
		double adjustedAutoPivotSpeed;
		double autoDiffPivotError;
		bool originallyHighGear;
		double pidExitCountdownStartTime;
		double pidExitDelay;
		bool mustReset;
		double gyroVal;
	
};

#endif /*DRIVECONTROLLER_H_*/
