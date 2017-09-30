#ifndef AUTONOMOUSCONTROLLER_H_
#define AUTONOMOUSCONTROLLER_H_

#include "ControlBoard.h"
#include "RemoteControl.h"
#include "DriveController.h"
#include "ShooterController.h"
#include "RobotModel.h"
#include <vector>
#include <string>

class AutoCommand {
public:
    virtual void Start() = 0;
    virtual bool IsDone() = 0;
};

enum AutoMode {
	kTestMode, kSimpleAuto
};

class DriveCommand : public AutoCommand {
public:
	DriveCommand(double myDistance, double mySpeed, DriveController *myDriveController, string myMsg = "")
		: distance(myDistance), 
		  speed(mySpeed),
		  driveController(myDriveController) {};
	virtual void Start() { 
		driveController->RequestAutoDrive(distance, speed); 
		//printf("just requested auto drive from auto controller\n");
	};
	virtual bool IsDone() { return driveController->AutoDriveDone(); };
private:
	double distance;
	double speed;
	DriveController *driveController;
	
};

class PivotCommand : public AutoCommand {
public:
	PivotCommand(double myAngle, double mySpeed, DriveController *myDriveController)
		: angle(myAngle), 
		  speed(mySpeed),
		  driveController(myDriveController){};
	virtual void Start() { driveController->RequestAutoPivot(angle, speed); };
	virtual bool IsDone() { return driveController->AutoPivotDone(); };
private:
	double angle;
	double speed;
	DriveController *driveController;
	
};

class WaitingCommand : public AutoCommand {
public:
	WaitingCommand(double myWaitTimeSec) : waitTimeSec(myWaitTimeSec) {
		timer = new Timer();
	};
	virtual void Start() { timer->Start(); };
	virtual bool IsDone() { return (timer->Get() >= waitTimeSec); };
		
private:
		double waitTimeSec;
		Timer *timer;
};

class FlywheelOnCommand : public AutoCommand {
public:
	FlywheelOnCommand(bool setOn, ShooterController *myShooterController) : on(setOn), shooterController(myShooterController) {};
	virtual void Start() { shooterController->RequestFlywheel(on); };
	virtual bool IsDone() { return true; }

private:
	bool on;
	ShooterController *shooterController;
};

class FlywheelSpeedSetCommand : public AutoCommand {
public:
	FlywheelSpeedSetCommand(double atSpeed, ShooterController *myShooterController) : speed(atSpeed), shooterController(myShooterController) {};
	virtual void Start() { shooterController->SetShooterSpeed(speed); };
	virtual bool IsDone() { return true; };
private:
	double speed;
	ShooterController *shooterController;
};

class FiringPistonCommand : public AutoCommand {
public:
	FiringPistonCommand(ShooterController* myShooterController) : shooterController(myShooterController) {};
	virtual void Start() {shooterController->RequestFiringPiston(); };
	virtual bool IsDone() { return shooterController->IsAutonomousFiringPistonDone(); };
private:
	ShooterController *shooterController;
};
#if 0
class ShooterAngleChangeCommand : public AutoCommand {
public:
	ShooterAngleChangeCommand(bool wantLowAngle, ShooterController* myShooterController) : lowAngle(wantLowAngle), shooterController(myShooterController) {};
	virtual void Start() { shooterController->RequestAutonomousShooterAngleChange(lowAngle); };
	virtual bool IsDone() { return shooterController->IsAutonomousAngleChangeDone(); }
private:
	ShooterController *shooterController;
	bool lowAngle;
};
#endif
class AutonomousController
{
public:
	AutonomousController(RobotModel *myRobot,
			RemoteController *myControlBoard,
			DriveController *myDriveController,
			ShooterController *myShooterController);
	void CreateQueue();
	void StartAutonomous();
	virtual ~AutonomousController();
	void Update(double currTimeSec, double deltaTimeSec);

private:
	vector<AutoCommand*> commandSequence;
	RobotModel *robot;
	RemoteController *controlBoard;
	DriveController *driveController;
	ShooterController *shooterController;
	unsigned int autoMode;
	unsigned int sequenceNumber;
	double initialWait;
	bool doneWithSequence;
};

#endif /*AUTONOMOUSCONTROLLER_H_*/
