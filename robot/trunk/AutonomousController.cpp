#include "AutonomousController.h"

AutonomousController::AutonomousController(RobotModel *myRobot,
		RemoteController *myControlBoard,
		DriveController *myDriveController,
		ShooterController *myShooterController) : 
		robot(myRobot),
		controlBoard(myControlBoard),
		driveController(myDriveController),
		shooterController(myShooterController)
{ }

void AutonomousController::CreateQueue() {

	sequenceNumber = 0;
	doneWithSequence = false;
	
	printf("Creating Queue\n");
	
	//double pauseBetween = robot->pini->getf("AUTONOMOUS", "PauseBetween", 2.01);
	//double pauseBetween = 0.1;

	initialWait = (robot->pini)->getf("AUTONOMOUS", "initialWaitTime", 1.01);
	//printf("initalWait time: %f \n", initialWait);
	
	
	
	autoMode = kSimpleAuto; 

	switch (autoMode) {
	case (kTestMode):
		printf("In kTestMode\n");
		//commandSequence.push_back(new PivotCommand(45, 0.8, driveController));
		commandSequence.push_back(new FlywheelOnCommand(true, shooterController));
		for (int i=0; i < 3; i++) {
			commandSequence.push_back(new WaitingCommand(2.0));
			commandSequence.push_back(new FiringPistonCommand(shooterController));
		}
		commandSequence.push_back(new WaitingCommand(2.0));
		commandSequence.push_back(new FlywheelOnCommand(false, shooterController));

	break;
	default: 
	case (kSimpleAuto):
		commandSequence.push_back(new WaitingCommand(initialWait));
		commandSequence.push_back(new FlywheelOnCommand(true, shooterController));
		commandSequence.push_back(new WaitingCommand(3.0));
			for (int i=0; i < 3; i++) {
				commandSequence.push_back(new FiringPistonCommand(shooterController));
				commandSequence.push_back(new WaitingCommand(1.0));
			}
			commandSequence.push_back(new WaitingCommand(2.0));
			commandSequence.push_back(new FlywheelOnCommand(false, shooterController));
	break;
	}
}

void AutonomousController::StartAutonomous() {
	printf("Starting Autonomous\n");
	CreateQueue();
	sequenceNumber = 0;
	doneWithSequence = false;
	if (commandSequence.size() > 0) {
		commandSequence.at(sequenceNumber)->Start();
		printf("Just started command %d\n", sequenceNumber);
	}
	else {
		doneWithSequence = true;
	}
}

void AutonomousController::Update(double currTimeSec, double deltaTimeSec) {
	if (!doneWithSequence) {
		if (commandSequence.at(sequenceNumber)->IsDone()) {
			sequenceNumber++;
			if (sequenceNumber >= commandSequence.size()) {
				doneWithSequence = true;
				printf("Done with autonomous.\n");
			} else {
				commandSequence.at(sequenceNumber)->Start();
				printf("Just started command %d at %f\n", sequenceNumber, currTimeSec);
			}
		}
	}
}



AutonomousController::~AutonomousController()
{
}
