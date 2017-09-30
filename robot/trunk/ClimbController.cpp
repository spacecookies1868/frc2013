#include "ClimbController.h"
#include <math.h>
#include "Debugging.h"

ClimbController::ClimbController(RobotModel *myRobot, RemoteController *myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;
	
	climbUpDesired = false;
	
	climbDownDesired = false;
	
	dumperDesired = false;

}

void ClimbController::Update(double currTimeSec, double deltaTimeSec){
	climbUpDesired = humanControl->ClimbUpDesired();
	climbDownDesired = humanControl->ClimbDownDesired();
	climbPistonExtendedDesired = humanControl->ClimbPistonExtendedDesired();
	dumperDesired = humanControl->DumperDesired();
	
	if (dumperDesired) {
		robot->ToggleDumper();
	}  
	
	if (climbUpDesired) {
		robot->SetClimberSpeed(1.0);
	}
	else if (climbDownDesired) {
		robot->SetClimberSpeed(-1.0);
		}
	
		else {
			robot->SetClimberSpeed(0.0);
		}
	
	if (climbPistonExtendedDesired){
		robot->ExtendClimb();
	}
	else {
		robot->RetractClimb();
	}
}


ClimbController::~ClimbController()
{
}
