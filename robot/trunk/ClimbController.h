
#ifndef CLIMBCONTROLLER_H_
#define CLIMBCONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"

class ClimbController
{
public: 
	ClimbController(RobotModel*, RemoteController*);
	void Update(double currTimeSec, double deltaTimeSec);
	virtual ~ClimbController();

	
private: 
	RobotModel* robot;
	RemoteController* humanControl;
	
	bool climbUpDesired;
	
	bool climbDownDesired;
	
	bool climbPistonExtendedDesired;
	
	bool dumperDesired;
	
};

#endif /*CLIMBCONTROLLER_H_*/
