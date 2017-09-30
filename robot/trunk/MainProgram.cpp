#include "WPILib.h"
#include "ControlBoard.h"
#include "DriveController.h"
#include "AutonomousController.h"
#include "ShooterController.h"
#include "ClimbController.h"
#include "RobotModel.h"

class MainProgram : public IterativeRobot
{
		
public:
	
	RemoteController *humanControl;
	RobotModel *robot;
	AutonomousController *autoController;
	DriveController *driveController;
    ShooterController *shooterController;
    ClimbController *climbController;
	ADXL345_I2C *accel;
	LiveWindow *lw;
	Ini *ini;
		
	double currTimeSec;
	double lastTimeSec;
	double deltaTimeSec;
		
	MainProgram(void) {
		// Constructor initializes robot and other controllers
		robot = new RobotModel();
		printf("New Robot constru\n");
		ini = robot->pini;
		humanControl = new ControlBoard(robot);
		driveController = new DriveController(robot, humanControl);
		shooterController = new ShooterController(robot, humanControl);
		climbController = new ClimbController(robot, humanControl);
		autoController = new AutonomousController(robot, humanControl, driveController, shooterController);
		
		accel = new ADXL345_I2C(1/*,DataFormat_Range(kRange_2G*/);
		
		lw = LiveWindow::GetInstance();
		
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
				
		SetPeriod(0.01);
	}
	
	void RobotInit(void) {
		lw->AddSensor("Gyro", "Gyro angle", robot->gyro);
		lw->AddSensor("Encoder", "Right encoder val", robot->rightWheelEncoder);
		lw->AddSensor("Encoder", "Left encoder val", robot->leftWheelEncoder);
		lw->AddSensor("Encoder", "Shooter encoder val", robot->shooterEncoder);
		lw->AddSensor("Compressor", "Compressor val", robot->compressor);
//		lw->AddActuator("Accelerometer", "Accelerometer reading", robot->accel);		
		// Actions that are performed once (and only once) when the robot is initialized
		robot->EnableCompressor();
	}
	
	void DisabledInit(void) {
		// Actions that are performed once as soon as the robot becomes disabled
		driveController->Reset();
		delete robot->pini;
			robot->pini = new Ini("/robot.ini");	

	}

	void AutonomousInit(void) {
		// Actions that are performed once as soon as the autonomous period begins
		robot->ResetGyro();
		shooterController->RefreshIni();
		shooterController->Reset();
		driveController->RefreshIni();
		driveController->Reset();
		driveController->Update(0,0);
		humanControl->ReadControls();
		autoController->StartAutonomous();
	}

	void TeleopInit(void) {
		// Actions that are performed once as soon as the teleop period begins	
		driveController->Reset();
		shooterController->Reset();
		driveController->RefreshIni();
		shooterController->RefreshIni();
		
	    robot->ExtendClimb(); 
	}
	
	void DisabledPeriodic(void)  {
		// Actions that are performed periodically while the robot is disabled
		//robot->dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "FW dial %f", (humanControl->GetFlywheelSpeedAdjustMultiplier() - 1));
		//robot->dsLCD->UpdateLCD();
	}

	void AutonomousPeriodic(void) {
		// Actions that are performed periodically during autonomous
		// Actions that are performed periodically during autonomous
		lastTimeSec = currTimeSec;
		currTimeSec = robot->timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;

		humanControl->ReadControls();
		autoController->Update(currTimeSec, deltaTimeSec);
		driveController->Update(currTimeSec, deltaTimeSec);
		shooterController->Update(currTimeSec, deltaTimeSec);
					
		//testing the beam sensor
		//DO_PERIODIC(500, printf("The status of the beam sensor is %d\n", robot->popUpBeamSensor->Get()));
	}

	void TeleopPeriodic(void) {
		// Actions that are performed periodically during teleop
		// Actions that are performed periodically during teleop
				
				lastTimeSec = currTimeSec;
				currTimeSec = robot->timer->Get();
				deltaTimeSec = currTimeSec - lastTimeSec;
				
				//robot->dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "FW dial %f        ", (humanControl->GetFlywheelSpeedAdjustMultiplier() - 1));
				//robot->PrintDebugInfoToLCD();
				//robot->dsLCD->UpdateLCD();
				
				humanControl->ReadControls();
				driveController->Update(currTimeSec, deltaTimeSec);
				shooterController->Update(currTimeSec, deltaTimeSec);
				climbController->Update(currTimeSec, deltaTimeSec);

				//testing the beam sensor
				//DO_PERIODIC(500, printf("The status of the beam sensor is %d\n", robot->popUpBeamSensor->Get()));
//				robot->dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "test");
//
				robot->dsLCD->UpdateLCD();
	}
	void TestPeriodic(){
		robot->dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Gyro angle %f", 
				robot->gyro->GetAngle());
		robot->dsLCD->UpdateLCD();
		lw->Run();
	}

	/*void DisabledContinuous(void) {
		// Actions that are performed repeatedly as fast as possible while disabled
	}

	void AutonomousContinuous(void)	{
		// Actions that are performed repeatedly as fast as possible during autonomous
	}

	void TeleopContinuous(void) {
		// Actions that are performed repeatedly as fast as possible during teleop
	} */
			
};

START_ROBOT_CLASS(MainProgram);

