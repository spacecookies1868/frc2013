
#include "WPILib.h"
#include "LinearVictor.h"
#include "ini.h"
#include "RemoteControl.h"

#ifndef ROBOTMODEL_H_
#define ROBOTMODEL_H_

//#define TALON 1

class RobotModel
{
public:
	
	enum Wheels { kLeftWheel, kRightWheel, kBothWheels };	

	Ini* pini;
	
#ifdef TALON
	Talon* driveLeftA;
	Talon* driveLeftB;

	
	Talon* driveRightA;
	Talon* driveRightB;

#else

	LinearVictor* driveLeftA;
	LinearVictor* driveLeftB;
    
	LinearVictor* driveRightA;
	LinearVictor* driveRightB;
#endif
	
    Talon* flywheelVictorA;
    Talon* flywheelVictorB;
    LinearVictor* climbVictorA;
    LinearVictor* climbVictorB;
    
    //Victor* victor_4;
    //Victor*victor_9;
    
    Talon* talon_10;
        
	Encoder* rightWheelEncoder;
	Encoder* leftWheelEncoder;
	Encoder* shooterEncoder;
	Counter* shooterCounter;

	Compressor* compressor;
		
	DriverStationLCD* dsLCD;
	
	ADXL345_I2C* accel;
	Gyro* gyro;
	Timer* timer;
	
	bool isLowGear;
	bool isFlywheelOn;

	bool isLowAngle;

	bool climbIsExtended;
	bool dumperIsExtended;
	
	RobotModel();

	void ResetGyro();
	float GetGyroAngle();
	
//	void PrintToLCD(int line, char message);
	double GetWheelEncoderValue(RobotModel::Wheels w);
	double GetWheelEncoderDistance(RobotModel::Wheels w);
	double GetRightWheelEncoderValue();
	double GetLeftWheelEncoderValue();	
	void EnableCompressor();
	void DisableCompressor();
	bool GetCompressorState();
	
	void ResetWheelEncoder(RobotModel::Wheels w = RobotModel::kBothWheels);
	
	void SetWheelSpeed(double speed, RobotModel::Wheels w);
	
	void ExtendFiringPiston();
	void RetractFiringPiston();
	
	void ExtendClimb();
	void RetractClimb();
	void ToggleClimb();
	
	void SwitchToHighAngle();
	void SwitchToLowAngle();
	void SwitchAngle();//toggle

	void ExtendDumper();
	void RetractDumper();
	void ToggleDumper();
	
#ifdef TEST_PNEUMATICS 
	void SetPneumaticsTrue();
	void SetPneumaticsFalse();
	void SwitchAllPneumatics();
	bool allPneumaticsTrue;
	bool AllPneumaticsTrue() { return allPneumaticsTrue; };
#endif
	
	void SetFlywheelSpeed(double speed);
	bool IsFlywheelOn() { return isFlywheelOn; };

	void SetClimberSpeed(double speed);

	bool IsLowGear();
	void ShiftToHighGear();
	void ShiftToLowGear();
	void ShiftGear();
	
	double GetCurrentTimeInSeconds();
	
	virtual ~RobotModel();	
	
private:
	
	Solenoid* shifterSolenoid;
	Solenoid* firingSolenoid; 
	Solenoid* shootingAngleSolenoid;
	Solenoid* climbExtendSolenoid;
	Solenoid* dumperExtendSolenoid;
	Solenoid* dumperRetractSolenoid;
	
#ifdef TEST_PNEUMATICS
	Solenoid* testSolenoid_4;
	Solenoid* testSolenoid_5;
	Solenoid* testSolenoid_6;
	Solenoid* testSolenoid_7;
	Solenoid* testSolenoid_8;
#endif
};

#endif /*ROBOTMODEL_H_*/
