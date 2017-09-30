#include "WPILib.h"
#include "LinearVictor.h"
#include "RobotModel.h"
//#include "RobotPorts2012.h"
#include "RobotPorts2013.h"

#include <math.h>

RobotModel::RobotModel() {
	
	pini = new Ini("/robot.ini");
	
#ifdef TALON
	driveLeftA = new Talon(LEFT_MOTOR_A_PWM_PORT);
	driveLeftB = new Talon(LEFT_MOTOR_B_PWM_PORT);
	
	driveRightA = new Talon(RIGHT_MOTOR_A_PWM_PORT);
	driveRightB = new Talon(RIGHT_MOTOR_B_PWM_PORT);
#else	

	driveLeftA  = new LinearVictor(LEFT_MOTOR_A_PWM_PORT);
	driveLeftB  = new LinearVictor(LEFT_MOTOR_B_PWM_PORT);
 
	driveRightA = new LinearVictor(RIGHT_MOTOR_A_PWM_PORT);
	driveRightB = new LinearVictor(RIGHT_MOTOR_B_PWM_PORT);
#endif

	flywheelVictorA = new Talon(SHOOTER_MOTOR_A_PWM_PORT);
	flywheelVictorB = new Talon(SHOOTER_MOTOR_B_PWM_PORT);
	
	climbVictorA = new LinearVictor(CLIMB_MOTOR_A_PWM_PORT);
	climbVictorB = new LinearVictor(CLIMB_MOTOR_B_PWM_PORT);
    
	//victor_4 = new Victor(4);
	//victor_9 = new Victor(9);
		
	talon_10 = new Talon(10);
	
	firingSolenoid = new Solenoid (FIRING_PISTON_SOLENOID_CHAN);
	shootingAngleSolenoid = new Solenoid(8);
	shifterSolenoid = new Solenoid(GEAR_SHIFT_SOLENOID_CHAN);
	climbExtendSolenoid = new Solenoid(CLIMB_SOLENOID_A_CHAN);
	dumperExtendSolenoid = new Solenoid(DUMPER_SOLENOID_A_CHAN);
	dumperRetractSolenoid = new Solenoid(DUMPER_SOLENOID_B_CHAN);
	
#ifdef TEST_PNEUMATICS
	testSolenoid_4 = new Solenoid(4);
	testSolenoid_5 = new Solenoid(5);
	testSolenoid_6 = new Solenoid(6);
	testSolenoid_7 = new Solenoid(7);
	testSolenoid_8 = new Solenoid(8);
#endif
	
	leftWheelEncoder = new Encoder(LEFT_WHEEL_ENCODER_A_PWM_PORT, LEFT_WHEEL_ENCODER_B_PWM_PORT, true);
	rightWheelEncoder = new Encoder(RIGHT_WHEEL_ENCODER_A_PWM_PORT, RIGHT_WHEEL_ENCODER_B_PWM_PORT, true);
	shooterEncoder = new Encoder(FLYWHEEL_ENCODER_A_PWM_PORT, FLYWHEEL_ENCODER_B_PWM_PORT, true) ;
	shooterCounter = new Counter(FLYWHEEL_ENCODER_A_PWM_PORT+2);
	
	leftWheelEncoder->Start();
	rightWheelEncoder->Start();
	shooterEncoder->Start();
	shooterCounter->Start();
	
	timer = new Timer();
	timer->Start();
	
	isLowGear = false;
	isLowAngle = true;
	
	dsLCD = DriverStationLCD::GetInstance();
			
	compressor = new Compressor(COMPRESSOR_PRESSURE_SWITCH_CHAN, COMPRESSOR_RELAY_CHAN);	
	
	gyro = new Gyro(GYRO_PORT);
	gyro->Reset();
	gyro->SetSensitivity(.007);	// used to be .0033, last year used .007
	
}

void RobotModel::EnableCompressor()  {
	compressor->Start();
}

void RobotModel::DisableCompressor()  {
	compressor->Stop();
}

bool RobotModel::GetCompressorState()  {
	return compressor->Enabled();
}

void RobotModel::ResetGyro()  {
	gyro->Reset();
}

float RobotModel::GetGyroAngle()  {
	//return gyro->GetAngle();
	return 0.0;
}

//void RobotModel::ExtendHangingPiston(){
	
//}

void RobotModel::ExtendFiringPiston() {
	firingSolenoid->Set(true); // assuming true extends it... this may need to be false
}

void RobotModel::RetractFiringPiston() {
	firingSolenoid->Set(false); //assuming false causes it to retract... this may been to be true
}

void RobotModel::SwitchToHighAngle(){
	shootingAngleSolenoid->Set(true); //assuming true is high angle, may need to switch
	isLowAngle = false;
}

#ifdef TEST_PNEUMATICS
void RobotModel::SetPneumaticsTrue(){
	shootingAngleSolenoid->Set(true);
	firingSolenoid->Set(true);
	//shifterSolenoid->Set(true);
	testSolenoid_4->Set(true);
	testSolenoid_5->Set(true);
	testSolenoid_6->Set(true);
	testSolenoid_7->Set(false);
	testSolenoid_8->Set(false);
	
	allPneumaticsTrue = true;
	//printf("Setting all pneumatics to true\n");
}

void RobotModel::SetPneumaticsFalse(){
	shootingAngleSolenoid->Set(false);
	firingSolenoid->Set(false);
	//shifterSolenoid->Set(false);
	testSolenoid_4->Set(false);
	testSolenoid_5->Set(false);
	testSolenoid_6->Set(false);
	testSolenoid_7->Set(true);
	testSolenoid_8->Set(true);
	
	allPneumaticsTrue = false;
	//printf("Setting all pneumatics to false\n");
}

void RobotModel::SwitchAllPneumatics(){
	if (allPneumaticsTrue){
		SetPneumaticsFalse();
	}
	else {
		SetPneumaticsTrue();
	}
}
#endif

void RobotModel::SwitchToLowAngle(){
	shootingAngleSolenoid->Set(false); //assuming false is low angle, may need to switch
	isLowAngle = true;
}

void RobotModel::SwitchAngle(){
	if (isLowAngle){
		SwitchToHighAngle();
	}
	else {
		SwitchToLowAngle();
	}
}

bool RobotModel::IsLowGear()  {
	return isLowGear;
}

void RobotModel::ShiftToLowGear() {
	shifterSolenoid->Set(false);
	isLowGear = true;
	// printf("Shifting to low gear: Solenoid %d is now TRUE\n", GEAR_SHIFT_SOLENOID_CHAN);
}

void RobotModel::ShiftToHighGear() {
	shifterSolenoid->Set(true);
	isLowGear = false;
	// printf("Shifting to high gear: Solenoid %d is now FALSE\n", GEAR_SHIFT_SOLENOID_CHAN);
}


void RobotModel::ShiftGear()  {
	if (isLowGear) {
		ShiftToHighGear();
	} else {
		ShiftToLowGear();
	}
}

void RobotModel::ExtendClimb() {
	climbExtendSolenoid->Set(true);
//	climbRetractSolenoid->Set(false); // this is only there if it is a double solenoid... currently using a single 
	climbIsExtended = true;
}

void RobotModel::RetractClimb() {
//	climbRetractSolenoid->Set(true); // also only there if using double solenoid
	climbExtendSolenoid->Set(false);
	climbIsExtended = false;
}

void RobotModel::ToggleClimb(){
	if (climbIsExtended){
		RetractClimb();
	}
	else {
		ExtendClimb();
	}
}

void RobotModel::ExtendDumper(){
	dumperExtendSolenoid->Set(true);
	dumperRetractSolenoid->Set(false);
	dumperIsExtended = true;
}

void RobotModel::RetractDumper(){
	dumperExtendSolenoid->Set(false);
	dumperRetractSolenoid->Set(true);
	dumperIsExtended = false;
}

void RobotModel::ToggleDumper(){
	if (dumperIsExtended){
		RetractDumper();
	}
	else
		ExtendDumper();
}

double RobotModel::GetWheelEncoderDistance(Wheels w) {
	switch (w) {
		case kLeftWheel:
			return ( leftWheelEncoder->Get() / 275.0 ) ;
		case kRightWheel:
			return ( -(rightWheelEncoder->Get()) / 275.0 );  //was 142, need 275 for 256 count encoders
		default: return 0.0;
	}
}

double RobotModel::GetWheelEncoderValue(Wheels w) {
	switch (w) {
	case kLeftWheel:
		return leftWheelEncoder->Get();
	case kRightWheel:
		return -(rightWheelEncoder->Get());
	default: return 0.0;
	}
}

double RobotModel::GetRightWheelEncoderValue()  {
	return(-rightWheelEncoder->Get());
}

double RobotModel::GetLeftWheelEncoderValue()  {
	return(leftWheelEncoder->Get());
}

void RobotModel::ResetWheelEncoder(Wheels w/* = kBothWheels*/) {
	switch (w) {
	case kLeftWheel:
		leftWheelEncoder->Reset();
		break;
	case kRightWheel:
		rightWheelEncoder->Reset();
		break;
	case kBothWheels:
		leftWheelEncoder->Reset();
		rightWheelEncoder->Reset();
		break;
	default: break;
	}
}

// Note: Pass this function positive values to move the robot forward,
// negative values to move the robot backwards. The function handles
// reversing the right side.

void RobotModel::SetWheelSpeed(double speed, Wheels w) {
		switch (w) {
		case kLeftWheel:
			driveLeftA->Set(-speed);
			driveLeftB->Set(-speed);
			break;
		case kRightWheel:
			driveRightA->Set(speed);
			driveRightB->Set(speed);

 			break;
		case kBothWheels:
			driveLeftA-> Set(-speed);
			driveLeftB-> Set(-speed);
			driveRightA->Set(speed);
			driveRightB->Set(speed);
			break;
		default: break;
		}
}

void RobotModel::SetFlywheelSpeed(double speed) {
	speed *= -1.0; // invert for proper direction;
	flywheelVictorA->Set(speed); 
	flywheelVictorB->Set(speed);
	// printf("Setting FlyWheel speed to: %f\n", speed);
	if (fabs(speed) > 0){
		isFlywheelOn = true;
	}
	else {
		isFlywheelOn = false;
	}
}

void RobotModel::SetClimberSpeed(double speed) {
	climbVictorA->Set(speed);
	climbVictorB->Set(speed);
}

double RobotModel::GetCurrentTimeInSeconds() {
	return timer->Get();
}

RobotModel::~RobotModel()
{
}
