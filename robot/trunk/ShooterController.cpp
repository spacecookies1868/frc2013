#include <math.h>
#include "ShooterController.h"
#include "Debugging.h"

ShooterController::ShooterController(RobotModel *myRobot, RemoteController *myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;
	m_stateVal = kInitialize;
	m_FlyWheelSpeed = 0.0 ;
	
	m_bRequestShooter = false;
	m_bFlywheelDesiredOn = false;
	m_bAutonomousFlywheelDesired = false;
	m_bAutonomousFiringPistonDone = true;
	
	
#ifdef TEST_PNEUMATICS
	m_bTestPneumaticsDesired = false;
#endif
	
	m_bFiringPistonDesired = false;

	m_initialFiringTime = 0.0;
	
	m_FlywheelMotorVal = 0.0;
	m_FlywheelSpeedAdjustMultiplier = 0.0;

	mustReset = false;

	rapidCount = 0;
	
	m_DesiredFlywheelSpeed = 0.0;
	
	//printf("In constructor of shooterController\n");
	
#ifdef TEST_SHOOTER
	m_bTestShooterDesired = false;
#endif
	ResetFilters();
	RefreshIni();
}

// take a new value in - filter it and return filtered value
// simple averaging filters of FILTER_SIZE
// can be templated as an exercise ...

void ShooterController::ResetFilters()
{
	for (int i = 0; i < IP_FILTER_SIZE; i++) {
		m_ifilter[i] = 0.0;
	}
	m_ifilterindex = 0;
	for (int i = 0; i < OP_FILTER_SIZE; i++) {
		m_ofilter[i] = 0;
	}
	m_ofilterindex = 0;
	
}

double ShooterController::InputFilter(double newvalue){
	double out = 0.0;
	m_ifilter[m_ifilterindex] = newvalue;
	m_ifilterindex++ ;
	if (m_ifilterindex == IP_FILTER_SIZE) {
		m_ifilterindex = 0;
	}
	for(int i = 0; i < IP_FILTER_SIZE; i++){
		out += m_ifilter[i];
	}
	return (out/(double)IP_FILTER_SIZE);
}

double ShooterController::OutputFilter(double newvalue){
	double out = 0.0;
	m_ofilter[m_ofilterindex] = newvalue;
	m_ofilterindex++ ;
	if (m_ofilterindex == OP_FILTER_SIZE) {
		m_ofilterindex = 0;
	}
	for(int i = 0; i < OP_FILTER_SIZE; i++){
		out += m_ofilter[i];
	}
	return (out/(double)OP_FILTER_SIZE);

}


void ShooterController::Update(double currTimeSec, double deltaTimeSec){
	m_curr_time = currTimeSec ;
		//m_bHumanRequestsFlywheelOff = humanControl->FlywheelDesiredOff();
		m_bRequestShooter = humanControl->ShootingDesired(); 
		m_FlywheelSpeedAdjustMultiplier = humanControl->GetFlywheelSpeedAdjustMultiplier();
		m_bFiringPistonDesired = humanControl->FiringPistonDesired();
		m_bShooterAngleChangeDesired = humanControl->AngleSwitchDesired();
		m_bRapidFireDesired = humanControl->RapidFireDesired();
		m_bShooterSwitchDesired = humanControl->ShooterSwitchDesired();

		
#ifdef TEST_PNEUMATICS
		m_bTestPneumaticsDesired = humanControl->TestPneumaticsDesired();
		if (m_bTestPneumaticsDesired){
			robot->SwitchAllPneumatics();
		}
#endif

#ifdef TEST_SHOOTER
		m_bTestShooterDesired = humanControl->TestShooterDesired();
		//DO_PERIODIC (50, printf("Flywheel Speed: %f\n", (-1.0 * robot->shooterEncoder->GetRate())));
		if (m_bTestShooterDesired){
			robot->SetFlywheelSpeed(humanControl->TestShooterPowerDesired());
			//robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Flywheel Test Power Desired Final: %d", (int)((humanControl->TestShooterPowerDesired())*100));
			//DO_PERIODIC(20, printf("Flywheel desired power: %f Flywheel Speed: %f\n", humanControl->TestShooterPowerDesired(), GetFlywheelSpeed()));
		}
		else {
			robot->SetFlywheelSpeed(0.0);
		}
		
#else

		if(m_bRequestShooter){
			m_bFlywheelDesiredOn = true;
			m_bAutonomousFlywheelDesired = false;
		}
		else
		{
			m_bFlywheelDesiredOn = false;
			//printf("turning off flywheel b/c operator pressed preset button again.\n");
		}
		/*if (m_bHumanRequestsFlywheelOff) {
			m_bFlywheelDesiredOn = false;
			printf("turning off flywheel b/c operator pressed preset button again.\n");
		}*/
		double desired = (m_DesiredFlywheelSpeed * m_FlywheelSpeedAdjustMultiplier);
		m_CurrentFlywheelSpeed = GetFlywheelSpeed();
		double tmp = m_CurrentFlywheelSpeed;

		if(m_bFlywheelDesiredOn || m_bAutonomousFlywheelDesired || m_bShooterSwitchDesired){
			double difError = 0.0;
			double speedFactor = 1.0;
			//double desired = (m_DesiredFlywheelSpeed * m_FlywheelSpeedAdjustMultiplier);
			static int cnt = 0;
			m_FlywheelMotorVal = .98 ;
			//m_CurrentFlywheelSpeed = GetFlywheelSpeed();
			m_CurrentFlywheelSpeed = InputFilter(m_CurrentFlywheelSpeed);

			//printf("About to call FlywheelPID\n");
			speedFactor = FlywheelPID((desired), m_CurrentFlywheelSpeed, &difError);
			m_FlywheelMotorVal = speedFactor ;
	//#if FILTER_SIGNALS
		//	m_FlywheelMotorVal = OutputFilter(m_FlywheelMotorVal);
	//#endif

			DO_PERIODIC(20, printf("** Time: %f err: %f desired: %f motor: %f\n", currTimeSec, (desired - m_CurrentFlywheelSpeed), (desired),  m_FlywheelMotorVal));
			DO_PERIODIC(20, printf("Flywheel current: %d (raw: %d), desired: %d \n", (int)(m_CurrentFlywheelSpeed), (int)(tmp),(int)(m_DesiredFlywheelSpeed * m_FlywheelSpeedAdjustMultiplier)));
			robot->SetFlywheelSpeed(m_FlywheelMotorVal);

			if(cnt++ % 500) {
				// we really shouldnt print every update - that causes performance issues.
				robot->dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "FW c %d d %d        ", (int)(m_CurrentFlywheelSpeed), (int)(m_DesiredFlywheelSpeed * m_FlywheelSpeedAdjustMultiplier) );
				robot->dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "FW counter rate: %d", (int)(GetFlywheelSpeed()));
				robot->dsLCD->UpdateLCD();
			}
		}
		else {
			robot->SetFlywheelSpeed(0.0);
			m_Flywheel_sum_error = 0.0;
			robot->dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "flywheel OFF                ");
			robot->dsLCD->UpdateLCD();
		}
#endif
		if (m_bShooterAngleChangeDesired){
			robot->SwitchAngle();
		}
		
		if (m_bAutonomousShooterAngleChangeDesired) {
			m_bAutonomousShooterAngleChangeDesired = false; 
			if (m_bAutonomousLowAngleDesired) {
				robot->SwitchToLowAngle();
			} else {
				robot->SwitchToHighAngle();
			}
		}
		
		if (mustReset) {
			m_stateVal = kReset;
			mustReset = false;
		}
		switch (m_stateVal){
		
		case(kReset):
				
			robot->SetFlywheelSpeed(0.0);
			m_bRequestShooter = false;
			m_bFlywheelDesiredOn = false;
			m_bHumanRequestsFlywheelOff = false;
			m_FlywheelMotorVal = 0.0;
			m_Flywheel_sum_error = 0.0;
			m_last_value = robot->shooterEncoder->Get();
			
			nextState = kIdle; 
			break;

		case (kInitialize):
			robot->SwitchToLowAngle();
			nextState = kIdle;
			break;
			
		case (kIdle):
				if (m_bRapidFireDesired){
					rapidCount = 3;
					nextState = kExtendFiringPiston;
				}
			if (m_bFiringPistonDesired || m_bAutonomousFiringPistonDesired) {
				nextState = kExtendFiringPiston; 
			}
			else { 
				nextState = kIdle;
			}
			break;
		case (kExtendFiringPiston):
				if(rapidCount > 0){
					rapidCount--;
				}
				robot->ExtendFiringPiston();
				m_initialFiringTime = m_curr_time;
				
				nextState = kWaitForFiringPiston;
			break;
		case (kWaitForFiringPiston):
				if ((m_curr_time - m_initialFiringTime) > 0.25) {
					nextState = kRetractFiringPiston;
				}
				else {
					nextState = kWaitForFiringPiston;
				}
				
			break;
		case (kWaitBetweenRapidFire):
				/*if ((m_curr_time - m_initialRapidTime) > 1.0){
					nextState = kExtendFiringPiston;
				}*/
				//if((fabs(desired - m_CurrentFlywheelSpeed))< 100)
				//{
					//nextState = kExtendFiringPiston;
			//	}
				//else {
					//nextState = kWaitBetweenRapidFire;
			//	}
		break;
		case (kRetractFiringPiston):
				robot->RetractFiringPiston();
				m_bAutonomousFiringPistonDesired = false;
				m_bAutonomousFiringPistonDone = true;
				nextState = kIdle;
				if (rapidCount > 0){
					nextState = kWaitBetweenRapidFire;
					m_initialRapidTime = m_curr_time;
				}
			break;
		}
		 m_stateVal = nextState;  

	}

	double ShooterController::GetFlywheelSpeed(){ // time and distance as vars?
		//m_curr_time = robot->timer->Get();
		//double delta = ( m_curr_time - m_last_time );
		//double encoder = robot->shooterEncoder->Get();
		//double speed = ( encoder - m_last_value) / (32.0*delta) ;
		// DO_PERIODIC(200, printf("** Delta Time: %f Curr: %f Last: %f Speed = %f Rate: %f\n", delta, (encoder), m_last_value, speed, rate));
		//m_last_time = m_curr_time ;
		//m_last_value = encoder ;
		// return speed ;
		double rate = -1.0 * robot->shooterEncoder->GetRate(); // negative rate from encoder
		return rate ;// encoder ticks/second
	}
	
	double ShooterController::GetFlywheelCounterRate(){
		double rate = robot->shooterCounter->GetPeriod(); // period per tick
		rate = 1/(rate);	// ticks per second (similar to GetRate()
		return rate; 
	}

	void ShooterController::SetShooterSpeed(double speed){
		m_DesiredFlywheelSpeed = speed;
	}
	void ShooterController::RequestFlywheel(bool on) {
		m_bAutonomousFlywheelDesired = on;
		/*if (on)
			printf("Request Flywheel being called as ON\n");
		else
			printf("Request Flywheel being called as OFF\n");
		*/
	}
	
	void ShooterController::RequestFiringPiston() {
		m_bAutonomousFiringPistonDesired = true;
		m_bAutonomousFiringPistonDone = false;
	}
	
	void ShooterController::RequestAutonomousShooterAngleChange(bool lowAngle) {
		m_bAutonomousShooterAngleChangeDesired = true;
		m_bAutonomousLowAngleDesired = lowAngle;
	}

	void ShooterController::Reset() {
		mustReset = true;
		m_bAutonomousFlywheelDesired = false;
		robot->SetFlywheelSpeed(0.0);
		ResetFilters();
	}


	double ShooterController::FlywheelPID(double desired, double current, double *diffFlywheelError) {
		//printf("In FlyWheelPID.\n");
		static float last_error = 0.0;
		float PFac = m_FlywheelPID_P;
		float IFac = m_FlywheelPID_I;
		float DFac = m_FlywheelPID_D;
		float NError = desired - current; // NError should be the new value and *diffFlywheelError is the old value

		// more than N speed- saturate
		// also means we start deceleration with N ticks to go
		//if(NError > 250.0) NError = 250.0; // these values

		double pterm = PFac * NError;

		// saturation of the terms
		if (pterm > 0.0) {pterm = min(pterm, 0.75);} //
		else {pterm = max(pterm, -0.75);}//

		double difErr = (NError - last_error);
		//DO_PERIODIC(250, printf("**difErr = NError - last_error = %f - %f = %f\n", NError, last_error, difErr));

		m_Flywheel_sum_error += NError ; // accumulate error for integral term
		// saturation of the I term - dont bother if I constant is zero (from Ini file)
		if(IFac != 0.0) {
			m_Flywheel_sum_error = (m_Flywheel_sum_error > (1.0/IFac)) ? (1.0/IFac) : m_Flywheel_sum_error ;
			m_Flywheel_sum_error = (m_Flywheel_sum_error < (-1.0/IFac)) ? (-1.0/IFac) : m_Flywheel_sum_error ;
		}

		double ierror = m_Flywheel_sum_error;
		double iterm = IFac * ierror ;



		if(difErr >= fabs(NError)) difErr = 0.0 ; // account for first time


		double dterm = DFac * (difErr);
		double motorValue = pterm + iterm + dterm;//??? is the motor value supposed to be the encoder value. 

		// saturation of the terms
		if (motorValue > 0.0) {motorValue = min(motorValue, 1.0);} //
		else {motorValue = max(motorValue, -1.0);}//


		// values are solely for testing purposes
		//DO_PERIODIC(50, printf("** pterm = PFac * NError = %f * %f = %f\n", PFac, NError, pterm));
		//DO_PERIODIC(50, printf("** dterm = DFac * difErr = %f * %f = %f\n", DFac, difErr, dterm));
		//DO_PERIODIC(20, printf("** Err: %f iErr: %f DifErr: %f \n", NError, ierror, difErr));
		//DO_PERIODIC(20, printf("** motorValue %f =  %f + %f + %f \n", motorValue, pterm, iterm, dterm));
		//DO_PERIODIC(20, printf("** Err: %f Motor: %f\n", NError, motorValue));


		last_error = NError;
		(*diffFlywheelError) = difErr;

		return motorValue;
	}

	void ShooterController::RefreshIni(){
		m_FlywheelPID_P = (robot->pini)->getf("SHOOTER", "FlywheelPFAC", 0.00041);
		m_FlywheelPID_I = (robot->pini)->getf("SHOOTER", "FlywheelIFAC", 0.0000021);
		m_FlywheelPID_D = (robot->pini)->getf("SHOOTER","FlywheelDFAC", 0.0001501);
		m_DesiredFlywheelSpeed = (robot->pini)->getf("SHOOTER", "FlywheelSpeed", 9501);
		//printf("INI file... FlywheelPFAC = %f Flywheel IFAC = %f FlywheelDFAC = %f\n", m_FlywheelPID_P, m_FlywheelPID_I,  m_FlywheelPID_D);
		//printf("INI file... Flywheel Speed: %f\n", m_DesiredFlywheelSpeed);

	}

	ShooterController::~ShooterController()
	{
	}
