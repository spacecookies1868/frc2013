#ifndef ROBOTPORTS2013_H_
#define ROBOTPORTS2013_H_

//********PWM PORTS (CORRECT FOR 2013 ROBOT)**********
static const int SHOOTER_MOTOR_A_PWM_PORT		= 1;
static const int SHOOTER_MOTOR_B_PWM_PORT 		= 2;

static const int LEFT_MOTOR_A_PWM_PORT 			= 5;
static const int LEFT_MOTOR_B_PWM_PORT 			= 6;

static const int RIGHT_MOTOR_A_PWM_PORT 		= 7; 
static const int RIGHT_MOTOR_B_PWM_PORT 		= 8;

static const int CLIMB_MOTOR_A_PWM_PORT 		= 4;
static const int CLIMB_MOTOR_B_PWM_PORT 		= 9; 

//***********DIGITAL I/O PORTS************************ 

static const int GYRO_PORT 						= 1; 

static const int LEFT_WHEEL_ENCODER_A_PWM_PORT 	= 2;
static const int LEFT_WHEEL_ENCODER_B_PWM_PORT 	= 3;

static const int RIGHT_WHEEL_ENCODER_A_PWM_PORT = 4;
static const int RIGHT_WHEEL_ENCODER_B_PWM_PORT = 5;
 
static const int FLYWHEEL_ENCODER_A_PWM_PORT 	= 6;
static const int FLYWHEEL_ENCODER_B_PWM_PORT 	= 7;

//*********SOLENOID PORTS**************************** 
static const int GEAR_SHIFT_SOLENOID_CHAN		= 1;

static const int FIRING_PISTON_SOLENOID_CHAN	= 3;

static const int CLIMB_SOLENOID_A_CHAN 			= 2;

static const int DUMPER_SOLENOID_A_CHAN			= 6;

static const int DUMPER_SOLENOID_B_CHAN			= 7;

//**************MISC***********************************
static const int COMPRESSOR_PRESSURE_SWITCH_CHAN = 1;
static const int COMPRESSOR_RELAY_CHAN 			= 1;

//**********JOYSTICK USB PORTS************************ Hansa and Sabrina wrote this. JK Rachel did all this//
static const int LEFT_JOY_USB_PORT 				= 1;
static const int RIGHT_JOY_USB_PORT 			= 2;
static const int OPERATOR_JOY_USB_PORT 			= 3;
static const int ISS_JOY_USB_PORT				= 4;
#endif /*ROBOTPORTS2013_H_*/
