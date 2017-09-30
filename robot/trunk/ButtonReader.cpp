#include "ButtonReader.h"

ButtonReader::ButtonReader(Joystick* myJoystick, int myButtonNum) {
	joystick = myJoystick;
	buttonNum = myButtonNum;
	currState = joystick->GetRawButton(buttonNum);
	lastState = currState;
}

bool ButtonReader::WasJustPressed() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
	return (lastState == false && currState == true);
}

bool ButtonReader::WasJustReleased() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
	return (lastState == true && currState == false);
}

bool ButtonReader::StateJustChanged() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
	return (lastState != currState);
}

bool ButtonReader::IsDown() {
	return (joystick->GetRawButton(buttonNum));
}

ButtonReader::~ButtonReader()
{
}

ToggleReader::ToggleReader(Joystick *myJoy, int upButton, int downButton){
	joy = myJoy;
	upB = upButton; 
	downB = downButton;
}

int ToggleReader::GetToggleState(){
	if (joy->GetRawButton(upB)) return 1;
	if (joy->GetRawButton(downB)) return -1;
	return 0;
}

