#ifndef BUTTONREADER_H_
#define BUTTONREADER_H_

#include "Joystick.h"

class ToggleReader {
public:
	ToggleReader(Joystick *myJoy, int upButton, int downButton);
	int GetToggleState();
	
private:
	Joystick *joy;
	int upB;
	int downB;
};

class ButtonReader
{
public:
	ButtonReader(Joystick *joy, int buttonNum);
	virtual ~ButtonReader();
	bool IsDown();
	bool WasJustPressed();
	bool WasJustReleased();
	bool StateJustChanged();
	
private:
	Joystick *joystick;
	int buttonNum;
	bool lastState;
	bool currState;
};

#endif /*BUTTONREADER_H_*/
