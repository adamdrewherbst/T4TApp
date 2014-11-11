#ifndef NAVIGATEMODE_H_
#define NAVIGATEMODE_H_

#include "Mode.h"

class NavigateMode : public Mode
{
public:
	NavigateMode();
	void setActive(bool active);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
};

#endif
