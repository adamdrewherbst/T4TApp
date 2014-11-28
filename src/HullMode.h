#ifndef HULLMODE_H_
#define HULLMODE_H_

#include "Mode.h"

class HullMode : public Mode
{
public:
	std::vector<short> _selection; //face indices
	std::vector<MyNode*> _selections;
	bool _shiftPressed, _ctrlPressed;
	
	HullMode();
	void setActive(bool active);
	bool setSubMode(short mode);
	void selectItem(const char *id);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
	void keyEvent(Keyboard::KeyEvent evt, int key);
	void clearSelection();
	void addFace(short face);
	void toggleFace(short face);
	void updateSelection();
};

#endif
