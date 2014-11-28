#ifndef HULLMODE_H_
#define HULLMODE_H_

#include "Mode.h"

class HullMode : public Mode
{
public:
	class Selection {
		public:
		HullMode *_mode;
		std::vector<short> _faces;
		MyNode *_node; //for display
		
		Selection(HullMode *mode, const char *id, Vector3 color = Vector3::unitX());
		void addFace(short face);
		void toggleFace(short face);
		void clear();
		void update();
	};
	Selection *_region, *_chain, *_currentSelection;
	bool _shiftPressed, _ctrlPressed;
	
	HullMode();
	void setActive(bool active);
	bool setSubMode(short mode);
	void selectItem(const char *id);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
	void keyEvent(Keyboard::KeyEvent evt, int key);
	void placeCamera();
};

#endif
