#ifndef ROBOT_H_
#define ROBOT_H_

#include "Project.h"

class Robot : public Project {
public:
	MyNode *_robot;
	PhysicsCharacter *_character;
	std::vector<std::string> _animations;

	Robot();
	void setupMenu();
	void setActive(bool active);
	bool setSubMode(short mode);
	void launch();
	void controlEvent(Control *control, EventType evt);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
};

#endif
