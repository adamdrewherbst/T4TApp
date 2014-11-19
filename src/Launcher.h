#ifndef LAUNCHER_H_
#define LAUNCHER_H_

#include "Project.h"

class Launcher : public Project {
public:
	class RubberBand : public Project::Element {
		public:
		RubberBand(Project *project);
	};

	RubberBand *_rubberBand;
	short _numLinks;
	Vector3 _anchor1, _anchor2; //rubber band endpoints
	
	MyNode *_cev; //use the crew exploration vehicle as the payload

	Launcher();
	bool setSubMode(short mode);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void launch();
};

#endif
