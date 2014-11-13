#ifndef SATELLITE_H_
#define SATELLITE_H_

#include "Project.h"

class Satellite : public Project {
public:
	class Body : public Project::Element {
		public:
		Body(Project *project);
	};

	class Instrument : public Project::Element {
		public:
		short _addingPanels; //how many solar panels I still need to add to power the current instrument
		float _maxMass;

		Instrument(Project *project, Element *parent);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void setNode(const char *id);
		void addNode(const Vector3 &position);
		void placeNode(const Vector3 &position, short n);
		void addPhysics(short n);
		float getMass(const char *type);
		short getPanelsNeeded(const char *type);
		float getTotalMass();
	};
	
	Body *_body;
	Instrument *_instruments;
	float _maxRadius, _maxLength;

	Satellite();
	bool setSubMode(short mode);
	void launch();
};

#endif
