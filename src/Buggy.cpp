#include "T4TApp.h"
#include "Modes.h"
#include "MyNode.h"

Buggy::Buggy() : Mode::Mode("buggy") {
	_body = addElement(new Body(this));
	_frontAxle = addElement(new Axle(this, "Front Axle", _body));
	_rearAxle = addElement(new Axle(this, "Rear Axle", _body));
	_frontWheels = addElement(new Wheels(this, "Front Wheels", _frontAxle));
	_rearWheels = addElement(new Wheels(this, "Rear Wheels", _rearAxle));
	setupMenu();
}

Buggy::Body::Body(Project *project) : Project::Element(project, "Body") {}

bool Buggy::Body::touchEvent(Touch::Event evt, int x, int y, unsigned int contactIndex) {
	return true;
}

Buggy::Axle::Axle(Project *project, const char *id, Element *parent) : Project::Element(project, id, parent) {}

bool Buggy::Axle::touchEvent(Touch::Event evt, int x, int y, unsigned int contactIndex) {
	Element::touchEvent(evt, x, y, contactIndex);
	//moving the axle in the yz-plane but only within the bounds of the body
	if(_parentTouch._hit) {
		getNode()->myTranslate(_planeTouch.getPoint(evt));
	}
}

Buggy::Wheels::Wheels(Project *project, const char *id, Element *parent) : Project::Element(project, id, parent) {}

bool Buggy::Wheels::touchEvent(Touch::Event evt, int x, int y, unsigned int contactIndex) {
	Element::touchEvent(evt, x, y, contactIndex);
	_parent->touchEvent(evt, x, y, contactIndex); //for now just move the axle
}

void Buggy::test() {
}


