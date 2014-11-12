#include "T4TApp.h"
#include "Buggy.h"
#include "MyNode.h"

Buggy::Buggy() : Project::Project("buggy") {

	app->addItem("axle1", 2, "general", "axle");
	app->addItem("wheel1", 2, "general", "wheel");
	app->addItem("wheel2", 2, "general", "wheel");

	_body = addElement(new Body(this));
	_frontAxle = addElement(new Axle(this, _body, "frontAxle", "Front Axle"));
	_rearAxle = addElement(new Axle(this, _body, "rearAxle", "Rear Axle"));
	_frontWheels = addElement(new Wheels(this, _frontAxle, "frontWheels", "Front Wheels"));
	_rearWheels = addElement(new Wheels(this, _rearAxle, "rearWheels", "Rear Wheels"));
	setupMenu();
	app->addListener(_controls, this);

	_ramp = MyNode::create("buggyRamp");
	_ramp->loadData("res/common/", false);
	_ramp->setTranslation(0, 2.5f, -7.5f);
	_ramp->setStatic(true);
	_ramp->addPhysics();
	_ramp->setVisible(false);
	_scene->addNode(_ramp);
}

void Buggy::setupMenu() {
	Project::setupMenu();
	_controls->setHeight(_controls->getHeight() + 50.0f);
	_launchButton = app->addButton <Button> (_controls, "launch", "Launch!");
	_launchButton->setEnabled(false);
}

void Buggy::setActive(bool active) {
	Project::setActive(active);
	_ramp->setVisible(false);
}

bool Buggy::setSubMode(short mode) {
	bool changed = Project::setSubMode(mode);
	switch(_subMode) {
		case 0: { //build
			break;
		} case 1: { //test
			_rootNode->enablePhysics(false);
			//Vector3 trans(0, 6, 0);
			//_rootNode->setMyTranslation(trans);
			setRampHeight(1);
			app->getPhysicsController()->setGravity(app->_gravity);
			app->setCameraEye(30, 0, M_PI/12);
			app->_ground->setVisible(true);
			_ramp->setVisible(true);
			_launched = false;
			_launchButton->setEnabled(true);
			break;
		}
	}
}

void Buggy::setRampHeight(float scale) {
	cout << "scaling ramp to " << scale << endl;
	_rampSlope = scale * 5.0f / 15.0f;
	_ramp->removePhysics();
	_ramp->setScaleY(scale);
	_ramp->setTranslationY(scale * 2.5f);
	_ramp->updateTransform();
	_ramp->addPhysics();
	//position the buggy near the top of the ramp
	_rootNode->updateTransform();
	BoundingBox box = _rootNode->getBoundingBox(true);
	Vector3 normal(0, 1, _rampSlope), trans(0, 0, -15 - box.min.z);
	normal.normalize();
	trans.y = -trans.z * _rampSlope;
	trans -= normal * box.min.y;
	_rootNode->setMyTranslation(trans);
	Quaternion rot(Vector3::unitX(), atan2(scale * 5.0f, 15.0f));
	_rootNode->setMyRotation(rot);
}

void Buggy::controlEvent(Control *control, EventType evt) {
	Project::controlEvent(control, evt);
	const char *id = control->getId();
	
	if(strcmp(id, "launch") == 0) {
		_rootNode->enablePhysics(true);
		app->getPhysicsController()->setGravity(app->_gravity);
		_rootNode->setActivation(DISABLE_DEACTIVATION);
		_launched = true;
	}
}

bool Buggy::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Project::touchEvent(evt, x, y, contactIndex);
	if(_subMode == 1 && !_launched) {
		if(_touching && _touchNode == _ramp) {
			float scale = _ramp->_baseScale.y * (1.0f - _touchPt.deltaPix().y / 400.0f);
			scale = fmin(4.0f, fmax(0.2f, scale));
			setRampHeight(scale);
		}
	}
	return true;
}

Buggy::Body::Body(Project *project) : Project::Element(project, NULL, "body", "Body") {
	_filter = "body";
}

bool Buggy::Body::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	return true;
}

Buggy::Axle::Axle(Project *project, Element *parent, const char *id, const char *name)
  : Project::Element(project, parent, id, name) {
  	_filter = "axle";
}

void Buggy::Axle::placeNode(const Vector3 &position, short n) {
	//z-axis of the cylinder should be along the buggy's x-axis
	_nodes[n]->setRotation(Vector3::unitY(), M_PI/2);
	_nodes[n]->setTranslation(0, position.y, position.z);
}

void Buggy::Axle::addPhysics(short n) {
	Element::addPhysics(n);
	app->getPhysicsController()->setConstraintNoCollide();
	app->addConstraint(_parent->getNode(), getNode(), -1, "fixed", Vector3::zero(), Vector3::zero(), true);
}

bool Buggy::Axle::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Element::touchEvent(evt, x, y, contactIndex);
	//moving the axle in the yz-plane but only within the bounds of the body
	if(_parentTouch._hit) {
		getNode()->baseTranslate(_planeTouch.delta());
	}
}

Buggy::Wheels::Wheels(Project *project, Element *parent, const char *id, const char *name)
  : Project::Element(project, parent, id, name) {
	_numNodes = 2;
	_filter = "wheel";
}

void Buggy::Wheels::placeNode(const Vector3 &position, short n) {
	//position determines how far we are along the axle
	float dir = n == 0 ? 1 : -1;
	Vector3 parent = _parent->getNode()->getTranslationWorld();
	_nodes[n]->setRotation(Vector3::unitY(), M_PI/2);
	_nodes[n]->setTranslation(dir * position.x, parent.y, parent.z);
}

void Buggy::Wheels::addPhysics(short n) {
	Element::addPhysics(n);
	MyNode *node = getNode(n);
	app->getPhysicsController()->setConstraintNoCollide();
	app->addConstraint(_parent->getNode(), node, -1, "hinge", node->getTranslationWorld(), Vector3::unitX(), true);
}

bool Buggy::Wheels::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Element::touchEvent(evt, x, y, contactIndex);
	_parent->touchEvent(evt, x, y, contactIndex); //for now just move the axle
}

