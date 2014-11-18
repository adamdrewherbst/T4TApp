#include "T4TApp.h"
#include "LandingPod.h"
#include "MyNode.h"

LandingPod::LandingPod() : Project::Project("landingPod") {

	app->addItem("hatch1", 2, "general", "hatch");
	app->addItem("hatch2", 2, "general", "hatch");

	_body = (Body*) addElement(new Body(this));
	_hatch = (Hatch*) addElement(new Hatch(this, _body));
	setupMenu();
	app->addListener(_controls, this);
}

void LandingPod::setupMenu() {
	Project::setupMenu();
	_hatchButton = app->addButton <Button> (NULL, "openHatch", "Open Hatch");
	_controls->insertControl(_hatchButton, 2);
	_hatchButton->setEnabled(false);
	_controls->setHeight(_controls->getHeight() + 70.0f);	
}

bool LandingPod::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Project::touchEvent(evt, x, y, contactIndex);
	return true;
}

void LandingPod::controlEvent(Control *control, EventType evt) {
	Project::controlEvent(control, evt);
	const char *id = control->getId();
	
	if(control == _hatchButton) {
		openHatch();
		_hatchButton->setEnabled(false);
	}
}

void LandingPod::setActive(bool active) {
	Project::setActive(active);
}

bool LandingPod::setSubMode(short mode) {
	bool changed = Project::setSubMode(mode);
	switch(_subMode) {
		case 0: { //build
			break;
		} case 1: { //test
			//put the buggy inside the pod
			
			//place the pod at a fixed height
			_rootNode->enablePhysics(false);
			_rootNode->placeRest();
			Vector3 trans(0, 10, 0);
			_rootNode->setMyTranslation(trans);
			app->getPhysicsController()->setGravity(Vector3::zero());
			app->_ground->setVisible(true);
			break;
		}
	}
	if(changed) _hatchButton->setEnabled(false);
	return changed;
}

void LandingPod::launch() {
	Project::launch();
	_rootNode->enablePhysics(true);
	app->getPhysicsController()->setGravity(app->_gravity);
	_rootNode->setActivation(DISABLE_DEACTIVATION);
	_hatchButton->setEnabled(true);
}

void LandingPod::openHatch() {
	//release the lock and give the hatch an outward kick
	if(_hatch->_lock.get() != nullptr) _hatch->_lock->setEnabled(false);
	//the torque is about the hinge axis
	MyNode *node = _hatch->getNode();
	node->getCollisionObject()->asRigidBody()->applyTorqueImpulse(-node->_parentAxis * 10.0f);
}

LandingPod::Body::Body(Project *project) : Project::Element::Element(project, NULL, "body", "Body") {
	_filter = "body";
}

LandingPod::Hatch::Hatch(Project *project, Element *parent)
  : Project::Element::Element(project, parent, "hatch", "Hatch") {
	_filter = "hatch";
}

void LandingPod::Hatch::placeNode(short n) {
	//put the bottom center of the bounding box where the user clicked
	MyNode *node = _nodes[n].get(), *parent = _parent->getNode();
	node->updateTransform();
	BoundingBox box = node->getBoundingBox(true);
	node->shiftModel(0, -box.min.y, 0);
	node->updateModel(false, false);
	Vector3 point = _project->getTouchPoint(), normal = _project->getTouchNormal(), axis;
	node->attachTo(parent, point, normal);
	//the hinge axis is the tangent to the surface that lies in the xz-plane
	Vector3 normalXZ(normal.x, 0, normal.z);
	if(normalXZ.length() < 1e-4) axis.set(1, 0, 0);
	else axis.set(-normal.z, 0, normal.x);
	node->_parentAxis = axis.normalize();
}

void LandingPod::Hatch::addPhysics(short n) {
	Project::Element::addPhysics(n);
	//the hinge should always be on the bottom edge so the buggy can roll out
	MyNode *node = _nodes[n].get(), *parent = _parent->getNode();
	app->getPhysicsController()->setConstraintNoCollide();
	app->addConstraint(parent, node, -1, "hinge", node->_parentOffset, node->_parentAxis, true);
	//fix the hatch in place until we have landed!
	_lock = ConstraintPtr(app->addConstraint(parent, node, -1, "fixed", node->_parentOffset, node->_parentAxis, false));
}


