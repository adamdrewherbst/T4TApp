#include "T4TApp.h"
#include "Robot.h"
#include "MyNode.h"

Robot::Robot() : Project::Project("robot") {
	_robot = MyNode::create("robot");
	_robot->loadData("res/common/", false);
	//load robot animations
	_animations.push_back("walk");
	short i, n = _animations.size();
	for(i = 0; i < n; i++) {
		_robot->loadAnimation("res/common/robot.animation", _animations[i].c_str());
	}
	setupMenu();
}

void Robot::setupMenu() {
	Project::setupMenu();
}

void Robot::setActive(bool active) {
	Project::setActive(active);
	if(active) {
		PhysicsRigidBody::Parameters params(20.0f);
		_robot->setCollisionObject(PhysicsCollisionObject::CHARACTER,
			PhysicsCollisionShape::capsule(1.7f, 6.0f, Vector3(0, 3.0f, 0), true), &params);
		_character = static_cast<PhysicsCharacter*>(_robot->getCollisionObject());
		_scene->addNode(_robot);
		app->_ground->setVisible(true);
	} else {
		_robot->setCollisionObject(PhysicsCollisionObject::NONE);
		_character = NULL;
	}
}

bool Robot::setSubMode(short mode) {
	bool changed = Project::setSubMode(mode);
	return changed;
}

void Robot::launch() {
	Project::launch();
}

void Robot::controlEvent(Control *control, EventType evt) {
	Project::controlEvent(control, evt);
	const char *id = control->getId();
	
	if(strcmp(id, "walk") == 0) {
		_robot->playAnimation("walk", true);
	} else if(strcmp(id, "stop") == 0) {
		_robot->stopAnimation();
	}
}

bool Robot::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Project::touchEvent(evt, x, y, contactIndex);
	return true;
}


