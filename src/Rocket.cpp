#include "T4TApp.h"

Rocket::Rocket() : Mode::Mode("rocket") {
	_scene = Scene::load("res/common/scene.gpb");
	_camera = _scene->getActiveCamera();
	
	_strawNode = MyNode::create("straw");
	_strawNode->loadData("res/common/");
	BoundingBox box = _strawNode->getModel()->getMesh()->getBoundingBox();
	_strawLength = box.max.z - box.min.z;
	_strawRadius = (box.max.x - box.min.x) / 2;
	setStrawConstraint();
	_scene->addNode(_strawNode);
	
	std::vector<std::string> balloonTypes;
	balloonTypes.push_back("long");
	balloonTypes.push_back("sphere");
	for(short i = 0; i < balloonTypes.size(); i++) {
		MyNode *balloon = MyNode::create(("balloon_" + balloonTypes[i]).c_str());
		balloon->loadData("res/common/");
		_balloons.push_back(std::shared_ptr<MyNode>(balloon));
	}
	
	_subModes.push_back("cutStraw");
	_subModes.push_back("addBalloon");
	_subModes.push_back("launch");
	
	_balloonMenu = (Container*) _container->getControl("balloonMenu");
	_balloonMenu->setVisible(false);
}

void Rocket::setActive(bool active) {
	Mode::setActive(active);
	if(active) {
		app->setActiveScene(_scene);
		app->cameraPush();
		app->setCameraEye(20, 0, 0);
		for(short i = 0; i < _balloons.size(); i++) _balloons[i]->_typeCount = 0;
	} else {
		app->cameraPop();
		app->showScene();
	}
}

bool Rocket::setSubMode(short mode) {
	bool changed = Mode::setSubMode(mode);
	Vector3 center = Vector3::zero();
	switch(_subMode) {
		case 0: { //cut straw
			break;
		} case 1: { //add balloon
			break;
		} case 2: { //launch
			center.set(0, 0, -10);
			break;
		}
	}
	Vector3 trans = center - _strawNode->getTranslationWorld();
	_strawNode->myTranslate(trans);
	for(short i = 0; i < 1 + _balloonNodes.size(); i++) {
		MyNode *node = i == 0 ? _strawNode : _balloonNodes[i-1].get();
		node->getCollisionObject()->asRigidBody()->setLinearVelocity(Vector3::zero());
	}
	_balloonMenu->setVisible(_subMode == 1);
	_launching = _subMode == 2;
	app->setCameraEye(_subMode == 2 ? 40 : 20, 0, M_PI/12);
	return changed;
}

void Rocket::setStrawConstraint() {
	Quaternion rot = Quaternion::identity();
	float angle = atan2(_strawRadius * 2, _strawLength);
	Vector3 trans = Vector3::zero(), linearLow(0, 0, -15), linearHigh(0, 0, 15), angularLow(0, -2*M_PI, -2*M_PI),
	  angularHigh(angle, 2*M_PI, 2*M_PI);
	PhysicsGenericConstraint *constraint = app->getPhysicsController()->createGenericConstraint(
	  _strawNode->getCollisionObject()->asRigidBody(), rot, trans);
	constraint->setLinearLowerLimit(linearLow);
	constraint->setLinearUpperLimit(linearHigh);
	constraint->setAngularLowerLimit(angularLow);
	constraint->setAngularUpperLimit(angularHigh);
}

void Rocket::update() {
	if(!_launching) return;
	//deflate each balloon by a fixed percentage
	for(short i = 0; i < _balloonNodes.size(); i++) {
		MyNode *balloon = _balloonNodes[i].get();
		float scale = balloon->getScaleX();
		if(scale > 0.2f) {
			scale *= 0.985f;
			balloon->setScale(scale);
		}
		//adjust it so it is still tangent to the straw
		Vector3 trans = balloon->getTranslationWorld() - _strawNode->getTranslationWorld();
		BoundingSphere sphere = balloon->getModel()->getMesh()->getBoundingSphere();
		float radius = sphere.radius;
		trans = trans.normalize() * (radius + _strawRadius);
		balloon->setTranslation(_strawNode->getTranslationWorld() + trans);
		//apply the air pressure force
		balloon->getCollisionObject()->asRigidBody()->applyForce(Vector3(0, 0, 40) * scale);
	}
}

void Rocket::controlEvent(Control *control, EventType evt) {
	Mode::controlEvent(control, evt);
	const char *id = control->getId();

	if(strncmp(id, "balloon_", 8) == 0) {
		for(short i = 0; i < _balloons.size(); i++) {
			if(strcmp(_balloons[i]->getId(), id) == 0) {
				_balloonNode = MyNode::cloneNode(_balloons[i].get());
				os.str("");
				os << "balloon" << ++_balloons[i]->_typeCount;
				_balloonNode->setId(os.str().c_str());
				BoundingBox box = _balloonNode->getModel()->getMesh()->getBoundingBox();
				_balloonRadius = (box.max.x - box.min.x) / 2;
				_balloonMenu->setVisible(false);
			}
		}
	}
}

bool Rocket::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			//cutting the straw
			if(_subMode == 0 && _touchNode == _strawNode) {
				Vector3 trans = _touchPoint - _strawNode->getTranslationWorld();
				Matrix straw = _strawNode->getWorldMatrix(), strawInv;
				straw.invert(&strawInv);
				strawInv.transformVector(&trans);
				float scale = ((_strawLength/2 - trans.z) / _strawLength) * _strawNode->getScaleZ();
				_strawNode->setScaleZ(scale);
				_strawLength = _originalStrawLength * scale;
				_strawNode->removePhysics();
				_strawNode->addPhysics();
				setStrawConstraint();
			}
			//attaching a balloon to the straw
			if(_subMode == 1 && _touchNode == _strawNode && _balloonNode != NULL) {
				Vector3 trans = _touchPoint - _strawNode->getTranslationWorld(), dir = trans.normalize();
				Matrix straw = _strawNode->getWorldMatrix(), strawInv;
				straw.invert(&strawInv);
				strawInv.transformVector(&trans);
				trans.z = 0;
				trans *= _balloonRadius / trans.length();
				straw.transformVector(&trans);
				_balloonNode->setTranslation(_touchPoint + trans);
				_balloonNode->setRotation(_strawNode->getRotation());
				//constrain the balloon so it is fixed to the straw
				_balloonNode->enablePhysics();
				app->addConstraint(_balloonNode, _strawNode, -1, "fixed", _touchPoint, dir, true);
				_scene->addNode(_balloonNode);
				_balloonNodes.push_back(std::shared_ptr<MyNode>(_balloonNode));
				_balloonRadii.push_back(_balloonRadius);
				_balloonNode = NULL;
			}
			break;
		} case Touch::TOUCH_MOVE: {
			break;
		} case Touch::TOUCH_RELEASE: {
			break;
		}
	}
	return true;
}

