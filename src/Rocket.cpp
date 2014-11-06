#include "T4TApp.h"
#include "Modes.h"
#include "MyNode.h"

Rocket::Rocket() : Mode::Mode("rocket") {
	_scene = Scene::load("res/common/scene.gpb");
	_camera = _scene->getActiveCamera();
	
	_strawNode = MyNode::create("straw");
	_strawNode->loadData("res/common/");
	BoundingBox box = _strawNode->getModel()->getMesh()->getBoundingBox();
	_strawLength = box.max.z - box.min.z;
	_originalStrawLength = _strawLength;
	_strawRadius = (box.max.x - box.min.x) / 2;
	_strawConstraint = NULL;
	setStrawConstraint();
	_scene->addNode(_strawNode);
	
	std::vector<std::string> balloonTypes;
	balloonTypes.push_back("long");
	balloonTypes.push_back("sphere");
	for(short i = 0; i < balloonTypes.size(); i++) {
		MyNode *balloon = MyNode::create(("balloon_" + balloonTypes[i]).c_str());
		balloon->loadData("res/common/", false);
		balloon->_objType = "none"; //no physics on the visual balloon - there is an anchor node for that
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
	app->removeConstraints(_strawNode);
	_balloonNodes.clear();
	_balloonType = -1;
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
			center.set(0, 0, -5);
			break;
		}
	}
	for(short i = 0; i < 1 + _balloonNodes.size(); i++) {
		MyNode *node = i == 0 ? _strawNode : _balloonNodes[i-1]._anchorNode.get();
		node->getCollisionObject()->asRigidBody()->setLinearVelocity(Vector3::zero());
	}
	Vector3 trans = center - _strawNode->getTranslationWorld();
	_strawConstraint->setEnabled(false);
	_strawNode->enablePhysics(false);
	_strawNode->myTranslate(trans);
	_strawNode->enablePhysics(true);
	_strawConstraint->setEnabled(true);
	for(short i = 0; i < _balloonNodes.size(); i++) {
		_balloonNodes[i].setScale(1);
	}
	_launching = _subMode == 2;
	_balloonMenu->setVisible(_subMode == 1);
	_balloonType = -1;
	app->setCameraEye(_subMode == 2 ? 50 : 20, 0, M_PI/12);
	return changed;
}

void Rocket::setStrawConstraint() {
	Quaternion rot = Quaternion::identity();
	float angle = atan2(_strawRadius * 2, _strawLength);
	Vector3 trans = Vector3::zero(), linearLow(0, 0, -5), linearHigh(0, 0, 5), angularLow(0, -2*M_PI, -2*M_PI),
	  angularHigh(angle, 2*M_PI, 2*M_PI);
	PhysicsGenericConstraint *constraint = app->getPhysicsController()->createGenericConstraint(
	  _strawNode->getCollisionObject()->asRigidBody(), rot, trans);
	constraint->setLinearLowerLimit(linearLow);
	constraint->setLinearUpperLimit(linearHigh);
	constraint->setAngularLowerLimit(angularLow);
	constraint->setAngularUpperLimit(angularHigh);
	if(_strawConstraint) free(_strawConstraint);
	_strawConstraint = constraint;
}

void Rocket::update() {
	if(!_launching) return;
	//deflate each balloon by a fixed percentage
	_launching = false;
	for(short i = 0; i < _balloonNodes.size(); i++) {
		Balloon &b = _balloonNodes[i];
		if(b._scale > 0.2f) {
			_launching = true;
			b.setScale(b._scale * 0.985f);
			//apply the air pressure force
			b._anchorNode->getCollisionObject()->asRigidBody()->applyForce(Vector3(0, 0, 40) * b._scale);
		}
	}
}

void Rocket::controlEvent(Control *control, EventType evt) {
	Mode::controlEvent(control, evt);
	const char *id = control->getId();

	if(strncmp(id, "balloon_", 8) == 0) {
		for(short i = 0; i < _balloons.size(); i++) {
			if(strcmp(_balloons[i]->getId(), id) == 0) {
				_balloonType = i;
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
				_strawNode->removePhysics();
				_strawNode->setScaleZ(scale);
				_strawLength = _originalStrawLength * scale;
				_strawNode->addPhysics();
				setStrawConstraint();
			}
			//attaching a balloon to the straw
			if(_subMode == 1 && _touchNode == _strawNode && _balloonType >= 0) {
				//get the unit direction vector 
				Vector3 trans = _touchPoint - _strawNode->getTranslationWorld(), strawAxis;
				_strawNode->getWorldMatrix().transformVector(Vector3::unitZ(), &strawAxis);
				strawAxis.normalize();
				trans -= strawAxis * trans.dot(strawAxis);
				trans.normalize();
				//constrain the balloon so it is fixed to the straw
				MyNode *balloonTemplate = _balloons[_balloonType].get(), *balloonNode = MyNode::cloneNode(balloonTemplate);
				os.str("");
				os << balloonTemplate->getId() << ++balloonTemplate->_typeCount;
				balloonNode->setId(os.str().c_str());
				MyNode *anchorNode = MyNode::create(MyNode::concat(2, "anchor_", balloonNode->getId()));
				BoundingBox box = balloonNode->getModel()->getMesh()->getBoundingBox();
				float balloonRadius = (box.max.x - box.min.x) / 2;
				float anchorRadius = balloonRadius * 0.5f; //best fit to the balloon shape as it deflates?
				anchorNode->setTranslation(_touchPoint + trans * anchorRadius);
				anchorNode->setRotation(_strawNode->getRotation());
				anchorNode->_objType = "sphere";
				anchorNode->_radius = anchorRadius;
				anchorNode->_mass = 0.5f;
				anchorNode->addPhysics();
				app->addConstraint(_strawNode, anchorNode, -1, "fixed", _touchPoint, trans, true);
				anchorNode->addChild(balloonNode);
				balloonNode->setTranslation((balloonRadius - anchorRadius) * trans);
				_balloonNodes.resize(_balloonNodes.size()+1);
				Balloon &balloon = _balloonNodes.back();
				balloon._rocket = this;
				balloon._balloonNode = std::shared_ptr<MyNode>(balloonNode);
				balloon._anchorNode = std::shared_ptr<MyNode>(anchorNode);
				balloon._balloonRadius = balloonRadius;
				balloon._anchorRadius = anchorRadius;
				balloon._scale = 1;
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

Rocket::Balloon::Balloon() {}

Rocket::Balloon::Balloon(Rocket *rocket) : _rocket(rocket) {}

void Rocket::Balloon::setScale(float scale) {
	_scale = scale;
	_balloonNode->setScale(_scale);
	//adjust it so it is still tangent to the straw
	Vector3 trans = _anchorNode->getTranslationWorld() - _rocket->_strawNode->getTranslationWorld(), strawAxis;
	_rocket->_strawNode->getWorldMatrix().transformVector(Vector3::unitZ(), &strawAxis);
	strawAxis.normalize();
	trans -= strawAxis * trans.dot(strawAxis);
	trans = trans.normalize() * (_scale * _balloonRadius - _anchorRadius);
	_balloonNode->setTranslation(trans);	
}


