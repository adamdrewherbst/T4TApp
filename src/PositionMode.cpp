#include "T4TApp.h"

T4TApp::PositionMode::PositionMode(T4TApp *app_) 
  : T4TApp::Mode::Mode(app_, "mode_Position", "res/common/position.form") {

	_subModeButton = (Button*) _controls->getControl("subMode");
	_axisButton = (Button*) _controls->getControl("axis");
	_valueSlider = (Slider*) _controls->getControl("axisValue");
	_staticCheckbox = (CheckBox*) _controls->getControl("static");
	_gridCheckbox = (CheckBox*) _controls->getControl("snap");
	_gridSlider = (Slider*) _controls->getControl("spacing");
	
	_subModes.push_back(std::string("Translating"));
	_subModes.push_back(std::string("Rotating"));
	_subModes.push_back(std::string("Scaling"));
	_subModes.push_back(std::string("Ground Face"));
	
	_axisNames.push_back("X");
	_axisNames.push_back("Y");
	_axisNames.push_back("Z");
	_axisNames.push_back("All");
}

void T4TApp::PositionMode::setActive(bool active) {
	Mode::setActive(active);
	setSubMode(0);
	setAxis(0);
	setSelectedNode(NULL);
	_valueSlider->setEnabled(false);
	_dragging = false;
}

bool T4TApp::PositionMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	Mode::touchEvent(evt, x, y, contactIndex);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			_groundFace = -1;
		    Camera* camera = app->_scene->getActiveCamera();
		    Ray ray;
		    camera->pickRay(app->getViewport(), x, y, &ray);
		    PhysicsController::HitResult hitResult;
		    if(!app->getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) break;
	    	Node *node = hitResult.object->getNode();
	    	if(node->getCollisionObject() == NULL) break;
		    if(strcmp(node->getId(), "grid") == 0) break;
			cout << "selected: " << node->getId() << endl;
			setSelectedNode(node);
			_dragging = true;
			_dragOffset.set(x, y);
			if(_subMode == 0) { //translate - treat it as if user clicked on point on grid directly below this object's center
				_basePoint = node->getTranslationWorld();
				_basePoint.y = 0;
				Vector2 centerPix;
				camera->project(app->getViewport(), _basePoint, &centerPix);
				_dragOffset.set(centerPix.x - x, centerPix.y - y);
				cout << "dragging " << node->getId() << " with offset " << app->printVector2(_dragOffset) << endl;
			} else if(_subMode == 1) { //rotate
			} else if(_subMode == 2) { //scale
			} else if(_subMode == 3) { //ground face - determine which face was selected
				_groundFace = _selectedNode->pt2Face(hitResult.point);
			}
			//disable all physics during the move - if a node is static, we must remove its physics and re-add it at the end
			app->_intersectNodeGroup.clear();
			app->_intersectNodeGroup.push_back(_selectedNode);
			app->enablePhysics(_selectedNode, false);
			break;
		}
		case Touch::TOUCH_MOVE: case Touch::TOUCH_RELEASE: {
			bool skip = !_dragging || _selectedNode == NULL;
			if(evt == Touch::TOUCH_RELEASE) _dragging = false;
			if(skip) break;
			bool finalize = evt == Touch::TOUCH_RELEASE;
			Ray ray;
			Camera *camera = app->_scene->getActiveCamera();
			if(_subMode == 0) { //translate
				camera->pickRay(app->getViewport(), x + _dragOffset.x, y + _dragOffset.y, &ray);
				float distance = ray.intersects(app->_groundPlane);
				if(distance == Ray::INTERSECTS_NONE) break;
				Vector3 point(ray.getOrigin() + ray.getDirection() * distance);
				Vector3 delta(point - _basePoint);
				_transDir = delta / delta.length();
				setPosition(delta.length(), finalize);
			} else if(_subMode == 1) { //rotate
				float angle = (x - _dragOffset.x) * 2 * M_PI / 500;
				while(angle < 0) angle += 2 * M_PI;
				while(angle > 2 * M_PI) angle -= 2 * M_PI;
				setPosition(angle, finalize);
			} else if(_subMode == 2) { //scale
				float exp = (x - _dragOffset.x) / 500.0f;
				if(exp < -1.0f) exp = -1.0f;
				if(exp > 1.0f) exp = 1.0f;
				float scale = pow(10.0f, exp);
				setPosition(scale, finalize);
			} else if(_subMode == 3) { //pick ground face
				if(!finalize) {
					camera->pickRay(app->getViewport(), x, y, &ray);
					PhysicsController::HitResult hitResult;
					if(app->getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) {
						Node *node = hitResult.object->getNode();
						if(node == _selectedNode) {
							unsigned short newFace = _selectedNode->pt2Face(hitResult.point);
							if(newFace >= 0) _groundFace = newFace;
						}
					}
				} else if(_groundFace >= 0) {
					_selectedNode->rotateFaceToPlane(_groundFace, app->_groundPlane);
					app->enablePhysics(_selectedNode);
					_groundFace = -1;
				}
			}
			break;
		}
	}
}

void T4TApp::PositionMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
	if(control == _subModeButton) {
		setSubMode((_subMode + 1) % _subModes.size());
	} else if(control == _valueSlider) {
		if(_subMode == 0) {
			_transDir.set(0, 0, 0);
			Node::sv(&_transDir, _axis, 1);
		}
		setPosition(_valueSlider->getValue(), evt == Control::Listener::RELEASE);
	} else if(control == _axisButton) {
		setAxis((_axis + 1) % _axisNames.size());
	} else if(control == _staticCheckbox) {
		if(_selectedNode != NULL) {
			_selectedNode->setStatic(_staticCheckbox->isChecked());
			app->removePhysics(_selectedNode);
			app->addPhysics(_selectedNode);
		}
	}
}

void T4TApp::PositionMode::setSelectedNode(Node *node) {
	//if(_selectedNode == node) return;
	_selectedNode = node;
	if(_selectedNode != NULL) {
		switch(_subMode) {
			case 0: { //translate
				_positionValue = 0.0f;
				break;
			} case 1: //rotate
				_positionValue = 0.0f;
				break;
			case 2: //scale
				_positionValue = 1.0f;
				break;
		}
		_baseRotation = _selectedNode->getRotation();
		_baseScale = _selectedNode->getScale();
		_baseTranslation = _selectedNode->getTranslationWorld();
	} else {
		_positionValue = 0.0f;
	}
	updateSlider();
	_staticCheckbox->setEnabled(_selectedNode != NULL);
	_staticCheckbox->setChecked(_selectedNode != NULL && _selectedNode->isStatic());
}

void T4TApp::PositionMode::updateSlider() {
	bool enable = _selectedNode != NULL;
	_valueSlider->setEnabled(enable);
	_valueSlider->setValue(_positionValue);
}

void T4TApp::PositionMode::setPosition(float value, bool finalize) {
	_positionValue = value;
	//perform the transformation
	switch(_subMode) {
		case 0: {//translate
			Vector3 delta(_transDir * value);
			_selectedNode->setTranslation(_baseTranslation + delta);
			_positionValue = Node::gv(&delta, _axis);
			break;
		} case 1: { //rotate
			Quaternion rot;
			Quaternion::createFromAxisAngle(Vector3(0, 1, 0), value, &rot);
			_selectedNode->setRotation(rot * _baseRotation);
			break;
		} case 2: { //scale
			Vector3 scale(_baseScale);
			if(_axis == 0) scale.x *= value;
			else if(_axis == 1) scale.y *= value;
			else if(_axis == 2) scale.z *= value;
			else if(_axis == 3) scale *= value;
			_selectedNode->setScale(scale);
			break;
		}
	}
	Vector3 trans(_selectedNode->getTranslationWorld());
	//when translating, snap object to grid if desired
	if(_subMode == 0 && _gridCheckbox->isChecked()) {
		float spacing = _gridSlider->getValue();
		trans.x = round(trans.x / spacing) * spacing;
		trans.z = round(trans.z / spacing) * spacing;
	}
	//if would intersect another object, place it on top of that object instead
	app->placeNode(_selectedNode, trans.x, trans.z);
	updateSlider();
	//toggle physics to keep bounding box with object, or re-enable if done moving
	if(!finalize) {
		PhysicsCollisionObject *obj = _selectedNode->getCollisionObject();
		if(obj) {
			PhysicsRigidBody *body = obj->asRigidBody();
			body->setEnabled(true); body->setEnabled(false);
		}
	} else {
		//after scaling, must re-create collision object
		if(_subMode == 2) {
			app->removeConstraints(_selectedNode);
			_selectedNode->setCollisionObject(PhysicsCollisionObject::NONE);
		}
		app->enablePhysics(_selectedNode);
		_selectedNode->updateData();
	}
}

void T4TApp::PositionMode::setSubMode(unsigned short mode) {
	_subMode = mode;
	_subModeButton->setText(_subModes[_subMode].c_str());
	if(_subMode == 0 && _axis == 3) setAxis(0);
	_axisButton->setEnabled(_subMode == 0 || _subMode == 2);
	Vector4 limits = Vector4::zero();
	switch(_subMode) {
		case 0: //translate
			limits.set(-25.0f, 25.0f, 0.1f, 0.0f);
			break;
		case 1: //rotate
			limits.set(0.0f, 2 * M_PI, 0.01f, 0.0f);
			break;
		case 2: //scale
			limits.set(0.1f, 10.0f, 0.1f, 1.0f);
			break;
	}
	bool enable = !limits.isZero();
	_valueSlider->setEnabled(enable);
	if(enable) {
		_valueSlider->setMin(limits.x);
		_valueSlider->setMax(limits.y);
		_valueSlider->setStep(limits.z);
		_valueSlider->setValue(limits.w);
	}
}

void T4TApp::PositionMode::setAxis(short axis) {
	if(_subMode == 0) {
		if(axis == 3) {
			setAxis(0);
			return;
		}
		_transDir.set(0, 0, 0);
		Node::sv(&_transDir, axis, 1);
	}
	_axisButton->setText(_axisNames[axis].c_str());
	_axis = axis;
}

