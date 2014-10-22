#include "T4TApp.h"

ProjectComponent::ProjectComponent(const char* id) : Mode::Mode(id) {

	_typeCount = -1;
	_sceneFile = "res/common/scene.gpb";

	_currentElement = 0;
	_moveMode = 0;
}

void ProjectComponent::controlEvent(Control *control, EventType evt) {
	Mode::controlEvent(control, evt);
	const char *id = control->getId();
	if(strncmp(id, "comp_", 5) == 0) {
		Element *el = _elements[_currentElement];
		MyNode *node = app->duplicateModelNode(id+5);
		std::stringstream ss;
		ss << _nodeId << "_" << el->_name;
		const std::string nodeID = ss.str();
		node->setId(nodeID.c_str());
		el->_node = node;
		_rootNode->addChild(node);
		placeElement();
		node->addCollisionObject();
		node->getCollisionObject()->asRigidBody()->setGravity(0, 0, 0);
		finishElement();
		app->_componentMenu->setVisible(false);
		_controls->setVisible(true);
	} else if(strcmp(id, "translate") == 0) {
		_moveMode = 0;
	} else if(strlen(id) == 7 && strncmp(id, "rotate", 6) == 0) {
		_moveMode = 1;
		_moveAxis = (short)(id[6] - 88); //char 7 is 'X', 'Y', or 'Z'
		app->setNavMode(-1);
	} else if(strcmp(id, "finishElement") == 0) {
		_currentElement++;
		if(_currentElement >= _elements.size()) finishComponent();
		else app->_componentMenu->setVisible(true);
	}
}

bool ProjectComponent::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	switch(evt)
	{
		case Touch::TOUCH_PRESS:
			if(app->_navMode < 0 && _moveMode >= 0 && _selectedNode != NULL) {
				Vector3 normal = app->getCameraNode()->getForwardVector();
				_plane.set(normal, -normal.dot(_selectedNode->getTranslationWorld()));
				_baseRotation = _selectedNode->getRotation();
				_baseTranslation = _selectedNode->getTranslationWorld();
				float distance = _ray.intersects(_plane);
				_touchPoint = _ray.getOrigin() + _ray.getDirection() * distance;
				_touchPix = _mousePix;
			}
			break;
		case Touch::TOUCH_MOVE:
			if(_touching && app->_navMode < 0 && _moveMode >= 0 && _selectedNode != NULL) {
				short e;
				for(e = 0; e <= _currentElement && _elements[e]->_node != _selectedNode; e++);
				if(e > _currentElement) break;
				Element *el = _elements[e];
				switch(_moveMode) {
					case 0: { //translate
						Vector3 translation = _mousePoint - _touchPoint;
						el->applyLimits(translation);
						_selectedNode->setTranslation(_baseTranslation + translation);
						break;
					} case 1: { //rotate
						if(!el->_rotable[_moveAxis]) break;
						Vector3 axis = MyNode::unitV(_moveAxis);
						Quaternion rotation;
						float angle = (_mousePix.x - _touchPix.x) / 400.0f;
						Quaternion::createFromAxisAngle(axis, angle, &rotation);
						_selectedNode->setRotation(rotation * _baseRotation);
						break;
					}
				}
			}
			break;
	}
}

ProjectComponent::Element* ProjectComponent::getEl() {
	return _elements[_currentElement];
}

MyNode* ProjectComponent::getNode(short n) {
	if(n < 0) n = _currentElement;
	return _elements[n]->_node;
}

void ProjectComponent::finishComponent() {
	for(short i = 0; i < _elements.size(); i++) {
		MyNode *node = getNode(i);
		if(_elements[i]->_isStatic) { //we didn't make it static before now, to allow user to adjust position
			node->removePhysics();
			node->setStatic(true);
			node->addPhysics();
		} else { //add gravity to this node
			node->getCollisionObject()->asRigidBody()->setGravity(app->getPhysicsController()->getGravity());
		}
	}
	app->_scene->addNode(_rootNode);
	setActive(false);
}

void ProjectComponent::addElement(const char *name, ProjectComponent::TouchCallback touchCallback, bool isStatic) {
	Element *el = new Element(this);
	el->_name = name;
	el->_callback = touchCallback;
	el->_isStatic = isStatic;
	el->_node = NULL;
	el->setMovable(false, false, false, -1);
	el->setRotable(false, false, false);
	for(short i = 0; i < 3; i++) el->setLimits(i, -MyNode::inf(), MyNode::inf());
	_elements.push_back(el);
}

void ProjectComponent::setActive(bool active) {
	Mode::setActive(active);
	app->_componentMenu->setVisible(active);
	if(active) {
		//determine the count of this component type based on the highest index for this element in the scene or in saved files
		_typeCount = 0;
		do {
			std::stringstream ss;
			ss << _id << ++_typeCount;
			_nodeId = ss.str();
		} while(app->_scene->findNode(_nodeId.c_str()) != NULL
			|| FileSystem::fileExists(("res/common/" + _nodeId + ".node").c_str()));
		app->hideScene();
		loadScene();
		app->_componentMenu->setState(Control::FOCUS);
		app->removeListener(app->_componentMenu, app);
		app->addListener(app->_componentMenu, this);
		_controls->setVisible(false);
	}else {
		releaseScene();
		app->showScene();
		app->_componentMenu->setState(Control::NORMAL);
		app->removeListener(app->_componentMenu, this);
		app->addListener(app->_componentMenu, app);
	}
}

void ProjectComponent::loadScene() {
	_scene = Scene::load(_sceneFile.c_str());
	_scene->setId("Machine");
    _scene->getActiveCamera()->setAspectRatio(app->getAspectRatio());

	//camera will always be pointing down the z axis from 20 units away
	Node *cameraNode = _scene->getActiveCamera()->getNode();
	Matrix lookAt;
	Vector3 scale, translation;
	Quaternion rotation;
	Matrix::createLookAt(Vector3(30.0f,2.0f,30.0f), Vector3(0.0f,0.0f,0.0f), Vector3(0.0f,1.0f,0.0f), &lookAt);
	lookAt.invert();
	lookAt.decompose(&scale, &rotation, &translation);
	cameraNode->setScale(scale);
	cameraNode->setRotation(rotation);
	cameraNode->setTranslation(translation);
	
	//create the root node for the component - not a physical node, just holds all the pieces
	_rootNode = MyNode::create(_nodeId.c_str());
	_rootNode->_type = "root";
	_scene->addNode(_rootNode);

	app->setActiveScene(_scene);
}

void ProjectComponent::releaseScene() {
	SAFE_RELEASE(_scene);
}

ProjectComponent::Element::Element(ProjectComponent *comp_) : comp(comp_) {}

void ProjectComponent::Element::setMovable(bool x, bool y, bool z, short ref) {
	_movable[0] = x;
	_movable[1] = y;
	_movable[2] = z;
	_moveRef = ref;
}

void ProjectComponent::Element::setRotable(bool x, bool y, bool z) {
	_rotable[0] = x;
	_rotable[1] = y;
	_rotable[2] = z;
}

void ProjectComponent::Element::setLimits(short axis, float lower, float upper) {
	_limits[axis][0] = lower;
	_limits[axis][1] = upper;
}

void ProjectComponent::Element::applyLimits(Vector3 &translation) {
	short i;
	for(i = 0; i < 3; i++) {
		if(!_movable[i]) MyNode::sv(translation, i, 0);
		else {
			Vector3 ref = translation, delta = Vector3::zero();
			if(_moveRef >= 0) {
				ref = translation - comp->_elements[_moveRef]->_node->getTranslationWorld();
				delta = translation - ref;
			}
			float val = MyNode::gv(ref, i);
			if(_limits[i][0] > -MyNode::inf() && val < _limits[i][0]) {
				MyNode::sv(ref, i, _limits[i][0]);
			}
			else if(_limits[i][1] < MyNode::inf() && val > _limits[i][1]) {
				MyNode::sv(ref, i, _limits[i][1]);
			}
			translation = ref + delta;
		}
	}
}

