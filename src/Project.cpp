#include "T4TApp.h"
#include "Modes.h"
#include "MyNode.h"

Project::Project(const char* id) : Mode::Mode(id) {

	_typeCount = -1;
	_scene = Scene::load("res/common/scene.gpb");
	_camera = _scene->getActiveCamera();
    _camera->setAspectRatio(app->getAspectRatio());

	_rootNode = MyNode::create(_nodeId.c_str());
	_rootNode->_type = "root";
	_scene->addNode(_rootNode);

	_currentElement = 0;
	_moveMode = 0;
}

void Project::setupMenu() {
	//add a button for each element to choose its item and edit it
	short i, n = _elements.size();
	_elementContainer = Container::create("elements", app->_theme->getStyle("hiddenContainer"), Layout::LAYOUT_VERTICAL);
	_elementContainer->setAutoWidth(true);
	_elementContainer->setHeight(n * 50.0f);
	for(i = 0; i < n; i++) {
		Button *button = app->addButton <Button> (_elementContainer, _elements[i]->_name.c_str(), _elements[i]->_name.c_str());
	}
	_controls->addControl(_elementContainer);
}

void Project::controlEvent(Control *control, EventType evt) {
	Mode::controlEvent(control, evt);
	const char *id = control->getId();
	
	if(_elementContainer->getControl(id) == control) {
		for(short i = 0; i < _elements.size(); i++) if(_elements[i]->_name.compare(id) == 0) {
			_currentElement = i;
			break;
		}
	} else if(strncmp(id, "comp_", 5) == 0) {
		getEl()->setNode(id+5);
	} else if(strcmp(id, "translate") == 0) {
		_moveMode = 0;
	} else if(strlen(id) == 7 && strncmp(id, "rotate", 6) == 0) {
		_moveMode = 1;
		_moveAxis = (short)(id[6] - 88); //char 7 is 'X', 'Y', or 'Z'
		app->setNavMode(-1);
	} else if(strcmp(id, "finishElement") == 0) {
		promptNextElement();
	}
}

bool Project::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	if(app->_navMode >= 0) return false;
	if(_touchNode != NULL && _touchNode->_element != NULL && _touchNode->_element->_project == this) {
		_touchNode->_element->touchEvent(evt, x, y, contactIndex);
	}
}

Project::Element* Project::getEl(short n) {
	if(n < 0) n = _currentElement;
	return _elements[n].get();
}

MyNode* Project::getNode(short n) {
	if(n < 0) n = _currentElement;
	if(_elements[n]->_nodes.empty()) return NULL;
	return _elements[n]->_nodes.back().get();
}

void Project::finish() {
	for(short i = 0; i < _elements.size(); i++) {
		MyNode *node = getNode(i);
		if(_elements[i]->_static) { //we didn't make it static before now, to allow user to adjust position
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

void Project::addElement(Element *element) {
	_elements.push_back(std::shared_ptr<Element>(element));
}

void Project::setActive(bool active) {
	Mode::setActive(active);
	app->_componentMenu->setVisible(active);
	_inSequence = active;
	if(active) {
		//determine the count of this component type based on the highest index for this element in the scene or in saved files
		_typeCount = 0;
		do {
			std::stringstream ss;
			ss << _id << ++_typeCount;
			_nodeId = ss.str();
		} while(app->_scene->findNode(_nodeId.c_str()) != NULL
			|| FileSystem::fileExists(("res/common/" + _nodeId + ".node").c_str()));
		app->setActiveScene(_scene);
		app->_componentMenu->setState(Control::FOCUS);
		app->removeListener(app->_componentMenu, app);
		app->addListener(app->_componentMenu, this);
		_controls->setVisible(false);
		_controls->getControl("finishElement")->setEnabled(true);
		app->getPhysicsController()->setGravity(Vector3::zero());
	}else {
		app->showScene();
		app->_componentMenu->setState(Control::NORMAL);
		app->removeListener(app->_componentMenu, this);
		app->addListener(app->_componentMenu, app);
		app->getPhysicsController()->setGravity(app->_gravity);
	}
}

void Project::promptNextElement() {
	if(_currentElement < _elements.size()) _currentElement++;
	else _inSequence = false;
	if(!_inSequence) return;
	_controls->setVisible(false);
	app->_componentMenu->setVisible(true);
}

Project::Element::Element(Project *project, const char *name, Element *parent)
  : _project(project), _name(name), _currentNodeId(NULL) {
	setParent(parent);
	_plane.set(Vector3::unitX(), 0); //usually keep things symmetric wrt yz-plane
	setMovable(false, false, false, -1);
	setRotable(false, false, false);
	for(short i = 0; i < 3; i++) setLimits(i, -MyNode::inf(), MyNode::inf());
}

void Project::Element::setParent(Element *parent) {
	_parent = parent;
	parent->addChild(this);
}

void Project::Element::addChild(Element *element) {
	if(std::find(_children.begin(), _children.end(), element) == _children.end()) {
		_children.push_back(element);
	}
}

void Project::Element::setMovable(bool x, bool y, bool z, short ref) {
	_movable[0] = x;
	_movable[1] = y;
	_movable[2] = z;
	_moveRef = ref;
}

void Project::Element::setRotable(bool x, bool y, bool z) {
	_rotable[0] = x;
	_rotable[1] = y;
	_rotable[2] = z;
}

void Project::Element::setLimits(short axis, float lower, float upper) {
	_limits[axis][0] = lower;
	_limits[axis][1] = upper;
}

void Project::Element::setPlane(const Plane &plane) {
	_plane = plane;
}

void Project::Element::applyLimits(Vector3 &translation) {
	short i;
	for(i = 0; i < 3; i++) {
		if(!_movable[i]) MyNode::sv(translation, i, 0);
		else {
			Vector3 ref = translation, delta = Vector3::zero();
			if(_moveRef >= 0) {
				ref = translation - _project->getNode(_moveRef)->getTranslationWorld();
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

void Project::Element::setNode(const char *id) {
	_currentNodeId = id;
	if(_parent == NULL) { //auto-place this node
		placeNode(Vector3::zero());
	} else { //prompt user to click on parent to place
		
	}
}

void Project::Element::placeNode(const Vector3 &position) {
	MyNode *node = _project->app->duplicateModelNode(_currentNodeId);
	node->_element = this;
	os.str("");
	os << _project->_nodeId << "_" << _name;
	node->setId(os.str().c_str());
	node->setTranslation(position);
	if(_multiple) _nodes.push_back(std::shared_ptr<MyNode>(node));
	else _nodes[0] = std::shared_ptr<MyNode>(node);
	_scene->addNode(node);
	addPhysics();
	_project->promptNextElement();
}

void Project::Element::addPhysics() {
	MyNode *node = _nodes.back();
	node->addPhysics();
	if(_parent) {
		MyNode *parentNode = _parent->getNode();
		if(parentNode) parentNode->addChild(node);
	}
}

MyNode* Project::Element::getNode(short n = 0) {
	return _nodes.size() > n ? _nodes[n] : NULL;
}

bool Project::Element::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	//element is generally positioned wrt its parent => get the touch point on the parent
	if(_parent && _parent->getNode()) {
		_parentTouch.set(evt, x, y, _parent->getNode());
	}
	_planeTouch.set(evt, x, y, _plane)
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			if(_nodeId) { //we have selected an item, now we are placing it on the parent
				
			}
			break;
		} case Touch::TOUCH_MOVE: {
			break;
		} case Touch::TOUCH_RELEASE: {
			break;
		}
	}
}


