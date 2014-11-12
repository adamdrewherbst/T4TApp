#include "T4TApp.h"
#include "Project.h"
#include "MyNode.h"

Project::Project(const char* id) : Mode::Mode(id) {

	_typeCount = -1;
	_scene = Scene::load("res/common/scene.gpb");
	_camera = _scene->getActiveCamera();
    _camera->setAspectRatio(app->getAspectRatio());

	_rootNode = MyNode::create(_id.c_str());
	_rootNode->_type = "root";

	_currentElement = 0;
	_moveMode = 0;

	_subModes.push_back("build");
	_subModes.push_back("test");
}

void Project::setupMenu() {
	//add a button for each element to choose its item and edit it
	short i, n = _elements.size();
	_controls->setHeight(_controls->getHeight() + n*50.0f + 50.0f);
	_elementContainer = Container::create("elements", app->_theme->getStyle("hiddenContainer"), Layout::LAYOUT_VERTICAL);
	_elementContainer->setAutoWidth(true);
	_elementContainer->setHeight(n * 50.0f);
	for(i = 0; i < n; i++) {
		Button *button = app->addButton <Button> (_elementContainer, _elements[i]->_name.c_str());
	}
	_controls->addControl(_elementContainer);

	//add a button for each action that any element has - we will enable them on the fly for the selected element
	_actionContainer = Container::create("actions", app->_theme->getStyle("hiddenContainer"), Layout::LAYOUT_FLOW);
	short numActions, j;
	for(i = 0; i < n; i++) {
		numActions = _elements[i]->_actions.size();
		for(j = 0; j < numActions; j++) {
			const char *action = _elements[i]->_actions[j].c_str();
			if(_actionContainer->getControl(action) != NULL) continue;
			ImageControl *button = app->addButton <ImageControl> (_actionContainer, action, action, "imageSquare");
			button->setImage(MyNode::concat(3, "res/png/", action, ".png"));
			button->setSize(50.0f, 50.0f);
		}
	}
	_actionFilter = new MenuFilter(_actionContainer);
}

void Project::controlEvent(Control *control, EventType evt) {
	Mode::controlEvent(control, evt);
	const char *id = control->getId();
	cout << "project control " << id << endl;

	if(_elementContainer->getControl(id) == control) {
		for(short i = 0; i < _elements.size(); i++) if(_elements[i]->_name.compare(id) == 0) {
			setCurrentElement(i);
			break;
		}
	} else if(strncmp(id, "comp_", 5) == 0) {
		app->_componentMenu->setVisible(false);
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
		//see if we are placing a node on its parent
		Element *current = getEl();
		if(evt == Touch::TOUCH_PRESS && current && current->_parent == _touchNode->_element && current->_currentNodeId) {
			current->addNode(_touchPoint);
		}
		//otherwise just trigger whatever node we clicked
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

Project::Element* Project::addElement(Element *element) {
	_elements.push_back(std::shared_ptr<Element>(element));
	return element;
}

void Project::addPhysics() {
	short i, n = _elements.size();
	for(i = 0; i < n; i++) {
		_elements[i]->addPhysics();
	}
}

void Project::setActive(bool active) {
	if(active) {
		app->cameraPush();
		app->setActiveScene(_scene);
	} else {
		app->cameraPop();
		app->showScene();
	}
	Mode::setActive(active);
	_currentElement = -1;
	if(active) {
		//determine the count of this component type based on the highest index for this element in the scene or in saved files
		_typeCount = 0;
		do {
			std::stringstream ss;
			ss << _id << ++_typeCount;
			_nodeId = ss.str();
		} while(app->_scene->findNode(_nodeId.c_str()) != NULL
			|| FileSystem::fileExists(("res/common/" + _nodeId + ".node").c_str()));
		_rootNode->setId(_nodeId.c_str());
		_scene->addNode(_rootNode);
		app->_ground->setVisible(false);
		app->_componentMenu->setState(Control::FOCUS);
		app->filterItemMenu();
		app->removeListener(app->_componentMenu, app);
		app->addListener(app->_componentMenu, this);
		Control *finish = _controls->getControl("finishElement");
		if(finish) finish->setEnabled(true);
		app->getPhysicsController()->setGravity(Vector3::zero());
		_inSequence = true;
		promptNextElement();
	}else {
		app->_componentMenu->setState(Control::NORMAL);
		app->filterItemMenu();
		app->removeListener(app->_componentMenu, this);
		app->addListener(app->_componentMenu, app);
		app->getPhysicsController()->setGravity(app->_gravity);
	}
}

bool Project::setSubMode(short mode) {
	bool building = _subMode == 0, changed = Mode::setSubMode(mode);
	if(building) {
		if(changed) _rootNode->setRest();
	} else _rootNode->placeRest();
	switch(_subMode) {
		case 0: { //build
			app->setCameraEye(30, -M_PI/3, M_PI/12);
			break;
		} case 1: { //test
			app->setCameraEye(40, 0, M_PI/12);
			break;
		}
	}
	return changed;
}

void Project::setCurrentElement(short n) {
	_currentElement = n;
	if(_currentElement >= 0) {
		std::vector<std::string> &actions = getEl()->_actions;
		_actionContainer->filterAll(true);
		for(short i = 0; i < actions.size(); i++) {
			_actionContainer->filter(actions[i].c_str(), false);
		}
	}
}

void Project::promptNextElement() {
	if(_currentElement < (short)_elements.size()-1) setCurrentElement(_currentElement+1);
	else _inSequence = false;
	if(!_inSequence) return;
	app->promptItem(getEl()->_filter);
}

Project::Element::Element(Project *project, const char *id, const char *name, Element *parent)
  : _project(project), _id(id), _name(name), _numNodes(1), _currentNodeId(NULL), _multiple(false) {
	app = (T4TApp*) Game::getInstance();
  	if(name == NULL) _name = _id;
	setParent(parent);
	_plane.set(Vector3::unitX(), 0); //usually keep things symmetric wrt yz-plane
	setMovable(false, false, false, -1);
	setRotable(false, false, false);
	for(short i = 0; i < 3; i++) setLimits(i, -MyNode::inf(), MyNode::inf());
}

void Project::Element::setParent(Element *parent) {
	_parent = parent;
	if(parent) parent->addChild(this);
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

bool Project::Element::setNode(const char *id) {
	_currentNodeId = id;
	if(_parent == NULL) { //auto-place this node
		addNode(Vector3::zero());
		return true;
	} else { //prompt user to click on parent to place
		
	}
	return false;
}

void Project::Element::addNode(const Vector3 &position) {
	bool append = _multiple || _nodes.empty();
	for(short i = 0; i < _numNodes; i++) {
		MyNode *node = app->duplicateModelNode(_currentNodeId);
		node->_element = this;
		std::ostringstream os;
		short count = 0;
		if(_numNodes > 1 || _multiple) do {
			os.str("");
			os << _project->_nodeId << "_" << _id << ++count;
		} while (_project->_scene->findNode(os.str().c_str()) != NULL);
		else os << _project->_nodeId << "_" << _id;
		node->setId(os.str().c_str());
		if(append) _nodes.push_back(std::shared_ptr<MyNode>(node));
		else _nodes[i] = std::shared_ptr<MyNode>(node);
		placeNode(position, i);
		addPhysics(i);
	}
	_currentNodeId = NULL;
	_project->promptNextElement();
}

void Project::Element::placeNode(const Vector3 &position, short n) {
	MyNode *node = _nodes[n].get(), *parent = _parent ? _parent->getNode() : _project->_rootNode;
	node->setTranslation(position);
	if(parent) parent->addChild(node);
}

void Project::Element::addPhysics(short n) {
	short i, numNodes = _nodes.size();
	for(short i = 0; i < numNodes; i++) {
		if(n < 0 || i == n) {
			MyNode *node = _nodes[i].get();
			node->addPhysics();
		}
	}
}

MyNode* Project::Element::getNode(short n) {
	return _nodes.size() > n ? _nodes[n].get() : NULL;
}

bool Project::Element::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	//element is generally positioned wrt its parent => get the touch point on the parent
	if(_parent && _parent->getNode()) {
		_parentTouch.set(evt, x, y, _parent->getNode());
	}
	_planeTouch.set(evt, x, y, _plane);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			for(short i = 0; i < _nodes.size(); i++) _nodes[i]->setBase();
			break;
		} case Touch::TOUCH_MOVE: {
			break;
		} case Touch::TOUCH_RELEASE: {
			break;
		}
	}
}


