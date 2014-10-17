#include "T4TApp.h"

T4TApp::ProjectComponent::ProjectComponent(const char* id) : T4TApp::Mode::Mode(id) {

	_currentElement = 0;
	_typeCount = -1;

	_sceneFile = "res/common/scene.gpb";
}

void T4TApp::ProjectComponent::controlEvent(Control *control, EventType evt) {
	const char *controlID = control->getId();
	if(strncmp(controlID, "comp_", 5) != 0) return;
	Element *el = _elements[_currentElement];
	MyNode *node = app->duplicateModelNode(controlID+5, el->_isStatic);
	std::stringstream ss;
	ss << _nodeId << "_" << el->_name;
	const std::string nodeID = ss.str();
	node->setId(nodeID.c_str());
	el->_node = node;
	_rootNode->addChild(node);
	placeElement();
	node->addCollisionObject();
	node->getCollisionObject()->setEnabled(false);
	finishElement();
	app->_componentMenu->setVisible(false);
}

bool T4TApp::ProjectComponent::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	switch(evt)
	{
		case Touch::TOUCH_PRESS:
			if((this->*_elements[_currentElement]->_callback)(evt, x, y)) {
				_currentElement++;
				if(_currentElement >= _elements.size()) {
					finishComponent();
				} else {
					app->_componentMenu->setVisible(true);
				}
			}
			break;
	}
}

T4TApp::ProjectComponent::Element* T4TApp::ProjectComponent::getEl() {
	return _elements[_currentElement];
}

MyNode* T4TApp::ProjectComponent::getNode(short n) {
	if(n < 0) n = _currentElement;
	return _elements[n]->_node;
}

void T4TApp::ProjectComponent::finishComponent() {
	app->_scene->addNode(_rootNode);
	setActive(false);
}

void T4TApp::ProjectComponent::addElement(const char *name, T4TApp::ProjectComponent::TouchCallback touchCallback, bool isStatic) {
	Element *el = new Element();
	el->_name = name;
	el->_callback = touchCallback;
	el->_isStatic = isStatic;
	el->_node = NULL;
	_elements.push_back(el);
}

void T4TApp::ProjectComponent::setActive(bool active) {
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
		app->setActiveScene(this->_scene);
		app->_componentMenu->setState(Control::FOCUS);
		app->removeListener(app->_componentMenu, app);
		app->addListener(app->_componentMenu, this);
	}else {
		releaseScene();
		app->showScene();
		app->_componentMenu->setState(Control::NORMAL);
		app->removeListener(app->_componentMenu, this);
		app->addListener(app->_componentMenu, app);
	}
}

void T4TApp::ProjectComponent::loadScene() {
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
}

void T4TApp::ProjectComponent::releaseScene() {
	SAFE_RELEASE(_scene);
}

