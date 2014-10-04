#include "T4TApp.h"

T4TApp::ProjectComponent::ProjectComponent(T4TApp *app_, const char* filename, const char* id, Theme::Style* buttonStyle, Theme::Style* formStyle) : app(app_) {

	_currentElement = 0;
	_typeCount = -1;

	_sceneFile = filename;

	//create the form to hold this button
	_container = Form::create(app->concat(2, "container_", id), formStyle, Layout::LAYOUT_VERTICAL);
	_container->setPosition(app->_sideMenu->getX(), 0.0f);
	_container->setWidth(app->getWidth() - _container->getX());
	_container->setAutoHeight(true);
	_container->setScroll(Container::SCROLL_VERTICAL);
	_container->setConsumeInputEvents(true);
	_container->setVisible(false);
	
	_id = id;
	_style = buttonStyle;
	setAutoWidth(true);
	setAutoHeight(true);
	setConsumeInputEvents(true);
	_container->addControl(this);
	app->_mainMenu->addControl(_container);
}

void T4TApp::ProjectComponent::controlEvent(Control *control, EventType evt) {
	const char *controlID = control->getId();
	if(strncmp(controlID, "comp_", 5) != 0) return;
	MyNode *node = app->duplicateModelNode(controlID+5, _isStatic[_currentElement]);
	std::stringstream ss;
	ss << _id << _typeCount << "_" << _elementNames[_currentElement];
	const std::string nodeID = ss.str();
	node->setId(nodeID.c_str());
	_rootNode->addChild(node);
	_allNodes.push_back(node);
	placeElement(node);
	node->addCollisionObject();
	node->getCollisionObject()->setEnabled(false);
	finishElement(node);
	app->_componentMenu->setVisible(false);
	_container->setVisible(true);
	addListener(this, Control::Listener::CLICK);
}

bool T4TApp::ProjectComponent::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	switch(evt)
	{
		case Touch::TOUCH_PRESS:
			if((this->*_elementCallbacks[_currentElement])(evt, x, y)) {
				_currentElement++;
				removeListener(this);
				_container->setVisible(false);
				if(_currentElement >= _elementNames.size()) {
					finishComponent();
				} else {
					app->_componentMenu->setVisible(true);
				}
			}
			break;
	}
}

void T4TApp::ProjectComponent::finishComponent() {
	//can't deep copy nodes to the app scene, so must write them out and read them in
	_rootNode->writeData();
	setActive(false);
	app->loadNode(_rootNode->getId());
}

void T4TApp::ProjectComponent::addElement(const char *name, T4TApp::ProjectComponent::TouchCallback touchCallback, bool isStatic) {
	_elementNames.push_back(std::string(name));
	_elementCallbacks.push_back(touchCallback);
	_isStatic.push_back(isStatic);
}

void T4TApp::ProjectComponent::setActive(bool active) {
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
	_rootNode->data->type = "root";
	_scene->addNode(_rootNode);
}

void T4TApp::ProjectComponent::releaseScene() {
	//must explicitly get rid of physics objects/constraints
	for(int i = 0; i < _allNodes.size(); i++) {
		_allNodes[i]->removePhysics();
	}
	SAFE_RELEASE(_scene);
}

