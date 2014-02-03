#include "T4TApp.h"

T4TApp::ProjectComponent::ProjectComponent(T4TApp *app_, const char* filename, const char* id, Theme::Style* buttonStyle, Theme::Style* formStyle) : app(app_) {

	_currentElement = 0;

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
	Node *node = app->duplicateModelNode(controlID+5, _isStatic[_currentElement]);
	node->getCollisionObject()->setEnabled(true);
	node->setId(_elementNames[_currentElement].c_str());
	_scene->addNode(node);
	_allNodes.push_back(node);
	placeElement(node);
cout << "added element " << _elementNames[_currentElement] << " static=" << _isStatic[_currentElement] << endl;
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
	setActive(false);
}

void T4TApp::ProjectComponent::addElement(const char *name, T4TApp::ProjectComponent::TouchCallback touchCallback, bool isStatic) {
	_elementNames.push_back(std::string(name));
	_elementCallbacks.push_back(touchCallback);
	_isStatic.push_back(isStatic);
}

void T4TApp::ProjectComponent::setActive(bool active) {
	app->_componentMenu->setVisible(active);
	if(active) {
		app->releaseScene();
		loadScene();
		app->setActiveScene(this->_scene);
		app->_componentMenu->setState(Control::FOCUS);
		app->_mainMenu->addListener(this, Control::Listener::CLICK);
		app->_componentMenu->addListener(this, Control::Listener::CLICK);
	}else {
		releaseScene();
		app->loadScene();
		app->_componentMenu->setState(Control::NORMAL);
		app->_mainMenu->removeListener(this);
		app->_componentMenu->removeListener(this);
	}
	const std::vector<Control*> buttons = app->_componentMenu->getControls();
	for(std::vector<Control*>::size_type i = 0; i != buttons.size(); i++) {
		if(active) buttons[i]->addListener(this, Control::Listener::CLICK);
		else buttons[i]->removeListener(this);
	}
}

void T4TApp::ProjectComponent::loadScene() {
	_scene = Scene::load(_sceneFile.c_str());
	_scene->setId("vehicle");
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
}

void T4TApp::ProjectComponent::releaseScene() {
	SAFE_RELEASE(_scene);
}

