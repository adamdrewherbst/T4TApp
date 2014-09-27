#include "T4TApp.h"

T4TApp::Mode::Mode(T4TApp *app_, const char* id, const char* filename) : app(app_) {
	//create the form to hold this button
	_container = Form::create("res/common/overlay.form");
	//_container->setId(app->concat(2, "container_", id));
	_container->setPosition(app->_sideMenu->getX(), 0.0f);
	_container->setWidth(app->getWidth() - _container->getX());
	_container->setScroll(Container::SCROLL_VERTICAL);
	_container->setConsumeInputEvents(true);
	_container->setVisible(false);
	
	_id = id;
	_style = _container->getTheme()->getStyle("buttonStyle");
	setAutoWidth(true);
	setAutoHeight(true);
	setConsumeInputEvents(true);
	_container->addControl(this);
	app->_mainMenu->addControl(_container);
	
	//load any custom controls this mode includes
	if(filename != NULL) {
		_controls = Form::create(filename);
		_container->addControl(_controls);
		_subControls = _controls->getControls();
		for(size_t i = 0; i < _subControls.size(); i++)
			_subControls[i]->addListener(this, Control::Listener::CLICK);
	}
	else _controls = NULL;
	
	setActive(false);
}

void T4TApp::Mode::draw() {
	if(_controls != NULL) _controls->draw();
}

void T4TApp::Mode::setActive(bool active) {
	_active = active;
	_container->setVisible(active);
	if(_controls != NULL) {
		for(size_t i = 0; i < _subControls.size(); i++)
			_subControls[i]->setEnabled(active);
	}
	if(active) {
		app->_mainMenu->addListener(this, Control::Listener::CLICK);
	} else {
		app->_mainMenu->removeListener(this);
	}
}

bool T4TApp::Mode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	switch(evt) {
		case Touch::TOUCH_PRESS:
			_touching = true;
			break;
		case Touch::TOUCH_RELEASE:
			_touching = false;
			break;
	}
}
