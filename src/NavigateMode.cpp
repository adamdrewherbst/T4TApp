#include "T4TApp.h"

NavigateMode::NavigateMode() 
  : Mode::Mode("navigate") {
	_doSelect = false;
}

void NavigateMode::setActive(bool active) {
	Mode::setActive(active);
	if(active) {
		app->setNavMode(0);
		app->_cameraMenu->setPosition(20, 10);
	} else {
		app->_cameraMenu->setPosition(900, 10);
	}
	//don't let the user turn off navigation in this mode
	app->_cameraMenu->getControl("eye")->setEnabled(!active);
}

bool NavigateMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	Mode::touchEvent(evt, x, y, contactIndex);
}

void NavigateMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
	Mode::controlEvent(control, evt);
}
