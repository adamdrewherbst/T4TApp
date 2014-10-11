#include "T4TApp.h"

T4TApp::NavigateMode::NavigateMode() 
  : T4TApp::Mode::Mode("navigate") {
	_doSelect = false;
	_subModes.push_back("Rotating");
	_subModes.push_back("Translating");
	_subModes.push_back("Zooming");
}

void T4TApp::NavigateMode::setSubMode(short mode) {
	Mode::setSubMode(mode);
	setCameraMode(_subMode);
}

bool T4TApp::NavigateMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	Mode::touchEvent(evt, x, y, contactIndex);
}

void T4TApp::NavigateMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
	Mode::controlEvent(control, evt);
	const char *id = control->getId();
	if(strcmp(id, "reset") == 0) {
		app->resetCamera();
	}
}
