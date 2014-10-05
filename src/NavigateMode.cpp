#include "T4TApp.h"

T4TApp::NavigateMode::NavigateMode() 
  : T4TApp::Mode::Mode("mode_Navigate", "res/common/navigate.form") {
	_doSelect = false;
	_subModes.push_back("Translating");
	_subModes.push_back("Rotating");
	_subModes.push_back("Zooming");
}

void T4TApp::NavigateMode::setSubMode(short mode) {
	setCameraMode(mode);
}

bool T4TApp::NavigateMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	Mode::touchEvent(evt, x, y, contactIndex);
	
	//if(_rotate) app->getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", evt, x, y, contactIndex);
}

void T4TApp::NavigateMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
	Mode::controlEvent(control, evt);
}
