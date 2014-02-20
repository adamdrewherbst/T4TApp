#include "T4TApp.h"

T4TApp::RotateMode::RotateMode(T4TApp *app_) 
  : T4TApp::Mode::Mode(app_, "mode_Rotate", NULL) {
	_rotate = false;
}

bool T4TApp::RotateMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	switch(evt) {
		case Touch::TOUCH_PRESS:
			_rotate = true;
			break;
		case Touch::TOUCH_RELEASE:
			_rotate = false;
			break;
	}
	if(_rotate) app->getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", evt, x, y, contactIndex);
}

void T4TApp::RotateMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
}
