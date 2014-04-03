#include "T4TApp.h"

T4TApp::ToolMode::ToolMode(T4TApp *app_, const char* id, const char* filename) 
  : T4TApp::Mode::Mode(app_, id, filename) {
	_toolType = id+5;
	_subMode = 0;
	_node = NULL;
	_touching = false;
	_rotate = false;
}

void T4TApp::ToolMode::setActive(bool active) {
	Mode::setActive(active);
	setNode(NULL);
	_touching = false;
}

void T4TApp::ToolMode::setNode(Node *node) {
	_node = node;
	app->getScriptController()->executeFunction<void>("camera_setNode", "s", _node != NULL ? _node->getId() : NULL);
	if(_node != NULL) {
		setAxis(0);
		app->_scene->addNode(_tool);
	} else {
		app->_scene->removeNode(_tool);
	}
}

void T4TApp::ToolMode::setAxis(int axis) {
	//translate the camera to look at the center of the node
	//and face along the <axis> direction in its model space
	float yaw = 0, pitch = 0;
	Vector3 sliceNormal, viewNormal, translation(_node->getTranslationWorld());
	Matrix rotation;
	_node->getRotation(&rotation);
	_tool->setRotation(rotation);
	switch(axis) {
		case 0: //x
			yaw = 0;
			pitch = 0;
			break;
		case 1: //y
			yaw = 0;
			pitch = M_PI/2;
			break;
		case 2: //z
			yaw = M_PI/2;
			pitch = 0;
			break;
	}
	_tool->setTranslation(translation);
	_toolBaseRotation = _tool->getRotation();
	app->getScriptController()->executeFunction<void>("camera_rotateTo", "ff", yaw, pitch);
	setView();
}

void T4TApp::ToolMode::setView() {
	//align the tool to the viewing axis
	Node *camNode = app->_scene->getActiveCamera()->getNode();
	Quaternion rot = camNode->getRotation(), offset;
	Quaternion::createFromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), M_PI/2, &offset);
	rot.multiply(offset);
	_tool->setRotation(rot);
	//make the view plane orthogonal to the viewing axis
	Vector3 cam(camNode->getTranslationWorld()), camToNode(_node->getTranslationWorld() - cam);
	_viewPlane.setNormal(camToNode);
}

bool T4TApp::ToolMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			if(_node == NULL) {
				setNode(app->getMouseNode(x, y));
				if(_node) cout << "going to tool " << _node->getId() << endl;
			}
			else {
				Ray ray;
				app->_activeScene->getActiveCamera()->pickRay(app->getViewport(), x, y, &ray);
				float distance = ray.intersects(_viewPlane);
				if(distance != Ray::INTERSECTS_NONE) {
					_touchStart.set(ray.getOrigin() + distance * ray.getDirection());
					_touching = true;
					_toolBaseRotation = _tool->getRotation();
				}
			}
			break;
		}
		case Touch::TOUCH_MOVE: {
			if(_node == NULL || !_touching) break;
			Ray ray;
			app->_activeScene->getActiveCamera()->pickRay(app->getViewport(), x, y, &ray);
			float distance = ray.intersects(_viewPlane);
			if(distance != Ray::INTERSECTS_NONE) {
				_touchPoint.set(ray.getOrigin() + distance * ray.getDirection());
				switch(_subMode) {
					case 0: //rotate
					{
						//alternative: rotate the tool in the view plane
						/*Vector3 v1(_touchStart - _viewPlaneOrigin), v2(_touchPoint - _viewPlaneOrigin);
						v1.normalize();
						v2.normalize();
						float cosAng = v1.dot(v2);
						v1.cross(v2);
						float sinAng = v1.dot(_viewPlane.getNormal());
						float angle = atan2(sinAng, cosAng);
						Quaternion rotation;
						Quaternion::createFromAxisAngle(_viewPlane.getNormal(), angle, &rotation);
						_tool->setRotation(rotation * _toolBaseRotation);//*/
						break;
					}
					case 1: //translate
						_tool->setTranslation(_node->getTranslationWorld() + _touchPoint-_touchStart);
						break;
				}
			}
			break;
		}
		case Touch::TOUCH_RELEASE:
			_touching = false;
			break;
	}
	if(_touching && _subMode == 0) {
		app->getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", evt, x, y, contactIndex);
		setView();
	}
}

void T4TApp::ToolMode::controlEvent(Control *control, Control::Listener::EventType evt) {
	
	const char *controlID = control->getId();
	cout << "tool mode prelim clicked " << controlID << endl;
	if(strncmp(_toolType.c_str(), controlID, _toolType.length()) != 0) return;
	cout << "tool mode clicked " << controlID << endl;
	controlID = controlID + _toolType.length()+1;

	if(strcmp(controlID, "axis") == 0 && _node != NULL) {
		const char *_axes[3] = {"X", "Y", "Z"};
		for(int i = 0; i < 3; i++) {
			if(strcmp(((Button*)control)->getText(), _axes[i]) == 0) {
				setAxis(i);
				((Button*)control)->setText(_axes[(i+1)%3]);
				cout << "set axis to " << _axes[i] << endl;
				break;
			}
		}
	}
	else if(strcmp(controlID, "rotate") == 0) _subMode = 0;
	else if(strcmp(controlID, "translate") == 0) _subMode = 1;
	else if(strcmp(controlID, "doTool") == 0) {
		toolNode();
		setActive(false);
	}
}

bool T4TApp::ToolMode::toolNode() {}
