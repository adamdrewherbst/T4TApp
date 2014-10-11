#include "T4TApp.h"

T4TApp::Mode::Mode(const char* id) {

	app = dynamic_cast<T4TApp*>(Game::getInstance());
	_scene = app->_scene;
	_camera = _scene->getActiveCamera();
	
	Theme::Style *hidden = Theme::create("res/common/default.theme")->getStyle("hidden");
	
	_id = id;
	_style = hidden;
	setAutoWidth(true);	
	setAutoHeight(true);
	setConsumeInputEvents(true);

	//add this button the container it belongs in
	_container = (Container*)app->_stage->getControl(("mode_" + _id).c_str());
	_container->setVisible(false);
	_container->addControl(this);
	
	//load any custom controls this mode includes
	_controls = (Container*)_container->getControl("controls");
	if(_controls != NULL) {
		_subModePanel = (Container*)_controls->getControl("subMode");
		_cameraModeButton = (Button*)_controls->getControl("cameraMode");
	}

	_plane = app->_groundPlane;
	_doSelect = true;
	_cameraMode = -1;
	_cameraBase = Camera::createPerspective(_camera->getFieldOfView(), _camera->getAspectRatio(),
	  _camera->getNearPlane(), _camera->getFarPlane());
	Node *cameraNode = Node::create((_id + "_camera").c_str());
	cameraNode->setCamera(_cameraBase);
	setActive(false);
}

void T4TApp::Mode::draw() {}

void T4TApp::Mode::setActive(bool active) {
	_active = active;
	setSelectedNode(NULL);
	_container->setVisible(active);
	if(active) {
		app->addListener(_container, this);
		setSubMode(0);
	} else {
		app->removeListener(_container, this);
	}
}

void T4TApp::Mode::setSubMode(short mode) {
	if(_subModes.empty()) return;
	_subMode = mode % _subModes.size();
}

void T4TApp::Mode::setSelectedNode(MyNode *node, Vector3 point) {
	_selectedNode = node;
	_selectPoint = point;
}

void T4TApp::Mode::setCameraMode(short mode) {
	_cameraMode = mode;
	if(_cameraModeButton) {
		if(mode == 0) _cameraModeButton->setText("Translating");
		else if(mode == 1) _cameraModeButton->setText("Rotating");
		else if(mode == 2) _cameraModeButton->setText("Zooming");
	}
}

void T4TApp::Mode::placeCamera() {
	Vector2 delta(_mousePix - _touchPix);
	switch(_cameraMode) {
		case 0: { //rotate
			Vector3 eye(_cameraBase->getNode()->getTranslationWorld() - _cameraCenterBase);
			float phi = atan2(eye.y, sqrt(eye.x*eye.x + eye.z*eye.z)), theta = atan2(eye.z, eye.x), radius = eye.length();
			float deltaPhi = delta.y * M_PI / 400.0f, deltaTheta = delta.x * M_PI / 400.0f;
			phi = fmin(89.9f * M_PI/180, fmax(-89.9f * M_PI/180, phi + deltaPhi));
			theta += deltaTheta;
			app->setCameraEye(radius, theta, phi);
			break;
		} case 1: { //translate
			Ray ray;
			_cameraBase->pickRay(_viewportBase, _mousePix.x, _mousePix.y, &ray);
			float distance = ray.intersects(_plane);
			Vector3 _newPoint(ray.getOrigin() + distance * ray.getDirection());
			app->setCameraTarget(_cameraCenterBase - (_newPoint - _touchPoint));
			break;
		} case 2: { //zoom
			Vector3 eye(_cameraBase->getNode()->getTranslationWorld() - _cameraCenterBase);
			float deltaRadius = -delta.y / 40.0f;
			float radius = fmin(120.0f, fmax(3.0f, eye.length() + deltaRadius));
			app->setCameraZoom(radius);
			break;
		}
	}
}

bool T4TApp::Mode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	_x = (int)(x + getX() + _container->getX() + app->_stage->getX());
	_y = (int)(y + getY() + _container->getY() + app->_stage->getY());
	_mousePix.set(_x, _y);
	_camera = _scene->getActiveCamera();
	_camera->pickRay(app->getViewport(), _x, _y, &_ray);
	if(_cameraMode >= 0) {
		float distance = _ray.intersects(_plane);
		_mousePoint.set(_ray.getOrigin() + distance * _ray.getDirection());
	}
	switch(evt) {
		case Touch::TOUCH_PRESS:
			cout << "mode ray: " << app->pv(_ray.getOrigin()) << " => " << app->pv(_ray.getDirection()) << endl;
			_touching = true;
			_touchPix = _mousePix;
			_touchNode = NULL;
			if(_cameraMode >= 0) {
				_touchPoint.set(_mousePoint);
				_cameraBase->getNode()->set(*_camera->getNode());
				_cameraCenterBase = app->_cameraCenter;
				_viewportBase = app->getViewport();
			} else if(_doSelect) {
				PhysicsController::HitResult hitResult;
				if(!app->getPhysicsController()->rayTest(_ray, _camera->getFarPlane(), &hitResult)) break;
				MyNode *node = dynamic_cast<MyNode*>(hitResult.object->getNode());
				if(!node || node->getCollisionObject() == NULL || app->auxNode(node)) break;
				_touchNode = node;
				_touchPoint.set(hitResult.point);
				setSelectedNode(_touchNode, _touchPoint);
				cout << "selected: " << node->getId() << " at " << app->pv(_touchPoint) << endl;
			}
			break;
		case Touch::TOUCH_MOVE:
			if(_touching && _cameraMode >= 0) {
				placeCamera();
			}
			break;
		case Touch::TOUCH_RELEASE:
			_touching = false;
			break;
	}
}

void T4TApp::Mode::controlEvent(Control *control, EventType evt) {
	const char *id = control->getId();
	if(_subModePanel && _subModePanel->getControl(id) == control) { //switching to next submode
		for(short i = 0; i < _subModes.size(); i++) {
			if(_subModes[i].compare(id) == 0) setSubMode(i);
		}
	}
}

