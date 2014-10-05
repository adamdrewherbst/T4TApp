#include "T4TApp.h"

T4TApp::Mode::Mode(const char* id, const char* filename) {
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
		_subModeButton = (Button*)_controls->getControl("subMode");
		_cameraModeButton = (Button*)_controls->getControl("cameraMode");
		_container->addControl(_controls);
		app->addListener(_controls, this);
	}
	else _controls = NULL;

	_plane = app->_groundPlane;
	_doSelect = true;
	setActive(false);
}

void T4TApp::Mode::draw() {
	if(_controls != NULL) _controls->draw();
}

void T4TApp::Mode::setActive(bool active) {
	_active = active;
	_container->setVisible(active);
	if(active) {
		app->addListener(app->_mainMenu, this);
		setSubMode(0);
	} else {
		app->removeListener(app->_mainMenu, this);
	}
}

void T4TApp::Mode::setSubMode(short mode) {
	_subMode = mode;
	if(_subModeButton) _subModeButton->setText(_subModes[_subMode].c_str());
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
	Camera *camera = app->_scene->getActiveCamera();
	Vector3 eye(_cameraBase - _cameraCenterBase), up;
	float phi = atan2(eye.y, sqrt(eye.x*eye.x + eye.z*eye.z)), theta = atan2(eye.z, eye.x), radius = eye.length();
	switch(_cameraMode) {
		case 0: { //translate
			Ray ray;
			camera->pickRay(_viewportBase, _mousePix.x, _mousePix.y, &ray);
			float distance = ray.intersects(_plane);
			Vector3 _newPoint(ray.getOrigin() + distance * ray.getDirection());
			_cameraCenter = _cameraCenterBase - (_newPoint - _touchPoint);
			break;
		} case 1: { //rotate
			float deltaPhi = delta.y * M_PI / 400.0f, deltaTheta = delta.x * M_PI / 400.0f;
			phi = fmin(M_PI/2, fmax(-M_PI/2, phi + deltaPhi));
			theta += deltaTheta;
			break;
		} case 2: { //zoom
			float deltaRadius = delta.y / 40.0f;
			radius = fmin(3.0f, fmax(120.0f, radius + deltaRadius));
			break;
		}
	}
	eye.set(radius * cos(theta) * cos(phi), radius * sin(phi), radius * cos(theta) * sin(phi));
	eye += _cameraCenter;
	up.set(-cos(theta) * sin(phi), cos(phi), -sin(theta) * sin(phi));
	Matrix cam;
	Matrix::createLookAt(eye, _cameraCenter, up, &cam);
	Vector3 scale, translation; Quaternion rotation;
	cam.decompose(&scale, &rotation, &translation);
	camera->getNode()->set(scale, rotation, translation);
}

bool T4TApp::Mode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	_mousePix.set(x, y);
	Camera* camera = app->_scene->getActiveCamera();
	Ray ray;
	camera->pickRay(app->getViewport(), x, y, &ray);
	if(_cameraMode >= 0) {
		float distance = ray.intersects(_plane);
		_mousePoint.set(ray.getOrigin() + distance * ray.getDirection());
	}
	switch(evt) {
		case Touch::TOUCH_PRESS:
			_touching = true;
			_touchPix = _mousePix;
			_touchNode = NULL;
			if(_cameraMode >= 0) {
				_touchPoint.set(_mousePoint);
				_cameraBase = camera->getNode()->getTranslationWorld();
				_cameraCenterBase = _cameraCenter;
				_viewportBase = app->getViewport();
			} else if(_doSelect) {
				PhysicsController::HitResult hitResult;
				if(!app->getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) break;
				MyNode *node = dynamic_cast<MyNode*>(hitResult.object->getNode());
				if(!node || node->getCollisionObject() == NULL || app->auxNode(node)) break;
				setSelectedNode(node);
				_touchNode = node;
				_touchPoint.set(hitResult.point);
				cout << "selected: " << node->getId() << " at " << app->printVector(_touchPoint) << endl;
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
	if(control && control == _subModeButton) { //switching to next submode
		setSubMode((_subMode + 1) % _subModes.size());
	}
}

