#include "T4TApp.h"
#include "Modes.h"
#include "MyNode.h"

Mode::Mode(const char* id) : _selectedNode(NULL), _touchNode(NULL), _touching(false), _doSelect(true) {

	app = dynamic_cast<T4TApp*>(Game::getInstance());
	_scene = app->_scene;
	_camera = app->getCamera();
	
	_id = id;
	_style = app->_theme->getStyle("hidden");
	setAutoWidth(true);	
	setAutoHeight(true);
	setConsumeInputEvents(true);

	//add this button to the container it belongs in
	_container = (Container*)app->_stage->getControl(("mode_" + _id).c_str());
	_container->setVisible(false);
	_container->addControl(this);
	
	//load any custom controls this mode includes
	_controls = (Container*)_container->getControl("controls");
	if(_controls != NULL) {
		_subModePanel = (Container*)_controls->getControl("subMode");
	}

	app->removeListener(_container, app);
	app->addListener(_container, this);

	_plane = app->_groundPlane;
	_cameraBase = Camera::createPerspective(_camera->getFieldOfView(), _camera->getAspectRatio(),
	  _camera->getNearPlane(), _camera->getFarPlane());
	Node *cameraNode = Node::create((_id + "_camera").c_str());
	cameraNode->setCamera(_cameraBase);
	_cameraStateBase = new cameraState();
	app->copyCameraState(app->_cameraState, _cameraStateBase);
	setActive(false);
}

void Mode::update() {}

void Mode::draw() {}

void Mode::setActive(bool active) {
	_active = active;
	setSelectedNode(NULL);
	_container->setVisible(active);
	if(active) {
		setSubMode(0);
		app->setNavMode(-1);
	} else {
	}
}

bool Mode::setSubMode(short mode) {
	if(_subModes.empty()) return false;
	short newMode = mode % _subModes.size();
	bool changed = newMode != _subMode;
	_subMode = newMode;
	return changed;
}

bool Mode::setSelectedNode(MyNode *node, Vector3 point) {
	bool changed = _selectedNode != node;
	_selectedNode = node;
	_selectPoint = point;
	return changed;
}

void Mode::placeCamera() {
	Vector2 delta(_mousePix - _touchPix);
	float radius = _cameraStateBase->radius, theta = _cameraStateBase->theta, phi = _cameraStateBase->phi;
	switch(app->_navMode) {
		case 0: { //rotate
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
			app->setCameraTarget(_cameraStateBase->target - (_newPoint - _touchPoint));
			break;
		} case 2: { //zoom
			float deltaRadius = -delta.y / 40.0f;
			radius = fmin(120.0f, fmax(3.0f, radius + deltaRadius));
			app->setCameraZoom(radius);
			break;
		}
	}
}

bool Mode::isSelecting() {
	return _doSelect && app->_navMode < 0;
}

bool Mode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	_x = (int)(x + getX() + _container->getX() + app->_stage->getX());
	_y = (int)(y + getY() + _container->getY() + app->_stage->getY());
	_touchPt.set(evt, _x, _y);
	_mousePix.set(_x, _y);
	_camera = _scene->getActiveCamera();
	_camera->pickRay(app->getViewport(), _x, _y, &_ray);
	float distance = _ray.intersects(_plane);
	if(distance != Ray::INTERSECTS_NONE) _mousePoint.set(_ray.getOrigin() + distance * _ray.getDirection());
	switch(evt) {
		case Touch::TOUCH_PRESS:
			cout << "mode ray: " << app->pv(_ray.getOrigin()) << " => " << app->pv(_ray.getDirection()) << endl;
			_touching = true;
			_touchPix = _mousePix;
			_touchNode = NULL;
			if(isSelecting()) {
				PhysicsController::HitResult hitResult;
				if(!app->getPhysicsController()->rayTest(_ray, _camera->getFarPlane(), &hitResult, app->_hitFilter)) break;
				MyNode *node = dynamic_cast<MyNode*>(hitResult.object->getNode());
				if(!node || node->getCollisionObject() == NULL || app->auxNode(node)) break;
				_touchNode = node;
				_touchNode->setBase();
				_touchPoint.set(hitResult.point);
				setSelectedNode(_touchNode, _touchPoint);
				cout << "selected: " << node->getId() << " at " << app->pv(_touchPoint) << endl;
			} else {
				_touchPoint.set(_mousePoint);
				_cameraBase->getNode()->set(*_camera->getNode());
				_viewportBase = app->getViewport();
				app->copyCameraState(app->_cameraState, _cameraStateBase);
				cout << "touched: camera at " << app->pcam(_cameraStateBase) << endl;
			}
			break;
		case Touch::TOUCH_MOVE:
			if(_touching && app->_navMode >= 0) {
				placeCamera();
			}
			break;
		case Touch::TOUCH_RELEASE:
			_touching = false;
			_touchNode = NULL;
			break;
	}
	return true;
}

void Mode::controlEvent(Control *control, EventType evt) {
	const char *id = control->getId();
	if(control != this)	app->setNavMode(-1);

	if(_subModePanel && _subModePanel->getControl(id) == control) {
		cout << "clicked submode " << id << endl;
		for(short i = 0; i < _subModes.size(); i++) {
			if(_subModes[i].compare(id) == 0) {
				cout << "matches " << i << endl;
				setSubMode(i);
			}
		}
	}
}


TouchPoint::TouchPoint() {
	app = (T4TApp*) Game::getInstance();
	_touching = false;
	_hit = false;
	_point[Touch::TOUCH_PRESS] = Vector3::zero();
	_point[Touch::TOUCH_MOVE] = Vector3::zero();
	_point[Touch::TOUCH_RELEASE] = Vector3::zero();
	_pix[Touch::TOUCH_PRESS] = Vector2::zero();
	_pix[Touch::TOUCH_MOVE] = Vector2::zero();
	_pix[Touch::TOUCH_RELEASE] = Vector2::zero();
}

void TouchPoint::set(Touch::TouchEvent evt, int x, int y) {
	if(evt == Touch::TOUCH_PRESS) _touching = true;
	else if(evt == Touch::TOUCH_RELEASE) _touching = true;
	_pix[evt].set(x, y);
}

void TouchPoint::set(Touch::TouchEvent evt, int x, int y, MyNode *node) {
	set(evt, x, y);
	Camera *camera = app->getCamera();
	Ray ray;
	camera->pickRay(app->getViewport(), x, y, &ray);
	PhysicsController::HitResult result;
	app->_nodeFilter->setNode(node);
	_hit = app->getPhysicsController()->rayTest(ray, camera->getFarPlane(), &result, app->_nodeFilter);
	if(_hit) _point[evt] = result.point;
}

void TouchPoint::set(Touch::TouchEvent evt, int x, int y, const Plane &plane) {
	set(evt, x, y);
	Camera *camera = app->getCamera();
	Ray ray;
	camera->pickRay(app->getViewport(), x, y, &ray);
	float distance = ray.intersects(plane);
	if(distance != Ray::INTERSECTS_NONE) _point[evt] = ray.getOrigin() + ray.getDirection() * distance;
}

Vector3 TouchPoint::getPoint(Touch::TouchEvent evt) {
	return _point[evt];
}

Vector2 TouchPoint::getPix(Touch::TouchEvent evt) {
	return _pix[evt];
}

Vector3 TouchPoint::delta() {
	return _point[Touch::TOUCH_MOVE] - _point[Touch::TOUCH_PRESS];
}

Vector2 TouchPoint::deltaPix() {
	return _pix[Touch::TOUCH_MOVE] - _pix[Touch::TOUCH_PRESS];
}


