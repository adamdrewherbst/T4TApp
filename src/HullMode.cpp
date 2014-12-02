#include "T4TApp.h"
#include "HullMode.h"
#include "MyNode.h"

HullMode::HullMode() : Mode::Mode("hull") {
	_scene = Scene::load("res/common/scene.gpb");
	_doSelect = false;
	_ctrlPressed = false;
	_shiftPressed = false;
	
	_region = new Selection(this, "region", Vector3(0.0f, 0.0f, 1.0f));
	_chain = new Selection(this, "chain", Vector3(1.0f, 0.0f, 0.0f));
	_hullNode = NULL;
	
	_axisContainer = (Container*) _controls->getControl("axisContainer");
}

void HullMode::setActive(bool active) {
	Mode::setActive(active);
	if(active) {
		app->setCameraEye(30, 0, M_PI/12);
		app->promptItem();
	}
}

bool HullMode::setSubMode(short mode) {
	bool changed = Mode::setSubMode(mode);
	_currentSelection = NULL;
	switch(_subMode) {
		case 0: { //select region
			_currentSelection = _region;
			break;
		} case 1: { //select chain
			_currentSelection = _chain;
			break;
		}
	}
	return changed;
}

void HullMode::controlEvent(Control *control, EventType evt) {
	Mode::controlEvent(control, evt);
	const char *id = control->getId();
	
	if(strcmp(id, "reverseFace") == 0) {
		if(_currentSelection) _currentSelection->reverseFaces();
	} else if(strcmp(id, "makeHull") == 0) {
		makeHulls();
	} else if(_axisContainer->getControl(id) == control) {
		Quaternion rot = Quaternion::identity();
		if(id[0] == 'X') rot = MyNode::getVectorRotation(Vector3::unitX(), Vector3::unitZ());
		else if(id[0] == 'Y') rot = MyNode::getVectorRotation(Vector3::unitY(), Vector3::unitZ());
		_hullNode->setMyRotation(rot);
		updateTransform();
	} else if(strcmp(id, "flipAxis") == 0) {
		Quaternion rot(Vector3::unitY(), M_PI);
		_hullNode->myRotate(rot);
		updateTransform();
	}
}

void HullMode::makeHulls() {
	std::vector<std::vector<short> > faces;
	if(_chain->nf() == 0) {
		faces.push_back(_region->_faces);
	} else {
		
	}
	std::set<short> hullSet;
	std::set<short>::const_iterator it;
	short nh = faces.size(), i;
	_hullNode->_hulls.resize(nh);
	for(i = 0; i < nh; i++) {
		hullSet.clear();
		short n = faces[i].size(), j;
		for(j = 0; j < n; j++) {
			Face &f = _hullNode->_faces[faces[i][j]];
			short m = f.size(), k;
			for(k = 0; k < m; k++) hullSet.insert(f[k]);
		}
		_hullNode->_hulls[i] = new MyNode::ConvexHull(_hullNode);
		MyNode::ConvexHull *hull = _hullNode->_hulls[i];
		for(it = hullSet.begin(); it != hullSet.end(); it++)
			hull->addVertex(_hullNode->_vertices[*it]);
	}
}

void HullMode::updateTransform() {
	_hullNode->updateTransform();
	_hullNode->updateCamera(false);
	_region->_node->set(_hullNode);
	_chain->_node->set(_hullNode);
}

void HullMode::selectItem(const char *id) {
	Mode::selectItem(id);
	if(_hullNode) _scene->removeNode(_hullNode);
	_hullNode = app->duplicateModelNode(id);
	_hullNode->setId(id);
	_hullNode->_color.set(1.0f, 1.0f, 0.0f);
	_hullNode->updateModel(false);
	_scene->addNode(_hullNode);
	updateTransform();
}

bool HullMode::keyEvent(Keyboard::KeyEvent evt, int key) {
	Mode::keyEvent(evt, key);
	if(evt == Keyboard::KEY_CHAR) return true;
	switch(key) {
		case Keyboard::KEY_SHIFT: {
			_shiftPressed = evt == Keyboard::KEY_PRESS;
			break;
		}
		case Keyboard::KEY_CTRL: {
			_ctrlPressed = evt == Keyboard::KEY_PRESS;
		}
	}
	return true;
}

bool HullMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	if(app->_navMode >= 0) return true;
	if(_currentSelection && isTouching()) {
		short face = _hullNode->pix2Face(_x, _y);
		if(face >= 0) {
			cout << "selected face " << face << ":" << endl;
			_hullNode->printFace(face);
			if(evt == Touch::TOUCH_PRESS && !_ctrlPressed && !_shiftPressed) {
				_currentSelection->clear();
			}
			if(_shiftPressed) {
				_currentSelection->toggleFace(face);
			} else {
				_currentSelection->addFace(face);
			}
		}
	}
	return true;
}

void HullMode::placeCamera() {
	Mode::placeCamera();
	_hullNode->updateCamera(false);
}

HullMode::Selection::Selection(HullMode *mode, const char *id, Vector3 color) : _mode(mode) {
	_node = MyNode::create(id);
	_node->_type = "red";
	_node->_color = color;
	_mode->_scene->addNode(_node);
}

short HullMode::Selection::nf() {
	return _faces.size();
}

void HullMode::Selection::addFace(short face) {
	std::vector<short>::iterator it = std::find(_faces.begin(), _faces.end(), face);
	if(it == _faces.end()) {
		_faces.push_back(face);
		update();
	}
}

void HullMode::Selection::toggleFace(short face) {
	std::vector<short>::iterator it = std::find(_faces.begin(), _faces.end(), face);
	if(it != _faces.end()) {
		_faces.erase(it);
	} else {
		_faces.push_back(face);
	}
	update();
}

void HullMode::Selection::update() {
	_node->clearMesh();
	MyNode *node = _mode->_hullNode;
	short i, j, n = _faces.size(), f, nv;
	Vector3 vec, normal;
	std::vector<unsigned short> newFace;
	std::vector<std::vector<unsigned short> > newTriangles;
	std::map<unsigned short, unsigned short> newInd;
	Face::boundary_iterator it;
	for(i = 0; i < n; i++) {
		Face face = node->_faces[_faces[i]];
		normal = face.getNormal(true);
		newInd.clear();
		for(it = face.vbegin(); it != face.vend(); it++) {
			vec = node->_vertices[it->first] + normal * 0.001f;
			newInd[it->first] = _node->nv();
			_node->addVertex(vec);
		}
		f = face.size();
		for(j = 0; j < f; j++) face[j] = newInd[face[j]];
		short nh = face.nh(), k;
		for(j = 0; j < nh; j++) {
			short h = face.holeSize(j);
			for(k = 0; k < h; k++) face._holes[j][k] = newInd[face._holes[j][k]];
		}
		short nt = face.nt();
		for(j = 0; j < nt; j++) {
			for(k = 0; k < 3; k++) face._triangles[j][k] = newInd[face._triangles[j][k]];
		}
		_node->addFace(face);
	}
	_node->update();
	_node->updateModel(false, false);
}

void HullMode::Selection::clear() {
	_faces.clear();
	update();
}

void HullMode::Selection::reverseFaces() {
	MyNode *node = _mode->_hullNode;
	short n = _faces.size(), i;
	for(i = 0; i < n; i++) {
		node->_faces[_faces[i]].reverse();
	}
	node->updateModel(false, false);
	update();
}


