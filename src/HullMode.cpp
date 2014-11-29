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
	_node = NULL;
}

void HullMode::setActive(bool active) {
	if(active) {
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
}

void HullMode::selectItem(const char *id) {
	Mode::selectItem(id);
	if(_node) _scene->removeNode(_node);
	_node = app->duplicateModelNode(id);
	_node->setId(id);
	_scene->addNode(_node);
}

bool HullMode::keyEvent(Keyboard::KeyEvent evt, int key) {
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
	if(_currentSelection && isTouching() && _touchPt._hit) {
		short face = _node->pix2Face(_x, _y);
		if(face >= 0) {
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
	_node->updateCamera();
}

HullMode::Selection::Selection(HullMode *mode, const char *id, Vector3 color) : _mode(mode) {
	_node = MyNode::create(id);
	_node->_type = "red";
	_node->_color = color;
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
	MyNode *node = _mode->_node;
	short i, j, n = _faces.size(), f, nv;
	Vector3 vec, normal;
	std::vector<unsigned short> newFace;
	for(i = 0; i < n; i++) {
		Face &face = node->_faces[_faces[i]];
		f = face.size();
		nv = _node->nv();
		normal = face.getNormal(true);
		newFace.resize(f);
		for(j = 0; j < f; j++) {
			vec = node->_vertices[face[j]] + normal * 0.001f;
			_node->addVertex(vec);
			newFace[j] = nv + j;
		}
		_node->addFace(newFace);
	}
	_node->update();
	_node->updateModel(false, false);
}

void HullMode::Selection::clear() {
	_faces.clear();
	update();
}


