#include "T4TApp.h"
#include "HullMode.h"
#include "MyNode.h"

HullMode::HullMode() : Mode::Mode("hull") {
	_scene = Scene::load("res/common/scene.gpb");
	_doSelect = false;
	_ctrlPressed = false;
	_shiftPressed = false;
}

void setActive(bool active) {
	if(active) {
		app->promptItem();
	}
}

bool setSubMode(short mode) {
}

void HullMode::controlEvent(Control *control, EventType evt) {
	Mode::controlEvent(control, evt);
	const char *id = control->getId();
}

void HullMode::selectItem(const char *id) {
	Mode::selectItem(id);
	_scene->removeNode(_node);
	_node = app->duplicateModelNode(id);
	_node->setId(id);
	_scene->addNode(_node);
}

void keyEvent(Keyboard::KeyEvent evt, int key) {
	if(evt == Keyboard::KEY_CHAR) return;
	switch(key) {
		case Keyboard::KEY_SHIFT: {
			_shiftPressed = evt == Keyboard::KEY_PRESS;
			break;
		}
		case Keyboard::KEY_CTRL: {
			_ctrlPressed = evt == KeyBoard::KEY_PRESS;
		}
	}
}

bool HullMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	if(_selectingFaces && isTouching() && _touchPt._hit) {
		short face = _node->pt2Face(_touchPt.getPoint(evt));
		if(face >= 0) {
			if(evt == Touch::TOUCH_PRESS && !_ctrlPressed && !_shiftPressed) {
				clearSelection();
			} else if(_shiftPressed) {
				toggleFace(face);
			} else {
				addFace(face);
			}
		}
	}
	return true;
}

void HullMode::clearSelection() {
	_selection.clear();
	updateSelection();
}

void HullMode::toggleFace(short face) {
	std::vector<short>::iterator it = std::find(_selection.begin(), _selection.end(), face);
	if(it != _selection.end()) {
		_selection.erase(it);
	} else {
		_selection.push_back(face);
	}
	updateSelection();
}

void HullMode::addFace(short face) {
	std::vector<short>::iterator it = std::find(_selection.begin(), _selection.end(), face);
	if(it == _selection.end()) {
		_selection.push_back(face);
		updateSelection();
	}
}

void HullMode::updateSelection() {
	
}


