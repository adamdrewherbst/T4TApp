#include "T4TApp.h"

StringMode::StringMode() : Mode::Mode("string") {}

bool StringMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
}

void StringMode::controlEvent(Control *control, Control::Listener::EventType evt) {
	Mode::controlEvent(control, evt);
}

void StringMode::makeString() {
	short n = _nodes.size(), i, j;
}


StringMode::NodeData::NodeData(MyNode *node) : _node(node), _mode(mode) {}

StringMode::NodeData::addFace(unsigned short f) {
	_faces.push_back(f);
}

StringMode::NodeData::getOutlines() {
	short i, j;
	std::vector<unsigned short> face;
	for(i = 0; i < _faces.size(); i++) {
		face = _node->_faces[_faces[i]];
		for(j = 0; j < face.size(); j++) {
			
		}
	}
}

