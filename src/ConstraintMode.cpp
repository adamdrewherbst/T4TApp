#include "T4TApp.h"

ConstraintMode::ConstraintMode()
  : Mode::Mode("constraint") {

	_currentNode = 0;
	
	_subModes.push_back("Axle");
	_subModes.push_back("Glue");
	_subModes.push_back("Socket");
	_subModes.push_back("Spring");
	
	_constraintTypes.push_back("hinge");
	_constraintTypes.push_back("fixed");
	_constraintTypes.push_back("socket");
	_constraintTypes.push_back("spring");
}

void ConstraintMode::setActive(bool active) {
	Mode::setActive(active);
	_currentNode = 0;
}

bool ConstraintMode::setSubMode(short mode) {
	bool changed = Mode::setSubMode(mode);
	_currentNode = 0;
	return changed;
}

bool ConstraintMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	Mode::touchEvent(evt, x, y, contactIndex);
	unsigned short i, j;
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			if(_touchNode == NULL) break;
			if(_currentNode == 1 && _touchNode == _nodes[0]) break; //don't allow self-constraint
			_nodes[_currentNode] = _touchNode;
			_touchNode->updateTransform();
			//find the clicked face
			if(_subMode == 0 || _subMode == 1 || _subMode == 2 || _subMode == 3) {
				_faces[_currentNode] = _selectedNode->pt2Face(_touchPoint,
				  _camera->getNode()->getTranslationWorld());
				if(_faces[_currentNode] < 0) break; //didn't hit a face - must reselect this node
			}
			_currentNode++;
			if(_currentNode == 2) { //we have both attachment points - add the constraint
				app->setAction("constraint", _nodes[0], _nodes[1]);
				_nodes[1]->rotateFaceToFace(_faces[1], _nodes[0], _faces[0]);
				_nodes[1]->translate(_nodes[0]->_worldNormals[_faces[0]] * 0.02f); //back away a tad
				for(i = 0; i < 2; i++) _nodes[i]->updateTransform();
				PhysicsConstraint *constraint = app->addConstraint(_nodes[0], _nodes[1], -1, _constraintTypes[_subMode].c_str(),
				  _nodes[0]->faceCenter(_faces[0]) + _nodes[0]->_worldNormals[_faces[0]] * 0.01f,
				  _nodes[0]->_worldNormals[_faces[0]]);
				//the second node clicked becomes a child of the first node clicked
				_nodes[0]->addChild(_nodes[1]);
				_nodes[1]->_constraintParent = _nodes[0];
				_nodes[1]->_parentOffset = _nodes[0]->faceCenter(_faces[0], true) + _nodes[0]->getScaleNormal(_faces[0]) * 0.01f;
				_nodes[1]->_parentAxis = _nodes[0]->getScaleNormal(_faces[0]);
				app->commitAction();
			}
			break;
		}
		case Touch::TOUCH_MOVE: {
			break;
		}
		case Touch::TOUCH_RELEASE: {
			break;
		}
	}
	return true;
}

void ConstraintMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
	Mode::controlEvent(control, evt);
}


