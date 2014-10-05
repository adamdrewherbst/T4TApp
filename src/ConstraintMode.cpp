#include "T4TApp.h"

T4TApp::ConstraintMode::ConstraintMode()
  : T4TApp::Mode::Mode("mode_Constraint", "res/common/constraint.form") {

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

void T4TApp::ConstraintMode::setActive(bool active) {
	Mode::setActive(active);
	_currentNode = 0;
}

void T4TApp::ConstraintMode::setSubMode(short mode) {
	Mode::setSubMode(mode);
	_currentNode = 0;
}

bool T4TApp::ConstraintMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	Mode::touchEvent(evt, x, y, contactIndex);
	unsigned short i, j;
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			if(_touchNode == NULL) break;
			_nodes[_currentNode] = _touchNode;
			_selectedNode->updateData();
			//find the clicked face
			if(_subMode == 0 || _subMode == 1 || _subMode == 2 || _subMode == 3) {
				_faces[_currentNode] = node->pt2Face(hitPoint);
				if(_faces[_currentNode] < 0) break; //didn't hit a face - must reselect this node
			}
			_currentNode++;
			if(_currentNode == 2) { //we have both attachment points - add the constraint
				_nodes[1]->rotateFaceToFace(_faces[1], _nodes[0], _faces[0]);
				_nodes[1]->translate(_nodes[0]->getData()->worldNormals[_faces[0]] * 0.02f); //back away a tad
				Vector3 normal; //default hinge axis is model space z-axis, we want face normal
				for(i = 0; i < 2; i++) {
					_nodes[i]->updateData();
					MyNode::nodeData *data = _nodes[i]->getData();
					_rot[i] = MyNode::getVectorRotation(Vector3(0, 0, 1), data->normals[_faces[i]]);
					_trans[i] = _nodes[i]->faceCenter(_faces[i], true);
					data->normals[_faces[i]].normalize(&normal);
					normal.x /= data->scale.x;
					normal.y /= data->scale.y;
					normal.z /= data->scale.z;
					_trans[i] += normal * 0.01f;
					PhysicsRigidBody *body = _nodes[i]->getCollisionObject()->asRigidBody();
				}
				PhysicsConstraint *constraint;
				constraint = app->addConstraint(_nodes[0], _nodes[1], -1, _constraintTypes[_subMode].c_str(),
				  &_rot[0], &_trans[0], &_rot[1], &_trans[1]);
				//the second node clicked becomes a child of the first node clicked
				_nodes[0]->addChild(_nodes[1]);
				_nodes[1]->_constraintParent = _nodes[0];
				_nodes[1]->parentOffset = _trans[0];
				Matrix rot;
				Matrix::createRotation(_rot[0], &rot);
				_nodes[1]->parentAxis.set(0, 0, 1);
				rot.transformVector(&_nodes[1]->parentAxis);
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
}

void T4TApp::ConstraintMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
	Mode::controlEvent(control, evt);
}


