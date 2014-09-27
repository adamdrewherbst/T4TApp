#include "T4TApp.h"

T4TApp::ConstraintMode::ConstraintMode(T4TApp *app_)
  : T4TApp::Mode::Mode(app_, "mode_Constraint", "res/common/constraint.form") {

	_currentNode = 0;
	
	_subModes.push_back(std::string("Axle"));
	_subModes.push_back(std::string("Glue"));
	_subModes.push_back(std::string("Socket"));
	_subModes.push_back(std::string("Spring"));
	_subMode = 0;
}

void T4TApp::ConstraintMode::setActive(bool active) {
	Mode::setActive(active);
	_currentNode = 0;
}

bool T4TApp::ConstraintMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	unsigned short i, j;
	switch(evt) {
		case Touch::TOUCH_PRESS: {
		    Camera* camera = app->_scene->getActiveCamera();
		    Ray ray;
		    camera->pickRay(app->getViewport(), x, y, &ray);
		    PhysicsController::HitResult hitResult;
		    if(!app->getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) break;
	    	Node *node = hitResult.object->getNode();
	    	Vector3 hitPoint = hitResult.point;
	    	if(node->getCollisionObject() == NULL) break;
		    if(strcmp(node->getId(), "grid") == 0) break;
			cout << "selected: " << node->getId() << " at " << hitPoint.x << "," << hitPoint.y << "," << hitPoint.z << endl;
			_nodes[_currentNode] = node;
			node->updateData();
			if(_subMode == 0 || _subMode == 1) { //axle and glue attach faces - find the clicked face
				_faces[_currentNode] = node->pt2Face(hitPoint);
				if(_faces[_currentNode] < 0) break; //didn't hit a face - must reselect this node
			} else if(_subMode == 2 || _subMode == 3) { //socket, spring
			}
			_currentNode++;
			if(_currentNode == 2) { //we have both attachment points - add the constraint
				if(_subMode == 0) { //axle
					_nodes[1]->rotateFaceToFace(_faces[1], _nodes[0], _faces[0]);
					_nodes[1]->translate(_nodes[0]->getData()->worldNormals[_faces[0]] * 0.2f); //back away a tad
					//default hinge axis is model space z-axis, we want face normal
					Vector3 normal;
					for(i = 0; i < 2; i++) {
						_nodes[i]->updateData();
						Node::nodeData *data = _nodes[i]->getData();
						_rot[i] = Node::getVectorRotation(Vector3(0, 0, 1), data->normals[_faces[i]]);
						_trans[i] = _nodes[i]->faceCenter(_faces[i], true);
						data->normals[_faces[i]].normalize(&normal);
						normal.x /= data->scale.x;
						normal.y /= data->scale.y;
						normal.z /= data->scale.z;
						_trans[i] += normal * 0.1f;
						_nodes[i]->getCollisionObject()->asRigidBody()->setEnabled(false);
						_nodes[i]->getCollisionObject()->asRigidBody()->setEnabled(true);
					}
					app->addConstraint(_nodes[0], _nodes[1], "hinge", &_rot[0], &_trans[0], &_rot[1], &_trans[1]);
				} else if(_subMode == 1) { //glue
					_nodes[1]->rotateFaceToFace(_faces[1], _nodes[0], _faces[0]);
					_nodes[1]->translate(_nodes[0]->getData()->worldNormals[_faces[0]] * 0.25f);
					for(i = 0; i < 2; i++) {
						_nodes[i]->getCollisionObject()->asRigidBody()->setEnabled(false);
						_nodes[i]->getCollisionObject()->asRigidBody()->setEnabled(true);
					}
					app->addConstraint(_nodes[0], _nodes[1], "fixed");
				} else if(_subMode == 2) { //socket
				} else if(_subMode == 3) { //spring
				}
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
	if(strcmp(control->getId(), "subMode") == 0) { //switching to new constraint type - clear any current attachment point
		_subMode = (_subMode + 1) % _subModes.size();
		((Button*)control)->setText(_subModes[_subMode].c_str());
		_currentNode = 0;
	}
}


