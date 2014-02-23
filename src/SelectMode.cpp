#include "T4TApp.h"

T4TApp::SelectMode::SelectMode(T4TApp *app_) 
  : T4TApp::Mode::Mode(app_, "mode_Select", "res/common/select.form") {

	_gridCheckbox = (CheckBox*) _controls->getControl("snap");
	_gridSlider = (Slider*) _controls->getControl("spacing");
}

bool T4TApp::SelectMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	switch(evt) {
		case Touch::TOUCH_PRESS: {
		    Camera* camera = app->_scene->getActiveCamera();
		    Ray ray;
		    camera->pickRay(app->getViewport(), x, y, &ray);
		    PhysicsController::HitResult hitResult;
		    if(!app->getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) break;
	    	Node *node = hitResult.object->getNode();
	    	if(node->getCollisionObject() == NULL) break;
		    if(strcmp(node->getId(), "grid") == 0) break;
			cout << "selected: " << node->getId() << endl;
			_selectedNode = node;
			//treat it as if the user clicked on the point on the grid directly below this object's center
			_dragOffset.set(0.0f, 0.0f);
			Vector3 center = node->getTranslation(), hitPoint = hitResult.point;
			Vector2 centerPix, hitPix;
			center.y = 0;
			Matrix viewProj = camera->getViewProjectionMatrix();
			camera->project(app->getViewport(), hitPoint, &hitPix);
			cout << "hit at " << app->printVector(hitPoint) << " => " << app->printVector2(hitPix) << endl;
			camera->project(app->getViewport(), center, &centerPix);
			_dragOffset.set(centerPix.x - x, centerPix.y - y);
			cout << "dragging " << node->getId() << " with offset " << app->printVector2(_dragOffset) << endl;
			//determine the set of nodes connected to this one by constraints - move them all together
			Node::nodeData *data = (Node::nodeData*)node->getUserPointer();
			_nodeGroup.clear();
			_groupOffset.clear();
			_nodeGroup.push_back(node);
			_groupOffset.push_back(Vector3::zero());
			for(int i = 0; i < data->constraints.size(); i++) {
				Node *other = app->_scene->findNode(data->constraints[i]->other.c_str());
				if(other == NULL) continue;
				_nodeGroup.push_back(other);
				_groupOffset.push_back(other->getTranslationWorld() - node->getTranslationWorld());
			}
			//disable all physics during the move - if a node is static, we must remove its physics and re-add it at the end
			for(int i = 0; i < _nodeGroup.size(); i++) {
				Node *n = _nodeGroup[i];
				PhysicsCollisionObject *obj = n->getCollisionObject();
				if(obj->isStatic()) {
					app->removeConstraints(n);
					n->setCollisionObject(PhysicsCollisionObject::NONE);
				} else {
					obj->asRigidBody()->setEnabled(false);
				}
			}
			break;
		}
		case Touch::TOUCH_MOVE: {
			if(_selectedNode == NULL) break;
			Ray ray;
			app->_scene->getActiveCamera()->pickRay(app->getViewport(), x + _dragOffset.x, y + _dragOffset.y, &ray);
			float distance = ray.intersects(app->_groundPlane);
			if(distance == Ray::INTERSECTS_NONE) break;
			app->_intersectPoint = ray.getOrigin() + ray.getDirection()*distance;
			app->_intersectPoint.y = (app->_intersectBox->max.y - app->_intersectBox->min.y) / 2.0f;
			//snap object to grid if desired
			if(_gridCheckbox->isChecked()) {
				float spacing = _gridSlider->getValue();
				app->_intersectPoint.x = round(app->_intersectPoint.x / spacing) * spacing;
				app->_intersectPoint.z = round(app->_intersectPoint.z / spacing) * spacing;
			}
			//if would intersect another object, place it on top of that object instead
			app->_intersectModel = NULL;
			app->_scene->visit(app, &T4TApp::checkTouchModel);
			for(int i = 0; i < _nodeGroup.size(); i++) {
				_nodeGroup[i]->setTranslation(app->_intersectPoint + _groupOffset[i]);
				PhysicsCollisionObject *obj = _nodeGroup[i]->getCollisionObject();
				if(obj) {
					PhysicsRigidBody *body = obj->asRigidBody();
					body->setEnabled(true); body->setEnabled(false);
				}
			}
			break;
		}
		case Touch::TOUCH_RELEASE: {
			//re-enable physics
			for(int i = 0; i < _nodeGroup.size(); i++) {
				Node *n = _nodeGroup[i];
				PhysicsCollisionObject *obj = n->getCollisionObject();
				if(obj == NULL) {
					app->addCollisionObject(n);
					app->addConstraints(n);
				} else {
					PhysicsRigidBody *body = obj->asRigidBody();
					body->setActivation(ACTIVE_TAG);
					body->setEnabled(true);
				}
			}
			_selectedNode = NULL;
			break;
		}
	}
}

void T4TApp::SelectMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
}


