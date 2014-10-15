#include "T4TApp.h"

T4TApp::TestMode::TestMode() 
  : T4TApp::Mode::Mode("test") {
}

bool T4TApp::TestMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	Mode::touchEvent(evt, x, y, contactIndex);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			float distance = _ray.intersects(app->_groundPlane);
			if(distance != Ray::INTERSECTS_NONE) {
				Vector3 point(_ray.getOrigin() + _ray.getDirection() * distance);
				MyNode *node = app->duplicateModelNode("sphere");
				node->setScale(0.3f);
				MyNode::nodeData *data = node->getData();
				data->scale = Vector3(0.3f, 0.3f, 0.3f);
				data->mass = 3.0f;
				node->addCollisionObject();
				PhysicsRigidBody *body = node->getCollisionObject()->asRigidBody();
				body->setEnabled(false);
				node->setTranslation(point.x, 10.0f, point.z);
				body->setEnabled(true);
				_scene->addNode(node);
				cout << "added ball at " << app->pv(node->getTranslationWorld()) << endl;
			}
			break;
		}
		case Touch::TOUCH_MOVE: break;
		case Touch::TOUCH_RELEASE: break;
	}
}

void T4TApp::TestMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
}


