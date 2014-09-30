#include "T4TApp.h"

T4TApp::TestMode::TestMode(T4TApp *app_) 
  : T4TApp::Mode::Mode(app_, "mode_Test", NULL) {
}

bool T4TApp::TestMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			Ray ray;
			Plane vertical(Vector3(0, 0, 1), 0), ground(Vector3(0, 1, 0), 0);
			app->_scene->getActiveCamera()->pickRay(app->getViewport(), x, y, &ray);
			float distance = ray.intersects(ground);
			if(distance != Ray::INTERSECTS_NONE) {
				Vector3 point(ray.getOrigin() + ray.getDirection() * distance);
				MyNode *node = app->duplicateModelNode("sphere");
				node->getData()->mass = 3.0f;
				node->addCollisionObject();
				PhysicsRigidBody *body = node->getCollisionObject()->asRigidBody();
				body->setEnabled(false);
				node->setTranslation(point.x, 10.0f, point.z);
				body->setEnabled(true);
				app->_scene->addNode(node);
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


