#include "T4TApp.h"

T4TApp::TestMode::TestMode(T4TApp *app_) 
  : T4TApp::Mode::Mode(app_, "mode_Test", NULL) {
}

bool T4TApp::TestMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			Ray ray;
			Plane vertical(Vector3(0, 0, 1), 0);
			app->_scene->getActiveCamera()->pickRay(app->getViewport(), x, y, &ray);
			float distance = ray.intersects(vertical);
			if(distance != Ray::INTERSECTS_NONE) {
				float worldX = ray.getOrigin().x + ray.getDirection().x * distance;
				Node *node = app->duplicateModelNode("sphere");
				app->addCollisionObject(node);
				PhysicsRigidBody *body = node->getCollisionObject()->asRigidBody();
				body->setEnabled(false);
				node->setTranslation(worldX, 10.0f, 0.0f);
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

