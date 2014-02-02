#include "T4TApp.h"

T4TApp::Lever::Lever(T4TApp *app_, Theme::Style *buttonStyle, Theme::Style *formStyle)
		: T4TApp::ProjectComponent::ProjectComponent(app_, "res/common/scene.gpb", "Lever", buttonStyle, formStyle) {
	addElement("Base", static_cast<T4TApp::ProjectComponent::TouchCallback>(&T4TApp::Lever::baseTouch), true);
	addElement("Arm", static_cast<T4TApp::ProjectComponent::TouchCallback>(&T4TApp::Lever::armTouch));
}

bool T4TApp::Lever::baseTouch(Touch::TouchEvent evt, int x, int y) {
	return true;
}

bool T4TApp::Lever::armTouch(Touch::TouchEvent evt, int x, int y) {
	PhysicsRigidBody* body = _allNodes[1]->getCollisionObject()->asRigidBody();
	body->setEnabled(false);
	//body->setAngularVelocity(0.0f, 0.0f, 100.0f);
	//_armConstraint->setEnabled(false);
	Vector3 axis;
	float ang = _allNodes[1]->getRotation(&axis);
	_allNodes[1]->rotate(Vector3(0.0f, 0.0f, 1.0f), (float)(0.5));
	cout << "rotating back by " << ang << " " << app->printVector(axis) << endl;
	//_allNodes[1]->translate(0.0f, 1.0f, 0.0f);
	body->setEnabled(true);
	body->setActivation(ACTIVE_TAG);
	Vector3 vec = _allNodes[1]->getTranslation(), vec2 = _allNodes[0]->getTranslation();
	cout << "arm now at " << app->printVector(vec) << ", base at " << app->printVector(vec2) << endl;
	Quaternion rot = _allNodes[1]->getRotation();
	cout << "rotation: " << app->printQuat(rot) << endl;
	return false;
}

void T4TApp::Lever::placeElement(Node *node) {
	switch(_currentElement) {
		case 0: //base
			node->setTranslation(Vector3(0.0f, 0.0f, 0.0f));
			break;
		case 1: //lever arm
			node->setTranslation(Vector3(0.0f, 4.0f, 0.0f));
			Quaternion rot;
			Quaternion::createFromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), (float)(0.0f), &rot);
			//add the hinge constraint between the base and arm
			_armConstraint = app->getPhysicsController()->createHingeConstraint(
				_allNodes[0]->getCollisionObject()->asRigidBody(),
				rot,
				Vector3(0.0f, 4.0f, 0.0f),
				_allNodes[1]->getCollisionObject()->asRigidBody(),
				rot,
				Vector3(0.0f, 0.0f, 0.0f)
			);//*/
			break;
	}
}

