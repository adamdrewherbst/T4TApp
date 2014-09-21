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
	return true;
}

void T4TApp::Lever::placeElement(Node *node) {
	BoundingBox box = node->getModel()->getMesh()->getBoundingBox();
	float x, y;
	switch(_currentElement) {
		case 0: //base
			y = (box.max.y - box.min.y) / 2.0f;
			node->setTranslation(Vector3(0.0f, y, 0.0f));
			break;
		case 1: //lever arm
			node->setTranslation(_allNodes[0]->getTranslationWorld() + Vector3(0.0f, 4.0f, 0.0f));
			break;
	}
}

void T4TApp::Lever::finishElement(Node *node) {
	switch(_currentElement) {
		case 0:
			break;
		case 1:
			Quaternion rot1, rot2;
			Quaternion::createFromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), (float)(0.0f), &rot1);
			Quaternion::createFromAxisAngle(Vector3(1.0f, 0.0f, 0.0f), (float)(0.0f), &rot2);
			Vector3 trans1(0.0f, 4.0f, 0.0f), trans2(0.0f, 0.0f, 0.0f);
			//add the hinge constraint between the base and arm
			_armConstraint = (PhysicsHingeConstraint*) app->addConstraint(_allNodes[0], _allNodes[1], "hinge",
				&rot1, &trans1, &rot2, &trans2);
			break;
	}
}

void T4TApp::Lever::releaseScene() {
	ProjectComponent::releaseScene();
	app->getPhysicsController()->removeConstraint(_armConstraint);
}

