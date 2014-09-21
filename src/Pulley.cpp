#include "T4TApp.h"

T4TApp::Pulley::Pulley(T4TApp *app_, Theme::Style *buttonStyle, Theme::Style *formStyle)
		: T4TApp::ProjectComponent::ProjectComponent(app_, "res/common/scene.gpb", "Pulley", buttonStyle, formStyle) {

	_wheelLinks = 10;
	_linkWidth = 0.2f;
	
	addElement("Base", static_cast<T4TApp::ProjectComponent::TouchCallback>(&T4TApp::Pulley::baseTouch), true);
	addElement("Wheel", static_cast<T4TApp::ProjectComponent::TouchCallback>(&T4TApp::Pulley::wheelTouch), true);
	addElement("LeftBucket", static_cast<T4TApp::ProjectComponent::TouchCallback>(&T4TApp::Pulley::bucketTouch));
	addElement("RightBucket", static_cast<T4TApp::ProjectComponent::TouchCallback>(&T4TApp::Pulley::bucketTouch));
}

bool T4TApp::Pulley::baseTouch(Touch::TouchEvent evt, int x, int y) {
	return true;
}

bool T4TApp::Pulley::wheelTouch(Touch::TouchEvent evt, int x, int y) {
	return true;
}

bool T4TApp::Pulley::bucketTouch(Touch::TouchEvent evt, int x, int y) {
	return true;
}

void T4TApp::Pulley::placeElement(Node *node) {
	BoundingBox box = node->getModel()->getMesh()->getBoundingBox();
	float x, y;
	switch(_currentElement) {
		case 0: //base
			y = (box.max.y - box.min.y) / 2.0f;
			node->setTranslation(Vector3(0.0f, y, 0.0f));
			break;
		case 1: { //wheel
			Vector3 base(_allNodes[0]->getTranslationWorld()), wheel(base + Vector3(0.0f, 6.0f, 0.0f));
			node->rotate(Vector3(0, 1, 0), M_PI/2);
			node->setTranslation(wheel);
			box = node->getModel()->getMesh()->getBoundingBox();
			_radius = (box.max.x - box.min.x) / 2.0f + _linkWidth/2;
			// L/2 / R = sin(180/N / 2) => L = 2*R*sin(9)
			_linkLength = 2 * _radius * sin(M_PI / (2*_wheelLinks));
			_dropLinks = (int)((wheel.y - _linkLength/2 - base.y) / (2.0f * _linkLength));
			_numLinks = _dropLinks + _wheelLinks+1 + _dropLinks;
			break;
		} case 2: case 3: { //left/right bucket
			//vertically halfway between base and wheel, rounded to the nearest chain link endpoint
			//horizontally at left/right edge of wheel
			Vector3 base(_allNodes[0]->getTranslationWorld()), wheel(_allNodes[1]->getTranslationWorld());
			y = _dropLinks * _linkLength + (box.max.y - box.min.y) / 2.0f;
			Vector3 pos(base.x + (2*_currentElement - 5) * _radius, wheel.y - y, wheel.z);
			node->setTranslation(pos);
			break;
		}
	}
}

void T4TApp::Pulley::finishElement(Node *node) {
	switch(_currentElement) {
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
		case 3: { //all components added - now create the rope as a set of chain links with socket constraints
			Vector3 wheel(_allNodes[1]->getTranslationWorld());
			unsigned short i, j, v = 0;
			//create the chain link model
			std::vector<float> vertices(12);
			float color[3] = {1.0f, 0.0f, 1.0f};
			for(i = 0; i < 2; i++) {
				for(j = 0; j < 2; j++) vertices[v++] = 0;
				vertices[v++] = (2*i-1) * _linkLength/2;
				for(j = 0; j < 3; j++) vertices[v++] = color[j];
			}
			VertexFormat::Element elements[] = {
				VertexFormat::Element(VertexFormat::POSITION, 3),
				VertexFormat::Element(VertexFormat::COLOR, 3)
			};
			Node *linkTemplate = Node::create("segment"), *link;
			Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), v/6, false);
			mesh->setPrimitiveType(Mesh::LINES);
			mesh->setVertexData(&vertices[0], 0, v/6);
			Model *model = Model::create(mesh);
			mesh->release();
			model->setMaterial("res/common/grid.material");
			node->setModel(model);
			model->release();
			PhysicsRigidBody::Parameters params;
			params.mass = 0.3f;
			std::vector<Node*> links(_numLinks);
			std::vector<Vector3> joints(_numLinks+1);
			//copy it into all the links
			float x, y, z = wheel.z, angle;
			Vector3 zAxis(0, 0, 1), joint, trans1, trans2;
			joints[0].set(wheel);
			joints[0].x -= _radius;
			joints[0].y -= (_dropLinks + 0.5f) * _linkLength;
			for(i = 0; i < _numLinks; i++) {
				link = app->duplicateModelNode("box");
				std::stringstream ss;
				ss << _id << _typeCount << "_link" << (i+1);
				const std::string nodeID = ss.str();
				link->setId(nodeID.c_str());
				app->addCollisionObject(link);
				if(i < _dropLinks) { //chain going up from left bucket
					x = wheel.x - _radius;
					y = wheel.y - (_dropLinks-i) * _linkLength;
					angle = 0;
				} else if(i < _dropLinks + _wheelLinks+1) { //chain going over wheel
					angle = (i - _dropLinks) * (M_PI / _wheelLinks);
					x = wheel.x -_radius * cos(angle);
					y = wheel.y + _radius * sin(angle);
				} else { //chain going down to right bucket
					x = wheel.x + _radius;
					y = wheel.y - (_dropLinks - (_numLinks-1 - i)) * _linkLength;
					angle = 0;
				}
				link->setScale(_linkWidth/3, _linkLength/3, _linkWidth/3);
				link->rotate(zAxis, -angle);
				link->setTranslation(x, y, z);
				links[i] = link;
				_scene->addNode(link);
				_allNodes.push_back(link);
				link->getCollisionObject()->setEnabled(false);
				//note the position of the joint between this link and the next
				joint.set(0, (_linkLength/2) / link->getScaleY(), 0);
				link->getWorldMatrix().transformPoint(&joint);
				joints[i+1].set(joint);
			}
			//connect each pair of adjacent links with a socket constraint
			PhysicsSocketConstraint *constraint;
			for(i = 0; i < _numLinks-1; i++) {
				trans1.set(joints[i+1] - links[i]->getTranslationWorld());
				trans2.set(joints[i+1] - links[i+1]->getTranslationWorld());
				constraint = (PhysicsSocketConstraint*) app->addConstraint(links[i], links[i+1], "socket", &trans1, &trans2);
				_constraints.push_back(constraint);
			}
			//connect each bucket to the adjacent chain link with a socket constraint
			for(i = 0; i < 2; i++) {
				j = i * (_numLinks-1);
				trans1.set(joints[i*_numLinks] - links[j]->getTranslationWorld());
				trans2.set(joints[i*_numLinks] - _allNodes[i+2]->getTranslationWorld());
				constraint = (PhysicsSocketConstraint*) app->addConstraint(links[j], _allNodes[i+2], "socket", &trans1, &trans2);
				_constraints.push_back(constraint);
			}
			break;
		}
	}
}

void T4TApp::Pulley::releaseScene() {
	ProjectComponent::releaseScene();
	unsigned short i, n = _constraints.size();
	for(i = 0; i < n; i++) {
		app->getPhysicsController()->removeConstraint(_constraints[i]);
	}
}

