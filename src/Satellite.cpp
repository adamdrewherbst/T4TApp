#include "T4TApp.h"
#include "Satellite.h"
#include "MyNode.h"

Satellite::Satellite() : Project::Project("satellite") {

	_body = addElement(new Body(this));
	_instruments = addElement(new Instrument(this, _body));

	_maxRadius = 2.5f;
	_maxLength = 10.0f;
}

bool Satellite::setSubMode(short mode) {
	bool changed = Project::setSubMode(mode);
	switch(_subMode) {
		case 0: { //build
			break;
		} case 1: { //test
			//check that it fits in the cylinder
			std::vector<MyNode*> nodes = _rootNode->getAllNodes();
			short i, j, n = nodes.size(), nv;
			float radius, maxRadius = 1e6, minZ = 1e6, maxZ = -1e6;
			Matrix m;
			_rootNode->getWorldMatrix().invert(&m);
			Vector3 vec;
			for(i = 0; i < n; i++) {
				MyNode *node = nodes[i];
				nv = node->nv();
				for(j = 0; j < nv; j++) {
					vec = node->_worldVertices[j];
					m.transformPoint(&vec);
					radius = sqrt(vec.x*vec.x + vec.y*vec.y);
					if(vec.z < minZ) minZ = vec.z;
					if(vec.z > maxZ) maxZ = vec.z;
					if(radius > maxRadius) {
						maxRadius = radius;
					}
				}
			}
			if(maxRadius > _maxRadius || (maxZ - minZ) > _maxLength) {
				cout << "Satellite does not fit in tube" << endl;
				break;
			}
			//drop it from a height of 1m
			_rootNode->enablePhysics(false);
			_rootNode->placeRest();
			Vector3 trans(0, 10, 0);
			_rootNode->setMyTranslation(trans);
			_rootNode->enablePhysics(true);
			app->getPhysicsController()->setGravity(app->_gravity);
			break;
		}
	}
	return changed;
}

Satellite::Body::Body(Project *project) : Project::Element::Element(project, NULL, "body", "Body") {
}

Satellite::Instrument::Instrument(Project *project, Element *parent)
  : Project::Element::Element(project, parent, "instrument", "Instrument") {
}

void Satellite::Instrument::placeNode(const Vector3 &position, short n) {
	short i, numNodes = _nodes.size();
	MyNode *body = _parent->getNode();
	for(i = 0; i < numNodes; i++) {
		if(n >= 0 && i != n) continue;
		MyNode *node = _nodes[i];
		BoundingBox box = node->getBoundingBox(true);
		Vector3 normal = Vector3::unitY();
		short f = body->pt2Face(position);
		if(f >= 0) normal = body->_faces[f].getNormal();
		Quaternion rot = MyNode::getVectorRotation(Vector3::unitZ());
		node->setMyRotation(rot);
		node->setMyTranslation(position - normal * box.min.y);
	}
}

void Satellite::Instrument::addPhysics(short n) {
}


