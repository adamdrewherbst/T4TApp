#include "T4TApp.h"

TouchMode::TouchMode() 
  : Mode::Mode("touch") {
	_face = MyNode::create("touchFace");
}

void TouchMode::setActive(bool active) {
	Mode::setActive(active);
	if(active) {
		_scene->addNode(_face);
	} else {
		_scene->removeNode(_face);
	}
}

bool TouchMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			Ray ray;
			_camera->pickRay(app->getViewport(), x, y, &ray);
		    PhysicsController::HitResult hitResult;
		    if(!app->getPhysicsController()->rayTest(ray, _camera->getFarPlane(), &hitResult)) break;
	    	MyNode *node = dynamic_cast<MyNode*>(hitResult.object->getNode());
	    	if(!node || node->getCollisionObject() == NULL) break;
		    if(strcmp(node->getId(), "grid") == 0) break;
			Vector3 point = hitResult.point, v1, v2, v3, p, coords, normal;
			cout << "touched " << node->getId() << " at " << point.x << "," << point.y << "," << point.z << endl;
			Matrix m;
			unsigned short i, j, k;
			short touchFace = -1;
			std::vector<unsigned short> face, triangle;
			for(i = 0; i < node->_faces.size() && touchFace < 0; i++) {
				face = node->_faces[i];
				for(j = 0; j < node->_triangles[i].size(); j++) {
					triangle = node->_triangles[i][j];
					v1.set(node->_worldVertices[face[triangle[1]]] - node->_worldVertices[face[triangle[0]]]);
					v2.set(node->_worldVertices[face[triangle[2]]] - node->_worldVertices[face[triangle[0]]]);
					v3.set(node->_normals[i]);
					p.set(point - node->_worldVertices[face[triangle[0]]]);
					//m.set(v1.x, v1.y, v1.z, 0, v2.x, v2.y, v2.z, 0, v3.x, v3.y, v3.z, 0, 0, 0, 0, 1);
					m.set(v1.x, v2.x, v3.x, 0, v1.y, v2.y, v3.y, 0, v1.z, v2.z, v3.z, 0, 0, 0, 0, 1);
					m.invert();
					m.transformVector(p, &coords);
					if(coords.x >= 0 && coords.y >= 0 && coords.x + coords.y <= 1 && fabs(coords.z) < 0.005f) {
						cout << "touching face " << i << endl;
						touchFace = i;
						break;
					}
				}
			}
			if(touchFace < 0) break;
			std::vector<float> vertices(6 * node->_triangles[touchFace].size() * 3);
			unsigned short v = 0;
			float color[3] = {1.0f, 0.0f, 1.0f};
			normal.set(node->_worldNormals[touchFace]);
			face = node->_faces[touchFace];
			for(i = 0; i < node->_triangles[touchFace].size(); i++) {
				triangle = node->_triangles[touchFace][i];
				for(j = 0; j < 3; j++) {
					v1.set(node->_worldVertices[face[triangle[j]]] + node->_worldNormals[touchFace] * 0.003f);
					vertices[v++] = v1.x;
					vertices[v++] = v1.y;
					vertices[v++] = v1.z;
					vertices[v++] = normal.x;
					vertices[v++] = normal.y;
					vertices[v++] = normal.z;
				}
			}
			VertexFormat::Element elements[] = {
				VertexFormat::Element(VertexFormat::POSITION, 3),
				VertexFormat::Element(VertexFormat::NORMAL, 3)
			};
			Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), v/6, false);
			mesh->setPrimitiveType(Mesh::TRIANGLES);
			mesh->setVertexData(&vertices[0], 0, v/6);
			Model *model = Model::create(mesh);
			mesh->release();
			model->setMaterial("res/common/models.material#red");
			_face->setModel(model);
			model->release();
			break;
		}
		case Touch::TOUCH_MOVE: break;
		case Touch::TOUCH_RELEASE: break;
	}
}

void TouchMode::controlEvent(Control *control, Control::Listener::EventType evt)
{
}


