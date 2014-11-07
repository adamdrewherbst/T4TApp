#include "T4TApp.h"
#include "MyNode.h"

void T4TApp::generateModels() {
	generateModel("box", "box", 4.0f, 2.0f, 6.0f);
	generateModel("sphere", "sphere", 1.0f, 10);
	generateModel("cylinder", "cylinder", 1.0f, 4.0f, 20);
	generateModel("halfpipe", "halfpipe", 1.0f, 12.0f, 0.2f, 20);
	generateModel("gear_basic", "gear", 0.6f, 0.9f, 1.2f, 5);
	generateModel("stringLink", "cylinder", 0.1f, 1.0f, 5);
	generateModel("straw", "cylinder", 0.25f, 5.0f, 8);
	generateModel("balloon_sphere", "sphere", 1.5f, 10);
	generateModel("balloon_long", "cylinder", 0.5f, 3.0f, 20);
	generateModel("axle1", "cylinder", 0.15f, 6.0f, 8);
	generateModel("wheel1", "cylinder", 1.2f, 0.5f, 20);
	generateModel("buggyRamp", "wedge", 15.0f, 15.0f, 5.0f);
}

void T4TApp::generateModel(const char *id, const char *type, ...) {
	va_list arguments;
	va_start(arguments, type);
	MyNode *node = MyNode::create(id);
	node->_type = type;
	node->_objType = "mesh";
	node->_mass = 10.0f;
	short nv, nf, ne, i, j, k, m, n;
	Vector3 vertex;
	std::vector<unsigned short> face;
	std::vector<std::vector<unsigned short> > triangles;
	MyNode::ConvexHull *hull;

	if(strcmp(type, "sphere") == 0) {
		float radius = (float)va_arg(arguments, double);
		int segments = va_arg(arguments, int);
		float theta, phi;
		node->_objType = "sphere";
		node->addVertex(0, 0, -radius);
		for(i = 1; i <= segments-1; i++) {
			phi = (i - segments/2) * M_PI/segments;
			for(j = 0; j < segments; j++) {
				theta = j * 2*M_PI / segments;
				node->addVertex(radius * cos(phi) * cos(theta), radius * cos(phi) * sin(theta), radius * sin(phi));
			}
		}
		node->addVertex(0, 0, radius);
		for(i = 0; i < segments; i++) {
			node->addFace(3, 1+i, 0, 1+(i+1)%segments);
			m = 1 + (segments-2)*segments;
			node->addFace(3, m+i, m+(i+1)%segments, m+segments);
		}
		for(i = 0; i < segments-2; i++) {
			m = 1 + i*segments;
			for(j = 0; j < segments; j++) {
				node->addFace(4, m+j, m+(j+1)%segments, m+segments+(j+1)%segments, m+segments+j);
			}
		}
		node->setOneHull();
	}
	else if(strcmp(type, "cylinder") == 0) {
		float radius = (float)va_arg(arguments, double);
		float height = (float)va_arg(arguments, double);
		int segments = va_arg(arguments, int);
		float angle;
		for(i = 0; i < segments; i++) {
			angle = i * 2*M_PI / segments;
			node->addVertex(radius * cos(angle), radius * sin(angle), height/2);
			node->addVertex(radius * cos(angle), radius * sin(angle), -height/2);
		}
		for(i = 0; i < segments; i++) {
			node->addFace(4, i*2, i*2+1, (i*2+3)%(2*segments), (i*2+2)%(2*segments));
		}
		std::vector<unsigned short> face(segments);
		std::vector<std::vector<unsigned short> > triangles;
		for(i = 0; i < 2; i++) {
			for(j = 0; j < segments; j++) face[j] = i == 0 ? 2*j : 2*segments-1 - 2*j;
			triangles.clear();
			node->addFace(face);
		}
		node->setOneHull();
	}
	else if(strcmp(type, "halfpipe") == 0) {
		float radius = (float)va_arg(arguments, double);
		float height = (float)va_arg(arguments, double);
		float thickness = (float)va_arg(arguments, double);
		int segments = va_arg(arguments, int);
		float angle, innerRadius = radius - thickness, outerRadius = radius;
		for(i = 0; i <= segments/2; i++) {
			angle = i * M_PI / (segments/2);
			node->addVertex(innerRadius * cos(angle), innerRadius * sin(angle), height/2);
			node->addVertex(innerRadius * cos(angle), innerRadius * sin(angle), -height/2);
			node->addVertex(outerRadius * cos(angle), outerRadius * sin(angle), height/2);
			node->addVertex(outerRadius * cos(angle), outerRadius * sin(angle), -height/2);
		}
		for(i = 0; i < segments/2; i++) {
			node->addFace(4, i*4, i*4+4, i*4+5, i*4+1);
			node->addFace(4, i*4+2, i*4+3, i*4+7, i*4+6);
			node->addFace(4, i*4+4, i*4, i*4+2, i*4+6);
			node->addFace(4, i*4+1, i*4+5, i*4+7, i*4+3);
		}
		node->addFace(4, 0, 1, 3, 2);
		m = (segments/2) * 4;
		node->addFace(4, m+1, m, m+2, m+3);
		
		for(i = 0; i < segments/2; i++) {
			hull = new MyNode::ConvexHull(node);
			for(j = 0; j < 8; j++) hull->addVertex(node->_vertices[i*4 + j]);
			hull->addFace(4, 0, 4, 5, 1);
			hull->addFace(4, 2, 3, 7, 6);
			hull->addFace(4, 4, 0, 2, 6);
			hull->addFace(4, 1, 5, 7, 3);
			hull->addFace(4, 0, 1, 3, 2);
			hull->addFace(4, 4, 6, 7, 5);
			node->_hulls.push_back(hull);
		}
	}
	else if(strcmp(type, "box") == 0) {
		float length = (float)va_arg(arguments, double);
		float height = (float)va_arg(arguments, double);
		float width = (float)va_arg(arguments, double);
		node->_objType = "box";
		node->_vertices.resize(8);
		for(i = 0; i < 2; i++) {
			for(j = 0; j < 2; j++) {
				for(k = 0; k < 2; k++) {
					node->_vertices[i*4 + j*2 + k].set((2*k-1) * length/2, (2*j-1) * height/2, (2*i-1) * width/2);
				}
			}
		}
		node->addFace(4, 2, 3, 1, 0);
		node->addFace(4, 4, 5, 7, 6);
		node->addFace(4, 1, 5, 4, 0);
		node->addFace(4, 2, 6, 7, 3);
		node->addFace(4, 4, 6, 2, 0);
		node->addFace(4, 1, 3, 7, 5);
		node->setOneHull();
	}
	else if(strcmp(type, "wedge") == 0) {
		float length = (float)va_arg(arguments, double);
		float width = (float)va_arg(arguments, double);
		float height = (float)va_arg(arguments, double);
		node->_vertices.resize(6);
		for(i = 0; i < 3; i++) {
			for(j = 0; j < 2; j++) {
				node->_vertices[i*2 + j].set((2*j-1) * width/2, (i/2) * height, (2*(i%2)-1) * length/2);
			}
		}
		node->addFace(4, 0, 1, 3, 2);
		node->addFace(4, 0, 4, 5, 1);
		node->addFace(4, 2, 3, 5, 4);
		node->addFace(3, 0, 2, 4);
		node->addFace(3, 1, 5, 3);
		node->setOneHull();
	}
	else if(strcmp(type, "gear") == 0) {
		float innerRadius = (float)va_arg(arguments, double);
		float outerRadius = (float)va_arg(arguments, double);
		float width = (float)va_arg(arguments, double);
		int teeth = va_arg(arguments, int);
		nv = teeth * 8;
		float angle, dAngle = 2*M_PI / teeth, gearWidth = innerRadius * sin(dAngle/2);
		Matrix rot;
		//vertices
		for(n = 0; n < teeth; n++) {
			angle = n * dAngle;
			Matrix::createRotation(Vector3(0, 0, 1), -angle, &rot);
			for(i = 0; i < 2; i++) {
				for(j = 0; j < 2; j++) {
					for(k = 0; k < 2; k++) {
						vertex.set(0, innerRadius, -width/2);
						if(i == 1) vertex.z += width;
						if(j == 1) vertex.y = outerRadius;
						if(k == 1) vertex.x += gearWidth;
						rot.transformPoint(&vertex);
						node->_vertices.push_back(vertex);
					}
				}
			}
		}
		//faces
		for(i = 0; i < 2; i++) {
			face.clear();
			for(j = 0; j < teeth; j++) {
				face.push_back(8 * j + 4*i);
				face.push_back(8 * j + 1 + 4*i);
			}
			node->addFace(face, i == 1);
		}
		for(n = 0; n < teeth; n++) {
			for(i = 0; i < 2; i++) {
				//tooth sides
				face.clear();
				triangles.clear();
				face.push_back(0);
				face.push_back(1);
				face.push_back(3);
				face.push_back(2);
				for(j = 0; j < 4; j++) face[j] += n*8 + i*4;
				node->addFace(face, i == 0);
				//tooth front/back
				face.clear();
				triangles.clear();
				face.push_back(0);
				face.push_back(2);
				face.push_back(6);
				face.push_back(4);
				for(j = 0; j < 4; j++) face[j] += n*8 + i;
				node->addFace(face, i == 0);
			}
			//tooth top
			face.clear();
			triangles.clear();
			face.push_back(2);
			face.push_back(6);
			face.push_back(7);
			face.push_back(3);
			for(j = 0; j < 4; j++) face[j] += n*8;
			node->addFace(face);
			//tooth connector
			face.clear();
			triangles.clear();
			face.push_back(1);
			face.push_back(5);
			face.push_back(12);
			face.push_back(8);
			for(j = 0; j < 4; j++) face[j] = (face[j] + n*8) % nv;
			node->addFace(face);
		}
		//convex hulls
		for(n = 0; n < teeth; n++) {
			hull = new MyNode::ConvexHull(node);
			for(i = 0; i < 8; i++) hull->addVertex(node->_vertices[i + n*8]);
			node->_hulls.push_back(hull);
		}
		hull = new MyNode::ConvexHull(node);
		for(n = 0; n < teeth; n++) {
			hull->addVertex(node->_vertices[0 + n*8]);
			hull->addVertex(node->_vertices[1 + n*8]);
			hull->addVertex(node->_vertices[4 + n*8]);
			hull->addVertex(node->_vertices[5 + n*8]);
		}
		node->_hulls.push_back(hull);
	}
	node->writeData("res/common/");
}

