#include "T4TApp.h"

T4TApp::DrillMode::DrillMode(T4TApp *app_) 
  : T4TApp::ToolMode::ToolMode(app_, "mode_Drill", "res/common/drill.form") {
	_axis.set(Vector3(0, 0, 0), Vector3(1, 0, 0));
	_radius = 0.2f;
	_segments = 20;
	_tool = Node::create("drill");
	usageCount = 0;
	
	//have the user start by selecting a drill bit - shape and size
	_bitMenu = Form::create("bitMenu", app->_formStyle, Layout::LAYOUT_FLOW);
	_bitMenu->setPosition(app->_sideMenu->getX() + app->_sideMenu->getWidth() + 25.0f, 25.0f);
	_bitMenu->setWidth(app->getWidth() - _bitMenu->getX() - 25.0f);
	_bitMenu->setHeight(app->getHeight() - 50.0f);
	_bitMenu->setScroll(Container::SCROLL_VERTICAL);
	_bitMenu->setConsumeInputEvents(true);
	_bitMenu->setVisible(false);
	_container->addControl(_bitMenu);

	float sizes[6] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 1.0f};
	int shapes[2] = {4, 12};
	for(int h = 0; h < 2; h++) {
		for(int s = 0; s < 6; s++) {
			char id[20];
			sprintf(id, "drill_bit_%d_%3.2f", shapes[h], sizes[s]);
			ImageControl* itemImage = app->addButton <ImageControl> (_bitMenu, id);
			itemImage->setImage("res/png/cowboys-helmet-nobkg.png");
			itemImage->setWidth(150.0f);
			itemImage->setHeight(150.0f);
			itemImage->removeListener(app);
			itemImage->addListener((Control::Listener*)this, Control::Listener::CLICK);
		}
	}
}

void T4TApp::DrillMode::setActive(bool active) {
	ToolMode::setActive(active);
	_controls->setVisible(false);
	_bitMenu->setVisible(active);
	std::vector<Control*> controls = _bitMenu->getControls();
	for(size_t i = 0; i < controls.size(); i++) controls[i]->setEnabled(active);
	if(active) {
		app->_mainMenu->addListener((Control::Listener*)this, Control::Listener::CLICK);
	} else {
		app->_mainMenu->removeListener((Control::Listener*)this);
	}
}

void T4TApp::DrillMode::draw() {
	Mode::draw();
	if(_bitMenu->isVisible()) _bitMenu->draw();
}

void T4TApp::DrillMode::controlEvent(Control *control, Control::Listener::EventType evt) {
	ToolMode::controlEvent(control, evt);
	const char *controlID = control->getId();
	if(strncmp(controlID, "drill_bit_", 10) == 0) {
		int shape;
		float size;
		std::stringstream ss(std::string(controlID+10));
		ss >> shape;
		ss.ignore();
		ss >> size;
		cout << "chose drill bit with " << shape << " segments of radius " << size << endl;
		_segments = shape;
		_radius = size;
		_bitMenu->setVisible(false);
		_controls->setVisible(true);

		//create the drill-bit node
		lines.resize(_segments);
		planes.resize(_segments);
		float length = 5.0f, color[3] = {1.0f, 1.0f, 1.0f}, angle, dAngle = 2*M_PI / _segments;
		int v = 0, vertexCount = 6*_segments*6;
		std::vector<float> vertices(vertexCount);
		for(int i = 0; i < _segments; i++) {
			angle = (2*M_PI * i) / _segments;
			for(int j = 0; j < 2; j++) {
				vertices[v++] = (2*j-1) * length;
				vertices[v++] = _radius * cos(angle);
				vertices[v++] = _radius * sin(angle);
				for(int i = 0; i < 3; i++) vertices[v++] = color[i];
			}
			for(int j = 0; j < 2; j++) {
				for(int k = 0; k < 2; k++) {
					vertices[v++] = (2*j-1) * length;
					vertices[v++] = _radius * cos(angle + k*dAngle);
					vertices[v++] = _radius * sin(angle + k*dAngle);
					for(int i = 0; i < 3; i++) vertices[v++] = color[i];
				}
			}
		}
		VertexFormat::Element elements[] = {
			VertexFormat::Element(VertexFormat::POSITION, 3),
			VertexFormat::Element(VertexFormat::COLOR, 3)
		};
		Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), v/6, false);
		mesh->setPrimitiveType(Mesh::LINES);
		mesh->setVertexData(&vertices[0], 0, v/6);
		Model *model = Model::create(mesh);
		mesh->release();
		model->setMaterial("res/common/grid.material");
		_tool->setModel(model);
		model->release();
	}
}

void T4TApp::DrillMode::setAxis(int axis) {
	ToolMode::setAxis(axis);
	Vector3 drillAxis;
	Matrix rotation;
	_node->getRotation(&rotation);
	switch(axis) {
		case 0: //x
			drillAxis.set(1, 0, 0);
			break;
		case 1: //y
			drillAxis.set(0, 1, 0);
			break;
		case 2: //z
			drillAxis.set(0, 0, 1);
			break;
	}
	rotation.transformVector(&drillAxis);
	_axis.setDirection(drillAxis);
}

//for debugging, zoom in on a face, highlight it, and show its drill intersections
void T4TApp::DrillMode::drawFace(int f) {

	Node::nodeData *data = (Node::nodeData*)_node->getUserPointer();

	//get the plane for this face
	std::vector<unsigned short> face = data->faces[f];
	Plane plane;
	Vector3 vec[3], v1, v2;
	for(int i = 0; i < 3; i++) vec[i].set(data->worldVertices[face[i]]);
	v1.set(vec[1]-vec[0]);
	v2.set(vec[2]-vec[1]);
	v1.cross(v2);
	plane.setNormal(v1);
	plane.setDistance(-vec[0].dot(plane.getNormal()));

	std::vector<float> vertices(6*2*face.size());
	int v = 0;
	Vector3 color(1, 0, 0);
	for(int i = 0; i < face.size(); i++) {
		for(int j = 0; j < 2; j++) {
			v1.set(data->worldVertices[face[(i+j)%face.size()]] + 0.01f * plane.getNormal());
			for(int k = 0; k < 3; k++) vertices[v++] = Node::gv(&v1, k);
			for(int k = 0; k < 3; k++) vertices[v++] = Node::gv(&color, k);
		}
	}
	app->_scene->addNode(app->createWireframe(vertices));
	//then the calculated drill points for the face
	color.set(1, 1, 1);
	vertices.clear();
	vertices.reserve(_segments*2*6);
	v = 0;
	for(int i = 0; i < _segments; i++) {
		if(drillInt.find(i) != drillInt.end() && drillInt[i].find(f) != drillInt[i].end()) {
			v1.set(newData.vertices[drillInt[i][f]] + 0.01f * plane.getNormal());
			for(int j = 0; j < 2; j++) {
				if(j == 0 && vertices.empty()) continue;
				for(int k = 0; k < 3; k++) vertices.push_back(Node::gv(&v1, k));
				for(int k = 0; k < 3; k++) vertices.push_back(Node::gv(&color, k));
			}
		}
	}
	if(!vertices.empty()) vertices.resize(vertices.size() - 6);
	if(!vertices.empty()) app->_scene->addNode(app->createWireframe(vertices));
	//then all drill intersections on this plane
	color.set(0, 1, 0);
	vertices.clear();
	vertices.resize(_segments*2*6);
	v = _segments*2*6 - 6;
	float distance;
	for(int i = 0; i < _segments; i++) {
		distance = lines[i].intersects(plane);
		v1.set(lines[i].getOrigin() + distance * lines[i].getDirection() + 0.01f*plane.getNormal());
		for(int j = 0; j < 2; j++) {
			for(int k = 0; k < 3; k++) vertices[v++] = Node::gv(&v1, k);
			for(int k = 0; k < 3; k++) vertices[v++] = Node::gv(&color, k);
			if(i == 0 && j == 0) v = 0;
		}
	}
	app->_scene->addNode(app->createWireframe(vertices));
	//move the camera to look at the face
	Vector3 eye, target, up;
	target.set(vec[0]);
	eye.set(target + 5.0f*plane.getNormal());
	up.set(vec[1]-vec[0]);
	app->getScriptController()->executeFunction<void>("camera_setVectors", "<Vector3><Vector3><Vector3>", &eye, &target, &up);
	app->frame();
}

void T4TApp::DrillMode::addEdge(unsigned short e1, unsigned short e2) {
	if(usedEdges.find(e1) != usedEdges.end() && std::find(usedEdges[e1].begin(), usedEdges[e1].end(), e2) != usedEdges[e1].end())
		return;
	newEdge[0] = e1;
	newEdge[1] = e2;
	newData.edgeInd[e1][e2] = newData.edges.size();
	newData.edgeInd[e2][e1] = newData.edges.size();
	newData.edges.push_back(newEdge);
	usedEdges[e1].push_back(e2);
	usedEdges[e2].push_back(e1);
}

void T4TApp::DrillMode::addFace(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles) {
	if(triangles.empty()) {
		//if no triangles given, we need to triangulate, and since it may be concave, we'll add each triangle as a separate face
		triangulate(face, triangles);
		std::vector<std::vector<unsigned short> > oneTriangle(1);
		for(int i = 0; i < 3; i++) oneTriangle[0].push_back(i);
		for(int i = 0; i < triangles.size(); i++) {
			addFaceHelper(triangles[i], oneTriangle);
		}
	} else addFaceHelper(face, triangles);
}

void T4TApp::DrillMode::addFaceHelper(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles) {
	newData.faces.push_back(face);
	newData.triangles.push_back(triangles);
	Vector3 v1(newData.vertices[face[1]] - newData.vertices[face[0]]),
		v2(newData.vertices[face[2]] - newData.vertices[face[1]]),
		normal;
	Vector3::cross(v1, v2, &normal);
	newData.normals.push_back(normal);
	unsigned short e1, e2;
	for(short n = 0; n < triangles.size(); n++) {
		for(short i = 0; i < 3; i++) {
			addEdge(face[triangles[n][i]], face[triangles[n][(i+1)%3]]);
		}
	}
}

//use ear clipping to triangulate the given polygon, which may be concave
// - http://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
void T4TApp::DrillMode::triangulate(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles) {

	std::vector<unsigned short> triangle(3);
	short i, j, k, faceSize = face.size(), ind, remaining = faceSize;
	std::vector<std::pair<unsigned short, unsigned short> > indClass(faceSize);
	
	//start by calculating the properly oriented face normal by Newell's method
	// - https://www.opengl.org/wiki/Calculating_a_Surface_Normal#Newell.27s_Method
	Vector3 v1, v2, normal(0, 0, 0);
	for(i = 0; i < faceSize; i++) {
		v1.set(newData.vertices[face[i]]);
		v2.set(newData.vertices[face[(i+1)%faceSize]]);
		normal.x += (v1.y - v2.y) * (v1.z + v2.z);
		normal.y += (v1.z - v2.z) * (v1.x + v2.x);
		normal.z += (v1.x - v2.x) * (v1.y + v2.y);
	}
	normal.normalize(&normal);

	//build the initial lists of convex/reflex/ear vertices
	for(i = 0; i < faceSize; i++) indClass[i] = std::pair<unsigned short, unsigned short>(i, 0);
	for(i = 0; i < faceSize; i++) indClass[i].second = vertexClass(face, normal, i, indClass);

	//iterate over the polygon removing ears
	while((remaining = indClass.size()) > 3) {
		ind = -1;
		for(i = 0; i < remaining; i++) if(indClass[i].second == 2) { ind = i; break; }
		if(ind < 0) break;
		triangle[0] = face[indClass[(ind-1+remaining)%remaining].first];
		triangle[1] = face[indClass[ind].first];
		triangle[2] = face[indClass[(ind+1)%remaining].first];
		triangles.push_back(triangle);
		indClass.erase(indClass.begin() + ind);
		if((remaining = indClass.size()) <= 3) break;
		//see if the neighboring vertices' classification has changed
		remaining--;
		for(j = ind-1; j <= ind; j++) {
			k = (j+remaining)%remaining;
			indClass[k].second = vertexClass(face, normal, k, indClass);
		}
	}
	if(remaining == 3) {
		for(i = 0; i < 3; i++) triangle[i] = face[indClass[i].first];
		triangles.push_back(triangle);
	}
}

//given a vertex(index) of a face(f), see if it is convex, reflex, or an ear (using the known reflex vertices of the face)
unsigned short T4TApp::DrillMode::vertexClass(std::vector<unsigned short>& face,
  Vector3& normal,
  unsigned short index,
  std::vector<std::pair<unsigned short, unsigned short> >& classes) {
	Vector3 v1, v2, v3, e1, e2, cross;
	float d00, d10, d11, d20, d21, denom, u, v, w;
	unsigned short faceSize = classes.size(), i, n2 = classes[index].first,
		n1 = classes[(index-1+faceSize)%faceSize].first, n3 = classes[(index+1)%faceSize].first;
	v1.set(newData.vertices[face[n1]]);
	v2.set(newData.vertices[face[n2]]);
	v3.set(newData.vertices[face[n3]]);
	e1.set(v2 - v1);
	e2.set(v3 - v2);
	Vector3::cross(e1, e2, &cross);
	if(cross.dot(normal) < 0) return 0; //reflex
	v1.set(-e1);
	v2.set(e2);
	d00 = v1.dot(v1);
	d10 = v2.dot(v1);
	d11 = v2.dot(v2);
	bool isEar = true;
	for(i = 0; i < faceSize; i++) {
		if(classes[i].second != 0) continue; //not a reflex point
		if(i == n1 || i == n2 || i == n3) continue;
		//compute the barycentric coords of the vertex wrt this triangle
		v3.set(newData.vertices[face[i]] - newData.vertices[face[n2]]);
		d20 = v3.dot(v1);
		d21 = v3.dot(v2);
		denom = d00 * d11 - d10 * d10;
		if(denom == 0) continue;
		v = (d11 * d20 - d10 * d21) / denom;
		w = (d00 * d21 - d10 * d20) / denom;
		u = 1.0f - v - w;
		if(u < 0.0f || u > 1.0f) continue;
		isEar = false;
		break;
	}
	return isEar ? 2 : 1; //ear : convex
}

//1 if face f1 occludes face f2 using drill axis as view direction, -1 if f2 occludes f1, 0 if no occlusion
short T4TApp::DrillMode::occlusion(unsigned short f1, unsigned short f2) {
	//just see if there are any edge intersections in the projected plane, and compare z value at intersection
	short i, j, s1 = data->faces[f1].size(), s2 = data->faces[f2].size();
	float d, a, b, z1, z2;
	Vector3 p1, p2, q1, q2, v1, v2, v;
	for(i = 0; i < s1; i++) {
		p1.set(drillVertices[data->faces[f1][i]]);
		q1.set(drillVertices[data->faces[f1][(i+1)%s1]]);
		v1.set(q1 - p1);
		for(j = 0; j < s2; j++) {
			//skip edges with either endpoint in common
			if(data->faces[f2][j] == data->faces[f1][i] || data->faces[f2][j] == data->faces[f1][(i+1)%s1]
			  || data->faces[f2][(j+1)%s2] == data->faces[f1][i] || data->faces[f2][(j+1)%s2] == data->faces[f1][(i+1)%s1])
				continue;
			p2.set(drillVertices[data->faces[f2][j]]);
			q2.set(drillVertices[data->faces[f2][(j+1)%s2]]);
			v2.set(q2 - p2);
			//p1 + a*v1 = p2 + b*v2 => [v1 v2][a -b] = p2 - p1 => [a -b] = ([v1 v2]^-1)(p2 - p1)
			// => d = 1/(v1x*v2y - v2x*v1y), v = p2 - p1, a = d * (v2y * vx - v2x * vy), b = -d * (-v1y * vx + v1x * vy)
			d = 1 / (v1.x * v2.y - v2.x * v1.y);
			v.set(p2 - p1);
			a = d * (v2.y * v.x - v2.x * v.y);
			b = -d * (-v1.y * v.x + v1.x * v.y);
			if(a < 0 || a > 1 || b < 0 || b > 1) continue;
			//z1 = p1z + a * v1z, z2 = p2z + b * v2z
			z1 = p1.z + a * v1.z;
			z2 = p2.z + b * v2.z;
			return z1 > z2 ? -1 : 1;
		}
	}
	return 0;
}

void T4TApp::DrillMode::partitionNode() {
	bool found;
	unsigned short i, j, k, patch, faceSize;
	facePatch.resize(data->faces.size());
	for(i = 0; i < data->faces.size(); i++) facePatch[i] = -1;
	do { //find the first unused face and recursively nucleate a patch from it
		found = false;
		for(i = 0; i < data->faces.size(); i++) {
			if(facePatch[i] < 0) {
				found = true;
				patch = patches.size();
				patches.resize(patch + 1);
				patchEdge.resize(patch + 1);
				patches[patch].push_back(i);
				facePatch[i] = patch;
				faceSize = data->faces[i].size();
				for(j = 0; j < faceSize; j++) {
					patchEdge[patch].push_back(data->edgeInd[data->faces[i][j]][data->faces[i][(j+1)%faceSize]]);
				}
				buildPatch(i);
				break;
			}
		}
	} while(found);
}

void T4TApp::DrillMode::buildPatch(unsigned short face) {
	unsigned short i, j, k, n1, n2, n3, n4, neighbor, faceSize, edge,
		patch = patches.size() - 1, patchSize;
	bool toward = data->normals[face].dot(_axis.getDirection()) > 0, hasOcclusion, found;
	for(i = 0; i < data->faceNeighbors[face].size(); i++) {
		neighbor = data->faceNeighbors[face][i];
		patchSize = patches[patch].size();
		faceSize = data->faces[neighbor].size();
		//make sure this neighbor is not already in a patch, and faces the same direction wrt the drill axis
		if(facePatch[neighbor] >= 0 || toward != (data->normals[neighbor].dot(_axis.getDirection()) > 0)) continue;
		//also make sure it is not occluding/occluded by any face already in this patch
		hasOcclusion = false;
		for(j = 0; j < patchSize; j++) {
			hasOcclusion = occlusion(patches[patch][j], neighbor) != 0;
			if(hasOcclusion) break;
		}
		if(hasOcclusion) continue;
		//add the triangle to the face
		patches[patch].push_back(neighbor);
		facePatch[neighbor] = patch;
		//for the patch boundary: replace all edges that are part of the new face, with the rest of the edges from the new face
		for(j = 0; j < faceSize; j++) {
			edge = data->edgeInd[data->faces[neighbor][j]][data->faces[neighbor][(j+1)%faceSize]];
			found = false;
			for(k = 0; k < patchEdge[patch].size(); k++) {
				if(edge == patchEdge[patch][k]) {
					patchEdge[patch].erase(patchEdge[patch].begin() + k);
					found = true;
					break;
				}
			}
			if(!found) patchEdge[patch].push_back(edge);
		}
		buildPatch(neighbor);
	}
}

void T4TApp::DrillMode::addDrillEdge(unsigned short v1, unsigned short v2, unsigned short lineNum, short face) {
	edgeLine[v1][v2] = lineNum;
	if(face >= 0) {
		edgeLine[v2][v1] = lineNum;
		Vector3 v(newData.vertices[v2] - newData.vertices[v1]);
		v.cross(data->normals[face]);
		leftEdge[v1][v2] = v.dot(planes[lineNum].getNormal()) > 0;
		leftEdge[v2][v1] = !leftEdge[v1][v2];
	}
}

bool T4TApp::DrillMode::toolNode() {
	ToolMode::toolNode();
	_node->updateData();
	data = (Node::nodeData*)_node->getUserPointer();
	
	usageCount++;

	//build drilled node into a new node data structure, save to disk, and reload
	newData.type = data->type;
	newData.objType = "mesh"; //can't keep sphere/box collision object once it has a hole in it!
	newData.mass = data->mass;
	newData.rotation = data->rotation;
	newData.translation = data->translation;
	newData.scale = data->scale;
	newData.constraints = data->constraints;
	newData.vertices.clear();
	newData.edges.clear();
	newData.faces.clear();
	newData.triangles.clear();
	newData.hulls.clear();
	newData.constraints = data->constraints;

	//initialize recycled member variables
	patches.clear();
	patchEdge.clear();
	usedEdges.clear();
	newEdge.resize(2);
	edgeLine.clear();
	leftEdge.clear();
	enterInt.clear();
	edgeInt.clear();
	drillInt.clear();
	drillError.clear();
	
	//temp variables
	short i, j, k, m, n, p, q;
	float f1, f2, f3, f4;
	Vector3 v1, v2, v3;
	std::vector<Vector3> v(4);

	//store the planes and lines of the drillbit segments for calculations
	float angle, dAngle = 2*M_PI/_segments, planeDistance = _radius * cos(dAngle/2);
	Matrix trans(_tool->getWorldMatrix());
	_axis.setOrigin(-50.0f, 0, 0);
	_axis.setDirection(1, 0, 0);
	_axis.transform(trans);
	for(i = 0; i < _segments; i++) {
		angle = (2*M_PI*i) / _segments;
		//line
		lines[i].setOrigin(-50.0f, _radius*cos(angle), _radius*sin(angle));
		lines[i].setDirection(1, 0, 0);
		lines[i].transform(trans);
		//plane
		planes[i].setNormal(0, cos(angle+dAngle/2), sin(angle+dAngle/2));
		planes[i].setDistance(-planeDistance);
		planes[i].transform(trans);
		v1.set(planes[i].getNormal());
		f1 = planes[i].getDistance();
		//make sure the plane normal points outward from the drill center
		if(v1.dot(-v1*f1 - _axis.getOrigin()) < 0) {
			planes[i].set(-v1, -f1);
		}
	}

	//take the drill axis as the z-axis and store the transformed coordinates of all vertices in the model
	Vector3 right(0, 0, 1), up(0, 1, 0), axis(_axis.getDirection());
	trans.transformVector(&right);
	trans.transformVector(&up);
	Matrix drillRot(right.x, up.x, axis.x, 0, right.y, up.y, axis.y, 0, right.z, up.z, axis.z, 0, 0, 0, 0, 1);
	drillRot.invert();
	drillVertices.resize(data->vertices.size());
	for(i = 0; i < data->vertices.size(); i++) {
		drillVertices[i].set(data->worldVertices[i] - _node->getTranslationWorld());
		drillRot.transformPoint(&drillVertices[i]);
	}

	//determine which vertices to discard because they are inside the drill cylinder
	std::vector<short> keep(data->vertices.size());
	Vector3 test;
	float testAngle, testRadius;
	for(i = 0; i < data->vertices.size(); i++) {
		keep[i] = -1;
		test.set(drillVertices[i]);
		testAngle = atan2(test.y, test.x);
		while(testAngle < 0) testAngle += 2*M_PI;
		testRadius = sqrt(test.x*test.x + test.y*test.y);
		float ang = fabs(fmod(testAngle, dAngle) - dAngle/2), radius = planeDistance / cos(ang);
		if(testRadius >= radius) {
			keep[i] = newData.vertices.size();
			newData.vertices.push_back(data->worldVertices[i]);
		}
	}

	//store the plane for each face in the model
	Vector3 direction, drillPt, drillVec, intersect;
	std::vector<Plane> faces(data->faces.size());
	for(i = 0; i < data->faces.size(); i++) {
		for(j = 0; j < 3; j++) v[j].set(data->worldVertices[data->faces[i][j]]);
		v1.set(v[1]-v[0]);
		v2.set(v[2]-v[1]);
		v1.cross(v2);
		faces[i].setNormal(v1);
		faces[i].setDistance(-v[0].dot(faces[i].getNormal()));
	}

	//partition the model surface into patches based on whether the face normals are along or against the drill axis
	partitionNode();

	//for each patch, check whether each drill line intersects it, using its 2D projection onto the drill plane
	Ray ray;
	unsigned short patchSize, intersectCount, neighbor, edgeInd;
	bool positive, skip;
	float epsilon = 0.001f, delta, denom, s, t, distance;
	std::map<unsigned short, unsigned short> drillLineInd;
	std::map<unsigned short, std::map<unsigned short, unsigned short> >::iterator it;
	std::map<unsigned short, unsigned short>::iterator it1;
	for(i = 0; i < patches.size(); i++) {
		patchSize = patches[i].size();
		for(j = 0; j < _segments; j++) {
			//use test-ray edge-intersection count to check if line is inside or outside patch
			drillPt.set(_radius * sin(j*2*M_PI / _segments), _radius * cos(j*2*M_PI / _segments), 0);
			direction.set(1, 0, 0);
			intersectCount = 0;
			for(k = 0; k < patchEdge[i].size(); k++) {
				edgeInd = patchEdge[i][k];
				v1.set(drillVertices[data->edges[edgeInd][0]] - drillPt);
				v2.set(drillVertices[data->edges[edgeInd][1]] - drillPt);
				v1.z = 0;
				v2.z = 0;
				Vector3::cross(v2, v1, &v3);
				positive = v3.z > 0;
				Vector3::cross(direction, v1, &v3);
				if(positive != (v3.z > 0)) continue;
				Vector3::cross(v2, direction, &v3);
				if(positive != (v3.z > 0)) continue;
				intersectCount++;
			}
			if(intersectCount % 2 == 0) continue;
			//we now know the line passes through this patch - use barycentric coords on the triangles to find out exactly where
			for(k = 0; k < patchSize; k++) {
				m = patches[i][k];
				for(n = 0; n < data->triangles[m].size(); n++) {
					v1.set(drillVertices[data->faces[m][data->triangles[m][n][1]]]
						- drillVertices[data->faces[m][data->triangles[m][n][0]]]);
					v2.set(drillVertices[data->faces[m][data->triangles[m][n][2]]]
						- drillVertices[data->faces[m][data->triangles[m][n][0]]]);
					drillVec.set(drillPt - drillVertices[data->faces[m][data->triangles[m][n][0]]]);
					//2D barycentric coords (ignore drill axis):
					//s*v1 + t*v2 = P => [v1 v2][s,t] = P => [s,t] = [v1 v2]^-1 * P
					// => [s,t] = 1/(v1.x * v2.y - v2.x * v1.y) [v2.y -v2.x -v1.y v1.x] [P.x P.y]
					//											[v2.y * P.x - v2.x * P.y, -v1.y * P.x + v1.x * P.y]
					denom = v1.x * v2.y - v2.x * v1.y;
					if(denom == 0) continue;
					s = (v2.y * drillVec.x - v2.x * drillVec.y) / denom;
					t = (-v1.y * drillVec.x + v1.x * drillVec.y) / denom;
					if(isinf(s) || isinf(t) || isnan(s) || isnan(t)) continue;
					if(s < -epsilon || t < -epsilon || s + t > 1 + epsilon) continue;
					if(s >=0 && t >= 0 && s+t <= 1) delta = 0;
					else {
						delta = fmax(delta, -s);
						delta = fmax(delta, -t);
						delta = fmax(delta, s+t - 1);
					}
					if(isinf(delta) || isnan(delta)) continue;
					//first see if this should replace an intersection in a neighboring triangle/face due to imprecision
					intersect.set(data->worldVertices[data->faces[m][data->triangles[m][n][0]]]
						+ (data->worldVertices[data->faces[m][data->triangles[m][n][1]]]
						  - data->worldVertices[data->faces[m][data->triangles[m][n][0]]]) * s
						+ (data->worldVertices[data->faces[m][data->triangles[m][n][2]]]
						  - data->worldVertices[data->faces[m][data->triangles[m][n][0]]]) * t);
					//already an intersection in another triangle of this face?
					if(drillInt[j].find(m) != drillInt[j].end()) {
						if(drillError[j][m] > delta) {
							newData.vertices[drillInt[j][m]].set(intersect);
							drillError[j][m] = delta;
						}
						continue;
					}
					//already an intersection in a neighboring face of the same patch?
					skip = false;
					for(p = 0; p < data->faceNeighbors[m].size(); p++) {
						neighbor = data->faceNeighbors[m][p];
						if(facePatch[neighbor] != i) continue;
						if(drillInt[j].find(neighbor) != drillInt[j].end()) {
							if(drillError[j][neighbor] < delta) {
								skip = true;
								break;
							} else {
								newData.vertices[drillInt[j][neighbor]] = intersect;
								drillLineInd[drillInt[j][neighbor]] = j;
								drillInt[j][m] = drillInt[j][neighbor];
								drillError[j][m] = delta;
								drillInt[j].erase(neighbor);
								drillError[j].erase(neighbor);
								skip = true;
								break;
							}
						}
					}
					if(skip) continue;
					//are we entering or leaving the model at this intersection?
					enterInt[newData.vertices.size()] = data->normals[m].dot(axis) < 0;
					//add the intersection data and add the vertex to the new model
					drillInt[j][m] = newData.vertices.size();
					drillError[j][m] = delta;
					newData.vertices.push_back(intersect);
				}
			}
		}
	}

	//find all intersections between edges of the model and the planes of the drill bit
	Ray edge;
	unsigned short e[2];
	short lineInd[2];
	std::vector<bool> hasInt(2);
	Vector4 intersect4;
	float intersectAngle, angleDiff, minAngleDiff, edgeLen, curDistance, angleEpsilon = 0.1f*M_PI/180;
	bool angleInside, prevAngleInside, distanceInside, prevDistanceInside, better, useInt;
	std::vector<std::vector<float> > minDistance(data->edges.size());
	std::map<unsigned short, std::map<unsigned short, bool> > isNewEdge;
	std::map<unsigned short, unsigned short> segmentInd; //which drill segment each new vertex lies in
	std::vector<std::vector<unsigned short> > segmentPoints(_segments); //the intersection points in each drill segment
	for(i = 0; i < data->edges.size(); i++) minDistance[i].resize(2);
	Vector3 dupTest;
	for(i = 0; i < data->edges.size(); i++) {
		for(j = 0; j < 2; j++) {
			hasInt[j] = false;
			lineInd[j] = -1;
			e[j] = data->edges[i][j];
			v[j].set(drillVertices[e[j]]);
			v[j].z = 0;
			minDistance[i][j] = 999999;
		}
		if(keep[e[0]] < 0 && keep[e[1]] < 0) continue;
		v[2].set(v[1]-v[0]);
		//validate the edge is really there
		if(v[2] == Vector3::zero()) continue;
		edge.set(v[0], v[2]);
		edgeLen = v[2].length();
		v[2].set(v[2] / edgeLen);
		//weed out edges that are completely outside the drill cross section
		v[3].set(v[1] - (v[2] * v[1].dot(v[2])));
		if(v[3].length() > _radius) continue;
		minAngleDiff = 1000; //(keep[e[0]] >= 0 && keep[e[1]] >= 0) ? -1000 : 1000;
		v1.set(v[0]);
		v2.set(v[1]);
		for(j = 0; j < _segments; j++) {
			//(v0 + a*v2) * r = planeDistance => a = (planeDistance - v0*r) / v2*r
			// v0 = endpoint, v2 = edge vector, r = radial vector to drill plane
			angle = j * dAngle + dAngle/2;
			v[3].set(sin(angle), cos(angle), 0);
			distance = (planeDistance - v1.dot(v[3])) / v[2].dot(v[3]);
			//only consider intersections outside the endpoints if we have already said that one endpoint is being discarded,
			//since in that case we need to find an intersection somewhere, and there may be imprecision
			if((distance <= 0 || distance >= edgeLen) && keep[e[0]] >= 0 && keep[e[1]] >= 0) continue;
			intersect.set(edge.getOrigin() + distance * edge.getDirection());
			//see if the intersection is in the correct angular range
			intersectAngle = atan2(intersect.x, intersect.y);
			while(intersectAngle < 0) intersectAngle += 2*M_PI;
			angle = (2*M_PI*j) / _segments;
			//again, if one endpoint is being discarded, allow tolerance in the angle check
			angleInside = (intersectAngle >= angle && intersectAngle <= angle+dAngle);
			if(angleInside) {
				angleDiff = -1;
			} else {
				angleDiff = fmin(fabs(intersectAngle-angle), fabs(intersectAngle-(angle+dAngle)));
				if(keep[e[0]] >= 0 && keep[e[1]] >= 0 && angleDiff > angleEpsilon) continue;
			}
			useInt = false;
			for(k = 0; k < 2; k++) {
				curDistance = k*edgeLen + (1-2*k)*distance;
				distanceInside = curDistance >= 0 && curDistance <= edgeLen;
				prevDistanceInside = minDistance[i][k] >= 0 && minDistance[i][k] <= edgeLen;
				prevAngleInside = minAngleDiff < 0;
				//priorities for deciding if this is better than an existing intersection:
				//1) we gain either the angle or distance being strictly within the bounds without losing the other
				if((angleInside && !prevAngleInside && (distanceInside || !prevDistanceInside)) ||
				   (distanceInside && !prevDistanceInside && (angleInside || !prevAngleInside))) better = true;
				//2) we move closer to the acceptable angle range
				else if(minAngleDiff >= angleEpsilon) better = angleDiff < minAngleDiff;
				//3) we are in the acceptable angle range and improve on an already valid distance
				else if(angleDiff < angleEpsilon && prevDistanceInside)
					better = distanceInside && curDistance < minDistance[i][k];
				//4) we are in the acceptable angle range and gain a valid distance
				else if(angleDiff < angleEpsilon && distanceInside) better = true;
				//5) default: we move closer to a valid distance
				else better = angleDiff < angleEpsilon && fmin(fabs(curDistance), fabs(curDistance - edgeLen))
					< fmin(fabs(minDistance[i][k]), fabs(minDistance[i][k] - edgeLen));
				if(better) {
					//if there is already an intersection for this edge, make sure we are not duplicating it due to imprecision
					if(hasInt[k] && v[k].distance(intersect) < 0.001f) continue;
					v[k].set(intersect);
					minAngleDiff = angleDiff;
					minDistance[i][k] = curDistance;
					lineInd[k] = j;
					hasInt[k] = true;
				}
			}
		}
		//if one endpoint is being discarded, we either need an intersection or we'll not discard it after all
		if(keep[e[0]] < 0 || keep[e[1]] < 0) {
			if(!hasInt[0] && !hasInt[1]) { //no intersection => keep it after all
				for(j = 0; j < 2; j++) if(keep[e[j]] < 0) {
					keep[e[j]] = newData.vertices.size();
					newData.vertices.push_back(data->worldVertices[e[j]]);
				}
				continue;
			}
			for(j = 0; j < 2; j++) if(!hasInt[j]) { //only found intersection in one direction => copy it to the other direction
				lineInd[j] = lineInd[1-j];
				v[j].set(v[1-j]);
				hasInt[j] = true;
			}
		}
		//don't allow both directions of the edge to intersect the same drill point, and don't allow edges where only one direction intersects the drill
		if((!hasInt[0] && hasInt[1]) || (hasInt[0] && !hasInt[1]) ||
		  (keep[e[0]] >= 0 && keep[e[1]] >= 0 && hasInt[0] && hasInt[1] && lineInd[0] == lineInd[1])) {
			continue;
		}
		else {
			//split this edge on the intersection point(s)
			if(hasInt[0] && hasInt[1]) {
				m = newData.vertices.size();
				for(j = 0; j < 2; j++) {
					n = keep[e[0]] >= 0 && keep[e[1]] >= 0 ? newData.vertices.size() : m;
					if(keep[e[j]] < 0) lineInd[j] = lineInd[1-j]; //if only keeping one endpoint, intersection is same both ways
					else {
						newData.vertices.push_back(data->worldVertices[e[j]]
							+ (data->worldVertices[e[(j+1)%2]] - data->worldVertices[e[j]]) * (minDistance[i][j] / edgeLen));
						segmentInd[n] = lineInd[j];
						segmentPoints[lineInd[j]].push_back(n);
					}
					edgeInt[e[j]][e[1-j]] = std::pair<unsigned short, unsigned short>(lineInd[j], n);
				}
			}
		}
	}

	//reconstruct the faces by leaving out the discarded points and incorporating the intersections
	std::vector<std::vector<unsigned short> > newFaces, newTriangles(1);
	std::vector<std::vector<short> > oldFaceInds;
	std::vector<unsigned short> newFace, faceKeep, newTriangle(3);
	std::vector<short> oldFaceInd, drillPoint;
	std::map<unsigned short, std::map<unsigned short, unsigned short> > faceEdgeInt;
	short faceSize, diff, bestLineDiff, newInd, start, current, next, ind, lastInter, startLine, endLine,
		line, newFaceSize, keepNext, dir, lineNum, drillIntCount;
	unsigned short eind[2];
	float minAngle, minDist, maxIntFraction;
	bool keepAll, drillInside, reverseFace;
	Matrix toolWorldModel;
	_tool->getWorldMatrix().invert(&toolWorldModel);
	Vector4 test4;
	for(i = 0; i < 3; i++) newTriangles[0].push_back(i);

	for(i = 0; i < data->faces.size(); i++) {
	
		//holds the (probably concave) polygons that this face will be split into - each of these will be triangulated
		//and each new triangle will be a separate face, to avoid concave faces
		newFaces.clear();
		//index of each vertex in the old face, if it is an original vertex
		oldFaceInds.clear();
		
		//determine the subset of vertices of this face that we are keeping
		faceSize = data->faces[i].size();
		faceKeep.clear();
		hasInt.resize(faceSize);
		drillPoint.resize(faceSize, -1);
		drillInside = false;
		keepAll = true; //true if all vertices are being kept AND there are no edge intersections with the drill
		faceEdgeInt.clear(); //make a temp index of edge intersections for this face
		
		for(j = 0; j < faceSize; j++) {
			for(k = 0; k < 2; k++) {
				for(m = 0; m < 2; m++) {
					eind[m] = (j+1-(m+k)%2)%faceSize;
					e[m] = data->faces[i][eind[m]];
				}
				hasInt[j] = edgeInt.find(e[0]) != edgeInt.end()	&& edgeInt[e[0]].find(e[1]) != edgeInt[e[0]].end();
				if(hasInt[j] && keep[e[0]] >= 0)
					faceEdgeInt[eind[0]][eind[1]] = edgeInt[e[0]][e[1]].second;
			}
			if(keep[data->faces[i][j]] >= 0) {
				faceKeep.push_back(j);
				keepAll = keepAll && !hasInt[j];
			}
			else keepAll = false;
		}
		short faceIntSize = faceEdgeInt.size();
		if(faceKeep.empty()) continue;
		if(keepAll) {
			newFace.resize(faceSize);
			for(j = 0; j < faceSize; j++) newFace[j] = keep[data->faces[i][j]];
			//see if the drill cross section is entirely inside the face, in which case treat it specially
			drillInside = true;
			if(faceKeep.size() == faceSize) { //implies that all vertices are being kept
				for(j = 0; j < _segments; j++) {
					if(drillInt.find(j) == drillInt.end() || drillInt[j].find(i) == drillInt[j].end()) {
						drillInside = false;
						break;
					}
				}
			}
			if(!drillInside) {
				addFace(newFace, data->triangles[i]);
				cout << "face " << i << " untouched" << endl;
				for(j = 0; j < faceSize; j++) cout << data->faces[i][j] << " ";
				cout << endl;
				for(j = 0; j < data->triangles[i].size(); j++) {
					cout << "\t";
					for(k = 0; k < 3; k++) cout << data->triangles[i][j][k] << " ";
					cout << endl;
				}
				continue;
			}
			else {
				newFaces.push_back(newFace);
				oldFaceInd.resize(faceSize);
				for(j = 0; j < faceSize; j++) oldFaceInd[j] = j;
				oldFaceInds.push_back(oldFaceInd);
			}
			faceKeep.clear();
		}
		
		while(!faceEdgeInt.empty()) {
			newFace.clear();
			oldFaceInd.clear();
			//begin each new face on an edge intersection point
			it = faceEdgeInt.begin();
			it1 = it->second.begin();
			newFace.push_back(it1->second);
			oldFaceInd.push_back(-1);
			dir = it1->first == (it->first+1)%faceSize ? -1 : 1;
			reverseFace = dir == -1;
			start = it->first;
			endLine = edgeInt[data->faces[i][it->first]][data->faces[i][it1->first]].first;
			lastInter = it1->second;
			it->second.erase(it1->first);
			if(it->second.empty()) faceEdgeInt.erase(it->first);
			//loop around to the next edge intersection point
			for(ind = start;
			  faceEdgeInt.find(ind) == faceEdgeInt.end() || faceEdgeInt[ind].find((ind+dir+faceSize)%faceSize) == faceEdgeInt[ind].end();
			  ind = (ind+dir+faceSize)%faceSize) {
				newFace.push_back(keep[data->faces[i][ind]]);
				oldFaceInd.push_back(ind);
				lastInter = keep[data->faces[i][ind]];
			}
			newFace.push_back(keep[data->faces[i][ind]]);
			oldFaceInd.push_back(ind);
			lastInter = keep[data->faces[i][ind]];
			//add the next edge intersection
			n = faceEdgeInt[ind][(ind+dir+faceSize)%faceSize];
			newFace.push_back(n);
			oldFaceInd.push_back(-1);
			lastInter = n;
			//and get rid of it so we don't try to reuse it
			faceEdgeInt[ind].erase((ind+dir+faceSize)%faceSize);
			if(faceEdgeInt[ind].empty()) faceEdgeInt.erase(ind);
			
			startLine = edgeInt[data->faces[i][ind]][data->faces[i][(ind+dir+faceSize)%faceSize]].first;

			//determine the range of angles wrt drill center covered by the face path so far
			//-then iterate the reverse direction along the drill edge to complete the face
			float angleRange = 0, lastAngle;
			for(k = 0; k < newFace.size(); k++) {
				test.set(newData.vertices[newFace[k]] - _node->getTranslationWorld());
				drillRot.transformPoint(&test);
				testAngle = atan2(test.y, test.x);
				if(k > 0) {
					while(fabs(testAngle - lastAngle) > M_PI) testAngle += testAngle > lastAngle ? -2*M_PI : 2*M_PI;
					angleRange += testAngle - lastAngle;
				}
				lastAngle = testAngle;
			}
			dir = angleRange > 0 ? -1 : 1;
			
			//iterate in the chosen direction, adding intersection points
			for(lineNum = (startLine + (dir == -1 ? 0 : 1) + _segments) % _segments;
			  lineNum != (endLine + (dir == -1 ? 0 : 1) + _segments) % _segments;
			  lineNum = (lineNum+dir+_segments) % _segments) {
				if(drillInt.find(lineNum) != drillInt.end() && drillInt[lineNum].find(i) != drillInt[lineNum].end()) {
					newFace.push_back(drillInt[lineNum][i]);
					oldFaceInd.push_back(-1);
					addDrillEdge(lastInter, drillInt[lineNum][i], (lineNum - (dir == -1 ? 0 : 1) + _segments) % _segments, i);
					lastInter = drillInt[lineNum][i];
				}
			}
			addDrillEdge(lastInter, newFace[0], endLine, i);
			//add the new face to the list of new faces, reversing it if we built it backwards
			if(!newFace.empty()) {
				m = newFaces.size();
				n = newFace.size();
				newFaces.resize(m+1);
				newFaces[m].resize(n);
				oldFaceInds.resize(m+1);
				oldFaceInds[m].resize(n);
				for(j = 0; j < n; j++) {
					newFaces[m][j] = reverseFace ? newFace[n-j-1] : newFace[j];
					oldFaceInds[m][j] = reverseFace ? oldFaceInd[n-j-1] : oldFaceInd[j];
				}
			}
		}
		if(drillInside) for(j = 0; j < _segments; j++) {
			newFaces[0].push_back(drillInt[j][i]);
			oldFaceInds[0].push_back(-1);
			addDrillEdge(drillInt[j][i], drillInt[(j+1)%_segments][i], j, i);
		}

		//triangulate each of the new polygons - also, note left/right edges within the drill bit
		for(j = 0; j < newFaces.size(); j++) {
			newFaceSize = newFaces[j].size();
			
			for(k = 0; k < faceSize; k++) drillPoint[k] = -1;

			//for each edge that is from the original face
			for(k = 0; k < newFaceSize; k++) {
				if(oldFaceInds[j][k] < 0 || (!drillInside && oldFaceInds[j][(k+1)%newFaceSize] < 0)) continue;
				for(m = (k+1)%newFaceSize; oldFaceInds[j][m] < 0; m = (m+1)%newFaceSize); //other endpoint of edge
				//find the drill bit intersection point closest to this line segment
				// - will be used to make the triangle with that edge - call this an "anchor point"
				v2.set(newData.vertices[newFaces[j][m]] - newData.vertices[newFaces[j][k]]);
				edgeLen = v2.length();
				v2.normalize(&v2);
				minDist = 9999999.0f;
				for(n = 0; n < newFaceSize; n++) {
					if(oldFaceInds[j][n] >= 0) continue;
					v3.set(newData.vertices[newFaces[j][k]] - newData.vertices[newFaces[j][n]]);
					f1 = v3.dot(v2);
					if(f1 > 0 || f1 < -edgeLen) { //closest distance to edge's line is not between endpoints
						distance = v3.length();
						v3.set(newData.vertices[newFaces[j][m]] - newData.vertices[newFaces[j][n]]);
						if(v3.length() < distance) distance = v3.length();
					} else {
						v3.set(v3 - v2 * f1);
						distance = v3.length();
					}
					if(distance < minDist) {
						minDist = distance;
						drillPoint[oldFaceInds[j][k]] = n;
					}
				}
			}

			//find the end of the sequence of vertices from the drill intersection
			start = 0;
			while((oldFaceInds[j][start] >= 0 || oldFaceInds[j][start+1] < 0) && start < newFaceSize-1) start++;
			if(oldFaceInds[j][start] >= 0) {
				GP_WARN("Couldn't find drill intersection starting point for face %d", i);
				return false;
			}
			
			//if the drill cross-section is entirely inside the face, determine its orientation relative to the face
			if(drillInside) {
				v[0].set(newData.vertices[drillInt[1][i]] - newData.vertices[drillInt[0][i]]);
				v[1].set(newData.vertices[drillInt[2][i]] - newData.vertices[drillInt[1][i]]);
				v[0].cross(v[1]);
				dir = v[0].dot(faces[i].getNormal()) > 0 ? 1 : -1;
			}
			else dir = -1;
			
			//fill in the triangles between the anchor points
			n = (start+1)%newFaceSize;
			if(!drillInside) m = start;
			else m = drillPoint[oldFaceInds[j][n]];
			do {
				if(m == drillPoint[oldFaceInds[j][n]]) {
					newTriangle[0] = n;
					do {
						n = (n+1)%newFaceSize;
					} while(oldFaceInds[j][n] < 0);
					newTriangle[1] = n;
					newTriangle[2] = m;
				} else {
					newTriangle[0] = m;
					newTriangle[1] = n;
					do {
						m = (m+dir+newFaceSize)%newFaceSize;
					} while(oldFaceInds[j][m] >= 0);
					newTriangle[2] = m;
				}
				for(k = 0; k < 3; k++) newTriangle[k] = newFaces[j][newTriangle[k]];
				addFace(newTriangle, newTriangles); //each triangle is a separate face - not optimal but much easier
			} while((!drillInside && (n+1)%newFaceSize != m)
				|| (drillInside && (m != drillPoint[(start+1)%newFaceSize] || n != (start+1)%newFaceSize)));
		}
	}
	
	//add the new faces formed by the walls of the drill cylinder
	unsigned short newFaceStart = newData.faces.size(), numInt;
	std::vector<std::vector<std::pair<unsigned short, float> > > lineInt(_segments);
	std::vector<std::pair<unsigned short, float> >::iterator vit;
	for(it = drillInt.begin(); it != drillInt.end(); it++) {
		lineNum = it->first;
		v1.set(lines[lineNum].getOrigin());
		//sort the drill line intersections by distance along the ray
		for(it1 = drillInt[lineNum].begin(); it1 != drillInt[lineNum].end(); it1++) {
			n = it1->second;
			v2.set(newData.vertices[n] - v1);
			edgeLen = v2.length();
			for(vit = lineInt[lineNum].begin();	vit != lineInt[lineNum].end() && vit->second < edgeLen; vit++);
			lineInt[lineNum].insert(vit, std::pair<unsigned short, float>(n, edgeLen));
		}
		//now they are ordered, note the edges they form
		numInt = lineInt[lineNum].size();
		for(i = 0; i < numInt; i++) {
			for(j = 0; j < 2; j++) e[j] = lineInt[lineNum][i+j].first;
			if(i < numInt-1 && enterInt[e[0]] && !enterInt[e[1]]) {
				addEdge(e[0], e[1]);
				addDrillEdge(e[0], e[1], lineNum, -1); //-1 indicates this edge is along a drill line, not in a drill plane
			}
			segmentPoints[lineNum].push_back(e[0]);
			segmentPoints[(lineNum-1+_segments)%_segments].push_back(e[0]);
		}
	}
	
	//for each segment of the drill bit, use its known set of points and edges to determine all its faces
	std::map<unsigned short, std::map<unsigned short, bool> > edges; //per segment
	std::map<unsigned short, std::map<unsigned short, bool> >::iterator eit;
	std::map<unsigned short, bool>::iterator eit2;
	for(i = 0; i < _segments; i++) {
		//make a list of all edges in this segment - use the direction where the model interior is to the left
		edges.clear();
		for(j = 0; j < segmentPoints[i].size(); j++) {
			p = segmentPoints[i][j];
			for(it1 = newData.edgeInd[p].begin(); it1 != newData.edgeInd[p].end(); it1++) {
				q = it1->first;
				if(edgeLine.find(p) == edgeLine.end() || edgeLine[p].find(q) == edgeLine[p].end()) continue;
				if(edgeLine[p][q] == i &&
				  (leftEdge.find(p) == leftEdge.end() || leftEdge[p].find(q) == leftEdge[p].end() || leftEdge[p][q] == true)) {
					edges[p][q] = true;
				}
				else if(edgeLine[p][q] == (i+1)%_segments &&
				  (leftEdge.find(p) == leftEdge.end() || leftEdge[p].find(q) == leftEdge[p].end())) {
					edges[q][p] = true;
				}
			}
		}
		//just keep building cycles until all edges are used
		newFace.clear();
		while(!edges.empty()) {
			if(newFace.empty()) {
				eit = edges.begin();
				p = eit->first;
				newFace.push_back(p);
			} else {
				p = newFace[newFace.size()-1];
			}
			if(edges.find(p) == edges.end()) {
				GP_WARN("Couldn't continue drill face from point %d", p);
				return false;
			}
			eit2 = edges[p].begin();
			q = eit2->first;
			edges[p].erase(q);
			if(edges[p].empty()) edges.erase(p);
			if(!newFace.empty() && q == newFace[0]) { //when cycle complete, triangulate and add the new face
				newTriangles.clear();
				addFace(newFace, newTriangles);
				newFace.clear();
			} else newFace.push_back(q);
		}
	}
	
	cout << "NEW FACES: " << endl;
	for(i = newFaceStart; i < newData.faces.size(); i++) {
		for(j = 0; j < newData.faces[i].size(); j++) cout << newData.faces[i][j] << " ";
		cout << endl;
		for(j = 0; j < newData.triangles[i].size(); j++) {
			for(k = 0; k < 3; k++) cout << " " << newData.triangles[i][j][k];
			cout << endl;
		}
	}
	
	//recalculate face neighbors
	newData.faceNeighbors.resize(newData.faces.size());
	for(i = 0; i < newData.faces.size(); i++) {
		p = newData.faces[i].size();
		for(j = 0; j < p; j++) {
			for(k = i+1; k < newData.faces.size(); k++) {
				q = newData.faces[k].size();
				for(m = 0; m < q; m++) {
					if(newData.faces[i][j] == newData.faces[k][m]
					  && newData.faces[i][(j+1)%p] == newData.faces[k][(m-1+q)%q]) {
						newData.faceNeighbors[i].push_back(k);
						newData.faceNeighbors[k].push_back(i);
					}
				}
			}
		}
	}
	
	//just add 1 convex hull for each convex face - is there a better way?
	for(i = 0; i < newData.faces.size(); i++)
		newData.hulls.push_back(newData.faces[i]);
	
	//transform the new vertices back to model space before saving the data
	Matrix worldModel;
	_node->getWorldMatrix().invert(&worldModel);
	Vector3 translation(_node->getTranslationWorld()), scale(_node->getScale());
	newData.translation.set(translation);
	newData.scale.set(scale);
	translation.x /= scale.x; translation.y /= scale.y; translation.z /= scale.z;
	for(i = 0; i < newData.vertices.size(); i++) {
		worldModel.transformVector(&newData.vertices[i]);
		newData.vertices[i] -= translation;
	}
	
	//write the new node data to a file with suffix '_slice' and read it back in
	std::string filename;
	char newID[40];
	for(i = 0; i < 40; i++) newID[i] = '\0';
	int count = 1;
	do {
		sprintf(newID, "%s_drill%d", data->type.c_str(), count++);
		filename = newID;
		filename = "res/common/" + filename + ".node";
	}while(FileSystem::fileExists(filename.c_str()));
	Node::writeData(&newData, filename.c_str());
	app->removeNode(_node, newID);
	app->loadNodeFromData(newID);
	//make sure the node is not hovering if its lower part was sliced off
	Node *newNode = app->_scene->findNode(newID);
	if(newNode) {
		translation.set(newNode->getTranslationWorld());
		app->placeNode(newNode, translation.x, translation.z);
	}
	return true;
}

