#include "T4TApp.h"

T4TApp::DrillMode::DrillMode() 
  : T4TApp::ToolMode::ToolMode("mode_Drill", "res/common/drill.form") {
	_axis.set(Vector3(0, 0, 0), Vector3(1, 0, 0));
	_radius = 0.2f;
	_segments = 20;
	_tool = MyNode::create("drill");
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
	_selectedNode->getRotation(&rotation);
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

	MyNode::nodeData *data = _selectedNode->getData();

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
			for(int k = 0; k < 3; k++) vertices[v++] = MyNode::gv(&v1, k);
			for(int k = 0; k < 3; k++) vertices[v++] = MyNode::gv(&color, k);
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
			v1.set(newData->vertices[drillInt[i][f]] + 0.01f * plane.getNormal());
			for(int j = 0; j < 2; j++) {
				if(j == 0 && vertices.empty()) continue;
				for(int k = 0; k < 3; k++) vertices.push_back(MyNode::gv(&v1, k));
				for(int k = 0; k < 3; k++) vertices.push_back(MyNode::gv(&color, k));
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
		v1.set(lines[i].getOrigin() + lines[i].getDirection() * distance + 0.01f*plane.getNormal());
		for(int j = 0; j < 2; j++) {
			for(int k = 0; k < 3; k++) vertices[v++] = MyNode::gv(&v1, k);
			for(int k = 0; k < 3; k++) vertices[v++] = MyNode::gv(&color, k);
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

void T4TApp::DrillMode::addDrillEdge(unsigned short v1, unsigned short v2, unsigned short lineNum) {
	_newNode->addEdge(v1, v2);
	segmentEdges[lineNum][v1] = v2;
}

bool T4TApp::DrillMode::toolNode() {
	ToolMode::toolNode();

	//initialize recycled member variables
	newEdge.resize(2);
	edgeLine.clear();
	leftEdge.clear();
	enterInt.clear();
	edgeInt.clear();
	drillInt.clear();
	segmentEdges.clear();
	
	//temp variables
	short i, j, k, m, n, p, q, r;
	float f1, f2, f3, f4, s, t, f[4];
	Vector3 v1, v2, v3, v4;
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
	Matrix drillRot;
	trans.invert(&drillRot);
	Matrix axisSwap(0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
	Matrix::multiply(axisSwap, drillRot, &drillRot);
	drillVertices.resize(data->vertices.size());
	for(i = 0; i < data->vertices.size(); i++) {
		drillVertices[i].set(data->worldVertices[i]);
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
			keep[i] = newData->vertices.size();
			newData->vertices.push_back(data->worldVertices[i]);
		}
	}

	//find all intersections between edges of the model and the planes of the drill bit
	Vector3 drillVec, intersect;
	unsigned short e[2], lineInd[2], lineNum;
	float d, edgeLen, distance, lineAngle;
	for(i = 0; i < data->edges.size(); i++) {
		for(j = 0; j < 2; j++) {
			e[j] = data->edges[i][j];
			v[j].set(drillVertices[e[j]]);
			v[j].z = 0;
		}
		if(keep[e[0]] < 0 && keep[e[1]] < 0) continue;
		v[2].set(v[1]-v[0]);
		//validate the edge is really there
		if(v[2] == Vector3::zero()) continue;
		//find the angles where this edge enters and exits the drill circle
		//|v0 + a*v2| = r => (v0x + a*v2x)^2 + (v0y + a*v2y)^2 = r^2 => |v2|^2 * a^2 + 2*(v0*v2)*a + |v0|^2 - r^2 = 0
		// => a = [-2*v0*v2 +/- sqrt(4*(v0*v2)^2 - 4*(|v2|^2)*(|v0|^2 - r^2))] / 2*|v2|^2
		// = [-v0*v2 +/- sqrt((v0*v2)^2 - (|v2|^2)*(|v0|^2 - r^2)) / |v2|^2
		f1 = v[0].dot(v[2]);
		f2 = v[0].lengthSquared() - _radius * _radius;
		f3 = v[2].lengthSquared();
		d = f1*f1 - f3*f2;
		if(d < 0) continue; //edge is completely outside drill circle
		s = (-f1 - sqrt(d)) / f3;
		t = (-f1 + sqrt(d)) / f3;
		//neither intersection should be outside the endpoints if we are keeping both of them
		if((s < 0 || t > 1) && keep[e[0]] >=0 && keep[e[1]] >= 0) continue;
		//for each edge direction, first get the segment from the angle where it intersects the circle
		for(j = 0; j < 2; j++) {
			if(keep[e[j]] < 0) continue;
			v1.set(v[0] + v[2] * (j == 0 ? s : t));
			angle = atan2(v1.x, v1.y);
			while(angle < 0) angle += 2*M_PI;
			lineInd[j] = (unsigned short)(angle / dAngle);
		}
		//if keeping both endpoints and they hit the same angle segment, the edge must not really touch the drill plane
		if(keep[e[0]] >= 0 && keep[e[1]] >= 0 && lineInd[0] == lineInd[1]) continue;
		//for each edge direction, find the point where it intersects the drill plane in the angle range found above
		for(j = 0; j < 2; j++) {
			if(keep[e[j]] < 0) continue;
			lineAngle = lineInd[j] * dAngle + dAngle/2;
			v2.set(sin(lineAngle), cos(lineAngle), 0);
			//(v0 + a*v2) * r = planeDistance => a = (planeDistance - v0*r) / v2*r
			distance = (planeDistance - v[0].dot(v2)) / v[2].dot(v2);
			n = newData->vertices.size();
			newData->vertices.push_back(data->worldVertices[e[0]]
			  + (data->worldVertices[e[1]] - data->worldVertices[e[0]]) * distance);
			edgeInt[e[j]][e[(j+1)%2]] = std::pair<unsigned short, unsigned short>(lineInd[j], n);
		}
		//if either vertex is not being kept, both directions intersect the drill at the same point
		for(j = 0; j < 2; j++) {
			if(keep[e[j]] >= 0) continue;
			edgeInt[e[j]][e[(j+1)%2]] = edgeInt[e[(j+1)%2]][e[j]];
		}
	}
	
	//loop around each original face, building modified faces according to the drill intersections
	bool ccw, drillInside, found;
	short dir, offset, mode, numInts;
	unsigned short lastInter;
	float denom, delta, epsilon = 0.001f, minDist;
	std::pair<unsigned short, unsigned short> ints[2];
	Plane facePlane;
	std::vector<unsigned short> newFace;
	std::vector<std::vector<unsigned short> > newTriangles;
	std::vector<unsigned short> drillPoint;
	std::set<unsigned short> keeping;
	std::set<unsigned short>::iterator kit;
	std::map<unsigned short, std::vector<std::pair<unsigned short, float> > > faceInts;
	std::vector<std::pair<unsigned short, float> > angles;
	std::vector<std::pair<unsigned short, float> >::iterator fit;
	for(i = 0; i < data->faces.size(); i++) {
		n = data->faces[i].size();
		keeping.clear();
		faceInts.clear();
		//index the edge intersections in this face by drill line number
		for(j = 0; j < n; j++) {
			p = data->faces[i][j];
			if(keep[p] < 0) continue;
			keeping.insert(j);
			q = data->faces[i][(j-1+n)%n]; //since we always walk the face in order, we only need to store back intersections
			if(edgeInt.find(p) != edgeInt.end() && edgeInt[p].find(q) != edgeInt[p].end()) {
				lineNum = edgeInt[p][q].first;
				v1.set(newData->vertices[edgeInt[p][q].second] - _axis.getOrigin());
				angle = atan2(v1.dot(right), v1.dot(up));
				while(angle < 0) angle += 2*M_PI;
				//within a given line number, if there are multiple intersections, order by angle wrt drill center
				// - order in the same direction we will walk the drill
				for(fit = faceInts[lineNum].begin(); fit != faceInts[lineNum].end() && 
				  (dir == 1 ? angle > fit->second : angle < fit->second); fit++);
				faceInts[lineNum].insert(fit, std::pair<unsigned short, float>(j, angle));
			}
		}
		if(keeping.empty()) continue; //not keeping any part of this face
		ccw = data->normals[i].dot(axis) < 0;
		dir = ccw ? 1 : -1;
		offset = dir == 1 ? 1 : 0;
		facePlane.set(data->normals[i],
		  -data->worldVertices[data->faces[i][0]].dot(data->normals[i]));

		//if there are edge intersections, build the set of new faces that this one is split into
		if(!faceInts.empty()) while(!keeping.empty()) {
			//pick any vertex we are keeping, from which to build a face
			kit = keeping.begin();
			j = *kit;
			keeping.erase(kit);
			newFace.clear();
			p = data->faces[i][j];
			mode = 0;
			numInts = 0;
			do {
				switch(mode) {
					case 0: //walking the original face
						newFace.push_back(keep[p]);
						keeping.erase(j);
						j = (j+1)%n;
						q = data->faces[i][j];
						if(edgeInt.find(p) != edgeInt.end() && edgeInt[p].find(q) != edgeInt[p].end()) {
							mode = 1;
							lineNum = edgeInt[p][q].first;
							p = edgeInt[p][q].second;
						} else {
							p = q;
						}
						break;
					case 1: //at an edge intersection
						mode = 2;
						newFace.push_back(p);
						lastInter = p;
						v1.set(newData->vertices[p] - _axis.getOrigin());
						angle = atan2(v1.dot(right), v1.dot(up));
						while(angle < 0) angle += 2*M_PI;
						break;
					case 2: //walking the drill
						//first see if there is an edge intersection exiting the drill in the current segment
						if(faceInts.find(lineNum) != faceInts.end()) {
							if(angle < 0) fit = faceInts[lineNum].begin();
							else for(fit = faceInts[lineNum].begin(); fit != faceInts[lineNum].end() && 
							  (dir == 1 ? fit->second < angle : fit->second > angle); fit++);
							if(fit != faceInts[lineNum].end()) {
								k = fit->first;
								p = data->faces[i][k];
								q = edgeInt[p][data->faces[i][(k-1+n)%n]].second;
								newFace.push_back(q);
								addDrillEdge(q, lastInter, lineNum);
								faceInts[lineNum].erase(fit);
								if(faceInts[lineNum].empty()) faceInts.erase(lineNum);
								mode = 0;
								j = k;
								break;
							}
						}
						//otherwise the next drill line must intersect the face
						k = (lineNum + offset) % _segments;
						m = newData->vertices.size();
						distance = lines[k].intersects(facePlane);
						newData->vertices.push_back(lines[k].getOrigin() + distance * lines[k].getDirection());
						newFace.push_back(m);
						drillInt[k][i] = m;
						addDrillEdge(m, lastInter, lineNum);
						lastInter = m;
						lineNum = (lineNum + dir + _segments) % _segments;
						angle = -1;
						numInts++;
						break;
				}
			} while((mode != 0 || keep[p] != newFace[0]) && numInts <= _segments);
			if(numInts > _segments) {
				GP_WARN("Couldn't exit drill on face %d", i);
				return false;
			}
			newTriangles.clear();
			_newNode->addFace(newFace, newTriangles);
		} else { //no edge intersections => only question is whether entire drill bit passes through this face
			//use drill center as indicator
			drillInside = false;
			for(j = 0; j < data->triangles[i].size(); j++) {
				v1.set(drillVertices[data->faces[i][data->triangles[i][j][1]]]
					- drillVertices[data->faces[i][data->triangles[i][j][0]]]);
				v2.set(drillVertices[data->faces[i][data->triangles[i][j][2]]]
					- drillVertices[data->faces[i][data->triangles[i][j][0]]]);
				drillVec.set(-drillVertices[data->faces[i][data->triangles[i][j][0]]]);
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
				drillInside = true;
				break;
			}
			if(drillInside) {
				//first get all the drill line intersections on this face
				for(j = 0; j < _segments; j++) {
					distance = lines[j].intersects(facePlane);
					drillInt[j][i] = newData->vertices.size();;
					newData->vertices.push_back(lines[j].getOrigin() + lines[j].getDirection() * distance);
				}
				//add edges on the drill segments making sure to get the right orientation
				for(j = 0; j < _segments; j++)
					addDrillEdge(drillInt[j][i], drillInt[(j-dir+_segments)%_segments][i], (j-offset+_segments)%_segments);
				//order the face vertices by angle wrt drill center
				angles.clear();
				for(j = 0; j < n; j++) {
					k = data->faces[i][j];
					angle = atan2(drillVertices[k].x, drillVertices[k].y);
					while(angle < 0) angle += 2*M_PI;
					for(fit = angles.begin(); fit != angles.end() && angle > fit->second; fit++);
					angles.insert(fit, std::pair<unsigned short, float>(j, angle));
				}
				//for each of two opposite drill vertices, find an edge to a face vertex, thus splitting the face into two
				for(j = 0; j < 2; j++) {
					p = drillInt[j * _segments/2][i];
					angle = dAngle * j * _segments/2;
					for(m = 0; m < n && angles[m].second < angle; m++);
					m %= n;
					//try vertices as close in angle to this drill point as possible until we find a valid edge
					// (doesn't intersect any existing edges of the face)
					offset = 1;
					do {
						q = angles[m].first;
						v1.set(_radius * sin(angle), _radius * cos(angle), 0);
						v2.set(drillVertices[data->faces[i][q]] - v1);
						found = true;
						for(k = (q+1)%n; k != (q-1+n)%n; k = (k+1)%n) {
							v3.set(drillVertices[data->faces[i][k]]);
							v4.set(drillVertices[data->faces[i][(k+1)%n]] - v3);
							//v1 + a*v2 = v3 + b*v4 => [v2 -v4][a b] = v3 - v1
							// => [a b] = [v2 -v4]^-1 * (v3 - v1)
							denom = -v2.x * v4.y + v4.x * v2.y;
							s = (-v4.y * (v3.x - v1.x) + v4.x * (v3.y - v1.y)) / denom;
							t = (-v2.y * (v3.x - v1.x) + v2.x * (v3.y - v1.y)) / denom;
							if(s >= 0 && s <= 1 && t >= 0 && t <= 1) {
								found = false;								
								break;
							}
						}
						if(found) break;
						m = (m + offset*(offset % 2 == 1 ? -1 : 1) + n) % n;
						offset++;
					} while(offset < n);
					if(offset == n) {
						GP_WARN("Can't get edge in face %d from drill point %d", i, p);
						return false;
					}
					e[j] = q;
				}
				//perform the split
				for(j = 0; j < 2; j++) {
					newFace.clear();
					newTriangles.clear();
					m = (j+1)%2;
					for(k = e[j]; k != (e[m]+1)%n; k = (k+1)%n)
						newFace.push_back(keep[data->faces[i][k]]);
					for(k = m * _segments/2; k != (j*_segments/2+dir+_segments) % _segments; k = (k+dir+_segments)%_segments)
						newFace.push_back(drillInt[k][i]);
					_newNode->addFace(newFace, newTriangles);
				}
			} else { //the drill does not touch this face => leave it as is
				_newNode->addFace(data->faces[i], data->triangles[i]);
			}
		}
	}
	
	//add the new faces formed by the walls of the drill cylinder
	unsigned short newFaceStart = newData->faces.size(), numInt;
	std::vector<std::vector<std::pair<unsigned short, float> > > lineInt(_segments);
	std::vector<std::pair<unsigned short, float> >::iterator vit;
	std::map<unsigned short, std::map<unsigned short, unsigned short> >::iterator it;
	std::map<unsigned short, unsigned short>::iterator it1;
	for(it = drillInt.begin(); it != drillInt.end(); it++) {
		lineNum = it->first;
		v1.set(lines[lineNum].getOrigin());
		//sort the drill line intersections by distance along the ray
		for(it1 = drillInt[lineNum].begin(); it1 != drillInt[lineNum].end(); it1++) {
			n = it1->second;
			v2.set(newData->vertices[n] - v1);
			edgeLen = v2.length();
			for(vit = lineInt[lineNum].begin();	vit != lineInt[lineNum].end() && vit->second < edgeLen; vit++);
			lineInt[lineNum].insert(vit, std::pair<unsigned short, float>(n, edgeLen));
			enterInt[n] = data->normals[it1->first].dot(axis) < 0;
		}
		//now they are ordered, note the edges they form
		numInt = lineInt[lineNum].size();
		for(i = 0; i < numInt; i++) {
			for(j = 0; j < 2; j++) e[j] = lineInt[lineNum][i+j].first;
			if(i < numInt-1 && enterInt[e[0]] && !enterInt[e[1]]) {
				addDrillEdge(e[0], e[1], lineNum);
				addDrillEdge(e[1], e[0], (lineNum-1+_segments)%_segments);
			}
		}
	}
	
	//for each segment of the drill bit, use its known set of points and edges to determine all its faces
	std::map<unsigned short, unsigned short>::iterator eit;
	for(i = 0; i < _segments; i++) {
		//just keep building cycles until all edges are used
		newFace.clear();
		while(!segmentEdges[i].empty()) {
			if(newFace.empty()) {
				eit = segmentEdges[i].begin();
				p = eit->first;
				newFace.push_back(p);
			} else {
				p = newFace[newFace.size()-1];
			}
			if(segmentEdges[i].find(p) == segmentEdges[i].end()) {
				GP_WARN("Couldn't continue drill face from point %d", p);
				return false;
			}
			q = segmentEdges[i][p];
			segmentEdges[i].erase(p);
			if(!newFace.empty() && q == newFace[0]) { //when cycle complete, triangulate and add the new face
				newTriangles.clear();
				_newNode->addFace(newFace, newTriangles);
				newFace.clear();
			} else newFace.push_back(q);
		}
	}

	//recalculate face neighbors
	newData->faceNeighbors.resize(newData->faces.size());
	for(i = 0; i < newData->faces.size(); i++) {
		p = newData->faces[i].size();
		for(j = 0; j < p; j++) {
			for(k = i+1; k < newData->faces.size(); k++) {
				q = newData->faces[k].size();
				for(m = 0; m < q; m++) {
					if(newData->faces[i][j] == newData->faces[k][m]
					  && newData->faces[i][(j+1)%p] == newData->faces[k][(m-1+q)%q]) {
						newData->faceNeighbors[i].push_back(k);
						newData->faceNeighbors[k].push_back(i);
					}
				}
			}
		}
	}
	
	//just add 1 convex hull for each convex face - is there a better way?
	for(i = 0; i < newData->faces.size(); i++)
		newData->hulls.push_back(newData->faces[i]);
	
	//transform the new vertices back to model space before saving the data
	Matrix worldModel;
	_selectedNode->getWorldMatrix().invert(&worldModel);
	Vector3 translation(_selectedNode->getTranslationWorld()), scale(_selectedNode->getScale());
	newData->translation.set(translation);
	newData->scale.set(scale);
	translation.x /= scale.x; translation.y /= scale.y; translation.z /= scale.z;
	for(i = 0; i < newData->vertices.size(); i++) {
		worldModel.transformVector(&newData->vertices[i]);
		newData->vertices[i] -= translation;
	}
	
	_newNode->setData(NULL);
	_selectedNode->setData(newData);
	_selectedNode->updateModelFromData();
	translation.set(_selectedNode->getTranslationWorld());
	app->placeNode(_selectedNode, translation.x, translation.z);
	return true;
}

