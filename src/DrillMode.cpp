#include "T4TApp.h"

T4TApp::DrillMode::DrillMode(T4TApp *app_) 
  : T4TApp::ToolMode::ToolMode(app_, "mode_Drill", "res/common/drill.form") {
	_axis.set(Vector3(0, 0, 0), Vector3(1, 0, 0));
	_radius = 0.2f;
	_segments = 20;
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
	_tool = Node::create("drill");
	_tool->setModel(model);
	model->release();
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

bool T4TApp::DrillMode::toolNode() {
	ToolMode::toolNode();
	_node->updateData();
	Node::nodeData *data = (Node::nodeData*)_node->getUserPointer();

	//build drilled node into a new node data structure, save to disk, and reload
	newData.type = data->type;
	newData.objType = "mesh"; //can't keep sphere/box collision object once it is deformed!
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
	
	short i, j, k, m, n;

	float angle, dAngle = 2*M_PI/_segments, planeDistance = _radius * cos(dAngle/2);
	Matrix trans(_tool->getWorldMatrix());
	for(i = 0; i < _segments; i++) {
		angle = (2*M_PI*i) / _segments;
		//line
		lines[i].setOrigin(-1000.0f, _radius*cos(angle), _radius*sin(angle));
		lines[i].setDirection(1, 0, 0);
		lines[i].transform(trans);
		//plane
		planes[i].setNormal(0, cos(angle+dAngle/2), sin(angle+dAngle/2));
		planes[i].setDistance(-planeDistance);
		planes[i].transform(trans);
	}
	_axis.setOrigin(-1000.0f, 0, 0);
	_axis.setDirection(1, 0, 0);
	_axis.transform(trans);
	edgeInt.clear();
	drillInt.clear();

	//determine which vertices to discard because they are inside the drill cylinder
	std::vector<short> keep(data->vertices.size());
	Matrix toolWorldModel;
	_tool->getWorldMatrix().invert(&toolWorldModel);
	Vector3 test;
	Vector4 test4;
	float testAngle, testRadius;
	for(i = 0; i < data->vertices.size(); i++) {
		keep[i] = -1;
		test.set(data->worldVertices[i]);
		test4.set(test.x, test.y, test.z, 1);
		toolWorldModel.transformVector(&test4);
		testAngle = atan2(test4.z, test4.y);
		testRadius = sqrt(test4.y*test4.y + test4.z*test4.z);
		float ang = fabs(fmod(testAngle, dAngle) - dAngle/2), radius = planeDistance / cos(ang);
		if(testRadius >= radius) {
			keep[i] = newData.vertices.size();
			newData.vertices.push_back(test);
		}
	}
	
	//find all intersections between the lines of the drill bit and faces of the model
	std::vector<Plane> faces(data->faces.size());
	Vector3 v1, v2, v3;
	std::vector<Vector3> v(4), inter(_segments);
	std::vector<bool> rayUsed(_segments);
	float distance, s, t, denom,
		dot[5]; //the distinct dot products needed to compute barycentric coordinates - http://geomalgorithms.com/a06-_intersect-2.html
	for(i = 0; i < data->faces.size(); i++) {
		for(j = 0; j < _segments; j++) rayUsed[j] = false;
		//get the plane for this face
		for(j = 0; j < 3; j++) v[j].set(data->worldVertices[data->faces[i][j]]);
		v1.set(v[1]-v[0]);
		v2.set(v[2]-v[1]);
		v1.cross(v2);
		faces[i].setNormal(v1);
		faces[i].setDistance(-v[0].dot(faces[i].getNormal()));
		//find all intersections for the drill with the plane
		for(j = 0; j < _segments; j++) {
			distance = lines[j].intersects(faces[i]);
			if(distance == Ray::INTERSECTS_NONE) inter[j].set(Vector3::zero());
			else inter[j].set(lines[j].getOrigin() + distance * lines[j].getDirection());
		}
		//check each triangle of the face for an intersection
		for(j = 0; j < data->triangles[i].size(); j++) {
			for(k = 0; k < 3; k++) v[k].set(data->worldVertices[data->faces[i][data->triangles[i][j][k]]]);
			v1.set(v[1] - v[0]);
			v2.set(v[2] - v[0]);
			for(k = 0; k < _segments; k++) {
				if(rayUsed[k]) continue;
				//calculate barycentric coords of the intersection wrt this triangle
				if(inter[k] == Vector3::zero()) continue;
				v3.set(inter[k] - v[0]);
				dot[0] = v1.dot(v2); dot[1] = v3.dot(v2); dot[2] = v2.dot(v2); dot[3] = v3.dot(v1); dot[4] = v1.dot(v1);
				denom = dot[0]*dot[0] - dot[4]*dot[2];
				if(denom == 0) {
					continue;
				}
				s = (dot[0]*dot[1] - dot[2]*dot[3]) / denom;
				t = (dot[0]*dot[3] - dot[4]*dot[1]) / denom;
				if(s < 0 || t < 0 || s+t > 1) {
					continue;
				}
				//the intersection point is inside this triangle
				newData.vertices.push_back(inter[k]);
				drillInt[k][i] = newData.vertices.size()-1;
				rayUsed[k] = true;
			}
		}
	}

	//find all intersections between edges of the model and the planes of the drill bit
	Ray edge;
	Vector3 intersect;
	unsigned short e[2];
	short lineInd[2];
	std::vector<bool> hasInt(2);
	std::vector<unsigned short> newEdge(2);
	Vector4 intersect4;
	float intersectAngle, angleDiff, minAngleDiff, edgeLen, curDistance;
	bool angleInside, better, useInt;
	std::vector<std::vector<float> > minDistance(data->edges.size());
	std::map<unsigned short, std::map<unsigned short, bool> > isNewEdge;
	for(i = 0; i < data->edges.size(); i++) minDistance[i].resize(2);
	Vector3 dupTest;
	for(i = 0; i < data->edges.size(); i++) {
		for(j = 0; j < 2; j++) {
			hasInt[j] = false;
			lineInd[j] = -1;
			e[j] = data->edges[i][j];
			v[j].set(data->worldVertices[e[j]]);
			minDistance[i][j] = 999999;
		}
		if(keep[e[0]] < 0 && keep[e[1]] < 0) continue;
		v[2].set(v[1]-v[0]);
		edge.set(v[0], v[2]);
		edgeLen = v[2].length();
		minAngleDiff = (keep[e[0]] >= 0 && keep[e[1]] >= 0) ? -1000 : 1000;
		for(j = 0; j < _segments; j++) {
			distance = edge.intersects(planes[j]);
			//only consider intersections outside the endpoints if we have already said that one endpoint is being discarded,
			//since in that case we need to find an intersection somewhere, and there may be imprecision
			if((distance <= 0 || distance >= edgeLen) && keep[e[0]] >= 0 && keep[e[1]] >= 0) continue;
			intersect.set(edge.getOrigin() + distance * edge.getDirection());
			//transform the intersection point back to the drillbit's model space and see if it is in the correct angular range
			intersect4.set(intersect.x, intersect.y, intersect.z, 1);
			toolWorldModel.transformVector(&intersect4);
			intersectAngle = atan2(intersect4.z, intersect4.y);
			while(intersectAngle < 0) intersectAngle += 2*M_PI;
			angle = (2*M_PI*j) / _segments;
			//again, if one endpoint is being discarded, allow tolerance in the angle check
			angleInside = (intersectAngle >= angle && intersectAngle <= angle+dAngle);
			if(angleInside) {
				angleDiff = -1;
			} else {
				if(keep[e[0]] >= 0 && keep[e[1]] >= 0) continue;
				angleDiff = fmin(fabs(intersectAngle-angle), fabs(intersectAngle-(angle+dAngle)));
			}
			useInt = false;
			for(k = 0; k < 2; k++) {
				curDistance = k*edgeLen + (1-2*k)*distance;
				if(minAngleDiff >= 0) better = angleDiff < minAngleDiff;
				else if(angleDiff < 0 && minDistance[i][k] >= 0 && minDistance[i][k] <= edgeLen)
					better = curDistance >= 0 && curDistance < minDistance[i][k];
				else if(angleDiff < 0 && curDistance >= 0 && curDistance <= edgeLen) better = true;
				else better = angleDiff < 0 && fmin(fabs(curDistance), fabs(curDistance - edgeLen))
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
		//don't allow both directions of the edge to intersect the same drill point, and don't allow edges where only one direction intersects the drill
		if((!hasInt[0] && hasInt[1]) || (hasInt[0] && !hasInt[1]) ||
		  (keep[e[0]] >= 0 && keep[e[1]] >= 0 && hasInt[0] && hasInt[1] && lineInd[0] == lineInd[1])) {
			continue;
		}
		else {
			//split this edge on the intersection point(s)
			if(hasInt[0] && hasInt[1]) {
				for(j = 0; j < 2; j++) {
					if(keep[e[j]] < 0) lineInd[j] = lineInd[1-j]; //if only keeping one endpoint, intersection is same both ways
					edgeInt[e[j]][e[1-j]] = std::pair<unsigned short, unsigned short>(lineInd[j], newData.vertices.size());
					if(keep[e[j]] >= 0) {
						newEdge[0] = keep[e[j]];
						newEdge[1] = newData.vertices.size();
						newData.vertices.push_back(v[j]);
						newData.edges.push_back(newEdge);
					}
				}
			}
			//or add it as is if it has no intersections
			if(!hasInt[0] && !hasInt[1] && keep[e[0]] >= 0 && keep[e[1]] >= 0) {
				newEdge[0] = keep[e[0]];
				newEdge[1] = keep[e[1]];
				newData.edges.push_back(newEdge);
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
	float minAngle, maxIntFraction;
	bool keepAll, drillInside, reverseFace;
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
		keepAll = true; //whether this face has no intersections, in which case leave it as is
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
				newData.faces.push_back(newFace);
				newData.triangles.push_back(data->triangles[i]);
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
			//begin each new face on an edge intersection point
			std::map<unsigned short, std::map<unsigned short, unsigned short> >::iterator it = faceEdgeInt.begin();
			std::map<unsigned short, unsigned short>::iterator it1 = it->second.begin();
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
				newEdge[0] = lastInter;
				newEdge[1] = keep[data->faces[i][ind]];
				newData.edges.push_back(newEdge);
				lastInter = newEdge[1];
			}
			newFace.push_back(keep[data->faces[i][ind]]);
			oldFaceInd.push_back(ind);
			newEdge[0] = lastInter;
			newEdge[1] = keep[data->faces[i][ind]];
			newData.edges.push_back(newEdge);
			lastInter = newEdge[1];
			//add the next edge intersection
			n = faceEdgeInt[ind][(ind+dir+faceSize)%faceSize];
			newFace.push_back(n);
			oldFaceInd.push_back(-1);
			newEdge[0] = lastInter;
			newEdge[1] = n;
			newData.edges.push_back(newEdge);
			lastInter = newEdge[1];
			//and get rid of it so we don't try to reuse it
			faceEdgeInt[ind].erase((ind+dir+faceSize)%faceSize);
			if(faceEdgeInt[ind].empty()) faceEdgeInt.erase(ind);
			//determine which way to go around the drill circle to connect the face
			//-choose the one with a higher percentage of drill line intersections (ideally should be 100%)
			startLine = edgeInt[data->faces[i][ind]][data->faces[i][(ind+dir+faceSize)%faceSize]].first;
			maxIntFraction = -1;
			for(k = -1; k <= 1; k += 2) {
				drillIntCount = 0;
				n = 0;
				for(lineNum = (startLine + (k == -1 ? 0 : 1) + _segments) % _segments;
				  lineNum != (endLine + (k == -1 ? 0 : 1) + _segments) % _segments;
				  lineNum = (lineNum+k+_segments) % _segments) {
				  	n++;
					if(drillInt.find(lineNum) != drillInt.end() && drillInt[lineNum].find(i) != drillInt[lineNum].end())
						drillIntCount++;
				}
				if(n == 0 || 1.0*drillIntCount/n > maxIntFraction) {
					maxIntFraction = 1.0*drillIntCount/n;
					dir = k;
				}
			}
			//iterate in the chosen direction, adding intersection points
			for(lineNum = (startLine + (dir == -1 ? 0 : 1) + _segments) % _segments;
			  lineNum != (endLine + (dir == -1 ? 0 : 1) + _segments) % _segments;
			  lineNum = (lineNum+dir+_segments) % _segments) {
				if(drillInt.find(lineNum) != drillInt.end() && drillInt[lineNum].find(i) != drillInt[lineNum].end()) {
					newFace.push_back(drillInt[lineNum][i]);
					oldFaceInd.push_back(-1);
					newEdge[0] = lastInter;
					newEdge[1] = drillInt[lineNum][i];
					newData.edges.push_back(newEdge);
					lastInter = drillInt[lineNum][i];
				}
			}
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
			newEdge[0] = drillInt[j][i];
			newEdge[1] = drillInt[(j+1)%_segments][i];
			newData.edges.push_back(newEdge);
		}

		//mark where the drill center intersects this face's plane, for computing 2d normals in the face plane
		distance = _axis.intersects(faces[i]);
		v1.set(_axis.getOrigin() + distance * _axis.getDirection());
		
		//triangulate each of the new polygons
		for(j = 0; j < newFaces.size(); j++) {
			newFaceSize = newFaces[j].size();
			for(k = 0; k < faceSize; k++) drillPoint[k] = -1;

			//for each edge that is from the original face
			for(k = 0; k < newFaceSize; k++) {
				if(oldFaceInds[j][k] < 0 || (!drillInside && oldFaceInds[j][(k+1)%newFaceSize] < 0)) continue;
				minAngle = -1;
				//find the drill bit intersection point whose radial normal is closest to that of the edge
				// - will be used to make the triangle with that edge
				v2.set(newData.vertices[newFaces[j][k]] - v1); //first calculate the perpendicular from the drill center to this edge
				for(m = (k+1)%newFaceSize; oldFaceInds[j][m] < 0; m = (m+1)%newFaceSize);
				v3.set(newData.vertices[newFaces[j][m]] - newData.vertices[newFaces[j][k]]);
				v3.normalize(&v3);
				v2.set(v2 - (v2.dot(v3)) * v3);
				v2.normalize(&v2);
				for(m = 0; m < newFaceSize; m++) {
					if(oldFaceInds[j][m] >= 0) continue;
					v3.set(newData.vertices[newFaces[j][m]] - v1); //drill spoke
					v3.normalize(&v3);
					angle = v3.dot(v2);
					if(angle > minAngle) {
						minAngle = angle;
						drillPoint[oldFaceInds[j][k]] = m;
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
				for(k = 0; k < 3; k++) { //add any edges of this triangle that aren't already edges of the face
					if(newTriangle[k] != (newTriangle[(k+1)%3]+1)%newFaceSize
					  && newTriangle[k] != (newTriangle[(k+1)%3]-1+newFaceSize)%newFaceSize) {
						newEdge[0] = newFaces[j][newTriangle[k]];
						newEdge[1] = newFaces[j][newTriangle[(k+1)%3]];
						newData.edges.push_back(newEdge);
					}
				}
				for(k = 0; k < 3; k++) newTriangle[k] = newFaces[j][newTriangle[k]];
				newData.faces.push_back(newTriangle);
				newData.triangles.push_back(newTriangles);
			} while((!drillInside && (n+1)%newFaceSize != m)
				|| (drillInside && (m != drillPoint[(start+1)%newFaceSize] || n != (start+1)%newFaceSize)));
		}
	}
	
	//add the new faces formed by the walls of the drill cylinder
	unsigned short newFaceStart = newData.faces.size();
	
	//just add 1 convex hull for each convex face
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


