#include "T4TApp.h"

T4TApp::DrillMode::DrillMode(T4TApp *app_) 
  : T4TApp::ToolMode::ToolMode(app_, "mode_Drill", "res/common/drill.form") {
	_axis.set(Vector3(0, 0, 0), Vector3(1, 0, 0));
	_radius = 0.2f;
	_segments = 20;
	//create the drill-bit node
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

bool T4TApp::DrillMode::toolNode() {
	ToolMode::toolNode();
	_node->updateData();
	Node::nodeData *data = (Node::nodeData*)_node->getUserPointer();

	//build drilled node into a new node data structure, save to disk, and reload
	Node::nodeData newData;
	newData.type = data->type;
	newData.objType = "mesh"; //can't keep sphere/box collision object once it is deformed!
	newData.mass = data->mass;
	newData.rotation = data->rotation;
	newData.translation = data->translation;
	newData.scale = data->scale;
	newData.constraints = data->constraints;
	
	unsigned short i, j, k, m, n;

	//store all the lines and planes of the drill bit
	Ray *lines = new Ray[_segments];
	Plane *planes = new Plane[_segments];
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
	//edgeInt[edge vertex 1][edge vertex 2] = (drill ray number, index of intersection point in new model's vertex list)
	//drillInt[drill ray number][face index in old model] = index of intersection point in new model's vertex list
	std::map<unsigned short, std::map<unsigned short, unsigned short> > drillInt;
	std::map<unsigned short, std::map<unsigned short, std::pair<unsigned short, unsigned short> > > edgeInt;
	
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
	Plane *faces = new Plane[data->faces.size()];
	Vector3 v1, v2, v3;
	std::vector<Vector3> v(4), inter(_segments);
	bool *rayUsed = new bool[_segments];
	float distance, s, t, denom,
		dot[5]; //the distinct dot products needed to compute barycentric coordinates - http://geomalgorithms.com/a06-_intersect-2.html
	for(i = 0; i < data->faces.size(); i++) {
		for(j = 0; j < _segments; j++) rayUsed[j] = false;
		//get the plane for this face
		for(j = 0; j < 3; j++) v[j].set(data->worldVertices[data->faces[i][j]]);
		v1.set(v[1]-v[0]);
		v2.set(v[2]-v[0]);
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
	std::vector<bool> hasInt(2);
	std::vector<unsigned short> newEdge(2);
	Vector4 intersect4;
	float intersectAngle, edgeLen, **minDistance = new float*[data->edges.size()];
	for(i = 0; i < data->edges.size(); i++) minDistance[i] = new float[2];
	Vector3 dupTest;
	for(i = 0; i < data->edges.size(); i++) {
		for(j = 0; j < 2; j++) {
			hasInt[j] = false;
			e[j] = data->edges[i][j];
			v[j].set(data->worldVertices[e[j]]);
			minDistance[i][j] = 999999;
		}
		if(keep[e[0]] < 0 && keep[e[1]] < 0) continue;
		v[2].set(v[1]-v[0]);
		edge.set(v[0], v[2]);
		edgeLen = v[2].length();
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
			angle = (2*M_PI*j) / _segments;
			if(intersectAngle < angle || intersectAngle > angle+dAngle) continue;
			bool useInt = false;
			for(k = 0; k < 2; k++) {
				if(k*edgeLen + (2*k-1)*distance < minDistance[i][k]) {
					//if there is already an intersection for this edge, make sure we are not duplicating it due to imprecision
					if(edgeInt.find(e[k]) != edgeInt.end() && edgeInt[e[k]].find(e[1-k]) != edgeInt[e[k]].end()) {
						int newInd = edgeInt[e[k]][e[1-k]].second;
						if(newData.vertices[newInd].distance(intersect) < 0.001f) continue;
					}
					useInt = true;
					hasInt[k] = true;
					edgeInt[e[k]][e[1-k]] = std::pair<unsigned short,unsigned short>(j, newData.vertices.size());
				}
			}
			if(useInt) newData.vertices.push_back(intersect);
		}
		//split this edge on the intersection point(s)
		if(!hasInt[0] || !hasInt[1] || edgeInt[e[0]][e[1]].second != edgeInt[e[1]][e[0]].second) {
			for(j = 0; j < 2; j++) if(keep[j] >= 0 && hasInt[j]) {
				newEdge[0] = keep[e[j]];
				newEdge[1] = edgeInt[e[j]][e[1-j]].second;
				newData.edges.push_back(newEdge);
			}
		}
		//or add it as is if it has no intersections
		if(!hasInt[0] && !hasInt[1] && keep[e[0]] >= 0 && keep[e[1]] >= 0) {
			newEdge[0] = keep[e[0]];
			newEdge[1] = keep[e[1]];
			newData.edges.push_back(newEdge);
		}
	}

	//reconstruct the faces by leaving out the discarded points and incorporating the intersections
	std::vector<unsigned short> newFace, faceKeep, newTriangle(3);
	std::vector<short> oldFaceInd, drillPoint;
	std::vector<std::vector<unsigned short> > newFaces, newTriangles(1);
	std::vector<std::vector<short> > oldFaceInds;
	unsigned short faceSize, diff, bestLineDiff, newInd, start, current, next, ind, lastInter, lineNum, startLine,
		line, newFaceSize;
	short keepNext, dir;
	float minAngle;
	bool keepAll, drillInside;
	newTriangles[0].resize(3);
	for(i = 0; i < 3; i++) newTriangles[0][i] = i;

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
		for(j = 0; j < faceSize; j++) {
			hasInt[j] = edgeInt.find(data->faces[i][j]) != edgeInt.end()
				&& edgeInt[data->faces[i][j]].find(data->faces[i][(j+1)%faceSize]) != edgeInt[data->faces[i][j]].end();
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
		
		while(!faceKeep.empty()) {
			newFace.clear();
			oldFaceInd.clear();
			//start from any vertex of this face that is not being discarded
			start = faceKeep.back();
			faceKeep.pop_back();
			//loop through the face vertices, adding new vertices as called for
			newFace.push_back(keep[data->faces[i][start]]);
			oldFaceInd.push_back(start);
			ind = start;
			do {
				current = data->faces[i][ind];
				next = data->faces[i][(ind+1) % faceSize];
				keepNext = keep[data->faces[i][next]];
				//if this edge intersects the drill, add intersection points until we get to the next face vertex
				if(edgeInt.find(current) != edgeInt.end() && edgeInt[current].find(next) != edgeInt[current].end()) {
					//first the intersection of the edge with the drill
					lastInter = edgeInt[current][next].second;
					newFace.push_back(lastInter);
					//then the adjacent intersections of the drill lines with the face
					lineNum = edgeInt[current][next].first;
					startLine = lineNum;
					dir = drillInt.find(lineNum) != drillInt.end() && drillInt[lineNum].find(i) != drillInt[lineNum].end() ? -1 : 1;
					if(dir == 1) {
						newFace.push_back(drillInt[lineNum][i]);
						oldFaceInd.push_back(-1);
						newEdge[0] = lastInter;
						newEdge[1] = drillInt[lineNum][i];
						newData.edges.push_back(newEdge);
					}
					for(lineNum = (lineNum+dir) % _segments;
					  lineNum != startLine
					  	&& drillInt.find(lineNum) != drillInt.end() && drillInt[lineNum].find(i) != drillInt[lineNum].end();
					  lineNum = (lineNum+dir) % _segments) {
						newFace.push_back(drillInt[lineNum][i]);
						oldFaceInd.push_back(-1);
						newEdge[0] = lastInter;
						newEdge[1] = drillInt[lineNum][i];
						newData.edges.push_back(newEdge);
						lastInter = drillInt[lineNum][i];	
					}
				
					//at this point we should be next to a drill plane that another edge of this face intersects
					lineNum = dir == 1 ? lineNum - 1 : lineNum;
					//we want it to be the one we're on, but we'll take +/-1 due to imprecision
					bestLineDiff = _segments;
					for(j = (ind+1)%faceSize; j != ind; j = (j+1)%faceSize) {
						for(k = 0; k < 2; k++) e[k] = data->faces[i][(j+k)%faceSize];
						if(keep[e[1]] >= 0
						  && edgeInt.find(e[0]) != edgeInt.end() && edgeInt[e[0]].find(e[1]) != edgeInt[e[0]].end()) {
							line = edgeInt[e[0]][e[1]].first;
							diff = abs(line - lineNum);
							if(diff > _segments/2) diff = _segments - diff;
							if(diff < bestLineDiff) {
								bestLineDiff = diff;
								newInd = edgeInt[e[0]][e[1]].second;
								ind = j;
							}
						}
					}
					if(bestLineDiff < 2) {
						//add the edge-drill intersection vertex
						newFace.push_back(newInd);
						//add the next vertex outside the drill
						if((ind+1)%faceSize != start) newFace.push_back(keep[(ind+1)%faceSize]);
						//add the last edge along the intersection rim
						newEdge[0] = lastInter;
						newEdge[1] = newInd;
						newData.edges.push_back(newEdge);
					} else {
						GP_WARN("DrillMode: couldn't find edge to exit drill to continue face");
						return false;
					}
				}
				else if(keepNext >= 0) {
					newFace.push_back(keepNext);
					oldFaceInd.push_back((ind+1)%faceSize);
				}
				ind = (ind+1) % faceSize;
			} while(ind != start);
			if(!newFace.empty()) {
				newFaces.push_back(newFace);
				oldFaceInds.push_back(oldFaceInd);
			}
		}
		if(drillInside) for(j = 0; j < _segments; j++) {
			newFaces[0].push_back(drillInt[j][i]);
			oldFaceInds[0].push_back(-1);
		}

		//mark where the drill center intersects this face's plane, for computing 2d normals in the face plane
		distance = _axis.intersects(faces[i]);
		v1.set(_axis.getOrigin() + distance * _axis.getDirection());
		
		//triangulate each of the new polygons
		for(j = 0; j < newFaces.size(); j++) {
			newFaceSize = newFaces[j].size();			

			//for each edge that is from the original face
			for(k = 0; k < newFaceSize; k++) {
				if(oldFaceInds[j][k] < 0 || (!drillInside && oldFaceInds[j][(k+1)%newFaceSize] < 0)) continue;
				minAngle = 2; //cosine metric
				//find the drill bit intersection point whose radial normal is closest to that of the edge
				// - will be used to make the triangle with that edge
				for(m = 0; m < newFaceSize; m++) {
					if(oldFaceInds[j][m] >= 0) continue;
					v2.set(newData.vertices[newFaces[j][k]]);
					for(n = (k+1)%newFaceSize; oldFaceInds[j][n] < 0; n = (n+1)%newFaceSize);
					v3.set(newData.vertices[newFaces[j][n]]);
					v[0].set(newData.vertices[newFaces[j][m]] - v1);
					v[1].set(v2 - v1);
					v[2].set(v3 - v1);
					v[3].set(v3 - v2);
					if(v[0].dot(v[1]) < 0 || v[0].dot(v[2]) < 0) continue;
					v[0].normalize(&v[0]);
					v[3].normalize(&v[3]);
					angle = fabs(v[0].dot(v[3]));
					if(angle < minAngle) {
						minAngle = angle;
						drillPoint[k] = m;
					}
				}
			}

			//find the beginning of the sequence of vertices from the drill intersection
			start = newFaceSize-1;
			while((oldFaceInds[j][start] >= 0 || oldFaceInds[j][start-1] < 0) && start >= 0) start--;
			if(oldFaceInds[j][start] >= 0) {
				GP_WARN("Couldn't find drill intersection starting point for face %d", i);
				return false;
			}
			n = (start-1+newFaceSize)%newFaceSize;
			if(!drillInside) m = start;
			else m = drillPoint[n];
			do {
				if(m == drillPoint[(n-1+newFaceSize)%newFaceSize]) {
					newTriangle[0] = newFaces[j][(n-1+newFaceSize)%newFaceSize];
					newTriangle[1] = newFaces[j][n];
					newTriangle[2] = newFaces[j][m];
					n = (n-1+newFaceSize)%newFaceSize;
				} else {
					newTriangle[0] = newFaces[j][n];
					newTriangle[1] = newFaces[j][m];
					newTriangle[2] = newFaces[j][(m+1)%newFaceSize];
					m = (m+1)%newFaceSize;
				}
				newData.faces.push_back(newTriangle);
				newData.triangles.push_back(newTriangles);
			} while((!drillInside && n != (m+1)%newFaceSize)
				|| (drillInside && (m != start || n != (start-1+newFaceSize)%newFaceSize)));
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


