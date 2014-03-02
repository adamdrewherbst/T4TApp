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

	//store all the lines and planes of the drill bit
	Ray *lines = new Ray[_segments], planes = *new Plane[_segments];
	float angle, dAngle = 2*M_PI/segments, planeDistance = _radius * cos(dAngle/2);
	Matrix trans(_tool->getWorldMatrix());
	for(int i = 0; i < _segments; i++) {
		//line
		angle = (2*M_PI*j) / _segments;
		lines[i].setOrigin(-1000.0f, _radius*cos(angle), _radius*sin(angle));
		lines[i].setDirection(1, 0, 0);
		lines[i].transform(trans);
		//plane
		planes[i].setNormal(0, cos(angle+dAngle/2), sin(angle+dAngle/2));
		planes[i].setDistance(-planeDistance);
		planes[i].transform(trans);
	}
	//edgeInt[edge vertex 1][edge vertex 2] = (drill ray number, index of intersection point in new model's vertex list)
	//drillInt[drill ray number][face index in old model] = index of intersection point in new model's vertex list
	std::map<unsigned short, std::map<unsigned short, unsigned short> > drillInt;
	std::map<unsigned short, std::map<unsigned short, std::pair<unsigned short, unsigned short> > > edgeInt;
	
	//determine which vertices to discard because they are inside the drill cylinder
	short *keep = new short[data->vertices.size()];
	Matrix toolWorldModel;
	tool->getWorldMatrix().invert(toolWorldModel);
	Vector3 test;
	Vector4 test4;
	float testAngle, testRadius;
	for(unsigned short i = 0; i < data->vertices.size(); i++) {
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
	Plane face;
	Vector3 v1, v2, v3;
	std::vector<Vector3> v(3), inter(_segments);
	bool *rayUsed = new bool[_segments];
	float distance, s, t, denom,
		dot[5]; //the distinct dot products needed to compute barycentric coordinates - http://geomalgorithms.com/a06-_intersect-2.html
	for(unsigned short i = 0; i < data->faces.size(); i++) {
		for(int j = 0; j < _segments; j++) rayUsed[j] = false;
		//get the plane for this face
		for(int j = 0; j < 3; j++) v[j].set(data->worldVertices[data->faces[i][j]]);
		v1.set(v[1]-v[0]);
		v2.set(v[2]-v[0]);
		v1.cross(v2);
		face.setNormal(v1);
		face.setDistance(-v[0].dot(face.getNormal()));
		//find all intersections for the drill with the plane
		for(int j = 0; j < _segments; j++) {
			distance = lines[j].intersects(face);
			if(distance == Ray::INTERSECTS_NONE) inter[j].set(Vector3::zero());
			else inter[j].set(lines[j].getOrigin() + distance * lines[j].getDirection());
		}
		//check each triangle of the face for an intersection
		for(int j = 0; j < data->triangles[i].size(); j++) {
			for(int k = 0; k < 3; k++) v[k].set(data->worldVertices[data->faces[i][data->triangles[i][j][k]]]);
			v1.set(v[1] - v[0]);
			v2.set(v[2] - v[0]);
			for(int k = 0; k < _segments; k++) {
				if(rayUsed[k]) continue;
				//calculate barycentric coords of the intersection wrt this triangle
				if(inter[k] == Vector3::zero()) continue;
				v3.set(inter[k] - v[0]);
				dot[0] = v1.dot(v2); dot[1] = v3.dot(v2); dot[2] = v2.dot(v2); dot[3] = v3.dot(v1); dot[4] = v1.dot(v1);
				denom = dot[0]*dot[0] - dot[4]*dot[2];
				if(denom == 0) continue;
				s = (dot[0]*dot[1] - dot[2]*dot[3]) / denom;
				t = (dot[0]*dot[3] - dot[4]*dot[1]) / denom;
				if(s < 0 || t < 0 || s+t > 1) continue;
				//the intersection point is inside this triangle
				newData.vertices.push_back(inter[k]);
				drillInt[k][i] = newData.vertices.size()-1;
				rayUsed[k] = true;
				break;
			}
		}
	}
	
	//find all intersections between edges of the model and the planes of the drill bit
	Ray edge;
	unsigned short e[2];
	bool hasInt[2];
	std::vector<unsigned short> newEdge;
	Vector4 intersect4;
	float intersectAngle, edgeLen, **minDistance = new float[data->edges.size()][2];
	Vector3 dupTest;
	for(unsigned short i = 0; i < data->edges.size(); i++) {
		for(int j = 0; j < 2; j++) {
			hasInt[j] = false;
			e[j] = data->edges[i][j];
			v[j].set(data->worldVertices[e[j]];
			minDistance[i][j] = 999999;
		}
		if(keep[e[0]] < 0 && keep[e[1]] < 0) continue;
		v[2].set(v[1]-v[0]);
		edge.set(v[0], v[2]);
		edgeLen = v[2].length();
		for(int j = 0; j < _segments; j++) {
			distance = edge.itersects(planes[j]);
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
			for(int k = 0; k < 2; k++) {
				if(keep[e[k]] >= 0 && k*edgeLen + (2*k-1)*distance < minDistance[i][k]) {
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
			for(int j = 0; j < 2; j++) if(hasInt[j]) {
				newEdge[0] = keep[e[j]];
				newEdge[1] = edgeInt[e[j]][e[1-j]].second;
				newData.edges.push_back(newEdge):
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
	std::vector<unsigned short> newFace, faceKeep;
	for(unsigned short i = 0; i < data->faces.size(); i++) {
	
		//determine the subset of vertices of this face that we are keeping
		faceKeep.clear();
		for(unsigned short j = 0; j < data->faces[i].size(); j++)
			if(keep[data->faces[i][j]] >= 0) faceKeep.push_back(j);
		if(faceKeep.empty()) continue;
		
		//first see if the drill cross section is entirely inside the face, in which case treat it specially
		
		while(!faceKeep.empty()) {
			//start from a vertex of this face that is not discarded
			unsigned short start = faceKeep.back(), current, next, ind, faceSize = data->faces[i].size(), lastInter;
			short keepNext;
			//loop through the face vertices, adding new vertices as called for
			newFace.clear();
			newFace.push_back(keep[data->faces[i][start]]);
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
					unsigned short lineNum = edgeInt[current][next].first, startLine = lineNum;
					short dir = drillInt.find(lineNum) != drillInt.end() && drillInt[lineNum].find(i) != drillInt[lineNum].end() ? -1 : 1;
					if(dir == 1) {
						newFace.push_back(drillInt[lineNum][i]);
						newEdge[0] = lastInter;
						newEdge[1] = drillInt[lineNum][i];
						newData.edges.push_back(newEdge);
					}
					for(lineNum = (lineNum+dir) % _segments;
					  lineNum != startNum
					  	&& drillInt.find(lineNum) != drillInt.end() && drillInt[lineNum].find(i) != drillInt[lineNum].end();
					  lineNum = (lineNum+dir) % _segments) {
						newFace.push_back(drillInt[lineNum][i]);
						newEdge[0] = lastInter;
						newEdge[1] = drillInt[lineNum][i];
						newData.edges.push_back(newEdge);
						lastInter = drillInt[lineNum][i];	
					}
				
					//at this point we should be next to a drill plane that another edge of this face intersects
					lineNum = dir == 1 ? lineNum - 1 : lineNum;
					//we want it to be the one we're on, but we'll take +/-1 due to imprecision
					unsigned short bestLineDiff = _segments, newInd;
					for(unsigned short j = (ind+1)%faceSize; j != ind; j = (j+1)%faceSize) {
						for(int k = 0; k < 2; k++) e[k] = data->faces[i][(j+k)%faceSize];
						if(edgeInt.find(e[0]) != edgeInt.end() && edgeInt[e[0]].find(e[1]) != edgeInt[e[0]].end()) {
							unsigned short line = edgeInt[e[0]][e[1]].first, diff = abs(line - lineNum);
							if(diff > _segments/2) diff = _segments - diff;
							if(diff < bestLineDiff) {
								bestLineDiff = diff;
								newInd = edgeInt[e[0]][e[1]].second;
							}
						}
					}
					if(bestLineDiff < 2) {
						newFace.push_back(newInd);
						newEdge[0] = lastInter;
						newEdge[1] = newInd;
						newData.edges.push_back(newEdge);
					} else {
						GP_WARN("DrillMode: couldn't find edge to exit drill to continue face");
						return false;
					}
				}
				else if(keepNext >= 0) newFace.push_back(keepNext);
				ind = (ind+1) % faceSize;
			} while(ind != start);
			if(!newFace.empty()) newData.faces.push_back(newFace);
		}
	}
	
	//add the new faces formed by the walls of the drill cylinder
	unsigned short newFaceStart = newData.faces.size();
	
	//just add 1 convex hull for each convex face
	for(int i = 0; i < newData.faces.size(); i++)
		newData.hulls.push_back(newData.faces[i]);
	
	//transform the new vertices back to model space before saving the data
	Matrix worldModel;
	_node->getWorldMatrix().invert(&worldModel);
	Vector3 translation(_node->getTranslationWorld()), scale(_node->getScale());
	newData.translation.set(translation);
	newData.scale.set(scale);
	translation.x /= scale.x; translation.y /= scale.y; translation.z /= scale.z;
	for(int i = 0; i < newData.vertices.size(); i++) {
		worldModel.transformVector(&newData.vertices[i]);
		newData.vertices[i] -= translation;
	}
	
	//write the new node data to a file with suffix '_slice' and read it back in
	std::string filename;
	char newID[40];
	for(int i = 0; i < 40; i++) newID[i] = '\0';
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


