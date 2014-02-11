#include "T4TApp.h"

T4TApp::SliceMode::SliceMode(T4TApp *app_) 
  : T4TApp::Mode::Mode(app_, "mode_Slice", "res/common/slice.form") {
	_subMode = 0;
	_node = NULL;
	_slicePlane.set(Vector3(1, 0, 0), 0);
}

void T4TApp::SliceMode::setActive(bool active) {
	Mode::setActive(active);
	_node = NULL;
}

void T4TApp::SliceMode::setAxis(int axis) {
	float yaw = 0, pitch = 0;
	Vector3 normal;
	switch(axis) {
		case 0: //x
			yaw = 0;
			pitch = 0;
			normal.set(1, 0, 0);
			break;
		case 1: //y
			yaw = 0;
			pitch = M_PI/2;
			normal.set(0, 1, 0);
			break;
		case 2: //z
			yaw = M_PI/2;
			pitch = 0;
			normal.set(0, 0, 1);
			break;
	}
	_slicePlane.setNormal(normal);
	app->getScriptController()->executeFunction<void>("camera_rotateTo", "ff", yaw, pitch);
}

bool T4TApp::SliceMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	switch(evt) {
		case Touch::TOUCH_PRESS:
			if(_node == NULL) {
				_node = app->getMouseNode(x, y);
				if(_node) {
					cout << "going to slice " << _node->getId() << endl;
					setAxis(0);
				}
			}
			else switch(_subMode) {
				case 0: //rotate
					break;
			}
			break;
	}
}

void T4TApp::SliceMode::controlEvent(Control *control, Control::Listener::EventType evt) {
	cout << "slice mode clicked " << control->getId() << endl;
	const char *controlID = control->getId();

	if(strcmp(controlID, "rotate") == 0) _subMode = 0;
	else if(strcmp(controlID, "translate") == 0) _subMode = 1;
	else if(strcmp(controlID, "doSlice") == 0) {
		sliceNode();
		setActive(false);
	}
}

void T4TApp::SliceMode::sliceNode() {

	_node->updateData(); //make sure the world coords are up to date
	Node::nodeData *data = (Node::nodeData*)_node->getUserPointer();
	unsigned short e1, e2, numKeep = 0;
	Vector3 v1, v2, planeOrigin = _slicePlane.getDistance() * _slicePlane.getNormal();

	//put the vertices that will be kept into a new nodeData struct
	Node::nodeData newData;
	newData.type = data->type;
	short *keep = new short[data->vertices.size()], //index in the new vertex list or -1 if discarding
		*replace = new short[data->vertices.size()]; //index of the new vertex that is replacing this one
	for(int i = 0; i < data->vertices.size(); i++) {
		keep[i] = -1;
		replace[i] = -1;
		float dot = (data->worldVertices[i] - planeOrigin).dot(_slicePlane.getNormal());
		if(dot > 0) continue;
		keep[i] = numKeep++;
		newData.vertices.push_back(data->vertices[i]);
	}

	//get all intersections between edges of this mesh and the slice plane
	Ray edge;
	Vector3 edgeVec, intersect;
	float distance, dot;
	std::map<unsigned short, std::map<unsigned short,unsigned short> > intersections;
	std::vector<unsigned short> newEdge(2);
	for(int i = 0; i < data->edges.size(); i++) {
		//see if this edge intersects the slice plane
		e1 = data->edges[i][0];
		e2 = data->edges[i][1];
		edgeVec.set(data->worldVertices[e2] - data->worldVertices[e1]);
		edge.set(data->worldVertices[e1], edgeVec);
		distance = edge.intersects(_slicePlane);
		if(distance != Ray::INTERSECTS_NONE) {
			intersect.set(edge.getOrigin() + edge.getDirection()*distance);
			dot = (intersect - data->worldVertices[e1]).dot(edgeVec) / edgeVec.lengthSquared();
		}
		if((distance == Ray::INTERSECTS_NONE) || dot < 0 || dot > 1) { //no intersection, so we are either keeping or discarding this edge
			if(keep[e1] >= 0 && keep[e2] >= 0) newData.edges.push_back(data->edges[i]);
			continue;
		}
		//add the new vertex at the intersection point
		newData.vertices.push_back(intersect);
		unsigned short newInd = newData.vertices.size()-1,
			kept = keep[e1] >= 0 ? e1 : e2, discarded = keep[e1] >= 0 ? e2 : e1;
		replace[discarded] = newInd;
		//add the shortened edge to the new mesh
		newEdge[0] = keep[kept];
		newEdge[1] = newInd;
		newData.edges.push_back(newEdge);
		//add this intersection to the map
		intersections[e1][e2] = newInd;
		intersections[e2][e1] = newInd;
	}

	//modify each polygon according to its intersections
	std::vector<unsigned short> newFace, newTriangle(3);
	std::vector<std::vector<unsigned short> > newTriangles, newEdges;
	bool faceAltered, keepTriangle;
	short usedNew;
	for(int i = 0; i < data->faces.size(); i++) {
		newFace.clear();
		faceAltered = false;
		usedNew = -1;
		for(int j = 0; j < data->faces[i].size(); j++) { //for each old polygon...
			e1 = data->faces[i][j];
			e2 = data->faces[i][(j+1)%data->faces[i].size()];
			if(keep[e1] >= 0) newFace.push_back(keep[e1]);
			else faceAltered = true;
			if(intersections.find(e1) == intersections.end() || intersections[e1].find(e2) == intersections[e1].end()) continue;
			newFace.push_back(intersections[e1][e2]);
			if(usedNew >= 0) { //we have a new edge formed by the 2 new vertices just added to this face
				//add them in reverse order so the new face on the slice plane has the right orientation
				newEdge[0] = intersections[e1][e2];
				newEdge[1] = usedNew;
				newData.edges.push_back(newEdge);
				newEdges.push_back(newEdge);
			}
			faceAltered = true;
			usedNew = intersections[e1][e2];
		}
		if(newFace.size() > 0) {
			newData.faces.push_back(newFace);
			if(faceAltered) { //retriangulate the polygon
				newTriangles.clear();
				/*for(int j = 0; j < data->triangles[i].size(); j++) { //keep as many of the old triangles as possible
					keepTriangle = true;
					for(int k = 0; k < 3; k++) if(!keep[data->triangles[i][j][k]) keepTriangle = false;
					if(keepTriangle) newTriangles.push_back(data->triangles[i][j]);
				}//*/
				for(int j = 1; j < newFace.size()-1; j++) {
					newTriangle[0] = 0;
					newTriangle[1] = j;
					newTriangle[2] = j+1;
					newTriangles.push_back(newTriangle);
				}
				newData.triangles.push_back(newTriangles);
			} else {
				newData.triangles.push_back(data->triangles[i]);
			}
		}
	}
	
	//add the brand new polygons formed by the slice plane
	unsigned short newFaceStart = newData.faces.size(); //note where the slice plane polygons begin
	newFace.clear();
	while(newEdges.size() > 0) {
		if(newFace.empty()) {
			newEdge = newEdges.back();
			newFace.push_back(newEdge[0]);
			newFace.push_back(newEdge[1]);
			newEdges.pop_back();
			continue;
		}
		unsigned short lastPoint = newFace[newFace.size()-1];
		bool found = false, done = false;
		//find the edge that connects to the endpoint of this one
		for(int i = 0; i < newEdges.size(); i++) {
			for(int j = 0; j < 2; j++) {
				if(newEdges[i][j] == lastPoint) {
					newEdge = newEdges[i];
					newEdges.erase(newEdges.begin()+i);
					found = true;
					done = newEdge[(j+1)%2] == newFace[0];
					newFace.push_back(newEdge[(j+1)%2]);
					break;
				}
			}
			if(found) { //see if we have closed this polygon (note: may be more than one)
				if(done) {
					newData.faces.push_back(newFace);
					newTriangles.clear();
					for(int j = 1; j < newFace.size()-1; j++) {
						newTriangle[0] = 0;
						newTriangle[1] = j;
						newTriangle[2] = j+1;
						newTriangles.push_back(newTriangle);
					}
					newData.triangles.push_back(newTriangles);
					newFace.clear();
				}
				break;
			}
		}
		if(!found) GP_ERROR("Didn't find edge to continue new polygon");
	}
	
	//update or discard all the old convex hulls
	std::vector<unsigned short> newHull;
	short *inHull = new short[newData.vertices.size()];
	for(int i = 0; i < newData.vertices.size(); i++) inHull[i] = false;
	for(int i = 0; i < data->hulls.size(); i++) {
		newHull.clear();
		for(int j = 0; j < data->hulls[i].size(); j++) {
			e1 = data->hulls[i][j];
			usedNew = keep[e1];
			if(usedNew < 0) usedNew = replace[e1];
			if(usedNew < 0) continue;
			newHull.push_back(usedNew);
			inHull[usedNew] = true;
		}
		if(newHull.size() > 0) newData.hulls.push_back(newHull);
	}
	//add 1 convex hull for each slice plane polygon if it has any vertices that aren't already in a hull
	for(int i = newFaceStart; i < newData.faces.size(); i++) {
		bool needHull = false;
		for(int j = 0; j < newData.faces[i].size() && !needHull; j++) needHull = !inHull[newData.faces[i][j]];
		if(needHull) newData.hulls.push_back(newData.faces[i]);
	}
	
	//write the new node data to a file with suffix '_slice' and read it back in
	char filename[100];
	for(int i = 0; i < 100; i++) filename[i] = '\0';
	int count = 1;
	do {
		sprintf(filename, "res/common/%s_slice%d.node", data->type, count++);
	}while(FileSystem::fileExists(filename));
	Node::writeData(&newData, filename);
	_node->reloadFromData(filename);
}


