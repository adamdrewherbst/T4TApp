#include "T4TApp.h"

T4TApp::SliceMode::SliceMode(T4TApp *app_) 
  : T4TApp::Mode::Mode(app_, "mode_Slice", "res/common/slice.form") {
	_subMode = 0;
	_node = NULL;
	_touching = false;
	_slicePlane.set(Vector3(0, 0, 1), 0);
	//create the knife node
	float spacing = 0.5f, radius = 3.25f, distance, color[3] = {1.0f, 1.0f, 1.0f}, vec[2];
	int v = 0, numLines = (int)(radius/spacing), vertexCount = 4*(2*numLines+1)*6;
	std::vector<float> vertices(vertexCount);
	for(int i = -numLines; i <= numLines; i++) {
		for(int j = 0; j < 2; j++) {
			for(int k = 0; k < 2; k++) {
				vec[j] = i*spacing;
				vec[1-j] = (2*k-1) * sqrt(radius*radius - vec[j]*vec[j]);
				vertices[v++] = vec[0];
				vertices[v++] = vec[1];
				vertices[v++] = 0;
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
	_knife = Node::create("knife");
	_knife->setModel(model);
	model->release();
	//app->_scene->addNode(_knife);
	//_knife->setActive(false);
}

void T4TApp::SliceMode::setActive(bool active) {
	Mode::setActive(active);
	setNode(NULL);
	_touching = false;
}

void T4TApp::SliceMode::setNode(Node *node) {
	_node = node;
	app->getScriptController()->executeFunction<void>("camera_setNode", "s", _node != NULL ? _node->getId() : NULL);
	if(_node != NULL) {
		setAxis(0);
		app->_scene->addNode(_knife);
	} else {
		app->_scene->removeNode(_knife);
	}
}

void T4TApp::SliceMode::setAxis(int axis) {
	//translate the camera to look at the center of the node
	//and face along the <axis> direction in its model space
	float yaw = 0, pitch = 0;
	Vector3 sliceNormal, viewNormal, translation(_node->getTranslationWorld());
	Matrix rotation;
	_node->getRotation(&rotation);
	_knife->setRotation(rotation);
	switch(axis) {
		case 0: //x
			yaw = 0;
			pitch = M_PI/2 - 0.1f;
			sliceNormal.set(1, 0, 0);
			viewNormal.set(0, -1, 0);
			_knife->rotateY(M_PI/2);
			break;
		case 1: //y
			yaw = M_PI/2;
			pitch = 0.1f;
			sliceNormal.set(0, 1, 0);
			viewNormal.set(0, 0, -1);
			_knife->rotateX(M_PI/2);
			break;
		case 2: //z
			yaw = 0.1f;
			pitch = 0;
			sliceNormal.set(0, 0, 1);
			viewNormal.set(-1, 0, 0);
			break;
	}
	_knife->setTranslation(translation);
	_knifeBaseRotation = _knife->getRotation();
	rotation.transformVector(&sliceNormal);
	_slicePlane.setNormal(sliceNormal);
	rotation.transformVector(&viewNormal);
	_viewPlane.setNormal(viewNormal);
	app->getScriptController()->executeFunction<void>("camera_rotateTo", "ff", yaw, pitch);
	setView();
}

void T4TApp::SliceMode::setView() {
	Vector3 cam(app->_activeScene->getActiveCamera()->getNode()->getTranslationWorld());
	Ray camToNode(cam, _node->getTranslationWorld() - cam);
	float distance = camToNode.intersects(_viewPlane);
	_viewPlaneOrigin.set(camToNode.getOrigin() + distance * camToNode.getDirection());
}

bool T4TApp::SliceMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			if(_node == NULL) {
				setNode(app->getMouseNode(x, y));
				if(_node) cout << "going to slice " << _node->getId() << endl;
			}
			else {
				Ray ray;
				app->_activeScene->getActiveCamera()->pickRay(app->getViewport(), x, y, &ray);
				float distance = ray.intersects(_viewPlane);
				if(distance != Ray::INTERSECTS_NONE) {
					_touchStart.set(ray.getOrigin() + distance * ray.getDirection());
					_touching = true;
					_knifeBaseRotation = _knife->getRotation();
				}
			}
			break;
		}
		case Touch::TOUCH_MOVE: {
			if(_node == NULL || !_touching) break;
			Ray ray;
			app->_activeScene->getActiveCamera()->pickRay(app->getViewport(), x, y, &ray);
			float distance = ray.intersects(_viewPlane);
			if(distance != Ray::INTERSECTS_NONE) {
				_touchPoint.set(ray.getOrigin() + distance * ray.getDirection());
				switch(_subMode) {
					case 0: //rotate
					{
						Vector3 v1(_touchStart - _viewPlaneOrigin), v2(_touchPoint - _viewPlaneOrigin);
						v1.normalize();
						v2.normalize();
						float cosAng = v1.dot(v2);
						v1.cross(v2);
						float sinAng = v1.dot(_viewPlane.getNormal());
						float angle = atan2(sinAng, cosAng);
						Quaternion rotation;
						Quaternion::createFromAxisAngle(_viewPlane.getNormal(), angle, &rotation);
						_knife->setRotation(rotation * _knifeBaseRotation);
						break;
					}
					case 1: //translate
						_knife->setTranslation(_node->getTranslationWorld() + _touchPoint-_touchStart);
						break;
				}
			}
			break;
		}
		case Touch::TOUCH_RELEASE:
			_touching = false;
			break;
	}
}

void T4TApp::SliceMode::controlEvent(Control *control, Control::Listener::EventType evt) {
	cout << "slice mode clicked " << control->getId() << endl;
	const char *controlID = control->getId();

	if(strcmp(controlID, "axis") == 0 && _node != NULL) {
		const char *_axes[3] = {"X", "Y", "Z"};
		for(int i = 0; i < 3; i++) {
			if(strcmp(((Button*)control)->getText(), _axes[i]) == 0) {
				setAxis(i);
				((Button*)control)->setText(_axes[(i+1)%3]);
				cout << "set axis to " << _axes[i] << endl;
				break;
			}
		}
	}
	else if(strcmp(controlID, "rotate") == 0) _subMode = 0;
	else if(strcmp(controlID, "translate") == 0) _subMode = 1;
	else if(strcmp(controlID, "doSlice") == 0) {
		sliceNode();
		setActive(false);
	}
}

bool T4TApp::SliceMode::sliceNode() {

	_node->updateData(); //make sure the world coords are up to date
	_slicePlane.set(Vector3(0, 0, 1), 0);
	Matrix trans;
	Matrix::createRotation(_knife->getRotation(), &trans);
	trans.translate(_knife->getTranslationWorld());
	_slicePlane.transform(trans);
	cout << "slicing " << _node->getId() << " at " << app->printVector(_slicePlane.getNormal()) << " => " << _slicePlane.getDistance() << endl;
	Node::nodeData *data = (Node::nodeData*)_node->getUserPointer();
	unsigned short e1, e2, numKeep = 0;
	Vector3 v1, v2, planeOrigin = _slicePlane.getDistance() * _slicePlane.getNormal();

	//put the vertices that will be kept into a new nodeData struct
	Node::nodeData newData;
	newData.type = data->type;
	std::vector<short> keep(data->vertices.size()), //index in the new vertex list or -1 if discarding
		replace(data->vertices.size()); //index of the new vertex that is replacing this one
	for(int i = 0; i < data->vertices.size(); i++) {
		keep[i] = -1;
		replace[i] = -1;
		float dot = (data->worldVertices[i] - planeOrigin).dot(_slicePlane.getNormal());
		if(dot > 0) continue;
		keep[i] = numKeep++;
		newData.vertices.push_back(data->worldVertices[i]);
	}
	if(numKeep == 0) return false;

	//get all intersections between edges of this mesh and the slice plane
	Ray edge;
	Vector3 edgeVec, intersect, newEdgeVec;
	float distance, dot;
	std::map<unsigned short, std::map<unsigned short,unsigned short> > intersections;
	std::vector<unsigned short> newEdge(2);
	for(int i = 0; i < data->edges.size(); i++) {
		e1 = data->edges[i][0];
		e2 = data->edges[i][1];
		//if both endpoints are on the same side of the slice plane, decide whether to keep the edge and move on
		if(keep[e1] >= 0 && keep[e2] >= 0) {
			newData.edges.push_back(data->edges[i]);
			continue;
		}
		if(keep[e1] < 0 && keep[e2] < 0) continue;
		//otherwise this edge must intersect the slice plane
		edgeVec.set(data->worldVertices[e2] - data->worldVertices[e1]);
		edge.set(data->worldVertices[e1], edgeVec);
		distance = edge.intersects(_slicePlane);
		if(distance != Ray::INTERSECTS_NONE) {
			intersect.set(edge.getOrigin() + edge.getDirection()*distance);
			newEdgeVec.set(intersect - data->worldVertices[e1]);
			dot = newEdgeVec.dot(edgeVec) / edgeVec.lengthSquared();
		}
		//limit the intersection point to the edge endpoints, in case computational geometry puts it outside the edge
		if(distance == Ray::INTERSECTS_NONE || dot <= 0 || dot >= 1) {
			//see if the entire edge is really on the keep side of the plane
			if(distance == Ray::INTERSECTS_NONE || (dot <= 0 && keep[e2] >= 0) || (dot >= 1 && keep[e1] >= 0))
				intersect.set(data->worldVertices[keep[e1] >= 0 ? e2 : e1]);
			//otherwise keep a tiny fraction of the edge so that we don't lose its direction vector
			else intersect.set(data->worldVertices[keep[e1] >= 0 ? e1 : e2] + (keep[e1] >= 0 ? 1 : -1)*0.01f*edgeVec);
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
					if(!done) newFace.push_back(newEdge[(j+1)%2]);
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
	
	//transform the new vertices back to model space before saving the data
	Matrix worldModel;
	_node->getWorldMatrix().invert(&worldModel);
	Vector3 translation(_node->getTranslationWorld());
	for(int i = 0; i < newData.vertices.size(); i++) {
		worldModel.transformVector(&newData.vertices[i]);
		newData.vertices[i] -= translation;
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
	return true;
}


