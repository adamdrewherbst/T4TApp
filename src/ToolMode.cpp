#include "T4TApp.h"

T4TApp::ToolMode::ToolMode() 
  : T4TApp::Mode::Mode("tool") {

	_tool = MyNode::create("tool_tool");
	_newNode = MyNode::create("newNode_tool");
	newData = _newNode->getData();
	
	_moveMenu = (Container*)_controls->getControl("moveMenu");
	setMoveMode(-1);

	//when drilling, have the user start by selecting a bit - shape and size
	_bitMenu = (Container*)_container->getControl("bitMenu");
	_bitMenu->setWidth(app->_componentMenu->getWidth());
	_bitMenu->setHeight(app->_componentMenu->getHeight());
	_bitMenu->setVisible(false);

	_subModes.push_back("saw");
	_subModes.push_back("drill");
	_tools.resize(_subModes.size());
	//create a mesh for every possible saw/drill bit - attach them to the tool node when selected
	for(short i = 0; i < _subModes.size(); i++) {
		switch(i) {
			case 0: //saw
				createBit(0);
				break;
			case 1: //drill - 4 sided (square) or 12 sided (~circle), sizes from 0.1 to 1 stepping by 0.1
				for(short n = 4; n <= 12; n += 8) {
					for(short s = 1; s <= 10; s++) {
						createBit(1, n, s * 0.1);
					}
				}
				break;
		}
	}
}

void T4TApp::ToolMode::createBit(short type, ...) {
	va_list args;
	va_start(args, type);
	
	Tool *tool = new Tool();
	tool->type = type;
	_tools[type].push_back(tool);
	
	std::vector<float> vertices;
	short i, j, k, m, v = 0;
	float color[3] = {1.0f, 1.0f, 1.0f};

	switch(type) {
		case 0: { //saw - make a hashed circle to show the cut plane
			float spacing = 0.5f, radius = 3.25f, vec[2];
			int numLines = (int)(radius/spacing);
			vertices.resize(4*(2*numLines+1)*6);
			for(i = -numLines; i <= numLines; i++) {
				for(j = 0; j < 2; j++) {
					for(k = 0; k < 2; k++) {
						vec[j] = i*spacing;
						vec[1-j] = (2*k-1) * sqrt(radius*radius - vec[j]*vec[j]);
						vertices[v++] = 0;
						vertices[v++] = vec[0];
						vertices[v++] = vec[1];
						for(m = 0; m < 3; m++) vertices[v++] = color[m];
					}
				}
			}
			tool->id = "saw";
			break;
		} case 1: { //drill - make an n-sided cylinder
			short segments = (short) va_arg(args, int);
			float radius = (float) va_arg(args, double);
			tool->iparam = (int*)malloc(1 * sizeof(int));
			tool->fparam = (float*)malloc(1 * sizeof(float));
			tool->iparam[0] = segments;
			tool->fparam[0] = radius;
			float length = 5.0f, angle, dAngle = 2*M_PI / segments;
			vertices.resize(6*segments*6);
			for(i = 0; i < segments; i++) {
				angle = (2*M_PI * i) / segments;
				for(j = 0; j < 2; j++) {
					vertices[v++] = radius * sin(angle);
					vertices[v++] = radius * cos(angle);
					vertices[v++] = (2*j-1) * length;
					for(k = 0; k < 3; k++) vertices[v++] = color[k];
				}
				for(j = 0; j < 2; j++) {
					for(k = 0; k < 2; k++) {
						vertices[v++] = radius * sin(angle + k*dAngle);
						vertices[v++] = radius * cos(angle + k*dAngle);
						vertices[v++] = (2*j-1) * length;
						for(m = 0; m < 3; m++) vertices[v++] = color[m];
					}
				}
			}
			//add a menu item for this bit
			std::ostringstream os;
			os << "drill_bit_" << segments << "_" << (int)(radius * 100 + 0.1);
			tool->id = os.str();
			std::string file = "res/png/" + tool->id + ".png";
			ImageControl *image = (ImageControl*) app->addButton<ImageControl>(_bitMenu, tool->id.c_str(), "");
			image->setZIndex(_bitMenu->getZIndex());
			image->setSize(100.0f, 100.0f);
			image->setImage(file.c_str());
			break;
		}
	}
	va_end(args);
	//create the mesh from the vertex data
	VertexFormat::Element elements[] = {
		VertexFormat::Element(VertexFormat::POSITION, 3),
		VertexFormat::Element(VertexFormat::COLOR, 3)
	};
	Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), v/6, false);
	mesh->setPrimitiveType(Mesh::LINES);
	mesh->setVertexData(&vertices[0], 0, v/6);
	Model *model = Model::create(mesh);
	model->setMaterial("res/common/grid.material");
	mesh->release();
	tool->model = model;
}

void T4TApp::ToolMode::setTool(short n) {
	_currentTool = n;
	_tool->setModel(getTool()->model);
}

T4TApp::ToolMode::Tool* T4TApp::ToolMode::getTool() {
	return _tools[_subMode][_currentTool];
}

void T4TApp::ToolMode::setActive(bool active) {
	Mode::setActive(active);
}

bool T4TApp::ToolMode::setSelectedNode(MyNode *node, Vector3 point) {
	bool changed = Mode::setSelectedNode(node, point);
	if(changed) app->setCameraNode(node);
	if(node != NULL) {
		_toolTrans.set(0, 0);
		_toolRot = 0;
		placeTool();
		_scene->addNode(_tool);
	} else {
		_scene->removeNode(_tool);
		_plane = app->_groundPlane;
		setMoveMode(-1);
	}
	_controls->getControl("doTool")->setEnabled(node != NULL);
	_controls->getControl("cancel")->setEnabled(node != NULL);
	return changed;
}

bool T4TApp::ToolMode::setSubMode(short mode) {
	bool changed = Mode::setSubMode(mode);
	if(changed) setTool(0);
	return changed;
}

void T4TApp::ToolMode::setMoveMode(short mode) {
	if(_selectedNode == NULL) return;
	_moveMode = mode;
	_doSelect = _moveMode < 0;
}

void T4TApp::ToolMode::placeCamera() {
	Mode::placeCamera();
	placeTool();
}

void T4TApp::ToolMode::placeTool() {
	if(_selectedNode == NULL) return;
	//tool is positioned at target node's center but with same orientation as camera
	Node *cam = _camera->getNode();
	Vector3 node = _selectedNode->getTranslationWorld();
	Matrix trans;
	trans.translate(node);
	trans.rotate(cam->getRotation());
	trans.translate(_toolTrans.x, _toolTrans.y, 0);
	trans.rotate(Vector3(0, 0, 1), _toolRot);
	_tool->set(trans);
	
	//view plane is orthogonal to the viewing axis and passing through the target node
	_plane.setNormal(node - cam->getTranslationWorld());
	_plane.setDistance(-_plane.getNormal().dot(node));
}

bool T4TApp::ToolMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			break;
		}
		case Touch::TOUCH_MOVE: {
			if(_touching && _moveMode >= 0) {
				Vector2 center;
				switch(_moveMode) {
					case 0: { //rotate
						_camera->project(app->getViewport(), _tool->getTranslationWorld(), &center);
						float angle = atan2(_x - center.x, _y - center.y);
						_toolRot = angle;
						break;
					} case 1: { //translate
						_camera->project(app->getViewport(), _selectedNode->getTranslationWorld(), &center);
						float angle = atan2(_x - center.x, _y - center.y);
						float length = (_mousePoint - _selectedNode->getTranslationWorld()).length();
						_toolTrans.set(length * sin(angle), -length * cos(angle));
						break;
					}
				}
				placeTool();
			}
			break;
		}
		case Touch::TOUCH_RELEASE:
			break;
	}
}

void T4TApp::ToolMode::controlEvent(Control *control, Control::Listener::EventType evt) {
	Mode::controlEvent(control, evt);
	
	const char *id = control->getId();
	
	if(_subModePanel->getControl(id) == control) {
		if(_tools[_subMode].size() > 1) {
			//the selected tool type has more than 1 bit to choose from => prompt which bit they want to use
			_bitMenu->setVisible(true);
		}
	} else if(_bitMenu->getControl(id) == control) {
		for(short i = 0; i < _tools[_subMode].size(); i++) {
			if(_tools[_subMode][i]->id.compare(id) == 0) setTool(i);
		}
		_bitMenu->setVisible(false);
	} else if(_moveMenu->getControl(id) == control) {
		if(strcmp(id, "rotate") == 0) {
			setMoveMode(0);
		} else if(strcmp(id, "translate") == 0) {
			setMoveMode(1);
		}
	} else if(strcmp(id, "doTool") == 0) {
		toolNode();
		setSelectedNode(NULL);
	} else if(strcmp(id, "cancel") == 0) {
		setSelectedNode(NULL);
	}
	
	if(control && control != this && _moveMenu->getControl(id) != control) {
		setMoveMode(-1);
	}
}

bool T4TApp::ToolMode::toolNode() {
	if(_selectedNode == NULL) return false;
	_selectedNode->updateData();
	data = _selectedNode->getData();
	
	usageCount++;

	//reset the data on the altered node
	newData = new MyNode::nodeData();
	_newNode->setData(newData);
	newData->type = data->type;
	newData->objType = "mesh"; //can't keep sphere/box collision object once it has been warped!
	newData->mass = data->mass;
	newData->rotation = data->rotation;
	newData->translation = data->translation;
	newData->scale = data->scale;
	newData->constraints = data->constraints;

	keep.resize(data->vertices.size());
	edgeInt.clear();	
	segmentEdges.clear();
	
	//store all model vertices in the tool frame
	Matrix trans(_tool->getWorldMatrix()), transInv;
	trans.invert(&transInv);
	_axis.setOrigin(0, 0, -50.0f);
	_axis.setDirection(0, 0, 1);
	_axis.transform(trans);
	for(i = 0; i < data->vertices.size(); i++) {
		toolVertices[i].set(data->worldVertices[i]);
		transInv.transformPoint(&toolVertices[i]);
	}

	bool success = false;
	switch(getTool()->type) {
		case 0:
			success = sawNode();
			break;
		case 1:
			success = drillNode();
			break;
	}
	if(!success) return false;
	
	
	//transform the new vertices back to model space before saving the data
	Matrix worldModel;
	_selectedNode->getWorldMatrix().invert(&worldModel);
	for(short i = 0; i < newData->vertices.size(); i++) {
		worldModel.transformPoint(&newData->vertices[i]);
	}

	_newNode->data = NULL;
	_selectedNode->setData(newData);
	_selectedNode->calculateHulls();
	_selectedNode->updateModelFromData();
}


/************ SAWING ************/

bool T4TApp::ToolMode::sawNode() {
	Plane _slicePlane(Vector3(0, 0, 1), 0);
	Matrix trans;
	Matrix::createRotation(_tool->getRotation(), &trans);
	_slicePlane.transform(trans);
	_slicePlane.setDistance(-_tool->getTranslationWorld().dot(_slicePlane.getNormal()));
	cout << "slicing " << _selectedNode->getId() << " at " << app->pv(_slicePlane.getNormal()) << " => " << _slicePlane.getDistance() << endl;
	unsigned short e1, e2, numKeep = 0;
	Vector3 v1, v2, planeOrigin = -_slicePlane.getDistance() * _slicePlane.getNormal();

	std::vector<short> keep(data->vertices.size()), //index in the new vertex list or -1 if discarding
		replace(data->vertices.size()); //index of the new vertex that is replacing this one
	for(int i = 0; i < data->vertices.size(); i++) {
		keep[i] = -1;
		replace[i] = -1;
		float dot = (data->worldVertices[i] - planeOrigin).dot(_slicePlane.getNormal());
		if(dot > 0) continue;
		keep[i] = numKeep++;
		newData->vertices.push_back(data->worldVertices[i]);
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
			newEdge[0] = keep[e1];
			newEdge[1] = keep[e2];
			newData->edges.push_back(newEdge);
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
		newData->vertices.push_back(intersect);
		unsigned short newInd = newData->vertices.size()-1,
			kept = keep[e1] >= 0 ? e1 : e2, discarded = keep[e1] >= 0 ? e2 : e1;
		replace[discarded] = newInd;
		//add the shortened edge to the new mesh
		newEdge[0] = keep[kept];
		newEdge[1] = newInd;
		newData->edges.push_back(newEdge);
		//add this intersection to the map
		intersections[e1][e2] = newInd;
		intersections[e2][e1] = newInd;
	}

	//modify each polygon according to its intersections
	std::vector<unsigned short> newFace, newTriangle(3);
	std::vector<std::vector<unsigned short> > newTriangles, newEdges;
	bool faceAltered, keepTriangle, intervening;
	short usedNew;
	for(int i = 0; i < data->faces.size(); i++) {
		newFace.clear();
		faceAltered = false;
		intervening = false;
		usedNew = -1;
		for(int j = 0; j < data->faces[i].size(); j++) { //for each old polygon...
			e1 = data->faces[i][j];
			e2 = data->faces[i][(j+1)%data->faces[i].size()];
			if(keep[e1] >= 0) {
				newFace.push_back(keep[e1]);
				if(usedNew >= 0) intervening = true;
			}
			else faceAltered = true;
			if(intersections.find(e1) == intersections.end() || intersections[e1].find(e2) == intersections[e1].end()) continue;
			newFace.push_back(intersections[e1][e2]);
			if(usedNew >= 0) { //we have a new edge formed by the 2 new vertices just added to this face
				//add them in reverse order so the new face on the slice plane has the right orientation
				newEdge[intervening ? 1 : 0] = intersections[e1][e2];
				newEdge[intervening ? 0 : 1] = usedNew;
				newData->edges.push_back(newEdge);
				newEdges.push_back(newEdge);
			}
			faceAltered = true;
			usedNew = intersections[e1][e2];
		}
		if(newFace.size() > 0) {
			newData->faces.push_back(newFace);
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
				newData->triangles.push_back(newTriangles);
			} else {
				newData->triangles.push_back(data->triangles[i]);
			}
		}
	}
	
	//add the brand new polygons formed by the slice plane
	unsigned short newFaceStart = newData->faces.size(); //note where the slice plane polygons begin
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
					newData->faces.push_back(newFace);
					newTriangles.clear();
					for(int j = 1; j < newFace.size()-1; j++) {
						newTriangle[0] = 0;
						newTriangle[1] = j;
						newTriangle[2] = j+1;
						newTriangles.push_back(newTriangle);
					}
					newData->triangles.push_back(newTriangles);
					newFace.clear();
				}
				break;
			}
		}
		if(!found) GP_ERROR("Didn't find edge to continue new polygon");
	}
	return true;
}


/************ DRILLING ***************/

bool T4TApp::ToolMode::drillNode() {

	Tool *tool = getTool();

	float _radius = tool->fparam[0];
	int _segments = tool->iparam[0];
	//store all the lines and planes of the drill bit
	std::vector<Ray> lines(_segments);
	std::vector<Plane> planes(_segments);
	//drillInt[drill ray number][face index in old model] = index of intersection point in new model's vertex list
	std::map<unsigned short, std::map<unsigned short, unsigned short> > drillInt;
	//for each edge in a drill plane, which drill plane it is in
	std::map<unsigned short, std::map<unsigned short, unsigned short> > edgeLine;
	//for each drill line intersection point, whether the interior of the object is in the forward drill axis direction
	std::map<unsigned short, bool> enterInt;

	//temp variables
	short i, j, k, m, n, p, q, r;
	float f1, f2, f3, f4, s, t, f[4];
	Vector3 v1, v2, v3, v4;
	std::vector<Vector3> v(4);

	//store the planes and lines of the drillbit segments for calculations
	float angle, dAngle = 2*M_PI/_segments, planeDistance = _radius * cos(dAngle/2);
	for(i = 0; i < _segments; i++) {
		angle = (2*M_PI*i) / _segments;
		//line
		lines[i].setOrigin(_radius*cos(angle), _radius*sin(angle), -50.0f);
		lines[i].setDirection(0, 0, 1);
		lines[i].transform(trans);
		//plane
		planes[i].setNormal(cos(angle+dAngle/2), sin(angle+dAngle/2), 0.0f);
		planes[i].setDistance(-planeDistance);
		planes[i].transform(trans);
		v1.set(planes[i].getNormal());
		f1 = planes[i].getDistance();
		//make sure the plane normal points outward from the drill center
		if(v1.dot(-v1*f1 - _axis.getOrigin()) < 0) {
			planes[i].set(-v1, -f1);
		}
	}

	Vector3 axis(_axis.getDirection()), right(-1, 0, 0), up(0, 1, 0);
	trans.transformVector(&right);
	trans.transformVector(&up);

	//determine which vertices to discard because they are inside the drill cylinder
	Vector3 test;
	float testAngle, testRadius;
	for(i = 0; i < data->vertices.size(); i++) {
		if(drillKeep(i, toolVertices[i])) {
			keep[i] = newData->vertices.size();
			newData->vertices.push_back(data->worldVertices[i]);
		} else keep[i] = -1;
	}

	//find all intersections between edges of the model and the planes of the drill bit
	Vector3 drillVec, intersect;
	unsigned short e[2], lineInd[2], lineNum;
	float d, edgeLen, distance, lineAngle, dist[2];
	for(i = 0; i < data->edges.size(); i++) {
		for(j = 0; j < 2; j++) {
			e[j] = data->edges[i][j];
			v[j].set(toolVertices[e[j]]);
			v[j].z = 0;
		}
		if(!getEdgeDrillInt(e, v, lineInd, dist)) continue;
		for(j = 0; j < 2; j++) {
			if(keep[e[j]] < 0) continue;
			n = newData->vertices.size();
			newData->vertices.push_back(data->worldVertices[e[0]]
			  + (data->worldVertices[e[1]] - data->worldVertices[e[0]]) * dist[j]);
			edgeInt[e[j]][e[(j+1)%2]] = std::pair<unsigned short, unsigned short>(lineInd[j], n);
		}
		//if either vertex is not being kept, both directions intersect the drill at the same point
		for(j = 0; j < 2; j++) if(keep[e[j]] < 0) edgeInt[e[j]][e[(j+1)%2]] = edgeInt[e[(j+1)%2]][e[j]];
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

		ccw = data->worldNormals[i].dot(axis) < 0;
		dir = ccw ? 1 : -1;
		offset = dir == 1 ? 1 : 0;
		facePlane.set(data->worldNormals[i],
		  -data->worldVertices[data->faces[i][0]].dot(data->worldNormals[i]));

		//index the edge intersections in this face by drill line number
		for(j = 0; j < n; j++) {
			p = data->faces[i][j];
			if(keep[p] < 0) continue;
			keeping.insert(j);
			q = data->faces[i][(j-1+n)%n]; //since we always walk the face in order, we only need to store back intersections
			if(edgeInt.find(p) != edgeInt.end() && edgeInt[p].find(q) != edgeInt[p].end()) {
				lineNum = edgeInt[p][q].first;
				v1.set(newData->vertices[edgeInt[p][q].second] - _axis.getOrigin());
				angle = atan2(v1.dot(up), -v1.dot(right));
				while(angle < 0) angle += 2*M_PI;
				//within a given line number, if there are multiple intersections, order by angle wrt drill center
				// - order in the same direction we will walk the drill
				for(fit = faceInts[lineNum].begin(); fit != faceInts[lineNum].end() && 
				  (dir == 1 ? angle > fit->second : angle < fit->second); fit++);
				faceInts[lineNum].insert(fit, std::pair<unsigned short, float>(j, angle));
			}
		}
		if(keeping.empty()) continue; //not keeping any part of this face

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
						angle = atan2(v1.dot(up), -v1.dot(right));
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
				v1.set(toolVertices[data->faces[i][data->triangles[i][j][1]]]
					- toolVertices[data->faces[i][data->triangles[i][j][0]]]);
				v2.set(toolVertices[data->faces[i][data->triangles[i][j][2]]]
					- toolVertices[data->faces[i][data->triangles[i][j][0]]]);
				drillVec.set(-toolVertices[data->faces[i][data->triangles[i][j][0]]]);
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
					angle = atan2(toolVertices[k].y, toolVertices[k].x);
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
						v1.set(_radius * cos(angle), _radius * sin(angle), 0);
						v2.set(toolVertices[data->faces[i][q]] - v1);
						found = true;
						for(k = (q+1)%n; k != (q-1+n)%n; k = (k+1)%n) {
							v3.set(toolVertices[data->faces[i][k]]);
							v4.set(toolVertices[data->faces[i][(k+1)%n]] - v3);
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
				newFace.clear();
				for(j = 0; j < data->faces[i].size(); j++) {
					newFace.push_back(keep[data->faces[i][j]]);
				}
				_newNode->addFace(newFace, data->triangles[i]);
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
			enterInt[n] = data->worldNormals[it1->first].dot(axis) < 0;
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
	
	//split convex hulls using the radial and tangential planes of the drill
	MyNode::convexHull *hull, *newHull;
	Matrix world = _selectedNode->getWorldMatrix(), tool = _tool->getWorldMatrix();
	tool.invert();
	tool *= world;
	std::vector<Vector3> face;
	for(i = 0; i < data->hulls.size(); i++) {
		hull = data->hulls[i];
		short nv = hull->vertices.size(), nf = hull->faces.size(), ne = hull->edges.size(), numKeep = 0;
		//get the drill-axis coords for the vertices of this hull
		toolVertices.resize(nv);
		keep.resize(nv);
		for(j = 0; j < nv; j++) {
			toolVertices[j] = hull->vertices[j];
			tool.transformPoint(&toolVertices[j]);
			keep[j] = drillKeep(j, toolVertices[j]) ? 1 : -1;
			if(keep[j] >= 0) numKeep++;
		}
		if(numKeep == 0) continue;
		//only split this hull if one or more of its vertices is inside the drill or edges intersects the drill bit
		bool found = numKeep < nv;
		if(!found) for(j = 0; j < ne; j++) {
			for(k = 0; k < 2; k++) {
				e[k] = hull->edges[j][k];
				v[k] = toolVertices[e[k]];
			}
			if(getEdgeDrillInt(e, v, lineInd, dist)) {
				found = true;
				break;
			}
		}
		if(!found) { //leave hull as is
			newHull = new MyNode::convexHull(*hull);
			newData->hulls.push_back(newHull);
			continue;
		}
		//for each drill segment, if the hull intersects its angle range, build a new hull from the intersection
		for(j = 0; j <= _segments; j++) {
			newHull = new MyNode::convexHull();
			edgeInt.clear();
			drillInt.clear();
			segmentEdges.clear();
			float minAngle = j * dAngle, maxAngle = (j+1) * dAngle;
			short numKeep, start, lastPlane, plane;
			//determine which vertices are included
			for(k = 0; k < nv; k++) {
				angle = atan2(toolVertices[k].y, toolVertices[k].x);
				if(keep[k] >= 0 && angle > minAngle && angle < maxAngle) {
					keep[k] = newHull->vertices.size();
					newHull->vertices.push_back(hull->vertices[k]);
					numKeep++;
				} else keep[k] = -1;
			}
			//find all edge intersections
			Vector3 best[2];
			short planeInd[2];
			for(k = 0; k < ne; k++) {
				for(m = 0; m < 2; m++) {
					e[m] = hull->edges[k][m];
					v[m] = toolVertices[e[m]];
					v[m].z = 0;
				}
				if(keep[e[0]] >= 0 && keep[e[1]] >= 0) continue;
				v[2] = v[1] - v[0];
				for(m = 0; m < 2; m++) {
					dist[m] = 1000;
					planeInd[m] = -1;
				}
				//first check the two radial planes
				for(m = 0; m < 2; m++) {
					// (v0 + a*v2) * n = 0 => a = -v0*n / v2*n
					angle = m == 0 ? minAngle : maxAngle;
					normal.set(-sin(angle), cos(angle), 0);
					f1 = v[2].dot(normal);
					if(f1 == 0) {
						if((keep[e[0]] < 0) != (keep[e[1]] < 0)) distance = keep[e[0]] >= 0 ? 0 : 1;
						else continue;
					} else distance = -v[0].dot(normal) / f1;
					if(distance < 0 || distance > 1) continue;
					v1 = v[0] + distance * v[2];
					v2.set(cos(angle), sin(angle), 0);
					if(v1.dot(v2) < _radius) continue;
					for(n = 0; n < 2; n++) {
						if(keep[e[n]] < 0) continue;
						f1 = n == 0 ? distance : 1 - distance;
						if(distance < dist[n]) {
							dist[n] = distance;
							best[n] = v1;
							planeInd[n] = m*2;
						}
					}
				}
				//then the drill plane
				for(m = 0; m < 1; m++) {
					// (v0 + a*v2) * r = planeDistance => a = (planeDistance - v0*r) / v2*r
					angle = minAngle + dAngle/2;
					normal.set(cos(angle), sin(angle), 0);
					f1 = v[2].dot(normal);
					if(f1 == 0) {
						if((keep[e[0]] < 0) != (keep[e[1]] < 0)) distance = keep[e[0]] >= 0 ? 0 : 1;
						else continue;
					} else distance = (planeDistance - v[0].dot(normal)) / f1;
					if(distance < 0 || distance > 1) continue;
					v1 = v[0] + distance * v[2];
					
				}
			}
			if(numKeep == 0 && edgeInt.empty()) continue;
			newData->hulls.push_back(newHull);
			//build modified faces
			for(k = 0; k < nf; k++) {
				faceSize = hull->faces[k].size();
				newFace.clear();
				lastPlane = -1;
				lastInter = -1;
				//determine the plane for this face in world space
				face.resize(faceSize);
				for(m = 0; m < faceSize; m++) face[m] = hull->vertices[hull->faces[k][m]];
				normal = MyNode::getNormal(face);
				facePlane.set(normal, -normal.dot(face[0]));
				facePlane.transform(world);
				//start on a vertex we are keeping, if any
				for(start = 0; start < faceSize && keep[hull->faces[k][start]] < 0; start++);
				//if not, look for an edge intersection exiting the segment
				if(start == faceSize) for(start = 0; start < faceSize; start++) {
					for(m = 0; m < 2; m++) e[m] = hull->faces[k][(start+m)%faceSize];
					if(edgeInt.find(e[1]) != edgeInt.end() && edgeInt[e[1]].find(e[0]) != edgeInt[e[1]].end()) {
						newFace.push_back(edgeInt[e[1]][e[0]].second);
						lastPlane = edgeInt[e[1]][e[0]].first;
						lastInter = newFace[0];
						start = (start+1) % faceSize;
						break;
					}
				}
				//if none of those either, we are discarding the whole face
				if(start == faceSize) continue;
				//otherwise, loop around the face adding kept vertices and intersections
				for(m = start; m != (start+1)%faceSize; m = (m+1)%faceSize) {
					for(n = 0; n < 2; n++) e[n] = hull->faces[k][(m+n)%faceSize];
					if(keep[e[0]] >= 0) newFace.push_back(keep[e[0]]);
					if(edgeInt.find(e[0]) != edgeInt.end() && edgeInt[e[0]].find(e[1]) != edgeInt[e[0]].end()) {
						plane = edgeInt[e[0]][e[1]].first;
						//if we are re-entering the segment on a different plane than we exited,
						//the drill lines in between those planes must intersect the face
						if(keep[e[0]] < 0 && lastPlane >= 0 && lastPlane != plane) {
							dir = plane > lastPlane ? 1 : -1;
							offset = dir == 1 ? 0 : -1;
							for(n = lastPlane + offset; n != plane + offset; n += dir) {
								distance = lines[j+n].intersects(facePlane);
								p = newHull->vertices.size();
								newFace.push_back(p);
								newHull->vertices.push_back(lines[j+n].getOrigin() + distance * lines[j+n].getDirection());
								addHullEdge(newHull, lastInter, p, lastPlane);
								lastPlane = n - offset;
							}
						}
						newFace.push_back(edgeInt[e[0]][e[1]].second);
						if(keep[e[0]] < 0) addHullEdge(newHull, lastInter, edgeInt[e[0]][e[1]].second, plane);
						if(edgeInt[e[1]][e[0]].second != edgeInt[e[0]][e[1]].second
						  && edgeInt[e[1]][e[0]].second != newFace[0]) {
							newFace.push_back(edgeInt[e[1]][e[0]].second);
							lastPlane = edgeInt[e[1]][e[0]].first;
							lastInter = edgeInt[e[1]][e[0]].second;
						} else if(keep[e[1]] < 0) lastPlane = plane;
					}
				}
				
			}
			//add faces on drill planes
			for(k = 0; k < 3; k++) {
				newFace.clear();
				while(!segmentEdges[k].empty()) {
					if(newFace.empty()) {
						eit = segmentEdges[k].begin();
						p = eit->first;
						newFace.push_back(p);
					} else {
						p = newFace[newFace.size()-1];
					}
					if(segmentEdges[k].find(p) == segmentEdges[k].end()) {
						GP_WARN("Couldn't continue hull face from point %d", p);
						return false;
					}
					q = segmentEdges[k][p];
					segmentEdges[k].erase(p);
					if(!newFace.empty() && q == newFace[0]) {
						newHull->faces.push_back(newFace);
						newFace.clear();
					} else newFace.push_back(q);
				}
			}
		}
	}
	return true;
}

bool T4TApp::ToolMode::drillKeep(unsigned short n, Vector3& v) {
	Vector3 test = toolVertices[n];
	float testAngle = atan2(test.y, test.x);
	if(testAngle < 0) testAngle += 2*M_PI;
	float testRadius = sqrt(test.x*test.x + test.y*test.y);
	float ang = fabs(fmod(testAngle, dAngle) - dAngle/2), radius = planeDistance / cos(ang);
	return testRadius >= radius;
}

bool T4TApp::ToolMode::getEdgeDrillInt(unsigned short *e, Vector3 *v, unsigned short *lineInd, float *distance) {
	Tool *tool = getTool();
	short _segments = tool->iparam[0];
	float _radius = tool->fparam[0], f1, f2, f3, angle, dAngle = 2*M_PI / _segments;

	if(keep[e[0]] < 0 && keep[e[1]] < 0) return false;
	v[2].set(v[1]-v[0]);
	//validate the edge is really there
	if(v[2] == Vector3::zero()) continue;
	//find the angles where this edge enters and exits the drill circle
	//|v0 + a*v2| = r => (v0x + a*v2x)^2 + (v0y + a*v2y)^2 = r^2 => |v2|^2 * a^2 + 2*(v0*v2)*a + |v0|^2 - r^2 = 0
	// => a = [-2*v0*v2 +/- sqrt(4*(v0*v2)^2 - 4*(|v2|^2)*(|v0|^2 - r^2))] / 2*|v2|^2
	// = [-v0*v2 +/- sqrt((v0*v2)^2 - (|v2|^2)*(|v0|^2 - r^2))] / |v2|^2
	f1 = v[0].dot(v[2]);
	f2 = v[0].lengthSquared() - _radius * _radius;
	f3 = v[2].lengthSquared();
	d = f1*f1 - f3*f2;
	if(d < 0) return false; //edge is completely outside drill circle
	s = (-f1 - sqrt(d)) / f3;
	t = (-f1 + sqrt(d)) / f3;
	//neither intersection should be outside the endpoints if we are keeping both of them
	if((s < 0 || t > 1) && keep[e[0]] >=0 && keep[e[1]] >= 0) return false;
	//for each edge direction, first get the segment from the angle where it intersects the circle
	for(j = 0; j < 2; j++) {
		if(keep[e[j]] < 0) continue;
		v1.set(v[0] + v[2] * (j == 0 ? s : t));
		angle = atan2(v1.y, v1.x);
		if(angle < 0) angle += 2*M_PI;
		lineInd[j] = (unsigned short)(angle / dAngle);
	}
	//if keeping both endpoints and they hit the same angle segment, the edge must not really touch the drill plane
	if(keep[e[0]] >= 0 && keep[e[1]] >= 0 && lineInd[0] == lineInd[1]) return false;
	//for each edge direction, find the point where it intersects the drill plane in the angle range found above
	for(j = 0; j < 2; j++) {
		if(keep[e[j]] < 0) continue;
		lineAngle = lineInd[j] * dAngle + dAngle/2;
		v2.set(cos(lineAngle), sin(lineAngle), 0);
		//(v0 + a*v2) * r = planeDistance => a = (planeDistance - v0*r) / v2*r
		distance[j] = (planeDistance - v[0].dot(v2)) / v[2].dot(v2);
	}
	return true;
}

void T4TApp::ToolMode::addDrillEdge(unsigned short v1, unsigned short v2, unsigned short lineNum) {
	_newNode->addEdge(v1, v2);
	segmentEdges[lineNum][v1] = v2;
}

void T4TApp::ToolMode::addHullEdge(MyNode::convexHull *hull, unsigned short v1, unsigned short v2, short segment) {
	if(hull->edgeInd.find(v1) != hull->edgeInd.end() && hull->edgeInd[v1].find(v2) != hull->edgeInd[v1].end()) return;
	hull->edgeInd[v1][v2] = hull->edges.size();
	hull->edgeInd[v2][v1] = hull->edges.size();
	std::vector<unsigned short> edge(2);
	edge[0] = v1;
	edge[1] = v2;
	hull->edges.push_back(edge);
	if(segment >= 0) segmentEdges[segment][v2] = v1;
}

