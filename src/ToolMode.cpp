#include "T4TApp.h"

ToolMode::ToolMode() 
  : Mode::Mode("tool") {

	_tool = MyNode::create("tool_tool");
	_newNode = MyNode::create("newNode_tool");
	
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

void ToolMode::createBit(short type, ...) {
	va_list args;
	va_start(args, type);
	
	Tool *tool = new Tool();
	tool->type = type;
	_tools[type].push_back(tool);
	
	std::vector<float> vertices;
	short i, j, k, m, v = 0;
	float color[3] = {1.0f, 1.0f, 1.0f};
	Vector3 vec;

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
					vec.set(radius * cos(angle), radius * sin(angle), (2*j-1) * length);
					tool->addVertex(vec);
					vertices[v++] = vec.x;
					vertices[v++] = vec.y;
					vertices[v++] = vec.z;
					for(k = 0; k < 3; k++) vertices[v++] = color[k];
				}
				for(j = 0; j < 2; j++) {
					for(k = 0; k < 2; k++) {
						vec.set(radius * cos(angle + k*dAngle), radius * sin(angle + k*dAngle), (2*j-1) * length);
						vertices[v++] = vec.x;
						vertices[v++] = vec.y;
						vertices[v++] = vec.z;
						for(m = 0; m < 3; m++) vertices[v++] = color[m];
					}
				}
			}
			std::vector<std::vector<unsigned short> > triangles(2);
			for(i = 0; i < 2; i++) triangles[i].resize(3);
			for(i = 0; i < 2; i++) for(j = 0; j < 3; j++) triangles[i][j] = j+i;
			triangles[1][0] = 0;
			tool->_triangles.resize(segments+2);
			for(i = 0; i < segments; i++) {
				j = (i+1)%segments;
				tool->addFace(4, i*2+1, i*2, j*2, j*2+1);
				tool->_triangles[i] = triangles;
			}
			std::vector<unsigned short> face(segments);
			for(i = 0; i < segments; i++) face[i] = 2*(segments-1 - i);
			tool->addFace(face);
			for(i = 0; i < segments; i++) face[i] = 2*i + 1;
			tool->addFace(face);
			triangles.resize(segments-2);
			for(i = 0; i < segments-2; i++) {
				triangles[i].resize(3);
				triangles[i][0] = 0;
				triangles[i][1] = i+1;
				triangles[i][2] = i+2;
			}
			tool->_triangles[segments] = triangles;
			tool->_triangles[segments+1] = triangles;
			//add a menu item for this bit
			std::ostringstream os;
			os << "drill_bit_" << segments << "_" << (int)(radius * 100 + 0.1);
			tool->id = os.str();
			std::string file = "res/png/" + tool->id + ".png";
			ImageControl *image = (ImageControl*) app->addButton<ImageControl>(_bitMenu, tool->id.c_str(), "", app->_theme->getStyle("imageSquare"));
			image->setZIndex(_bitMenu->getZIndex());
			image->setSize(100.0f, 100.0f);
			image->setImage(file.c_str());
			image->addListener(this, Control::Listener::CLICK);
			break;
		}
	}
	va_end(args);
	tool->model = app->createModel(vertices, true, "grid");
}

void ToolMode::setTool(short n) {
	_currentTool = n;
	Tool *tool = getTool();
	_tool->setModel(tool->model);
	_tool->_vertices = tool->_vertices;
	_tool->_faces = tool->_faces;
	_tool->_triangles = tool->_triangles;
	_tool->update();
}

ToolMode::Tool* ToolMode::getTool() {
	return _tools[_subMode][_currentTool];
}

void ToolMode::setActive(bool active) {
	Mode::setActive(active);
}

bool ToolMode::setSelectedNode(MyNode *node, Vector3 point) {
	bool changed = Mode::setSelectedNode(node, point);
	if(changed) app->setCameraNode(node);
	if(node != NULL) {
		_toolTrans.set(0, 0);
		_toolRot = 0;
		placeTool();
		_scene->addNode(_tool);
		//app->showFace(_selectedNode, _selectedNode->_faces[3], true);
	} else {
		_scene->removeNode(_tool);
		_plane = app->_groundPlane;
		setMoveMode(-1);
	}
	_controls->getControl("doTool")->setEnabled(node != NULL);
	_controls->getControl("cancel")->setEnabled(node != NULL);
	return changed;
}

bool ToolMode::setSubMode(short mode) {
	bool changed = Mode::setSubMode(mode);
	if(changed) setTool(0);
	return changed;
}

void ToolMode::setMoveMode(short mode) {
	if(_selectedNode == NULL) return;
	_moveMode = mode;
	_doSelect = _moveMode < 0;
}

void ToolMode::placeCamera() {
	Mode::placeCamera();
	placeTool();
}

void ToolMode::placeTool() {
	if(_selectedNode == NULL) return;
	//tool is positioned at target node's center but with same orientation as camera
	Node *cam = _camera->getNode();
	Vector3 node = _selectedNode->getTranslationWorld();
	Matrix trans;
	trans.translate(node);
	trans.rotate(cam->getRotation());
	trans.translate(_toolTrans.x, _toolTrans.y, 0);
	trans.rotate(Vector3::unitZ(), _toolRot);
	trans.rotate(Vector3::unitY(), M_PI);
	_tool->set(trans);
	
	//view plane is orthogonal to the viewing axis and passing through the target node
	_plane.setNormal(node - cam->getTranslationWorld());
	_plane.setDistance(-_plane.getNormal().dot(node));
}

bool ToolMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			break;
		}
		case Touch::TOUCH_MOVE: {
			if(_touching && _moveMode >= 0 && app->_navMode < 0) {
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

void ToolMode::controlEvent(Control *control, Control::Listener::EventType evt) {
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

void ToolMode::setMesh(Meshy *mesh) {
	_mesh = mesh;
	short nv = _mesh->_vertices.size(), i, j, k;
	keep.resize(nv);
	toolVertices.resize(nv);
	Matrix tool = _tool->getWorldMatrix();
	tool.invert();
	for(i = 0; i < nv; i++) {
		tool.transformPoint(_mesh->_worldVertices[i], &toolVertices[i]);
	}
}

bool ToolMode::toolNode() {
	if(_selectedNode == NULL) return false;
	_node = _selectedNode;
	_node->updateTransform();
	setMesh(_node);
	_newMesh = _newNode;
	_newNode->set(_node->getWorldMatrix()); //align the transforms

	//reset the data on the altered node
	_newNode->_vertices.clear();
	_newNode->_faces.clear();
	_newNode->_triangles.clear();
	_newNode->_edges.clear();
	_newNode->_edgeInd.clear();
	_newNode->_hulls.clear();

	short i, j, k;	
	usageCount++;
	
	edgeInt.clear();
	toolInt.clear();
	segmentEdges.clear();
	
	Matrix tool = _tool->getWorldMatrix();
	Vector3 center = _tool->getTranslationWorld();
	axis = Vector3::unitZ();
	tool.transformVector(&axis);
	right = -Vector3::unitX();
	tool.transformVector(&right);
	up = Vector3::unitY();
	tool.transformVector(&up);
	_axis.set(center - axis * 50, axis);

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
	
	//transform the new model and convex hull vertices back to model space and store in the original node
	Matrix worldModel;
	_node->getWorldMatrix().invert(&worldModel);
	_node->_vertices = _newNode->_vertices;
	for(i = 0; i < _node->nv(); i++) {
		worldModel.transformPoint(&_node->_vertices[i]);
	}
	_node->_faces = _newNode->_faces;
	_node->_triangles = _newNode->_triangles;
	_node->_objType = "mesh";
	_node->_hulls = _newNode->_hulls;
	for(i = 0; i < _node->_hulls.size(); i++) {
		MyNode::ConvexHull *hull = _node->_hulls[i];
		hull->_node = _node;
		for(j = 0; j < hull->nv(); j++) {
			worldModel.transformPoint(&hull->_vertices[j]);
		}
		hull->setNormals();
	}

	//put all the changes into the simulation
	_node->updateModel();
}


/************ SAWING ************/

bool ToolMode::sawNode() {
	return true;
}


/************ DRILLING ***************/

bool ToolMode::drillNode() {

	Tool *tool = getTool();

	float _radius = tool->fparam[0];
	int _segments = tool->iparam[0];

	//temp variables
	short i, j, k, m, n, p, q, r;
	float f1, f2, f3, f4, s, t, f[4];
	Vector3 v1, v2, v3, v4, v[4], drillVec;

	//store the planes and lines of the drillbit segments for calculations
	Matrix nodeWorld = _selectedNode->getWorldMatrix(), toolWorld = _tool->getWorldMatrix();
	float angle, dAngle = 2*M_PI/_segments, planeDistance = _radius * cos(dAngle/2);
	lines.resize(_segments);
	planes.resize(_segments);
	for(i = 0; i < _segments; i++) {
		angle = (2*M_PI*i) / _segments;
		//line
		lines[i].setOrigin(_radius*cos(angle), _radius*sin(angle), -50.0f);
		lines[i].setDirection(0, 0, 1);
		lines[i].transform(toolWorld);
		//plane
		planes[i].setNormal(cos(angle+dAngle/2), sin(angle+dAngle/2), 0.0f);
		planes[i].setDistance(-planeDistance);
		planes[i].transform(toolWorld);
		v1.set(planes[i].getNormal());
		f1 = planes[i].getDistance();
		//make sure the plane normal points outward from the drill center
		if(v1.dot(-v1*f1 - _axis.getOrigin()) < 0) {
			planes[i].set(-v1, -f1);
		}
	}

	//determine which vertices to discard because they are inside the drill cylinder
	for(i = 0; i < _mesh->_vertices.size(); i++) {
		if(drillKeep(i)) {
			keep[i] = _newMesh->_vertices.size();
			_newMesh->_vertices.push_back(_selectedNode->_worldVertices[i]);
		} else keep[i] = -1;
	}

	//find all intersections between edges of the model and the planes of the drill bit
	getEdgeInt(&ToolMode::getEdgeDrillInt);
	
	//loop around each original face, building modified faces according to the drill intersections
	bool ccw, drillInside, found;
	short dir, offset, mode, numInts, lineNum;
	unsigned short lastInter, e[2];
	float denom, delta, epsilon = 0.001f, minDist, distance;
	std::pair<unsigned short, unsigned short> ints[2];
	Plane facePlane;
	std::vector<unsigned short> newFace;
	std::vector<unsigned short> drillPoint;
	std::set<unsigned short> keeping;
	std::set<unsigned short>::iterator kit;
	std::map<unsigned short, std::vector<std::pair<unsigned short, float> > > faceInts;
	std::vector<std::pair<unsigned short, float> > angles;
	std::vector<std::pair<unsigned short, float> >::iterator fit;
	for(i = 0; i < _mesh->_faces.size(); i++) {
		n = _mesh->_faces[i].size();
		keeping.clear();
		faceInts.clear();

		ccw = _mesh->_worldNormals[i].dot(axis) < 0;
		dir = ccw ? 1 : -1;
		offset = dir == 1 ? 1 : 0;
		facePlane.set(_mesh->_worldNormals[i],
		  -_mesh->_worldVertices[_mesh->_faces[i][0]].dot(_mesh->_worldNormals[i]));

		//index the edge intersections in this face by drill line number
		for(j = 0; j < n; j++) {
			p = _mesh->_faces[i][j];
			if(keep[p] < 0) continue;
			keeping.insert(j);
			q = _mesh->_faces[i][(j-1+n)%n]; //since we always walk the face in order, we only need to store back intersections
			if(edgeInt.find(p) != edgeInt.end() && edgeInt[p].find(q) != edgeInt[p].end()) {
				lineNum = edgeInt[p][q].first;
				v1.set(_newMesh->_vertices[edgeInt[p][q].second] - _axis.getOrigin());
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
			p = _mesh->_faces[i][j];
			mode = 0;
			numInts = 0;
			do {
				switch(mode) {
					case 0: //walking the original face
						newFace.push_back(keep[p]);
						keeping.erase(j);
						j = (j+1)%n;
						q = _mesh->_faces[i][j];
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
						v1.set(_newMesh->_vertices[p] - _axis.getOrigin());
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
								p = _mesh->_faces[i][k];
								q = edgeInt[p][_mesh->_faces[i][(k-1+n)%n]].second;
								newFace.push_back(q);
								addToolEdge(q, lastInter, lineNum);
								faceInts[lineNum].erase(fit);
								if(faceInts[lineNum].empty()) faceInts.erase(lineNum);
								mode = 0;
								j = k;
								break;
							}
						}
						//otherwise the next drill line must intersect the face
						k = (lineNum + offset) % _segments;
						m = _newMesh->_vertices.size();
						distance = lines[k].intersects(facePlane);
						_newMesh->_vertices.push_back(lines[k].getOrigin() + distance * lines[k].getDirection());
						newFace.push_back(m);
						toolInt[k][i] = m;
						addToolEdge(m, lastInter, lineNum);
						lastInter = m;
						lineNum = (lineNum + dir + _segments) % _segments;
						angle = -1;
						numInts++;
						break;
				}
			} while((mode != 0 || keep[p] != newFace[0]) && numInts <= _segments);
			if(numInts > _segments) {
				GP_ERROR("Couldn't exit drill on face %d", i);
			}
			_newNode->addFace(newFace);
		} else { //no edge intersections => only question is whether entire drill bit passes through this face
			//use drill center as indicator
			drillInside = false;
			for(j = 0; j < _selectedNode->_triangles[i].size(); j++) {
				v1.set(toolVertices[_selectedNode->_faces[i][_selectedNode->_triangles[i][j][1]]]
					- toolVertices[_selectedNode->_faces[i][_selectedNode->_triangles[i][j][0]]]);
				v2.set(toolVertices[_selectedNode->_faces[i][_selectedNode->_triangles[i][j][2]]]
					- toolVertices[_selectedNode->_faces[i][_selectedNode->_triangles[i][j][0]]]);
				drillVec.set(-toolVertices[_selectedNode->_faces[i][_selectedNode->_triangles[i][j][0]]]);
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
					toolInt[j][i] = _newMesh->_vertices.size();;
					_newMesh->_vertices.push_back(lines[j].getOrigin() + lines[j].getDirection() * distance);
				}
				//add edges on the drill segments making sure to get the right orientation
				for(j = 0; j < _segments; j++)
					addToolEdge(toolInt[j][i], toolInt[(j-dir+_segments)%_segments][i], (j-offset+_segments)%_segments);
				//order the face vertices by angle wrt drill center
				angles.clear();
				for(j = 0; j < n; j++) {
					k = _mesh->_faces[i][j];
					angle = atan2(toolVertices[k].y, toolVertices[k].x);
					while(angle < 0) angle += 2*M_PI;
					for(fit = angles.begin(); fit != angles.end() && angle > fit->second; fit++);
					angles.insert(fit, std::pair<unsigned short, float>(j, angle));
				}
				//for each of two opposite drill vertices, find an edge to a face vertex, thus splitting the face into two
				for(j = 0; j < 2; j++) {
					p = toolInt[j * _segments/2][i];
					angle = dAngle * j * _segments/2;
					for(m = 0; m < n && angles[m].second < angle; m++);
					m %= n;
					//try vertices as close in angle to this drill point as possible until we find a valid edge
					// (doesn't intersect any existing edges of the face)
					offset = 1;
					do {
						q = angles[m].first;
						v1.set(_radius * cos(angle), _radius * sin(angle), 0);
						v2.set(toolVertices[_mesh->_faces[i][q]] - v1);
						found = true;
						for(k = (q+1)%n; k != (q-1+n)%n; k = (k+1)%n) {
							v3.set(toolVertices[_mesh->_faces[i][k]]);
							v4.set(toolVertices[_mesh->_faces[i][(k+1)%n]] - v3);
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
						GP_ERROR("Can't get edge in face %d from drill point %d", i, p);
					}
					e[j] = q;
				}
				//perform the split
				for(j = 0; j < 2; j++) {
					newFace.clear();
					m = (j+1)%2;
					for(k = e[j]; k != (e[m]+1)%n; k = (k+1)%n)
						newFace.push_back(keep[_mesh->_faces[i][k]]);
					for(k = m * _segments/2; k != (j*_segments/2+dir+_segments) % _segments; k = (k+dir+_segments)%_segments)
						newFace.push_back(toolInt[k][i]);
					_newNode->addFace(newFace);
				}
			} else { //the drill does not touch this face => leave it as is
				newFace.clear();
				for(j = 0; j < _mesh->_faces[i].size(); j++) {
					newFace.push_back(keep[_mesh->_faces[i][j]]);
				}
				_newNode->addFace(newFace);
			}
		}
	}
	addToolFaces();
	
	//split convex hulls using the radial and tangential planes of the drill
	MyNode::ConvexHull *hull, *newHull;
	std::vector<Vector3> face;
	Vector3 normal;
	std::vector<bool> keepDrill;
	short nh = _node->_hulls.size();
	for(i = 0; i < nh; i++) {
		hull = _node->_hulls[i];
		setMesh(hull);
		short nv = hull->nv(), nf = hull->nf(), numKeep = 0, faceSize;
		keepDrill.resize(nv);
		for(j = 0; j < nv; j++) {
			keepDrill[j] = drillKeep(j);
			if(keepDrill[j]) numKeep++;
		}
		if(numKeep == 0) continue;
		//for each drill segment, if the hull intersects its angle range, build a new hull from the intersection
		for(j = 0; j < _segments; j++) {
			_hullSlice = j;
			newHull = new MyNode::ConvexHull(_newNode);
			_newMesh = newHull;
			edgeInt.clear();
			toolInt.clear();
			segmentEdges.clear();
			float minAngle = j * dAngle, maxAngle = (j+1) * dAngle;
			short start, lastPlane, plane;
			//determine which vertices are included
			numKeep = 0;
			for(k = 0; k < nv; k++) {
				angle = atan2(toolVertices[k].y, toolVertices[k].x);
				if(angle < 0) angle += 2*M_PI;
				if(keepDrill[k] && angle > minAngle && angle < maxAngle) {
					keep[k] = newHull->_vertices.size();
					newHull->_vertices.push_back(hull->_worldVertices[k]);
					numKeep++;
				} else keep[k] = -1;
			}
			//find all edge intersections
			getEdgeInt(&ToolMode::getHullSliceInt);
			if(numKeep == 0 && edgeInt.empty()) continue;
			_newNode->_hulls.push_back(newHull);
			//build modified faces
			for(k = 0; k < nf; k++) {
				faceSize = hull->_faces[k].size();
				newFace.clear();
				lastPlane = -1;
				lastInter = -1;
				//determine the plane for this face in world space
				normal = hull->_worldNormals[k];
				facePlane.set(normal, -normal.dot(hull->_worldVertices[hull->_faces[k][0]]));
				//start on a vertex we are keeping, if any
				for(start = 0; start < faceSize && keep[hull->_faces[k][start]] < 0; start++);
				//if not, look for an edge intersection exiting the segment
				if(start == faceSize) for(start = 0; start < faceSize; start++) {
					for(m = 0; m < 2; m++) e[m] = hull->_faces[k][(start+m)%faceSize];
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
				for(r = 0; r < faceSize; r++) {
					m = (start+r) % faceSize;
					for(n = 0; n < 2; n++) e[n] = hull->_faces[k][(m+n)%faceSize];
					if(keep[e[0]] >= 0) newFace.push_back(keep[e[0]]);
					if(edgeInt.find(e[0]) != edgeInt.end() && edgeInt[e[0]].find(e[1]) != edgeInt[e[0]].end()) {
						plane = edgeInt[e[0]][e[1]].first;
						//if we are re-entering the segment on a different plane than we exited,
						//the drill lines in between those planes must intersect the face
						if(keep[e[0]] < 0 && lastPlane >= 0 && lastPlane != plane) {
							dir = plane > lastPlane ? 1 : -1;
							offset = dir == 1 ? 0 : -1;
							for(n = lastPlane + offset; n != plane + offset; n += dir) {
								Ray line = lines[(j+n) % _segments];
								distance = line.intersects(facePlane);
								p = newHull->nv();
								v1 = line.getOrigin() + line.getDirection() * distance;
								cout << "hull " << j << " adding " << app->pv(v1) << endl;
								newHull->addVertex(v1);
								newFace.push_back(p);
								toolInt[n+1][k] = p;
								addToolEdge(p, lastInter, n - offset);
								lastInter = p;
							}
						}
						newFace.push_back(edgeInt[e[0]][e[1]].second);
						if(keep[e[0]] < 0) addToolEdge(edgeInt[e[0]][e[1]].second, lastInter, plane);
						if(edgeInt[e[1]][e[0]].second != edgeInt[e[0]][e[1]].second
						  && edgeInt[e[1]][e[0]].second != newFace[0]) {
							newFace.push_back(edgeInt[e[1]][e[0]].second);
							lastPlane = edgeInt[e[1]][e[0]].first;
							lastInter = edgeInt[e[1]][e[0]].second;
						} else if(keep[e[1]] < 0) {
							lastPlane = plane;
							lastInter = edgeInt[e[0]][e[1]].second;
						}
					}
				}
				if(!newFace.empty()) newHull->addFace(newFace);
			}
			addToolFaces();
		}
	}
	return true;
}

bool ToolMode::drillKeep(unsigned short n) {
	Tool *tool = getTool();
	short _segments = tool->iparam[0];
	float _radius = tool->fparam[0], dAngle = 2*M_PI / _segments, planeDistance = _radius * cos(dAngle/2);

	Vector3 test = toolVertices[n];
	float testAngle = atan2(test.y, test.x);
	if(testAngle < 0) testAngle += 2*M_PI;
	float testRadius = sqrt(test.x*test.x + test.y*test.y);
	float ang = fabs(fmod(testAngle, dAngle) - dAngle/2), radius = planeDistance / cos(ang);
	return testRadius >= radius;
}

void ToolMode::getEdgeInt(bool (ToolMode::*getInt)(unsigned short*, short*, float*)) {
	short i, j, k, ne = _mesh->ne(), line[2];
	unsigned short e[2];
	float dist[2];
	for(i = 0; i < ne; i++) {
		for(j = 0; j < 2; j++) e[j] = _mesh->_edges[i][j];
		if(!((this->*getInt)(e, line, dist))) continue;
		for(j = 0; j < 2; j++) if(line[j] >= 0) {
			k = _newMesh->_vertices.size();
			_newMesh->_vertices.push_back(_mesh->_worldVertices[e[j]]
			  + (_mesh->_worldVertices[e[(j+1)%2]] - _mesh->_worldVertices[e[j]]) * dist[j]);
			edgeInt[e[j]][e[(j+1)%2]] = std::pair<unsigned short, unsigned short>(line[j], k);
		}
		for(j = 0; j < 2; j++) if(line[j] < 0) edgeInt[e[j]][e[(j+1)%2]] = edgeInt[e[(j+1)%2]][e[j]];
	}
}

bool ToolMode::getEdgeDrillInt(unsigned short *e, short *lineInd, float *distance) {
	if(keep[e[0]] < 0 && keep[e[1]] < 0) return false;

	Tool *tool = getTool();
	short _segments = tool->iparam[0], i, j, k;
	float _radius = tool->fparam[0], f1, f2, f3, d, s, t, angle, dAngle = 2*M_PI / _segments,
	  planeDistance = _radius * cos(dAngle/2);
	Vector3 v[3], v1, v2;

	for(i = 0; i < 2; i++) {
		v[i] = toolVertices[e[i]];
		v[i].z = 0;
		lineInd[i] = -1;
	}
	v[2].set(v[1]-v[0]);
	//validate the edge is really there
	if(v[2] == Vector3::zero()) return false;
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
		lineInd[j] = (short)(angle / dAngle);
	}
	//if keeping both endpoints and they hit the same angle segment, the edge must not really touch the drill plane
	if(keep[e[0]] >= 0 && keep[e[1]] >= 0 && lineInd[0] == lineInd[1]) return false;
	//for each edge direction, find the point where it intersects the drill plane in the angle range found above
	for(j = 0; j < 2; j++) {
		if(keep[e[j]] < 0) continue;
		angle = lineInd[j] * dAngle + dAngle/2;
		v2.set(cos(angle), sin(angle), 0);
		//(v0 + a*v2) * r = planeDistance => a = (planeDistance - v0*r) / v2*r
		distance[j] = (planeDistance - v[0].dot(v2)) / v[2].dot(v2);
		if(j == 1) distance[j] = 1 - distance[j];
	}
	return true;
}

bool ToolMode::getHullSliceInt(unsigned short *e, short *planeInd, float *dist) {
	if(keep[e[0]] >= 0 && keep[e[1]] >= 0) return false;
	
	Tool *tool = getTool();	
	short _segments = tool->iparam[0], i, j, k, m, n;
	float err[2], angle, dAngle = 2*M_PI / _segments, minAngle = _hullSlice * dAngle, maxAngle = (_hullSlice+1) * dAngle,
	  _radius = tool->fparam[0], planeDistance = _radius * cos(dAngle/2), f1, f2, f3, a, distance, error;
	Vector3 v[3], v1, v2, normal, tangent;
	for(m = 0; m < 2; m++) {
		v[m] = toolVertices[e[m]];
		v[m].z = 0;
		err[m] = 1000;
		dist[m] = 1000;
		planeInd[m] = -1;
	}
	v[2] = v[1] - v[0];
	//check radial plane 1, drill plane, radial plane 2
	bool radial, better;
	short keeper;
	for(m = 0; m < 3; m++) {
		radial = m%2 == 0;
		keeper = -1;
		if((keep[e[0]] < 0) != (keep[e[1]] < 0)) keeper = keep[e[0]] < 0 ? 1 : 0;
		switch(m) {
			case 0: angle = minAngle; break;
			case 1: angle = minAngle + dAngle/2; break;
			case 2: angle = maxAngle; break;
		}
		v1.set(cos(angle), sin(angle), 0);
		v2.set(-sin(angle), cos(angle), 0);
		normal = radial ? v2 : v1;
		tangent = radial ? v1 : v2;
		distance = radial ? 0 : planeDistance;
		// (v0 + a*v2) * n = distance => a = (distance - v0*n) / v2*n
		f1 = v[2].dot(normal);
		f2 = v[0].dot(normal);
		if(f1 == 0) { //edge is parallel to plane
			if(keeper >= 0 && fabs(f2) < 0.001f) a = keeper * 1.0f;
			else continue;
		} else a = (distance - f2) / f1;
		if((a < 0 || a > 1) && keeper < 0) continue;
		v1 = v[0] + a * v[2];
		if(radial) error = _radius - v1.dot(tangent);
		else error = fabs(v1.dot(tangent)) - _radius*sin(dAngle/2);
		if(error <= 0) error = 0;
		else if(keeper < 0) continue;
		for(n = 0; n < 2; n++) {
			if(keeper == 1-n) continue;
			f1 = n == 0 ? a : 1 - a;
			if(keeper == n && (err[n] > 0 || dist[n] < 0 || dist[n] > 1)) {
				float distErr = dist[n] < 0 || dist[n] > 1 ? fmin(fabs(dist[n] - 1), fabs(-dist[n])) : 0,
				  newErr = a < 0 || a > 1 ? fmin(fabs(a - 1), fabs(-a)) : 0;
				better = (distErr > err[n] && newErr < distErr) || (err[n] > distErr && error < err[n]);
			} else better = f1 >= 0 && f1 <= 1 && f1 < dist[n];
			if(better) {
				dist[n] = f1;
				planeInd[n] = m;
				err[n] = error;
			}
		}
	}
	if(keeper < 0) return planeInd[0] >= 0 && planeInd[1] >= 0 && planeInd[0] != planeInd[1];
	else return planeInd[keeper] >= 0;
}

void ToolMode::addToolFaces() {
	Tool *tool = getTool();
	short _segments = tool->iparam[0], i, j, k, m, n, p, q, lineNum, numInt, e[2];
	Vector3 v1, v2;
	float edgeLen;
	//for any tool line that has intersections, order them by distance along the line
	std::vector<std::vector<std::pair<unsigned short, float> > > lineInt(_segments);
	std::map<unsigned short, bool> enterInt;
	std::vector<std::pair<unsigned short, float> >::iterator vit;
	std::map<unsigned short, std::map<unsigned short, unsigned short> >::iterator it;
	std::map<unsigned short, unsigned short>::iterator it1;
	Vector3 axis = _axis.getDirection();
	for(it = toolInt.begin(); it != toolInt.end(); it++) {
		lineNum = it->first;
		v1.set(lines[lineNum].getOrigin());
		//sort the drill line intersections by distance along the ray
		for(it1 = toolInt[lineNum].begin(); it1 != toolInt[lineNum].end(); it1++) {
			n = it1->second;
			v2.set(_newMesh->_vertices[n] - v1);
			edgeLen = v2.length();
			for(vit = lineInt[lineNum].begin();	vit != lineInt[lineNum].end() && vit->second < edgeLen; vit++);
			lineInt[lineNum].insert(vit, std::pair<unsigned short, float>(n, edgeLen));
			enterInt[n] = _mesh->_worldNormals[it1->first].dot(axis) < 0;
		}
		//now they are ordered, note the edges they form
		numInt = lineInt[lineNum].size();
		for(i = 0; i < numInt; i++) {
			for(j = 0; j < 2; j++) e[j] = lineInt[lineNum][i+j].first;
			if(i < numInt-1 && enterInt[e[0]] && !enterInt[e[1]]) {
				addToolEdge(e[0], e[1], lineNum);
				addToolEdge(e[1], e[0], (lineNum-1+_segments)%_segments);
			}
		}
	}
	
	//copy edge map for debugging
	std::map<unsigned short, std::map<unsigned short, unsigned short> > oldEdges = segmentEdges;

	//for each segment of the drill bit, use its known set of points and edges to determine all its faces
	std::map<unsigned short, unsigned short>::iterator eit;
	std::vector<unsigned short> newFace;
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
				GP_ERROR("Couldn't continue tool face from point %d", p);
			}
			q = segmentEdges[i][p];
			segmentEdges[i].erase(p);
			if(!newFace.empty() && q == newFace[0]) { //when cycle complete, triangulate and add the new face
				_newMesh->addFace(newFace);
				newFace.clear();
			} else newFace.push_back(q);
		}
	}
}

void ToolMode::addToolEdge(unsigned short v1, unsigned short v2, unsigned short lineNum) {
	_newMesh->addEdge(v1, v2);
	segmentEdges[lineNum][v1] = v2;
}

void ToolMode::showFace(Meshy *mesh, std::vector<unsigned short> &face, bool world) {
	_node->setWireframe(true);
	app->_drawDebug = false;
	app->showFace(mesh, face, world);
}

