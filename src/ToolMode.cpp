#include "T4TApp.h"

ToolMode *ToolMode::_instance;
//triangulation of faces with holes
GLUtesselator *ToolMode::_tess; //for triangulating polygons
GLenum ToolMode::_tessType;
//vertices is what tesselation returns to us, buffer is the list of vertices we are using for tesselation
std::vector<unsigned short> ToolMode::_tessVertices, ToolMode::_tessBuffer;

ToolMode::ToolMode() 
  : Mode::Mode("tool") {

	_instance = this;

	_tool = MyNode::create("tool_tool");
	_newNode = MyNode::create("newNode_tool");
	
	_tess = gluNewTess();
	gluTessCallback(_tess, GLU_TESS_BEGIN, (GLvoid (*) ()) &tessBegin);
	gluTessCallback(_tess, GLU_TESS_END, (GLvoid (*) ()) &tessEnd);
	gluTessCallback(_tess, GLU_TESS_VERTEX, (GLvoid (*) ()) &tessVertex);
	gluTessCallback(_tess, GLU_TESS_COMBINE, (GLvoid (*) ()) &tessCombine);
	gluTessCallback(_tess, GLU_TESS_ERROR, (GLvoid (*) ()) &tessError);
	
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
			os.str("");
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
	_bitMenu->setVisible(false);
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
	return true;
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
	} else if(_moveMenu->getControl(id) == control && _selectedNode != NULL) {
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
	_hullSlice = -1;

	//reset the data on the altered node
	_newNode->_vertices.clear();
	_newNode->_faces.clear();
	_newNode->_triangles.clear();
	_newNode->_edges.clear();
	_newNode->_edgeInd.clear();
	for(std::vector<MyNode::ConvexHull*>::iterator it= _newNode->_hulls.begin(); it != _newNode->_hulls.end(); it++) delete *it;
	_newNode->_hulls.clear();
	_newNode->_objType = "mesh";

	short i, j, k;	
	usageCount++;
	
	edgeInt.clear();
	toolInt.clear();
	segmentEdges.clear();
	
	_toolWorld = _tool->getWorldMatrix();
	Vector3 center = Vector3::zero();
	axis = Vector3::unitZ();
	right = -Vector3::unitX();
	up = Vector3::unitY();
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
	
	app->setAction("tool", _node);

	//transform the new model and convex hull vertices back to model space and store in the original node
	Matrix worldModel;
	_node->getWorldMatrix().invert(&worldModel);
	_node->copyMesh(_newNode);
	for(i = 0; i < _node->nv(); i++) {
		worldModel.transformPoint(&_node->_vertices[i]);
	}
	for(i = 0; i < _node->_hulls.size(); i++) {
		MyNode::ConvexHull *hull = _node->_hulls[i];
		for(j = 0; j < hull->nv(); j++) {
			worldModel.transformPoint(&hull->_vertices[j]);
		}
		hull->setNormals();
	}
	//put all the changes into the simulation
	_node->updateModel();

	app->commitAction();
}

bool ToolMode::checkEdgeInt(unsigned short v1, unsigned short v2) {
	if(edgeInt.find(v1) == edgeInt.end() || edgeInt[v1].find(v2) == edgeInt[v1].end()) return false;
	_tempInt = edgeInt[v1][v2];
	return true;
}

void ToolMode::addToolEdge(unsigned short v1, unsigned short v2, unsigned short lineNum) {
	_newMesh->addEdge(v1, v2);
	segmentEdges[lineNum][v1] = v2;
}

short ToolMode::addToolInt(Vector3 &v, unsigned short line, unsigned short face, short segment) {
	short n = _newMesh->nv();
	_newMesh->addVertex(v);
	os.str("");
	os << "line " << line << " => face " << face;
	_newMesh->setVInfo(n, os.str().c_str());
	toolInt[line][face] = n;
	if(segment >= 0) {
		_newFace.push_back(n);
		addToolEdge(n, _lastInter, segment);
		_lastInter = n;
	}
	return n;
}

void ToolMode::showFace(Meshy *mesh, std::vector<unsigned short> &face, bool world) {
	_node->setWireframe(true);
	app->_drawDebug = false;
	app->showFace(mesh, face, world);
	mesh->printFace(face);
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
			_newMesh->_vertices.push_back(toolVertices[e[j]]
			  + (toolVertices[e[(j+1)%2]] - toolVertices[e[j]]) * dist[j]);
			os.str("");
			os << "edge " << e[j] << "-" << e[(j+1)%2] << " => line " << line[j];
			_newMesh->setVInfo(k, os.str().c_str());
			edgeInt[e[j]][e[(j+1)%2]] = std::pair<unsigned short, unsigned short>(line[j], k);
		}
		for(j = 0; j < 2; j++) if(line[j] < 0) edgeInt[e[j]][e[(j+1)%2]] = edgeInt[e[(j+1)%2]][e[j]];
	}
}

float ToolMode::getDrillEdgeIntDistance(unsigned short line1, Vector3 &v1, unsigned short line2, Vector3 &v2) {
	if(parallel) return v2.z - v1.z;
	float ang = dir * (atan2(v2.y, v2.x) - atan2(v1.y, v1.x));
	if(ang < 0) ang += 2*M_PI;
	return ang;
}

void ToolMode::getNewFaces() {
	short nf = _mesh->nf(), i, j, k, m, n, p, q, r, refLine;
	float faceDistance;
	Plane facePlane;
	Vector3 faceNormal, refVec;
	bool found;
	std::set<unsigned short> keeping;
	std::set<unsigned short>::iterator kit;
	std::pair<unsigned short, unsigned short> inter;
	std::list<std::pair<float, unsigned short> > ints[2];
	std::list<std::pair<float, unsigned short> >::const_iterator it1, it2;
	for(i = 0; i < nf; i++) {
		n = _mesh->_faces[i].size();

		ccw = toolNormals[i].z < 0;
		dir = ccw ? 1 : -1;
		offset = dir == 1 ? 1 : 0;
		facePlane.set(_mesh->_worldNormals[i],
		  -_mesh->_worldVertices[_mesh->_faces[i][0]].dot(_mesh->_worldNormals[i]));
		facePlane.transform(_toolWorld);
		faceNormal = facePlane.getNormal();
		faceDistance = facePlane.getDistance();
		parallel = faceNormal.z == 0;

		//determine which vertices of this face we are keeping
		keeping.clear();
		for(j = 0; j < n; j++) if(keep[_mesh->_faces[i][j]]) keeping.insert(j);
		if(keeping.empty()) continue; //not keeping any part of this face

		//pair up the edge intersections in this face so that we can jump from one to the next to complete the new face

		//start on an edge with an intersection that enters the tool ("forward")
		for(j = 0; j < n; j++) {
			for(k = 0; k < 2; k++) e[k] = _mesh->_faces[i][(j+k)%n];
			for(k = 0; k < 2; k++) {
				if(checkEdgeInt(e[k], e[(k+1)%2]) && enteringTool(e[k], e[(k+1)%2])) {
					found = true;
					refLine = _tempInt.first;
					refVec = _newMesh->_vertices[_tempInt.second];
					break;
				}
			}
			if(found) break;
		}
		//store all forward and backward intersections separately, sorted by "distance" from the intersecton above
		for(j = 0; j < 2; j++) ints[j].clear();
		for(j = 0; j < n; j++) {
			for(k = 0; k < 2; k++) e[k] = _mesh->_faces[i][(j+k)%n];
			for(k = 0; k < 2; k++) {
				if(edgeInt.find(e[k]) != edgeInt.end() && edgeInt[e[k]].find(e[(k+1)%2]) != edgeInt[e[k]].end()) {
					if(k == 1 && edgeInt[e[0]][e[1]].second)
					inter = edgeInt[e[k]][e[(k+1)%2]];
					v1 = _newMesh->_vertices[inter.second];
					//store the "distance" (tool-specific metric function) from the first forward intersection
					if(!found) {
						distance = 0;
						found = true;
						refLine = inter.first;
						refVec = v1;
					} else distance = (this->*edgeIntDistance)(inter.first, v1, refLine, refVec);
					ints[k].insert(std::pair<float, unsigned short>(distance, j));
				}
			}
		}
		//sort forward and backward separately by distance
		for(j = 0; j < 2; j++) ints[j].sort();
		//first forward intersection bridges to first backward, etc.
		edgeLoop.clear();
		for(it1 = ints[0].begin(), it2 = ints[1].begin(); it1 != ints[0].end(); it1++, it2++)
			edgeLoop[it1->second] = it2->second;

		//if there are edge intersections, build the set of new faces that this one is split into
		if(!edgeLoop.empty()) while(!edgeLoop.empty()) {
			//pick a for
			kit = keeping.begin();
			j = *kit;
			keeping.erase(kit);
			_newFace.clear();
			p = _mesh->_faces[i][j];
			mode = 0;
			numInts = 0;
			do {
				switch(mode) {
					case 0: //walking the original face
						_newFace.push_back(keep[p]);
						keeping.erase(j);
						k = (j+1)%n;
						q = _mesh->_faces[i][k];
						if(edgeInt.find(p) != edgeInt.end() && edgeInt[p].find(q) != edgeInt[p].end()) {
							mode = 1;
							startLine = (edgeInt[p][q].first + offset) % _segments;
							p = edgeInt[p][q].second;
						} else {
							p = q;
							j = k;
						}
						break;
					case 1: //at an edge intersection - bridge to the next edge intersection
						//add the current intersection
						_newFace.push_back(p);
						_lastInter = p;
						//find the next intersection
						j = edgeLoop[j];
						k = (j+1)%n;
						endLine = (edgeInt[k][j].first + offset) % _segments;
						q = edgeInt[k][j].second;
						//there's no real answer for a parallel face, so try to get it somewhere on the face
						if(parallel) distance = (_newMesh->_vertices[p].z + _newMesh->_vertices[q].z) / 2;
						//make the bridge
						for(lineNum = startLine; lineNum != endLine; lineNum = (lineNum + dir + _segments) % segments) {
							v1.set(_radius * cos(lineNum*dAngle), _radius * sin(lineNum*dAngle), 0);
							if(parallel) v1.z = distance;
							else v1.z = -(v1.dot(faceNormal) + faceDistance) / faceNormal.z;
							addToolInt(v1, lineNum, i, (lineNum-offset+_segments)%_segments);
						}
						//add the next intersection
						_newFace.push_back(q);
						addToolEdge(q, _lastInter, (endLine-offset+_segments)%_segments);
						j = k;
						mode = 0;
						break;
				}
			} while((mode != 0 || keep[p] != _newFace[0]) && numInts <= _segments);
			if(numInts > _segments) {
				GP_ERROR("Couldn't exit drill on face %d", i);
			}
			_newNode->addFace(_newFace);
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
					v1.set(_radius * cos(j*dAngle), _radius * sin(j*dAngle), 0);
					distance = -(v1.dot(faceNormal) + faceDistance) / faceNormal.z;
					addToolInt(v1, j, i);
				}
				//add edges on the drill segments making sure to get the right orientation
				for(j = 0; j < _segments; j++)
					addToolEdge(toolInt[j][i], toolInt[(j-dir+_segments)%_segments][i], (j-offset+_segments)%_segments);
				//order the face vertices by angle wrt drill center
				angles.clear();
				for(j = 0; j < n; j++) {
					k = _mesh->_faces[i][j];
					angle = atan2(toolVertices[k].y, toolVertices[k].x);
					if(angle < 0) angle += 2*M_PI;
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
					_newFace.clear();
					m = (j+1)%2;
					for(k = e[j]; k != (e[m]+1)%n; k = (k+1)%n)
						_newFace.push_back(keep[_mesh->_faces[i][k]]);
					for(k = m * _segments/2; k != (j*_segments/2+dir+_segments) % _segments; k = (k+dir+_segments)%_segments)
						_newFace.push_back(toolInt[k][i]);
					_newNode->addFace(_newFace);
				}
			} else { //the drill does not touch this face => leave it as is
				_newFace.clear();
				for(j = 0; j < n; j++) _newFace.push_back(keep[_mesh->_faces[i][j]]);
				_newNode->addFace(_newFace);
			}
		}		
	}
}

void ToolMode::addToolFaces() {
	Tool *tool = getTool();
	short _segments, i, j, k, m, n, p, q, lineNum, toolLineNum, numInt, e[2];
	float distance, dAngle, angle;
	_segments = _subMode == 1 ? _hullSlice >= 0 ? 3 : tool->iparam[0] : 1;
	if(_subMode == 1) dAngle = 2*M_PI / tool->iparam[0];
	Vector3 v1, v2;
	//for any tool line that has intersections, order them by distance along the line
	std::vector<std::vector<std::pair<unsigned short, float> > > lineInt(_segments);
	std::map<unsigned short, bool> enterInt;
	std::vector<std::pair<unsigned short, float> >::iterator vit;
	std::map<unsigned short, std::map<unsigned short, unsigned short> >::iterator it;
	std::map<unsigned short, unsigned short>::iterator it1;
	for(it = toolInt.begin(); it != toolInt.end(); it++) {
		lineNum = it->first;
		//sort the drill line intersections by distance along the ray
		for(it1 = toolInt[lineNum].begin(); it1 != toolInt[lineNum].end(); it1++) {
			n = it1->second;
			distance = _newMesh->_vertices[n].z;
			lineInt[lineNum].push_back(std::pair<float, unsigned short>(distance, n));
			enterInt[n] = toolNormals[it1->first].z < 0;
		}
		lineInt[lineNum].sort();
		//now they are ordered, note the edges they form
		numInt = lineInt[lineNum].size();
		for(i = 0; i < numInt-1; i++) {
			for(j = 0; j < 2; j++) e[j] = lineInt[lineNum][i+j].first;
			//due to round off error, can't afford to be too zealous - so just treat each adjacent pair as an edge
			if(i % 2 == 0) { //i < numInt-1 && enterInt[e[0]] && !enterInt[e[1]]) {
				addToolEdge(e[0], e[1], lineNum);
				addToolEdge(e[1], e[0], (lineNum-1+_segments)%_segments);
			}
		}
	}
	
	//copy edge map for debugging
	std::map<unsigned short, std::map<unsigned short, unsigned short> > oldEdges = segmentEdges;

	//for each segment of the tool, use its known set of points and edges to determine all its faces
	std::map<unsigned short, unsigned short>::iterator eit;
	std::map<unsigned short, Vector2> planeVertices;
	std::vector<std::vector<unsigned short> > facesCCW, facesCW;
	std::vector<std::vector<unsigned short> >::iterator fit, fit1;
	std::map<unsigned short, std::vector<unsigned short> > holes;
	std::vector<unsigned short> cycle;
	Vector3 normal, tangent, vec;
	int numVertices;
	bool clockwise;
	_tessBuffer.clear();
	for(i = 0; i < _segments; i++) {
		if(_subMode == 1 && _hullSlice >= 0) {
			angle = (_hullSlice + i/2.0f) * dAngle;
			if(i%2 == 0) normal.set((i-1) * sin(angle), (1-i) * cos(angle), 0);
			else normal.set(cos(angle), sin(angle), 0);
			_toolNorm.transformVector(&normal);
		} else normal = planes[i].getNormal();
		Vector3::cross(axis, normal, &tangent);
		tangent.normalize();
		holes.clear();
		facesCCW.clear();
		facesCW.clear();
		numVertices = 0;
		//we will need the 2D plane coordinates of all vertices that are part of edges on this plane
		planeVertices.clear();
		for(eit = segmentEdges[i].begin(); eit != segmentEdges[i].end(); eit++) {
			for(j = 0; j < 2; j++) {
				m = j == 0 ? eit->first : eit->second;
				if(planeVertices.find(m) != planeVertices.end()) continue;
				vec = _newMesh->_vertices[m];
				planeVertices[m].set(vec.dot(axis), vec.dot(tangent));
			}
		}
		//just keep building cycles until all edges are used
		_newFace.clear();
		while(!segmentEdges[i].empty()) {
			if(_newFace.empty()) {
				eit = segmentEdges[i].begin();
				p = eit->first;
				_newFace.push_back(p);
			} else {
				p = _newFace.back();
			}
			if(segmentEdges[i].find(p) == segmentEdges[i].end()) {
				GP_WARN("Couldn't continue tool face from point %d", p);
				segmentEdges[i].erase(p);
				_newFace.clear();
				continue;
			}
			q = segmentEdges[i][p];
			segmentEdges[i].erase(p);
			if(!_newFace.empty() && q == _newFace[0]) { //when cycle complete, store the cycle with its orientation
				clockwise = _newMesh->getNormal(_newFace, true).dot(normal) > 0;
				if(clockwise) facesCW.push_back(_newFace);
				else facesCCW.push_back(_newFace);
				numVertices += _newFace.size();
				_newFace.clear();
			} else _newFace.push_back(q);
		}
		//CCW cycles are the outer boundaries of the faces, CW cycles are holes inside them
		//start by finding which CCW each CW best fits inside
		Vector2 v1, v2, v3, norm;
		for(p = 0; p < facesCW.size(); p++) {
			cycle = facesCW[p];
			m = cycle.size();
			float min = 2;
			short best = -1;
			for(q = 0; q < facesCCW.size(); q++) {
				_newFace = facesCCW[q];
				n = _newFace.size();
				float avg = 0;
				for(j = 0; j < m; j++) {
					norm.set(0, 0);
					for(k = 0; k < 2; k++) {
						v1 = planeVertices[cycle[(j+k)%m]] - planeVertices[cycle[(j+k-1+m)%m]];
						v1.set(-v1.y, v1.x);
						norm += v1.normalize();
					}
					norm.normalize();
					short inter = 0;
					for(k = 0; k < n; k++) {
						v2 = planeVertices[_newFace[k]];
						v3 = planeVertices[_newFace[(k+1)%n]] - v2;
						float det = norm.x * v3.y - norm.y * v3.x;
						if(det == 0) continue;
						v2 -= planeVertices[cycle[j]];
						float dist = (-norm.y * v2.x + norm.x * v2.y) / det;
						if(dist > 0 && dist < 1) inter++;
					}
					avg += inter % 2;
				}
				avg /= m;
				if(avg < min) {
					min = avg;
					best = q;
				}
			}
			holes[best].push_back(p);
		}
		//use gluTesselator to triangulate the polygons
		GLdouble coords[3];
		_tessBuffer.resize(numVertices);
		for(j = 0; j < facesCCW.size(); j++) {
			if(holes[j].empty()) {
				_newMesh->addFace(facesCCW[j]);
			} else {
				cout << "Tesselating" << endl;
				_tessVertices.clear();
				short bufferInd = 0;
				gluTessBeginPolygon(_tess, NULL);
				for(k = 0; k < 1 + holes[j].size(); k++) {
					cycle = k == 0 ? facesCCW[j] : facesCW[holes[j][k-1]];
					if(k > 0) cout << "  ";
					for(m = 0; m < cycle.size(); m++) cout << cycle[m] << " ";
					cout << endl;
					//_newMesh->printFace(cycle);
					gluTessBeginContour(_tess);
					for(m = 0; m < cycle.size(); m++) {
						vec = _newMesh->_vertices[cycle[m]];
						for(n = 0; n < 3; n++) coords[n] = MyNode::gv(vec, n);
						_tessBuffer[bufferInd] = cycle[m];
						unsigned short *data = &_tessBuffer[bufferInd++];
						//cout << "adding vertex " << cycle[m] << " => " << data << endl;
						gluTessVertex(_tess, coords, data);
					}
					gluTessEndContour(_tess);
				}
				gluTessEndPolygon(_tess);
			}
		}
	}
}

void ToolMode::tessBegin(GLenum type) {
	_tessType = type;
}

void ToolMode::tessEnd() {
	short i, j, n = _tessVertices.size();
	std::vector<unsigned short> triangle(3), face;
	std::vector<std::vector<unsigned short> > triangles;
	switch(_tessType) {
		case GL_TRIANGLE_FAN:
			face = _tessVertices;
			for(i = 0; i < n-2; i++) {
				triangle[0] = 0;
				for(j = 1; j <= 2; j++) triangle[j] = i + j;
				triangles.push_back(triangle);
			}
			_instance->_newMesh->addFace(face, triangles);
			break;
		case GL_TRIANGLE_STRIP:
			for(i = 0; i < n-2; i++) {
				for(j = 0; j < 3; j++) triangle[i] = _tessVertices[i + (i%2 == 0 ? j : 2-j)];
				_instance->_newMesh->addFace(triangle);
			}
			break;
		case GL_TRIANGLES:
			for(i = 0; i < n; i++) {
				for(j = 0; j < 3; j++) triangle[i] = _tessVertices[i*3 + j];
				_instance->_newMesh->addFace(triangle);
			}
			break;
		case GL_LINE_LOOP:
			break;
	}
	_tessVertices.clear();
}

void ToolMode::tessVertex(unsigned short *vertex) {
	_tessVertices.push_back(*vertex);
	//cout << "got vertex " << *vertex << " from " << vertex << endl;
}

void ToolMode::tessCombine(GLdouble coords[3], unsigned short *vertex[4], GLfloat weight[4], unsigned short *dataOut) {
	short n = _instance->_newMesh->_vertices.size();
	_instance->_newMesh->addVertex((float)coords[0], (float)coords[1], (float)coords[2]);
	*dataOut = n;
	cout << "tess combining " << *vertex[0] << "," << *vertex[1] << "," << *vertex[2] << "," << *vertex[3] << " => " << n << endl;
}

void ToolMode::tessError(GLenum errno) {
	cout << "Tesselation error " << errno << endl;
}

/************ SAWING ************/

bool ToolMode::sawNode() {
	short i, j, k, m, n, p, q, r, nh = _node->_hulls.size();
	float f1, f2, f3, f4;
	
	planes.resize(1);
	planes[0].set(Vector3::unitX(), 0);
	planes[0].transform(_tool->getWorldMatrix());
	
	for(r = 0; r < 1 + nh; r++) {
		if(r > 0) {
			setMesh(_node->_hulls[r-1]);
			MyNode::ConvexHull *hull = new MyNode::ConvexHull(_newNode);
			_newNode->_hulls.push_back(hull);
			_newMesh = hull;
		}
		short nv = _mesh->nv(), nf = _mesh->nf();

		for(i = 0; i < nv; i++) {
			if(toolVertices[i].x < 0) {
				keep[i] = _newMesh->_vertices.size();
				_newMesh->_vertices.push_back(toolVertices[i]);
			} else keep[i] = -1;
		}

		edgeInt.clear();	
		getEdgeInt(&ToolMode::getEdgeSawInt);
	
		std::vector<unsigned short> face, keeping;
		std::list<std::pair<float, unsigned short> > intList;
		std::list<std::pair<float, unsigned short> >::iterator lit;
		std::map<unsigned short, unsigned short> intPair;
		for(i = 0; i < nf; i++) {
			face = _mesh->_faces[i];
			n = face.size();
			//sort the edge intersections by distance
			Vector3 first, dir = Vector3::zero(), v1;
			short intCount = 0;
			keeping.clear();
			intList.clear();
			for(j = 0; j < n; j++) {
				p = face[j];
				q = face[(j+1)%n];
				if(keep[p] >= 0) keeping.push_back(j);
				if(checkEdgeInt(p, q)) {
					intCount++;
					v1 = _newMesh->_vertices[_tempInt.second];
					if(intCount == 1) first = v1;
					else if(intCount == 2) {
						dir = v1 - first;
						dir.normalize();
					}
					float distance = (v1 - first).dot(dir);
					intList.push_back(std::pair<float, unsigned short>(distance, j));
				}
			}
			intList.sort();
			intPair.clear();
			for(lit = intList.begin(); lit != intList.end(); lit++) {
				p = (lit++)->second;
				q = lit->second;
				intPair[p] = q;
				intPair[q] = p;
			}
		
			while(!keeping.empty()) {
				_newFace.clear();
				j = keeping.back();
				do {
					p = face[j];
					q = face[(j+1)%n];
					if(keep[p] >= 0) {
						keeping.erase(std::find(keeping.begin(), keeping.end(), j));
						_newFace.push_back(keep[p]);
					}
					if(checkEdgeInt(p, q)) {
						_newFace.push_back(_tempInt.second);
						j = intPair[j];
						m = edgeInt[face[j]][face[(j+1)%n]].second;
						_newFace.push_back(m);
						addToolEdge(m, _tempInt.second, 0);
					}
					j = (j+1)%n;
				} while(keep[face[j]] != _newFace[0]);
				if(_newFace.size() > 0) _newMesh->addFace(_newFace);
			}
		}
		addToolFaces();
	}
	return true;
}

bool ToolMode::getEdgeSawInt(unsigned short *e, short *lineInd, float *dist) {
	if((keep[e[0]] < 0) == (keep[e[1]] < 0)) return false;
	short keeper = keep[e[0]] < 0 ? 1 : 0, other = 1-keeper;
	float delta = toolVertices[e[keeper]].x - toolVertices[e[other]].x;
	dist[keeper] = delta == 0 ? 0 : toolVertices[e[keeper]].x / delta;
	lineInd[keeper] = 0;
	lineInd[other] = -1;
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
	Matrix nodeWorld = _selectedNode->getWorldMatrix();
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
			_newMesh->_vertices.push_back(toolVertices[i]);
		} else keep[i] = -1;
	}

	//find all intersections between edges of the model and the planes of the drill bit
	getEdgeInt(&ToolMode::getEdgeDrillInt);
	
	//loop around each original face, building modified faces according to the drill intersections
	bool ccw, drillInside, found;
	short dir, offset, mode, numInts, best, lineNum, startLine, endLine;
	unsigned short e[2];
	float denom, delta, epsilon = 0.001f, minDist, distance, faceDistance;
	Plane facePlane;
	Vector3 faceNormal;
	std::vector<unsigned short> drillPoint;
	std::set<unsigned short> keeping;
	std::set<unsigned short>::iterator kit;
	std::list<std::pair<float, unsigned short> > ints[2];
	std::map<unsigned short, unsigned short> edgeLoop;
	for(i = 0; i < _mesh->_faces.size(); i++) {
		n = _mesh->_faces[i].size();
		faceInts.clear();

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
				_newFace.clear();
				lastPlane = -1;
				_lastInter = -1;
				//determine the plane for this face in world space
				normal = hull->_worldNormals[k];
				facePlane.set(normal, -normal.dot(hull->_worldVertices[hull->_faces[k][0]]));
				facePlane.transform(_toolWorld);
				faceNormal = facePlane.getNormal();
				faceDistance = facePlane.getDistance();
				
				//start on a vertex we are keeping, if any
				for(start = 0; start < faceSize && keep[hull->_faces[k][start]] < 0; start++);
				//if not, look for an edge intersection exiting the segment
				if(start == faceSize) for(start = 0; start < faceSize; start++) {
					for(m = 0; m < 2; m++) e[m] = hull->_faces[k][(start+m)%faceSize];
					if(edgeInt.find(e[1]) != edgeInt.end() && edgeInt[e[1]].find(e[0]) != edgeInt[e[1]].end()) {
						_newFace.push_back(edgeInt[e[1]][e[0]].second);
						lastPlane = edgeInt[e[1]][e[0]].first;
						_lastInter = _newFace[0];
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
					if(keep[e[0]] >= 0) _newFace.push_back(keep[e[0]]);
					if(edgeInt.find(e[0]) != edgeInt.end() && edgeInt[e[0]].find(e[1]) != edgeInt[e[0]].end()) {
						plane = edgeInt[e[0]][e[1]].first;
						p = edgeInt[e[0]][e[1]].second;
						//if we are re-entering the segment on a different plane than we exited,
						//the drill lines in between those planes must intersect the face
						if(keep[e[0]] < 0 && lastPlane >= 0 && lastPlane != plane) {
							dir = plane > lastPlane ? 1 : -1;
							offset = dir == 1 ? 0 : -1;
							if(parallel) distance = (_newMesh->_vertices[lastInter].z + _newMesh->_vertices[p].z) / 2;
							short startInter = lastInter, endInter = edgeInt[e[0]][e[1]].second;
							for(n = lastPlane + offset; n != plane + offset; n += dir) {
								Ray line = lines[(j+n) % _segments];
								distance = line.intersects(facePlane);
								if(distance == 0) { //face plane is parallel to drill axis
									
								}
								p = newHull->nv();
								v1 = line.getOrigin() + line.getDirection() * distance;
								//cout << "hull " << j << " adding " << app->pv(v1) << endl;
								newHull->addVertex(v1);
								os.str("");
								os << "line " << n+1 << " => face " << k;
								newHull->setVInfo(p, os.str().c_str());
								_newFace.push_back(p);
								toolInt[n+1][k] = p;
								addToolEdge(p, lastInter, n - offset);
								lastInter = p;
							}
						}
						_newFace.push_back(edgeInt[e[0]][e[1]].second);
						if(keep[e[0]] < 0) addToolEdge(edgeInt[e[0]][e[1]].second, lastInter, plane);
						if(edgeInt[e[1]][e[0]].second != edgeInt[e[0]][e[1]].second
						  && edgeInt[e[1]][e[0]].second != _newFace[0]) {
							_newFace.push_back(edgeInt[e[1]][e[0]].second);
							lastPlane = edgeInt[e[1]][e[0]].first;
							lastInter = edgeInt[e[1]][e[0]].second;
						} else if(keep[e[1]] < 0) {
							lastPlane = plane;
							lastInter = edgeInt[e[0]][e[1]].second;
						}
					}
				}
				if(!_newFace.empty()) newHull->addFace(_newFace);
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

