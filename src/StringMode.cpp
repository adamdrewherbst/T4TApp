#include "T4TApp.h"

StringMode::StringMode() : Mode::Mode("string") {
	_path = MyNode::create("stringPath");
	_path->_type = "grid";
	_path->_color.set(1.0f, 0.0f, 0.0f);
	_path->_chain = true;
	_path->_lineWidth = 10.0f;
}

void StringMode::setActive(bool active) {
	Mode::setActive(active);
	clearPath();
}

bool StringMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			if(_touchNode == NULL) break;
			addNode(_touchNode, _touchPoint);
			break;
		} case Touch::TOUCH_MOVE: {
			break;
		} case Touch::TOUCH_RELEASE: {
			break;
		}
	}
	return true;
}

void StringMode::controlEvent(Control *control, Control::Listener::EventType evt) {
	Mode::controlEvent(control, evt);
	const char *id = control->getId();
	if(strcmp(id, "cancel") == 0) {
		clearPath();
	} else if(strcmp(id, "makeString") == 0) {
		makeString();
	}
}

void StringMode::addNode(MyNode *node, Vector3 point) {
	NodeData *data = new NodeData(node, this);
	data->_point = point;
	_nodes.push_back(data);
	short n = _nodes.size(), i;
	if(n == 1) _origin = node->getTranslationWorld();
	if(n >= 3) getPlane();
	if(n >= 2) getPath();
}

//find the best-fit plane to my set of nodes
bool StringMode::getPlane() {
	short n = _nodes.size(), i, j;
	std::vector<Vector3> p(n);
	//center is average of all node centers
	Vector3 center, v1, v2;
	for(i = 0; i < n; i++) {
		p[i] = _nodes[i]->_node->getTranslationWorld();
		center += p[i];
	}
	center *= 1.0f / n;
	//normal is eigenvector for smallest eigenvalue of cross product matrix
	Matrix m = Matrix::zero();
	float xx, xy, xz, yy, yz, zz;
	for(i = 0; i < n; i++) {
		v1 = p[i] - center;
		xx = v1.x * v1.x;
		xy = v1.x * v1.y;
		xz = v1.x * v1.z;
		yy = v1.y * v1.y;
		yz = v1.y * v1.z;
		zz = v1.z * v1.z;
		m.m[0] += xx;
		m.m[1] += xy;
		m.m[2] += xz;
		m.m[4] += xy;
		m.m[5] += yy;
		m.m[6] += yz;
		m.m[8] += xz;
		m.m[9] += yz;
		m.m[10] += zz;
	}
	m.m[15] = 1;
	//get eigenvalue
	Vector3 normal;
	float det = m.determinant();
	if(det == 0) {
		v1 = p[0] - center;
		v2 = p[1] - center;
		Vector3::cross(v1, v2, &normal);
	} else {
		m.invert();
		float max = 0, val;
		for(i = 0; i < 3; i++) {
			for(j = 0; j < 3; j++) {
				val = fabs(m.m[i*4 + j]);
				if(val > max) max = val;
			}
		}
		m.scale(1 / max);
		for(i = 0; i < 3; i++) m *= m;
		normal.set(1, 1, 1);
		Vector3 last = normal;
		for(i = 0; i < 100; i++) {
			m.transformVector(&normal);
			normal.normalize();
			v1 = normal - last;
			if(v1.lengthSquared() < 1e-16f) break;
			last = normal;
		}
	}
	_stringPlane.set(normal, -normal.dot(center));
	_normal = normal.normalize();
	_axis = _nodes.back()->_node->getTranslationWorld() - _nodes.front()->_node->getTranslationWorld();
	_axis.normalize();
	Vector3::cross(_normal, _axis, &_up);
	return true;
}

bool StringMode::getPath() {
	short n = _nodes.size(), i, j;
	//calculate the 2D convex hull of each outline in the sequence
	for(i = 0; i < n; i++) if(!_nodes[i]->getHull()) return false;
	//start on the center of the first node, facing toward the next node's center
	short ind = 0, mode = 0, vertex = 0, maxInd;
	Vector2 dir, side, v1, v2, v3;
	Vector3 vec = _nodes[1]->_planeCenter;
	dir.set(vec.x, vec.y);
	dir.normalize();
	float f1, f2, f3, angle, maxAngle, maxAfter;
	bool switchMode = false;
	_path->clearMesh();
	//at each step, see if we can jump to the next hull in the sequence
	do {
		NodeData *node = _nodes[ind], *next = _nodes[ind+1];
		if(mode == 0) side.set(-dir.y, dir.x);
		else side.set(dir.y, -dir.x);
		v1 = node->_hull[vertex];
		_path->addVertex(_origin + v1.x * _axis + v1.y * _up);
		if(ind == n-1) break; //we touched the last node, so we're done
		f1 = switchMode ? -1 : 1;
		maxAngle = f1 * -1e6;
		for(i = 0; i < next->_hull.size(); i++) {
			v2 = next->_hull[i] - v1;
			angle = atan2(v2.dot(side), v2.dot(dir));
			if(angle > M_PI/2) angle -= 2*M_PI;
			if(f1*angle > f1*maxAngle) {
				maxAngle = angle;
				maxInd = i;
			}
		}
		if(maxAngle >= 0) { //we can jump to the next hull
			if(ind < n-2 && !switchMode) { //first check if jumping to the following hull would bypass the next one...
				NodeData *after = _nodes[ind+2];
				maxAfter = -1e6;
				for(i = 0; i < after->_hull.size(); i++) {
					v2 = after->_hull[i] - v1;
					angle = atan2(v2.dot(side), v2.dot(dir));
					if(angle > M_PI/2) angle -= 2*M_PI;
					if(angle > maxAfter) maxAfter = angle;
				}
				if(maxAfter > maxAngle) { //...in which case, switch directions to wrap around the next hull first
					switchMode = true;
					continue;
				}
			}
			//jump to the next hull
			ind++;
			vertex = maxInd;
			if(switchMode) mode = 1-mode;
			switchMode = false;
		}
	} while(true);
	_path->updateModel(false);
	_scene->addNode(_path);
	return true;
}

void StringMode::makeString() {
}

void StringMode::clearPath() {
	for(std::vector<NodeData*>::iterator it = _nodes.begin(); it != _nodes.end(); it++) delete *it;
	_nodes.clear();
	_scene->removeNode(_path);
}


StringMode::NodeData::NodeData(MyNode *node, StringMode *mode) : _node(node), _mode(mode) {}

void StringMode::NodeData::updateVertices() {
	short n = _node->nv(), i;
	_planeVertices.resize(n);
	Vector3 v1;
	for(i = 0; i < n; i++) {
		v1 = _node->_worldVertices[i] - _mode->_origin;
		_planeVertices[i].set(v1.dot(_mode->_axis), v1.dot(_mode->_up), v1.dot(_mode->_normal));
	}
	v1 = _node->getTranslationWorld() - _mode->_origin;
	_planeCenter.set(v1.dot(_mode->_axis), v1.dot(_mode->_up), v1.dot(_mode->_normal));
}

//given the face the user clicked on, find the contiguous region of the surface on the same side of the string plane,
//and get all its edge intersections with the plane as an unordered set
bool StringMode::NodeData::getOutline() {
	updateVertices();
	//get the clicked face from the clicked point
	short f = _node->pt2Face(_point);
	if(f < 0) return false;
	//find a vertex on the face that is on the same side of the plane as the clicked point
	std::vector<unsigned short> face = _node->_faces[f];
	Plane plane = _mode->_stringPlane;
	float distance = plane.getDistance();
	Vector3 normal = plane.getNormal(), v1 = _point + normal * distance, v2, v3;
	short i, n = face.size(), sign = _point.dot(normal) > 0 ? 1 : -1;
	for(i = 0; i < n; i++) {
		v1 = _node->_worldVertices[face[i]] + normal * distance;
		if(v1.dot(normal) / sign > 0) break;
	}
	if(i == n) return false;
	//branch out from that vertex until we find every edge intersection with the plane
	std::map<unsigned short, std::set<unsigned short> > edges;
	std::map<unsigned short, std::set<unsigned short> >::iterator it;
	std::set<unsigned short> used;
	std::set<unsigned short>::iterator sit;
	short p = face[i], q, r;
	std::map<unsigned short, unsigned short>::const_iterator eit;
	for(eit = _node->_edgeInd[p].begin(); eit != _node->_edgeInd[p].end(); eit++) edges[p].insert(eit->first);
	used.insert(p);
	while(!edges.empty()) {
		//get an edge
		it = edges.begin();
		p = it->first;
		q = *it->second.begin();
		edges[p].erase(q);
		if(edges[p].empty()) edges.erase(p);
		//if it intersects the plane, add its intersection
		v1 = _planeVertices[p];
		v2 = _planeVertices[q];
		if((v1.z < 0) != (v2.z < 0)) {
			distance = fabs(v1.z / (v2.z - v1.z));
			_outline.push_back(Vector2(v1.x + distance * (v2.x - v1.x), v1.y + distance * (v2.y - v1.y)));
			continue;
		}
		//otherwise, branch out from its second endpoint
		for(eit = _node->_edgeInd[q].begin(); eit != _node->_edgeInd[q].end(); eit++) {
			r = eit->first;
			if(used.find(r) == used.end()) edges[q].insert(r);
		}
		used.insert(q);
	}
	return true;
}

//Jarvis' walk method to find convex hull
bool StringMode::NodeData::getHull() {
	_hull.clear();
	if(this == _mode->_nodes.front()) {
		_hull.push_back(Vector2(0, 0));
		return true;
	}
	if(this == _mode->_nodes.back()) {
		MyNode *head = _mode->_nodes.front()->_node;
		Vector3 delta = _node->getTranslationWorld() - head->getTranslationWorld();
		_hull.push_back(Vector2(delta.length(), 0));
		return true;
	}
	getOutline();
	short i, j, n = _outline.size(), start, p, maxInd;
	Vector2 v1, v2;
	float minX = 1e6, angle, maxAngle;
	bool *used = new bool[n];
	//get the left-most point
	for(i = 0; i < n; i++) {
		used[i] = false;
		v1 = _outline[i];
		if(v1.x < minX) {
			minX = v1.x;
			start = i;
		}
	}
	if(start < 0) return false;
	p = start;
	Vector2 dir(1, 0), right;
	do {
		_hull.push_back(_outline[p]);
		if(p != start) used[p] = true;
		//get the right-most unused point viewed from the current point
		maxAngle = -1e6;
		maxInd = -1;
		right.set(dir.y, -dir.x);
		for(i = 0; i < n; i++) {
			if(i == p || used[i]) continue;
			v1 = _outline[i] - _outline[p];
			angle = atan2(v1.dot(right), v1.dot(dir));
			if(angle > maxAngle) {
				maxAngle = angle;
				maxInd = i;
			}
		}
		if(maxInd < 0) {
			return false;
		}
		dir = _outline[p] - _outline[maxInd];
		dir.normalize();
		p = maxInd;
	} while(p != start);
	return true;
}


