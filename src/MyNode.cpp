#include "T4TApp.h"
#include "MyNode.h"
//for convex hull decomposition

//triangulation of faces with holes
GLUtesselator *Face::_tess; //for triangulating polygons
Face *Face::_tessFace;
GLenum Face::_tessType;
//vertices is what tesselation returns to us, buffer is the list of vertices we are using for tesselation
std::vector<unsigned short> Face::_tessVertices, Face::_tessBuffer;
short Face::_tessBufferInd;


Face::Face() : _mesh(NULL) {}

Face::Face(Meshy *mesh) : _mesh(mesh) {}

unsigned short Face::size() { return _border.size(); }

unsigned short Face::nv() { return _next.size(); }

unsigned short Face::nh() { return _holes.size(); }

unsigned short Face::nt() { return _triangles.size(); }

unsigned short& Face::operator[](unsigned short index) { return _border[index]; }

unsigned short Face::front() { return _border.front(); }

unsigned short Face::back() { return _border.back(); }

Face::boundary_iterator Face::vbegin() { return _next.begin(); }

Face::boundary_iterator Face::vend() { return _next.end(); }

bool Face::hasHoles() { return !_holes.empty(); }

unsigned short Face::holeSize(unsigned short h) { return _holes[h].size(); }

unsigned short Face::hole(unsigned short h, unsigned short ind) { return _holes[h][ind]; }

unsigned short Face::triangle(unsigned short t, unsigned short ind) { return _triangles[t][ind]; }

void Face::clear() {
	_border.clear();
	_triangles.clear();
	_holes.clear();
	_next.clear();
}

void Face::push_back(unsigned short vertex) {
	_border.push_back(vertex);
}

void Face::set(const std::vector<unsigned short> &border) {
	clear();
	_border = border;
}

void Face::resize(unsigned short size) {
	clear();
	_border.resize(size);
}

void Face::addEdge(unsigned short e1, unsigned short e2, bool boundary) {
	if(boundary) _next[e1] = e2;
	_mesh->addEdge(e1, e2, boundary ? _index : -1);
}

void Face::addHole(const std::vector<unsigned short> &hole) {
	_holes.push_back(hole);
}

void Face::setTransform() {
	Vector3 normal = _mesh->getNormal(_border, true);
	_plane.set(normal, -normal.dot(_mesh->_vertices[_border[0]]));
}

void Face::updateTransform() {
	_worldPlane = _plane;
	_worldPlane.transform(_mesh->_worldMatrix);
}

void Face::updateEdges() {
	short i, j, n, nh = _holes.size(), nt = _triangles.size();
	std::vector<unsigned short> cycle;
	n = _border.size();
	for(i = 0; i < n; i++) {
		//mark the CCW direction of the boundary edge with my face index
		addEdge(_border[i], _border[(i+1)%n], true);
	}
	for(i = 0; i < nh; i++) {
		n = _holes[i].size();
		for(j = 0; j < n; j++) { //same for hole boundaries
			addEdge(_holes[i][j], _holes[i][(j+1)%n], true);
		}
	}
	for(i = 0; i < nt; i++) {
		//don't mark inner triangle edges with my face index, to distinguish them
		for(j = 0; j < 3; j++) {
			addEdge(_triangles[i][j], _triangles[i][(j+1)%3]);
		}
	}
}

Plane Face::getPlane(bool modelSpace) {
	return modelSpace ? _plane : _worldPlane;
}

Vector3 Face::getNormal(bool modelSpace) {
	return modelSpace ? _plane.getNormal() : _worldPlane.getNormal();
}

float Face::getDistance(bool modelSpace) {
	return modelSpace ? _plane.getDistance() : _worldPlane.getDistance();
}

void Face::triangulate() {
	updateEdges();
	
	_tessFace = this;
	_triangles.clear();
	_tessVertices.clear();
	_tessBuffer.resize(nv() * nv()); //allow room for any edge intersections
	_tessBufferInd = 0;

	short i, j, k, nh = _holes.size(), n;
	Vector3 vec;
	GLdouble coords[3];

	gluTessBeginPolygon(_tess, NULL);
	for(i = 0; i < 1 + nh; i++) {
		std::vector<unsigned short> &cycle = i == 0 ? _border : _holes[i-1];
		gluTessBeginContour(_tess);
		n = cycle.size();
		for(j = 0; j < n; j++) {
			vec = _mesh->_vertices[cycle[j]];
			for(k = 0; k < 3; k++) coords[k] = MyNode::gv(vec, k);
			_tessBuffer[_tessBufferInd] = cycle[j];
			unsigned short *data = &_tessBuffer[_tessBufferInd++];
			gluTessVertex(_tess, coords, data);
		}
		gluTessEndContour(_tess);
	}
	gluTessEndPolygon(_tess);
}

void Face::initTess() {
	_tess = gluNewTess();
	gluTessCallback(_tess, GLU_TESS_BEGIN, (GLvoid (*) ()) &tessBegin);
	gluTessCallback(_tess, GLU_TESS_END, (GLvoid (*) ()) &tessEnd);
	gluTessCallback(_tess, GLU_TESS_VERTEX, (GLvoid (*) ()) &tessVertex);
	gluTessCallback(_tess, GLU_TESS_COMBINE, (GLvoid (*) ()) &tessCombine);
	gluTessCallback(_tess, GLU_TESS_ERROR, (GLvoid (*) ()) &tessError);
}

void Face::tessBegin(GLenum type) {
	_tessType = type;
}

void Face::tessEnd() {
	short i, j, n = _tessVertices.size();
	std::vector<unsigned short> triangle(3);
	switch(_tessType) {
		case GL_TRIANGLE_FAN:
			for(i = 0; i < n-2; i++) {
				triangle[0] = _tessVertices[0];
				for(j = 1; j <= 2; j++) triangle[j] = _tessVertices[i + j];
				_tessFace->_triangles.push_back(triangle);
			}
			break;
		case GL_TRIANGLE_STRIP:
			for(i = 0; i < n-2; i++) {
				for(j = 0; j < 3; j++) triangle[j] = _tessVertices[i + (i%2 == 0 ? j : 2-j)];
				_tessFace->_triangles.push_back(triangle);
			}
			break;
		case GL_TRIANGLES:
			for(i = 0; i < n/3; i++) {
				for(j = 0; j < 3; j++) triangle[j] = _tessVertices[i*3 + j];
				_tessFace->_triangles.push_back(triangle);
			}
			break;
		case GL_LINE_LOOP:
			break;
	}
	_tessVertices.clear();
}

void Face::tessVertex(unsigned short *vertex) {
	_tessVertices.push_back(*vertex);
}

void Face::tessCombine(GLdouble coords[3], unsigned short *vertex[4], GLfloat weight[4], unsigned short **dataOut) {
	short n = _tessFace->_mesh->nv();
	_tessFace->_mesh->addVertex((float)coords[0], (float)coords[1], (float)coords[2]);
	_tessBuffer[_tessBufferInd] = n;
	*dataOut = &_tessBuffer[_tessBufferInd++];
	cout << "tess combining ";
	for(short i = 0; i < 4 && vertex[i] > &_tessBuffer[0]; i++) cout << *vertex[i] << ",";
	cout << " => " << n << endl;
}

void Face::tessError(GLenum errno) {
	cout << "Tesselation error " << errno << endl;
}


Meshy::Meshy() {
}

short Meshy::nv() {
	return _vertices.size();
}

short Meshy::nf() {
	return _faces.size();
}

short Meshy::ne() {
	return _edges.size();
}

void Meshy::addVertex(const Vector3 &v) {
	_vertices.push_back(v);
}

void Meshy::addVertex(float x, float y, float z) {
	_vertices.push_back(Vector3(x, y, z));
}

void Meshy::setVInfo(unsigned short v, const char *info) {
	if(_vInfo.size() <= v) _vInfo.resize(v+1);
	_vInfo[v] = info;
}

void Meshy::printVertex(unsigned short n) {
	bool doWorld = _worldVertices.size() > n;
	Vector3 v = _vertices[n];
	cout << "VERTEX " << n << " <" << v.x << "," << v.y << "," << v.z << ">";
	if(doWorld) {
		v = _worldVertices[n];
		cout << " => <" << v.x << "," << v.y << "," << v.z << ">";
	}
	if(_vInfo.size() > n) cout << ": " << _vInfo[n] << endl;
}

void Meshy::addEdge(unsigned short e1, unsigned short e2, short faceInd) {
	if(faceInd >= 0 || _edgeInd.find(e1) == _edgeInd.end() || _edgeInd[e1].find(e2) == _edgeInd[e1].end()) {;
		_edgeInd[e1][e2] = faceInd;
		if(faceInd == -1) _edgeInd[e2][e1] = -1;
	}
}

void Meshy::addFace(Face &face) {
	face._mesh = this;
	face._index = _faces.size();
	if(dynamic_cast<MyNode*>(this) != NULL) face.triangulate();
	else face.updateEdges();
	_faces.push_back(face);
}

void Meshy::addFace(std::vector<unsigned short> &face, bool reverse) {
	unsigned short i, n = face.size(), temp;
	if(reverse) {
		for(i = 0; i < n/2; i++) {
			temp = face[i];
			face[i] = face[n-1 - i];
			face[n-1 - i] = temp;
		}
	}
	Face f(this);
	f._border = face;
	addFace(f);
}

void Meshy::addFace(short n, ...) {
	va_list arguments;
	va_start(arguments, n);
	std::vector<unsigned short> face;
	for(short i = 0; i < n; i++) {
		face.push_back((unsigned short)va_arg(arguments, int));
	}
	addFace(face);
}

void Meshy::addFace(std::vector<unsigned short> &face, std::vector<std::vector<unsigned short> > &triangles) {
	Face f(this);
	f._border = face;
	f._triangles = triangles;
	addFace(f);
}

void Meshy::printFace(std::vector<unsigned short> &face) {
	short i, n = face.size(), nInfo = _vInfo.size();
	Vector3 v;
	bool doWorld = _worldVertices.size() == _vertices.size();
	for(i = 0; i < n; i++) {
		v = doWorld ? _worldVertices[face[i]] : _vertices[face[i]];
		cout << face[i] << " <" << v.x << "," << v.y << "," << v.z << ">";
		if(nInfo > face[i]) cout << ": " << _vInfo[face[i]];
		cout << endl;
	}
}

void Meshy::printFace(unsigned short f) {
	Face &face = _faces[f];
	short i, j, n, nInfo = _vInfo.size(), nh = face.nh();
	Vector3 v;
	bool doWorld = _worldVertices.size() == _vertices.size();
	for(i = 0; i < 1+nh; i++) {
		std::vector<unsigned short> &cycle = i==0 ? face._border : face._holes[i-1];
		if(i == 0) cout << "BORDER:" << endl;
		else cout << "HOLE " << i-1 << ":" << endl;
		n = cycle.size();
		for(j = 0; j < n; j++) {
			v = doWorld ? _worldVertices[cycle[j]] : _vertices[cycle[j]];
			cout << cycle[j] << " <" << v.x << "," << v.y << "," << v.z << ">";
			if(nInfo > cycle[j]) cout << ": " << _vInfo[cycle[j]];
			cout << endl;
		}
	}
}

void Meshy::printFaces() {
	short i, j, nf = this->nf();
	for(i = 0; i < nf; i++) {
		Face &face = _faces[i];
		cout << face._index << ": ";
		for(j = 0; j < face.size(); j++) cout << face[j] << " ";
		cout << endl;
	}
}

void Meshy::printTriangles(short face) {
	short i, j, k, nf = this->nf(), nt;
	for(i = 0; i < nf; i++) {
		if(face >= 0 && i != face) continue;
		cout << i << ": ";
		nt = _faces[i].nt();
		for(j = 0; j < nt; j++) {
			for(k = 0; k < 3; k++) cout << _faces[i].triangle(j, k) << " ";
			if(j < nt-1) cout << ", ";
		}
		cout << endl;
	}
}

void Meshy::update() {
	setNormals();
	updateEdges();
	updateTransform();
}

void Meshy::updateTransform() {
	_worldMatrix = _node->getWorldMatrix();
	_normalMatrix = _node->getInverseTransposeWorldMatrix();
	unsigned short i, nv = _vertices.size(), nf = _faces.size();
	_worldVertices.resize(nv);
	_vInfo.resize(nv);
	for(i = 0; i < nv; i++) _worldMatrix.transformPoint(_vertices[i], &_worldVertices[i]);
	for(i = 0; i < nf; i++) _faces[i].updateTransform();
}

void Meshy::updateEdges() {
	_edges.clear(); //pairs of vertices
	_edgeInd.clear(); //vertex neighbor list
	unsigned short i, nf = _faces.size();
	for(i = 0; i < nf; i++) {
		_faces[i]._index = i;
		_faces[i].updateEdges();
	}
}

void Meshy::setNormals() {
	unsigned short i, nf = _faces.size();
	for(i = 0; i < nf; i++) _faces[i].setTransform();
}

//calculate the properly oriented face normal by Newell's method
// - https://www.opengl.org/wiki/Calculating_a_Surface_Normal#Newell.27s_Method
Vector3 Meshy::getNormal(std::vector<unsigned short>& face, bool modelSpace) {
	Vector3 v1, v2, normal(0, 0, 0);
	unsigned short i, n = face.size();
	for(i = 0; i < n; i++) {
		if(modelSpace) {
			v1.set(_vertices[face[i]]);
			v2.set(_vertices[face[(i+1)%n]]);
		} else {
			v1.set(_worldVertices[face[i]]);
			v2.set(_worldVertices[face[(i+1)%n]]);
		}
		normal.x += (v1.y - v2.y) * (v1.z + v2.z);
		normal.y += (v1.z - v2.z) * (v1.x + v2.x);
		normal.z += (v1.x - v2.x) * (v1.y + v2.y);
	}
	return normal.normalize();
}

Vector3 Meshy::getNormal(std::vector<Vector3> &face) {
	short n = face.size(), i;
	Vector3 v1, v2, normal(0, 0, 0);
	for(i = 0; i < n; i++) {
		v1 = face[i];
		v2 = face[(i+1)%n];
		normal.x += (v1.y - v2.y) * (v1.z + v2.z);
		normal.y += (v1.z - v2.z) * (v1.x + v2.x);
		normal.z += (v1.x - v2.x) * (v1.y + v2.y);
	}
}

void Meshy::copyMesh(Meshy *src) {
	_vertices = src->_vertices;
	_vInfo = src->_vInfo;
	_faces = src->_faces;
	for(short i = 0; i < _faces.size(); i++) _faces[i]._mesh = this;
	_edgeInd = src->_edgeInd;
	_edges = src->_edges;
}

void Meshy::clearMesh() {
	_vertices.clear();
	_faces.clear();
	_edges.clear();
	_edgeInd.clear();
	_vInfo.clear();
}


MyNode::MyNode(const char *id) : Node::Node(id), Meshy::Meshy()
{
	init();
}

MyNode* MyNode::create(const char *id) {
	return new MyNode(id);
}

void MyNode::init() {
	_node = this;
    app = (T4TApp*) Game::getInstance();
    _staticObj = false;
    _constraintParent = NULL;
    _chain = false;
    _loop = false;
    _wireframe = false;
    _lineWidth = 1.0f;
    _color.set(-1.0f, -1.0f, -1.0f); //indicates no color specified
    _objType = "none";
    _mass = 0;
    _radius = 0;
}

MyNode* MyNode::cloneNode(Node *node) {
	Node *clone = node->clone();
	MyNode *copy = new MyNode(clone->getId());
	MyNode *myNode = dynamic_cast<MyNode*>(node);
	if(myNode) {
		copy->copyMesh(myNode);
		copy->_type = myNode->_type;
		copy->_mass = myNode->_mass;
		copy->_chain = myNode->_chain;
		copy->_loop = myNode->_loop;
		copy->_wireframe = myNode->_wireframe;
		copy->_lineWidth = myNode->_lineWidth;
		copy->_color = myNode->_color;
	}
	copy->setModel(clone->getModel());
	copy->setCamera(clone->getCamera());
	copy->setScale(clone->getScale());
	copy->setRotation(clone->getRotation());
	copy->setTranslation(clone->getTranslation());
	SAFE_RELEASE(clone);
	return copy;
}

float MyNode::gv(Vector3 &v, int dim) {
	switch(dim) {
		case 0: return v.x;
		case 1: return v.y;
		case 2: return v.z;
	}
	return 0;
}

void MyNode::sv(Vector3 &v, int dim, float val) {
	switch(dim) {
		case 0: v.x = val; break;
		case 1: v.y = val; break;
		case 2: v.z = val; break;
	}
}

Vector3 MyNode::unitV(short axis) {
	switch(axis) {
		case 0: return Vector3::unitX();
		case 1: return Vector3::unitY();
		case 2: return Vector3::unitZ();
	}
	return Vector3::zero();
}

float MyNode::inf() {
	return std::numeric_limits<float>::infinity();
}

Quaternion MyNode::getVectorRotation(Vector3 v1, Vector3 v2) {
	Vector3 axis;
	Vector3::cross(v1, v2, &axis);
 	float angle = acos(v1.dot(v2) / (v1.length() * v2.length()));
	Quaternion rot;
	Quaternion::createFromAxisAngle(axis, angle, &rot);
	return rot;
}

Matrix MyNode::getRotTrans() {
	Matrix m;
	m.translate(getTranslationWorld());
	m.rotate(getRotation());
	return m;
}

Matrix MyNode::getInverseRotTrans() {
	Matrix m;
	m.translate(getTranslationWorld());
	m.rotate(getRotation());
	m.invert();
	return m;
}

Matrix MyNode::getInverseWorldMatrix() {
	Matrix m(getWorldMatrix());
	m.invert();
	return m;
}

BoundingBox MyNode::getWorldBox() {
	Model *model = getModel();
	if(model == NULL) return BoundingBox::empty();
	BoundingBox box = model->getMesh()->getBoundingBox();
	Vector3 scale = getScale(), center = box.getCenter(), min = box.min - center, max = box.max - center;
	min.x *= scale.x;
	min.y *= scale.y;
	min.z *= scale.z;
	max.x *= scale.x;
	max.y *= scale.y;
	max.z *= scale.z;
	return BoundingBox(center + min, center + max);
}

//given a point in space, find the best match for the face that contains it
short MyNode::pt2Face(Vector3 point, Vector3 viewer) {
	unsigned short i, j, k, n;
	short touchFace = -1;
	std::vector<unsigned short> face, triangle;
	Vector3 v1, v2, v3, p, coords;
	Matrix m;
	float minDistance = 9999.0f;
	Vector3 view(viewer - point);
	for(i = 0; i < _faces.size(); i++) {
		n = _faces[i].nt();
		for(j = 0; j < n; j++) {
			triangle = _faces[i]._triangles[j];
			v1.set(_worldVertices[triangle[1]] - _worldVertices[triangle[0]]);
			v2.set(_worldVertices[triangle[2]] - _worldVertices[triangle[0]]);
			v3.set(_faces[i].getNormal());
			//face must be facing toward the viewer, otherwise they couldn't have clicked it
			if(!viewer.isZero() && v3.dot(view) < 0) continue;
			p.set(point - _worldVertices[triangle[0]]);
			m.set(v1.x, v2.x, v3.x, 0, v1.y, v2.y, v3.y, 0, v1.z, v2.z, v3.z, 0, 0, 0, 0, 1);
			m.invert();
			m.transformVector(p, &coords);
			if(coords.x >= 0 && coords.y >= 0 && coords.x + coords.y <= 1 && fabs(coords.z) < minDistance) {
				touchFace = i;
				minDistance = fabs(coords.z);
				cout << "best match " << i << " at " << minDistance << endl;
				break;
			}
		}
	}
	return touchFace;
}

Plane MyNode::facePlane(unsigned short f, bool modelSpace) {
	return modelSpace ? _faces[f]._plane : _faces[f]._worldPlane;
}

Vector3 MyNode::faceCenter(unsigned short f, bool modelSpace) {
	Vector3 center(0, 0, 0);
	unsigned short i, n = _faces[f].size();
	for(i = 0; i < n; i++) {
		center += modelSpace ? _vertices[_faces[f][i]] : _worldVertices[_faces[f][i]];
	}
	center *= 1.0f / n;
	return center;
}

//position node so that given face is flush with given plane
void MyNode::rotateFaceToPlane(unsigned short f, Plane p) {
	float angle;
	Vector3 axis, face, plane;
	//get model space face normal
	face = getScaleNormal(f);
	//get axis/angle rotation required to align face normal with plane normal
	plane.set(-p.getNormal());
	setRotation(getVectorRotation(face, plane));
	//translate node so it is flush with the plane
	Vector3 vertex(_vertices[_faces[f][0]]);
	getWorldMatrix().transformPoint(&vertex);
	float distance = vertex.dot(plane) - p.getDistance();
	translate(-plane * distance);
	updateTransform();
}

void MyNode::rotateFaceToFace(unsigned short f, MyNode *other, unsigned short g) {
	Plane p = other->facePlane(g);
	rotateFaceToPlane(f, p);
	//also align centers of faces
	Vector3 center1(0, 0, 0), center2(0, 0, 0);
	translate(other->faceCenter(g) - faceCenter(f));
	updateTransform();
}

void MyNode::triangulate(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles) {
	//make a copy of the face so we don't modify the original
	unsigned short n = face.size(), i;
	std::vector<unsigned short> copy(n), inds(n);
	for(i = 0; i < n; i++) {
		copy[i] = face[i];
		inds[i] = i;
	}
	triangulateHelper(copy, inds, triangles, getNormal(face, true));
}

void MyNode::triangulateHelper(std::vector<unsigned short>& face,
  std::vector<unsigned short>& inds,
  std::vector<std::vector<unsigned short> >& triangles,
  Vector3 normal) {
	unsigned short i, j, n = face.size();
	short v = -1;
	bool valid;
	Vector3 v1, v2, v3, coords;
	Matrix m;
	std::vector<unsigned short> triangle(3);
	//find 3 consecutive vertices whose triangle has the right orientation and does not contain any other vertices
	if(n == 3) v = 1;
	else {
		for(i = 1; i <= n; i++) {
			v1.set(_vertices[face[i-1]] - _vertices[face[i%n]]);
			v2.set(_vertices[face[(i+1)%n]] - _vertices[face[i%n]]);
			Vector3::cross(v2, v1, &v3);
			if(v3.dot(normal) < 0) continue;
			m.set(v1.x, v2.x, v3.x, 0, v1.y, v2.y, v3.y, 0, v1.z, v2.z, v3.z, 0, 0, 0, 0, 1);
			m.invert();
			//get barycentric coords of all other vertices of this face in the proposed triangle
			valid = true;
			for(j = (i+2)%n; j != i-1; j = (j+1)%n) {
				m.transformVector(_vertices[face[j]] - _vertices[face[i%n]], &coords);
				if(coords.x >= 0 && coords.y >= 0 && coords.x + coords.y <= 1) {
					valid = false;
					break;
				}
			}
			if(valid) {
				v = i;
				break;
			}
		}
	}
	if(v < 0) {
		GP_WARN("Couldn't triangulate face");
		return;
	}
	triangle[0] = inds[v-1];
	triangle[1] = inds[v % n];
	triangle[2] = inds[(v+1)%n];
	triangles.push_back(triangle);
	face.erase(face.begin() + (v%n));
	inds.erase(inds.begin() + (v%n));
	if(n > 3) triangulateHelper(face, inds, triangles, normal);
}

void MyNode::setWireframe(bool wireframe) {
	_wireframe = wireframe;
}

void MyNode::copyMesh(Meshy *mesh) {
	MyNode *src = dynamic_cast<MyNode*>(mesh);
	if(!src) return;
	Meshy::copyMesh(mesh);
	short nh = src->_hulls.size(), i;
	_hulls.clear();
	_hulls.resize(nh);
	for(i = 0; i < nh; i++) {
		ConvexHull *hull = src->_hulls[i], *newHull = new ConvexHull(this);
		newHull->copyMesh(hull);
		_hulls[i] = newHull;
	}
	_objType = src->_objType;
}

void MyNode::clearMesh() {
	Meshy::clearMesh();
	for(std::vector<ConvexHull*>::iterator it = _hulls.begin(); it != _hulls.end(); it++) delete *it;
	_hulls.clear();
}

void MyNode::addHullFace(MyNode::ConvexHull *hull, short f) {
	hull->addFace(_faces[f]);
}

void MyNode::setOneHull() {
	_hulls.clear();
	ConvexHull *hull = new ConvexHull(this);
	short i;
	for(i = 0; i < nv(); i++) hull->addVertex(_vertices[i]);
	for(i = 0; i < nf(); i++) addHullFace(hull, i);
	_hulls.push_back(hull);
}

std::string MyNode::resolveFilename(const char *filename) {
	std::string path;
	int n = filename == NULL ? 0 : strlen(filename);
	if(filename == NULL) path = app->getSceneDir() + getId() + ".node";
	else if(filename[n-1] == '/') path = filename + _id + ".node";
	else if(strstr(filename, "/") == NULL) path = app->getSceneDir() + filename;
	else path = filename;
	return path;
}

void MyNode::loadData(const char *file, bool doPhysics)
{
	std::string filename = resolveFilename(file);
	std::unique_ptr<Stream> stream(FileSystem::open(filename.c_str()));
	if (stream.get() == NULL)
	{
		GP_ERROR("Failed to open file '%s'.", filename.c_str());
		return;
	}
	stream->rewind();
	
	_typeCount = 0;

	char *str, line[2048];
	short i, j, k, m, n;
    float x, y, z, w;
	std::istringstream in;
	str = stream->readLine(line, 2048);
	in.str(str);
	in >> _type;
	if(_type.compare("root") != 0) { //this is a physical node, not just a root node
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> x >> y >> z >> w;
		setRotation(Vector3(x, y, z), (float)(w*M_PI/180.0));
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> x >> y >> z;
		setTranslation(x, y, z);
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> x >> y >> z;
		setScale(x, y, z);
		str = stream->readLine(line, 2048);
		short nv = atoi(str);
		for(i = 0; i < nv; i++) {
			str = stream->readLine(line, 2048);
			in.str(str);
			in >> x >> y >> z;
			_vertices.push_back(Vector3(x, y, z));
		}
		//faces, along with their constituent triangles
		str = stream->readLine(line, 2048);
		short nf = atoi(str), faceSize, numHoles, holeSize, numTriangles;
		_faces.resize(nf);
		for(i = 0; i < nf; i++) {
			Face &face = _faces[i];
			face._mesh = this;
			face._index = i;
			str = stream->readLine(line, 2048);
			in.str(str);
			in >> faceSize;
			face._border.resize(faceSize);
			in >> numHoles;
			face._holes.resize(numHoles);
			in >> numTriangles;
			face._triangles.resize(numTriangles);
			str = stream->readLine(line, 2048);
			in.str(str);
			for(j = 0; j < faceSize; j++) {
				in >> n;
				face._border[j] = n;
			}
			for(j = 0; j < numHoles; j++) {
				str = stream->readLine(line, 2048);
				holeSize = atoi(str);
				face._holes[j].resize(holeSize);
				str = stream->readLine(line, 2048);
				in.str(str);
				for(k = 0; k < holeSize; k++) {
					in >> n;
					face._holes[j][k] = n;
				}
			}
			for(j = 0; j < numTriangles; j++) {
				str = stream->readLine(line, 2048);
				in.str(str);
				face._triangles[j].resize(3);
				for(k = 0; k < 3; k++) {
					in >> n;
					face._triangles[j][k] = n;
				}
			}
		}
		//physics
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> _objType;
		str = stream->readLine(line, 2048);
		short nh = atoi(str);
		_hulls.resize(nh);
		for(i = 0; i < nh; i++) {
			str = stream->readLine(line, 2048);
			nv = atoi(str);
			_hulls[i] = new ConvexHull(this);
			ConvexHull *hull = _hulls[i];
			hull->_vertices.resize(nv);
			for(j = 0; j < nv; j++) {
				str = stream->readLine(line, 2048);
				in.str(str);
				in >> x >> y >> z;
				hull->_vertices[j].set(x, y, z);
			}
			str = stream->readLine(line, 2048);
			nf = atoi(str);
			hull->_faces.resize(nf);
			for(j = 0; j < nf; j++) {
				Face &face = hull->_faces[j];
				face._mesh = hull;
				face._index = j;
				str = stream->readLine(line, 2048);
				faceSize = atoi(str);
				str = stream->readLine(line, 2048);
				in.str(str);
				face.resize(faceSize);
				for(k = 0; k < faceSize; k++) {
					in >> face[k];
				}
			}
		}
		str = stream->readLine(line, 2048);
		short nc = atoi(str);
		_constraints.resize(nc);
		std::string word;
		for(i = 0; i < nc; i++) {
			_constraints[i] = new nodeConstraint();
			str = stream->readLine(line, 2048);
			in.str(str);
			in >> word;
			_constraints[i]->type = word.c_str();
			in >> word;
			_constraints[i]->other = word.c_str();
			in >> x >> y >> z >> w;
			_constraints[i]->rotation.set(x, y, z, w);
			in >> x >> y >> z;
			_constraints[i]->translation.set(x, y, z);
			_constraints[i]->id = -1;
		}
		str = stream->readLine(line, 2048);
		_mass = atof(str);
		str = stream->readLine(line, 2048);
		_staticObj = atoi(str) > 0;
	}
	//see if this node has any children
	str = stream->readLine(line, 2048);
	int numChildren = atoi(str);
	std::string childId;
	for(i = 0; i < numChildren; i++) {
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> childId;
		MyNode *child = MyNode::create(childId.c_str());
		child->loadData(NULL, doPhysics);
		addChild(child);
	}
    stream->close();
	updateModel(doPhysics);
	if(getCollisionObject() != NULL) getCollisionObject()->setEnabled(false);
}

void MyNode::writeData(const char *file) {
	std::string filename = resolveFilename(file);
	std::unique_ptr<Stream> stream(FileSystem::open(filename.c_str(), FileSystem::WRITE));
	if (stream.get() == NULL)
	{
		GP_ERROR("Failed to open file '%s'.", filename.c_str());
		return;
	}
	short i, j, k;
	std::string line;
	std::ostringstream os;
	os << _type << endl;
	line = os.str();
	stream->write(line.c_str(), sizeof(char), line.length());
	if(_type.compare("root") != 0) {
		os.str("");
		Vector3 axis, vec, translation, scale;
		Quaternion rotation;
		if(getParent() != NULL && isStatic()) {
			Matrix m = getWorldMatrix();
			//Matrix::multiply(getParent()->getWorldMatrix(), getWorldMatrix(), &m);
			m.decompose(&scale, &rotation, &translation);
		} else {
			scale = getScale();
			rotation = getRotation();
			translation = getTranslation();
		}
		float angle = rotation.toAxisAngle(&axis) * 180.0f/M_PI;
		os << axis.x << "\t" << axis.y << "\t" << axis.z << "\t" << angle << endl;
		os << translation.x << "\t" << translation.y << "\t" << translation.z << endl;
		os << scale.x << "\t" << scale.y << "\t" << scale.z << endl;
		os << _vertices.size() << endl;
		for(i = 0; i < _vertices.size(); i++) {
			for(j = 0; j < 3; j++) os << gv(_vertices[i],j) << "\t";
			os << endl;
		}
		line = os.str();
		stream->write(line.c_str(), sizeof(char), line.length());

		os.str("");
		os << _faces.size() << endl;
		for(i = 0; i < _faces.size(); i++) {
			short n = _faces[i].size(), nh = _faces[i].nh(), nt = _faces[i].nt();
			os << n << "\t" << nh << "\t" << nt << endl;
			for(j = 0; j < n; j++) os << _faces[i][j] << "\t";
			os << endl;
			for(j = 0; j < nh; j++) {
				n = _faces[i].holeSize(j);
				for(k = 0; k < n; k++) os << _faces[i].hole(j, k) << "\t";
				os << endl;
			}
			for(j = 0; j < nt; j++) {
				for(k = 0; k < 3; k++) os << _faces[i].triangle(j, k) << "\t";
				os << endl;
			}
		}
		line = os.str();
		stream->write(line.c_str(), sizeof(char), line.length());
		os.str("");
		os << _objType << endl;
		os << _hulls.size() << endl;
		for(i = 0; i < _hulls.size(); i++) {
			ConvexHull *hull = _hulls[i];
			os << hull->_vertices.size() << endl;
			for(j = 0; j < hull->_vertices.size(); j++) {
				vec = hull->_vertices[j];
				os << vec.x << "\t" << vec.y << "\t" << vec.z << endl;
			}
			os << hull->_faces.size() << endl;
			for(j = 0; j < hull->_faces.size(); j++) {
				os << hull->_faces[j].size() << endl;
				for(k = 0; k < hull->_faces[j].size(); k++) os << hull->_faces[j][k] << "\t";
				os << endl;
			}
		}
		line = os.str();
		stream->write(line.c_str(), sizeof(char), line.length());
		os.str("");
		os << _constraints.size() << endl;
		for(i = 0; i < _constraints.size(); i++) {
			os << _constraints[i]->type << "\t" << _constraints[i]->other << "\t";
			os << _constraints[i]->rotation.x << "\t";
			os << _constraints[i]->rotation.y << "\t";
			os << _constraints[i]->rotation.z << "\t";
			os << _constraints[i]->rotation.w << "\t";
			os << _constraints[i]->translation.x << "\t";
			os << _constraints[i]->translation.y << "\t";
			os << _constraints[i]->translation.z << endl;
		}
		float mass = (getCollisionObject() != NULL) ? getCollisionObject()->asRigidBody()->getMass() : _mass;
		os << mass << endl;
		os << (_staticObj ? 1 : 0) << endl;
		line = os.str();
		stream->write(line.c_str(), sizeof(char), line.length());
	}
	//write any child nodes to their respective files
	std::vector<MyNode*> children;
	for(MyNode *child = dynamic_cast<MyNode*>(getFirstChild()); child; child = dynamic_cast<MyNode*>(child->getNextSibling())) {
		children.insert(children.begin(), child);
	}
	os.str("");
	os << children.size() << endl;
	for(i = 0; i < children.size(); i++) os << children[i]->getId() << endl;
	line = os.str();
	stream->write(line.c_str(), sizeof(char), line.length());
	stream->close();
	for(i = 0; i < children.size(); i++) children[i]->writeData();
}

void MyNode::updateTransform() {
	Meshy::updateTransform();
	for(short i = 0; i < _hulls.size(); i++) _hulls[i]->updateTransform();
}

void MyNode::updateEdges() {
	Meshy::updateEdges();
	for(short i = 0; i < _hulls.size(); i++) _hulls[i]->updateEdges();
}

void MyNode::setNormals() {
	Meshy::setNormals();
	for(short i = 0; i < _hulls.size(); i++) _hulls[i]->setNormals();
}

void MyNode::updateModel(bool doPhysics) {
	if(_type.compare("root") != 0) {
		//must detach from parent while setting transformation since physics object is off
		Node *parent = getParent();
		if(parent != NULL) {
			addRef();
			parent->removeChild(this);
		}
		removePhysics();

		//update the mesh to contain the new coordinates
		float radius = 0, f1;
		unsigned short i, j, k, m, n, v = 0, nv = this->nv(), nf = this->nf();
		Vector3 min(1000,1000,1000), max(-1000,-1000,-1000);

		//first find our new bounding box and bounding sphere, and position our node at their center
		// - otherwise Bullet applies gravity at node origin, not COM (why?) so produces torque
		for(i = 0; i < nv; i++) {
			for(j = 0; j < 3; j++) {
				f1 = gv(_vertices[i], j);
				if(f1 < gv(min, j)) sv(min, j, f1);
				if(f1 > gv(max, j)) sv(max, j, f1);
			}
		}
		Vector3 center = min + (max - min)/2.0f, vec, normal;
		translate(center);
		for(i = 0; i < nv; i++) {
			_vertices[i] -= center;
			f1 = _vertices[i].length();
			if(f1 > radius) radius = f1;
		}
		update();
		BoundingBox box(min - center, max - center);
		BoundingSphere sphere(Vector3::zero(), radius);

		//then create the new model
		std::vector<float> vertices;
		if(_chain) {
			n = _loop ? nv : nv-1;
			vertices.resize(2 * n * 6);
			for(i = 0; i < n; i++) {
				for(j = 0; j < 2; j++) {
					for(k = 0; k < 3; k++) vertices[v++] = gv(_vertices[(i+j)%nv], k);
					for(k = 0; k < 3; k++) vertices[v++] = gv(_color, k);
				}
			}
		} else {
			n = 0;
			for(i = 0; i < nf; i++) n += _faces[i].nt() * 3;
			vertices.resize(n * 6);
			for(i = 0; i < nf; i++) {
				n = _faces[i].nt();
				normal = _faces[i].getNormal(true);
				for(j = 0; j < n; j++) {
					for(k = 0; k < 3; k++) {
						vec = _vertices[_faces[i].triangle(j, k)];
						for(m = 0; m < 3; m++) vertices[v++] = gv(vec, m);
						for(m = 0; m < 3; m++) vertices[v++] = gv(normal, m);
					}
				}
			}
		}
		app->createModel(vertices, _chain, _type.c_str(), this);
		Mesh *mesh = getModel()->getMesh();
		mesh->setBoundingBox(box);
		mesh->setBoundingSphere(sphere);
		if(_color.x >= 0) setColor(_color.x, _color.y, _color.z); //updates the model's color
		
		//update convex hulls and constraints to reflect shift in node origin
		short nh = _hulls.size(), nc = _constraints.size();
		for(i = 0; i < nh; i++) {
			nv = _hulls[i]->nv();
			for(j = 0; j < nv; j++) _hulls[i]->_vertices[j] -= center;
			_hulls[i]->updateTransform();
		}
		for(i = 0; i < nc; i++) {
			nodeConstraint *constraint = _constraints[i];
			constraint->translation -= center;
			MyNode *other = getConstraintNode(constraint);
			if(other && other->_constraintParent == this) {
				other->_parentOffset -= center;
			}
		}
		if(doPhysics) addPhysics(false);
		if(parent != NULL) {
			parent->addChild(this);
			release();
		}
	}
	/*for(MyNode *child = dynamic_cast<MyNode*>(getFirstChild()); child; child = dynamic_cast<MyNode*>(child->getNextSibling())) {
		child->updateModel(doPhysics);
	}//*/
}

void MyNode::setColor(float r, float g, float b) {
	_color.set(r, g, b);
	Model *model = getModel();
	if(model) {
		Material *mat = model->getMaterial();
		if(mat) {
			Technique *tech = mat->getTechniqueByIndex(0);
			if(tech) {
				Pass *pass = tech->getPassByIndex(0);
				if(pass) {
					Effect *effect = pass->getEffect();
					if(effect) {
						Uniform *amb = effect->getUniform("u_ambientColor");
						if(amb) {
							pass->getParameter("u_ambientColor")->setValue(_color);
						}
					}
				}
			}
		}
	}
}

bool MyNode::isStatic() {
	return _staticObj;
}

void MyNode::setStatic(bool stat) {
	_staticObj = stat;
}

void MyNode::calculateHulls() {
/*	//put my mesh into HACD format
	std::vector< HACD::Vec3<HACD::Real> > points;
	std::vector< HACD::Vec3<long> > triangles;
	short i, j, k;

	Vector3 v;
	std::vector<unsigned short> face, triangle;
	for(i = 0; i < _vertices.size(); i++ ) 
	{
		v = _vertices[i];
		HACD::Vec3<HACD::Real> vertex(v.x, v.y, v.z);
		points.push_back(vertex);
	}
	for(i = 0; i < _faces.size(); i++)
	{
		face = _faces[i];
		for(j = 0; j < _triangles[i].size(); j++) {
			triangle = _triangles[i][j];
			HACD::Vec3<long> tri(face[triangle[0]], face[triangle[1]], face[triangle[2]]);
			triangles.push_back(tri);
		}
	}

	//initialize HACD and run it on my mesh
	HACD::HACD myHACD;
	myHACD.SetPoints(&points[0]);
	myHACD.SetNPoints(points.size());
	myHACD.SetTriangles(&triangles[0]);
	myHACD.SetNTriangles(triangles.size());
	myHACD.SetCompacityWeight(0.1);
	myHACD.SetVolumeWeight(0.0);

	// HACD parameters
	// Recommended parameters: 2 100 0 0 0 0
	size_t nClusters = 2;
	double concavity = 1;
	bool invert = false;
	bool addExtraDistPoints = false;
	bool addNeighboursDistPoints = false;
	bool addFacesPoints = false;       

	myHACD.SetNClusters(nClusters);                     // minimum number of clusters
	myHACD.SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
	myHACD.SetConcavity(concavity);                     // maximum concavity
	myHACD.SetAddExtraDistPoints(addExtraDistPoints);   
	myHACD.SetAddNeighboursDistPoints(addNeighboursDistPoints);   
	myHACD.SetAddFacesPoints(addFacesPoints); 

	myHACD.Compute();
	nClusters = myHACD.GetNClusters();
	
	//store the resulting hulls back into my data
	_hulls.clear();
	for(i = 0; i < nClusters; i++)
	{
		size_t nPoints = myHACD.GetNPointsCH(i), nTriangles = myHACD.GetNTrianglesCH(i);
		HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
		HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
		myHACD.GetCH(i, pointsCH, trianglesCH);

		ConvexHull *hull = new ConvexHull(this);
		for(j = 0; j < nPoints; j++) hull->addVertex(pointsCH[j].X(), pointsCH[j].Y(), pointsCH[j].Z());
		_hulls.push_back(hull);

		delete [] pointsCH;
		delete [] trianglesCH;
	}*/
}

Vector3 MyNode::getScaleVertex(short v) {
	Vector3 ret = _vertices[v], scale = getScale();
	ret.x *= scale.x;
	ret.y *= scale.y;
	ret.z *= scale.z;
	return ret;
}

Vector3 MyNode::getScaleNormal(short f) {
	Vector3 ret = _faces[f].getNormal(true), scale = getScale();
	ret.x /= scale.x;
	ret.y /= scale.y;
	ret.z /= scale.z;
	return ret.normalize();
}

char* MyNode::concat(int n, ...)
{
	const char** strings = new const char*[n];
	int length = 0;
	va_list arguments;
	va_start(arguments, n);
	for(int i = 0; i < n; i++) {
		strings[i] = (const char*) va_arg(arguments, const char*);
		length += strlen(strings[i]);
	}
	va_end(arguments);
	char *dest = (char*)malloc((length+1)*sizeof(char));
	int count = 0;
	for(int i = 0; i < length+1; i++) dest[i] = '\0';
	for(int i = 0; i < n; i++) {
		strcpy(dest + count*sizeof(char), strings[i]);
		count += strlen(strings[i]);
	}
	dest[length] = '\0';
	return dest;
}

/*********** TRANSFORM ***********/

void MyNode::set(const Matrix& trans) {
	PhysicsCollisionObject *obj = getCollisionObject();
	bool doPhysics = obj != NULL && obj->isEnabled();
	if(doPhysics) enablePhysics(false);

	Vector3 translation, scale;
	Quaternion rotation;
	trans.decompose(&scale, &rotation, &translation);
	setScale(scale);
	setRotation(rotation);
	setTranslation(translation);
	
	if(doPhysics) enablePhysics(true);
}

void MyNode::set(Node *other) {
	set(other->getWorldMatrix());
}

void MyNode::myTranslate(const Vector3& delta) {
	for(MyNode *child = dynamic_cast<MyNode*>(getFirstChild()); child; child = dynamic_cast<MyNode*>(child->getNextSibling())) {
		child->myTranslate(delta);
	}
	cout << "moving " << getId() << " by " << app->pv(delta) << endl;
	if(getParent() == NULL || (getCollisionObject() != NULL && !isStatic())) translate(delta);
}

void MyNode::setMyTranslation(const Vector3& translation) {
	myTranslate(translation - getTranslationWorld());
}

void MyNode::myRotate(const Quaternion& delta) {
	Vector3 baseTrans(getTranslationWorld()), offset, offsetRot;
	Matrix rot;
	Matrix::createRotation(delta, &rot);
	for(MyNode *child = dynamic_cast<MyNode*>(getFirstChild()); child; child = dynamic_cast<MyNode*>(child->getNextSibling())) {
		offset = child->getTranslationWorld() - baseTrans;
		rot.transformVector(offset, &offsetRot);
		child->myRotate(delta);
		child->myTranslate(offsetRot - offset);
	}
	if(getParent() == NULL || !isStatic()) setRotation(delta * getRotation());
}

void MyNode::setMyRotation(const Quaternion& rotation) {
	Quaternion rotInv, delta;
	getRotation().inverse(&rotInv);
	delta = rotation * rotInv;
	Vector3 axis;
	float angle = delta.toAxisAngle(&axis);
	cout << "rotating by " << angle << " about " << app->pv(axis) << " [" << delta.x << "," << delta.y << "," << delta.z << "," << delta.w << "]" << endl;
	myRotate(delta);
}

void MyNode::myScale(const Vector3& scale) {
	this->scale(scale);
}

void MyNode::setMyScale(const Vector3& scale) {
	setScale(scale);
}

/*********** PHYSICS ************/

void MyNode::addCollisionObject() {
	if(_type.compare("root") == 0) return;
	PhysicsRigidBody::Parameters params;
	params.mass = _staticObj ? 0.0f : _mass;
	if(_objType.compare("mesh") == 0) {
		Mesh *mesh = getModel()->getMesh();
		mesh->vertices = &_vertices;
		mesh->hulls = new std::vector<std::vector<Vector3> >();
		for(short i = 0; i < _hulls.size(); i++) {
			mesh->hulls->push_back(_hulls[i]->_vertices);
		}
		setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::mesh(mesh), &params);
	} else if(_objType.compare("box") == 0) {
		setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &params);
	} else if(_objType.compare("sphere") == 0) {
		setCollisionObject(PhysicsCollisionObject::RIGID_BODY,
		  _radius > 0 ? PhysicsCollisionShape::sphere(_radius) : PhysicsCollisionShape::sphere(), &params);
	} else if(_objType.compare("capsule") == 0) {
		setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::capsule(), &params);
	}
}

void MyNode::addPhysics(bool recur) {
	if(_objType.compare("none") == 0) return;
	if(getCollisionObject() == NULL) {
		addCollisionObject();
		app->addConstraints(this);
	}
	if(recur) {
		for(MyNode *node = dynamic_cast<MyNode*>(getFirstChild()); node; node = dynamic_cast<MyNode*>(node->getNextSibling())) {
			node->addPhysics();
		}
	}
}

void MyNode::removePhysics(bool recur) {
	if(getCollisionObject() != NULL) {
		app->removeConstraints(this);
		setCollisionObject(PhysicsCollisionObject::NONE);
	}
	if(recur) {
		for(MyNode *node = dynamic_cast<MyNode*>(getFirstChild()); node; node = dynamic_cast<MyNode*>(node->getNextSibling())) {
			node->removePhysics();
		}
	}
}

void MyNode::enablePhysics(bool enable, bool recur) {
	PhysicsCollisionObject *obj = getCollisionObject();
	if(obj != NULL && obj->isEnabled() == enable) return;
	if(recur) {
		//first enable/disable physics on all child nodes
		for(MyNode *node = dynamic_cast<MyNode*>(getFirstChild()); node; node = dynamic_cast<MyNode*>(node->getNextSibling())) {
			node->enablePhysics(enable);
		}
	}
	//then handle my own constraints and collision object
	if(enable) {
		if(obj == NULL) {
			addPhysics(false);
		} else {
			PhysicsRigidBody *body = obj->asRigidBody();
			body->setActivation(ACTIVE_TAG);
			body->setEnabled(true);
			app->enableConstraints(this, true);
		}
	} else if(obj != NULL) {
		if(obj->isStatic()) {
			removePhysics(false);
		} else {
			app->enableConstraints(this, false);
			obj->asRigidBody()->setEnabled(false);
		}
	}
}

bool MyNode::physicsEnabled() {
	PhysicsCollisionObject *obj = getCollisionObject();
	return obj != NULL && obj->isEnabled();
}

nodeConstraint* MyNode::getNodeConstraint(MyNode *other) {
	for(short i = 0; i < _constraints.size(); i++) {
		if(_constraints[i]->other.compare(other->getId()) == 0) return _constraints[i];
	}
	return NULL;
}

MyNode* MyNode::getConstraintNode(nodeConstraint *constraint) {
	Node *node = app->_scene->findNode(constraint->other.c_str());
	if(node == NULL) return NULL;
	return dynamic_cast<MyNode*>(node);
}

MyNode::ConvexHull::ConvexHull(Node *node) {
	_node = node;
}

