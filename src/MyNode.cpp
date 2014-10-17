#include "MyNode.h"
//for convex hull decomposition
#include "hacdCircularList.h"
#include "hacdVector.h"
#include "hacdICHull.h"
#include "hacdGraph.h"
#include "hacdHACD.h"

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

void Meshy::addEdge(unsigned short e1, unsigned short e2) {
	if(_edgeInd.find(e1) != _edgeInd.end() && _edgeInd[e1].find(e2) != _edgeInd[e1].end()) return;
	std::vector<unsigned short> edge(2);
	edge[0] = e1;
	edge[1] = e2;
	_edgeInd[e1][e2] = _edges.size();
	_edgeInd[e2][e1] = _edges.size();
	_edges.push_back(edge);
}

void Meshy::addFace(std::vector<unsigned short> &face, bool reverse) {
	if(reverse) {
		unsigned short i, n = face.size(), temp;
		for(i = 0; i < n/2; i++) {
			temp = face[i];
			face[i] = face[n-1 - i];
			face[n-1 - i] = temp;
		}
	}
	_faces.push_back(face);
	short i, n = face.size();
	for(i = 0; i < n; i++) addEdge(face[i], face[(i+1)%n]);
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

void Meshy::update() {
	setNormals();
	updateEdges();
	updateTransform();
}

void Meshy::updateTransform() {
	Matrix world = _node->getWorldMatrix(), norm = _node->getInverseTransposeWorldMatrix();
	unsigned short i, nv = _vertices.size(), nf = _faces.size();
	_worldVertices.resize(nv);
	for(i = 0; i < nv; i++) world.transformPoint(_vertices[i], &_worldVertices[i]);
	_worldNormals.resize(nf);
	for(i = 0; i < nf; i++) norm.transformVector(_normals[i], &_worldNormals[i]);
}

void Meshy::updateEdges() {
	unsigned short i, j, n;
	_edges.clear();
	_edgeInd.clear();
	for(i = 0; i < _faces.size(); i++) {
		n = _faces[i].size();
		for(j = 0; j < n; j++) {
			addEdge(_faces[i][j], _faces[i][(j+1)%n]);
		}
	}
}

void Meshy::setNormals() {
	unsigned short i, nf = _faces.size();
	_normals.resize(nf);
	for(i = 0; i < nf; i++) {
		_normals[i] = getNormal(_faces[i], true);
	}
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
}

MyNode* MyNode::cloneNode(Node *node) {
	Node *clone = node->clone();
	MyNode *copy = new MyNode(clone->getId());
	copy->setModel(clone->getModel());
	copy->setCamera(clone->getCamera());
	copy->setScale(clone->getScale());
	copy->setRotation(clone->getRotation());
	copy->setTranslation(clone->getTranslation());
	SAFE_RELEASE(clone);
	return copy;
}

float MyNode::gv(Vector3 *v, int dim) {
	switch(dim) {
		case 0: return v->x;
		case 1: return v->y;
		case 2: return v->z;
	}
	return 0;
}

void MyNode::sv(Vector3 *v, int dim, float val) {
	switch(dim) {
		case 0: v->x = val; break;
		case 1: v->y = val; break;
		case 2: v->z = val; break;
	}
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
		face = _faces[i];
		for(j = 0; j < _triangles[i].size(); j++) {
			triangle = _triangles[i][j];
			v1.set(_worldVertices[face[triangle[1]]] - _worldVertices[face[triangle[0]]]);
			v2.set(_worldVertices[face[triangle[2]]] - _worldVertices[face[triangle[0]]]);
			v3.set(_worldNormals[i]);
			//face must be facing toward the viewer, otherwise they couldn't have clicked it
			if(!viewer.isZero() && _worldNormals[i].dot(view) < 0) continue;
			p.set(point - _worldVertices[face[triangle[0]]]);
			m.set(v1.x, v2.x, v3.x, 0, v1.y, v2.y, v3.y, 0, v1.z, v2.z, v3.z, 0, 0, 0, 0, 1);
			//m.set(v1.x, v1.y, v1.z, 0, v2.x, v2.y, v2.z, 0, v3.x, v3.y, v3.z, 0, 0, 0, 0, 1);
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
	Vector3 normal(_worldNormals[f]), vertex(_worldVertices[_faces[f][0]]);
	normal.normalize(&normal);
	Plane plane(normal, -vertex.dot(normal));
	if(modelSpace) plane.transform(getInverseRotTrans());
	return plane;
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

void MyNode::addFace(std::vector<unsigned short>& face, bool reverse) {
	Meshy::addFace(face, reverse);
	std::vector<std::vector<unsigned short> > triangles;
	triangulate(face, triangles);
	_triangles.push_back(triangles);
}

void MyNode::addFace(short n, ...) {
	va_list arguments;
	va_start(arguments, n);
	std::vector<unsigned short> face;
	for(short i = 0; i < n; i++) {
		face.push_back((unsigned short)va_arg(arguments, int));
	}
	addFace(face);
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
	std::auto_ptr<Stream> stream(FileSystem::open(filename.c_str()));
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
		str = stream->readLine(line, 2048);
		short nf = atoi(str), faceSize, numTriangles, numNeighbors;
		std::vector<unsigned short> triangle(3), face;
		_triangles.resize(nf);
		//faces, along with their constituent triangles and neighboring faces (sharing an edge)
		for(i = 0; i < nf; i++) {
			str = stream->readLine(line, 2048);
			faceSize = atoi(str);
			face.clear();
			str = stream->readLine(line, 2048);
			in.str(str);
			for(j = 0; j < faceSize; j++) {
				in >> n;
				face.push_back(n);
			}
			_faces.push_back(face);
			//triangles
			str = stream->readLine(line, 2048);
			numTriangles = atoi(str);
			for(j = 0; j < numTriangles; j++) {
				str = stream->readLine(line, 2048);
				in.str(str);
				for(k = 0; k < 3; k++) {
					in >> n;
					triangle[k] = n;
				}
				_triangles[i].push_back(triangle);
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
				str = stream->readLine(line, 2048);
				faceSize = atoi(str);
				str = stream->readLine(line, 2048);
				in.str(str);
				face.clear();
				for(k = 0; k < faceSize; k++) {
					in >> n;
					face.push_back(n);
				}
				hull->_faces[j] = face;
			}
			hull->updateEdges();
			hull->setNormals();
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
	std::auto_ptr<Stream> stream(FileSystem::open(filename.c_str(), FileSystem::WRITE));
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
		Vector3 axis, vec, translation = getTranslation(), scale = getScale();
		float angle = getRotation().toAxisAngle(&axis) * 180.0f/M_PI;
		os << axis.x << "\t" << axis.y << "\t" << axis.z << "\t" << angle << endl;
		os << translation.x << "\t" << translation.y << "\t" << translation.z << endl;
		os << scale.x << "\t" << scale.y << "\t" << scale.z << endl;
		os << _vertices.size() << endl;
		for(i = 0; i < _vertices.size(); i++) {
			for(j = 0; j < 3; j++) os << gv(&_vertices[i],j) << "\t";
			os << endl;
		}
		line = os.str();
		stream->write(line.c_str(), sizeof(char), line.length());

		os.str("");
		os << _faces.size() << endl;
		for(i = 0; i < _faces.size(); i++) {
			os << _faces[i].size() << endl;
			for(j = 0; j < _faces[i].size(); j++) os << _faces[i][j] << "\t";
			os << endl << _triangles[i].size() << endl;
			for(j = 0; j < _triangles[i].size(); j++) {
				for(k = 0; k < 3; k++) os << _triangles[i][j][k] << "\t";
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
	for(short i = 0; i < _hulls.size(); i++) {
		_hulls[i]->updateTransform();
	}
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
		update();

		std::string materialFile = concat(2, "res/common/models.material#", _type.c_str());
		//update the mesh to contain the new coordinates
		float *vertices, radius = 0;
		unsigned short *triangles, i, j, k;
		int numVertices = 0, numTriangles = 0, v = 0, t = 0, f = 0;
		Vector3 min(1000,1000,1000), max(-1000,-1000,-1000);
		for(int i = 0; i < _faces.size(); i++) {
			numVertices += _faces[i].size();
			numTriangles += _triangles[i].size();
		}
		vertices = new float[numVertices*6];
		triangles = new unsigned short[numTriangles*3];
		for(i = 0; i < _faces.size(); i++) {
			for(j = 0; j < _faces[i].size(); j++) {
				for(k = 0; k < 3; k++) {
					vertices[v++] = gv(&_vertices[_faces[i][j]],k);
					radius = fmaxf(radius, _vertices[_faces[i][j]].length());
					if(vertices[v-1] < gv(&min, k)) sv(&min, k, vertices[v-1]);
					if(vertices[v-1] > gv(&max, k)) sv(&max, k, vertices[v-1]);
				}
				for(k = 0; k < 3; k++) vertices[v++] = gv(&_normals[i],k);
			}
			for(j = 0; j < _triangles[i].size(); j++) {
				for(k = 0; k < 3; k++) triangles[t++] = f + _triangles[i][j][k];
			}
			f += _faces[i].size();
		}
		VertexFormat::Element elements[] =
		{
			VertexFormat::Element(VertexFormat::POSITION, 3),
			VertexFormat::Element(VertexFormat::NORMAL, 3)
		};
		Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), numVertices, false);
		mesh->setVertexData(&vertices[0], 0, numVertices);
		MeshPart *part = mesh->addPart(Mesh::TRIANGLES, Mesh::INDEX16, numTriangles*3);
		part->setIndexData(triangles, 0, numTriangles*3);
		mesh->setBoundingBox(BoundingBox(min, max));
		mesh->setBoundingSphere(BoundingSphere(Vector3::zero(), radius));
		Model *model = Model::create(mesh);
		mesh->release();
		model->setMaterial(materialFile.c_str());
		setModel(model);
		model->release();
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

bool MyNode::isStatic() {
	return _staticObj;
}

void MyNode::setStatic(bool stat) {
	_staticObj = stat;
}

void MyNode::calculateHulls() {
	//put my mesh into HACD format
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
	}
}

Vector3 MyNode::getScaleVertex(short v) {
	Vector3 ret = _vertices[v], scale = getScale();
	ret.x *= scale.x;
	ret.y *= scale.y;
	ret.z *= scale.z;
	return ret;
}

Vector3 MyNode::getScaleNormal(short f) {
	Vector3 ret = _normals[f], scale = getScale();
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
	Vector3 translation, scale;
	Quaternion rotation;
	trans.decompose(&scale, &rotation, &translation);
	setScale(scale);
	setRotation(rotation);
	setTranslation(translation);
}

void MyNode::myTranslate(const Vector3& delta) {
	for(MyNode *child = dynamic_cast<MyNode*>(getFirstChild()); child; child = dynamic_cast<MyNode*>(child->getNextSibling())) {
		child->myTranslate(delta);
	}
	//cout << "moving " << getId() << " by " << app->pv(delta) << endl;
	if(getParent() == NULL || !isStatic()) translate(delta);
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
		setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::sphere(), &params);
	} else if(_objType.compare("capsule") == 0) {
		setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::capsule(), &params);
	}
}

void MyNode::addPhysics(bool recur) {
	if(getCollisionObject() != NULL) return;
	addCollisionObject();
	app->addConstraints(this);
	if(recur) {
		for(MyNode *node = dynamic_cast<MyNode*>(getFirstChild()); node; node = dynamic_cast<MyNode*>(node->getNextSibling())) {
			node->addPhysics();
		}
	}
}

void MyNode::removePhysics(bool recur) {
	if(getCollisionObject() == NULL) return;
	app->removeConstraints(this);
	setCollisionObject(PhysicsCollisionObject::NONE);
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

MyNode::nodeConstraint* MyNode::getNodeConstraint(MyNode *other) {
	for(short i = 0; i < _constraints.size(); i++) {
		if(_constraints[i]->other.compare(other->getId()) == 0) return _constraints[i];
	}
	return NULL;
}

MyNode::ConvexHull::ConvexHull(Node *node) {
	_node = node;
}

