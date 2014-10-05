#include "MyNode.h"

MyNode::MyNode(const char *id) : Node::Node(id)
{
	init();
}

MyNode* MyNode::create(const char *id) {
	return new MyNode(id);
}

void MyNode::init() {
    app = (T4TApp*) Game::getInstance();
    _constraintParent = NULL;
    data = new nodeData();
    data->rotation.set(0, 0, 0, 1);
    data->scale.set(1, 1, 1);
    data->staticObj = false;
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
short MyNode::pt2Face(Vector3 point) {
	unsigned short i, j, k, n;
	short touchFace = -1;
	std::vector<unsigned short> face, triangle;
	Vector3 v1, v2, v3, p, coords;
	Matrix m;
	float minDistance = 9999.0f;
	for(i = 0; i < data->faces.size(); i++) {
		face = data->faces[i];
		for(j = 0; j < data->triangles[i].size(); j++) {
			triangle = data->triangles[i][j];
			v1.set(data->worldVertices[face[triangle[1]]] - data->worldVertices[face[triangle[0]]]);
			v2.set(data->worldVertices[face[triangle[2]]] - data->worldVertices[face[triangle[0]]]);
			v3.set(data->worldNormals[i]);
			p.set(point - data->worldVertices[face[triangle[0]]]);
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
	Vector3 normal(data->worldNormals[f]), vertex(data->worldVertices[data->faces[f][0]]);
	normal.normalize(&normal);
	Plane plane(normal, -vertex.dot(normal));
	if(modelSpace) plane.transform(getInverseRotTrans());
	return plane;
}

Vector3 MyNode::faceCenter(unsigned short f, bool modelSpace) {
	Vector3 center(0, 0, 0);
	unsigned short i, n = data->faces[f].size();
	for(i = 0; i < n; i++) {
		center += modelSpace ? data->vertices[data->faces[f][i]] : data->worldVertices[data->faces[f][i]];
	}
	center *= 1.0f / n;
	return center;
}

//position node so that given face is flush with given plane
void MyNode::rotateFaceToPlane(unsigned short f, Plane p) {
	float angle;
	Vector3 axis, face, plane;
	//get model space face normal
	data->normals[f].normalize(&face);
	//get axis/angle rotation required to align face normal with plane normal
	plane.set(-p.getNormal());
	setRotation(getVectorRotation(face, plane));
	//translate node so it is flush with the plane
	Vector3 vertex(data->vertices[data->faces[f][0]]);
	getWorldMatrix().transformPoint(&vertex);
	float distance = vertex.dot(plane) - p.getDistance();
	translate(-plane * distance);
	updateData();
}

void MyNode::rotateFaceToFace(unsigned short f, MyNode *other, unsigned short g) {
	Plane p = other->facePlane(g);
	rotateFaceToPlane(f, p);
	//also align centers of faces
	Vector3 center1(0, 0, 0), center2(0, 0, 0);
	translate(other->faceCenter(g) - faceCenter(f));
	updateData();
}

void MyNode::addEdge(unsigned short e1, unsigned short e2) {
	if(data->edgeInd.find(e1) != data->edgeInd.end() && data->edgeInd[e1].find(e2) != data->edgeInd[e1].end()) return;
	std::vector<unsigned short> edge(2);
	edge[0] = e1;
	edge[1] = e2;
	data->edgeInd[e1][e2] = data->edges.size();
	data->edgeInd[e2][e1] = data->edges.size();
	data->edges.push_back(edge);
}

void MyNode::updateEdges() {
	unsigned short i, j, n;
	for(i = 0; i < data->faces.size(); i++) {
		n = data->faces[i].size();
		for(j = 0; j < n; j++) {
			addEdge(data->faces[i][j], data->faces[i][(j+1)%n]);
		}
	}
}

void MyNode::addFace(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles, bool reverse) {
	if(reverse) {
		unsigned short i, n = face.size(), temp;
		for(i = 0; i < n/2; i++) {
			temp = face[i];
			face[i] = face[n-1 - i];
			face[n-1 - i] = temp;
		}
	}
	if(triangles.empty()) triangulate(face, triangles);
	addFaceHelper(face, triangles);
}

void MyNode::addFaceHelper(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles) {
	data->faces.push_back(face);
	data->triangles.push_back(triangles);
	data->normals.push_back(getNormal(face, true));
	data->worldNormals.push_back(getNormal(face));
	unsigned short n = face.size(), i;
	for(i = 0; i < n; i++) addEdge(face[i], face[(i+1)%n]);
}

//calculate the properly oriented face normal by Newell's method
// - https://www.opengl.org/wiki/Calculating_a_Surface_Normal#Newell.27s_Method
Vector3 MyNode::getNormal(std::vector<unsigned short>& face, bool modelSpace) {
	Vector3 v1, v2, normal(0, 0, 0);
	unsigned short i, n = face.size();
	for(i = 0; i < n; i++) {
		v1.set(data->worldVertices[face[i]]);
		v2.set(data->worldVertices[face[(i+1)%n]]);
		normal.x += (v1.y - v2.y) * (v1.z + v2.z);
		normal.y += (v1.z - v2.z) * (v1.x + v2.x);
		normal.z += (v1.x - v2.x) * (v1.y + v2.y);
	}
	if(modelSpace) getInverseRotTrans().transformVector(&normal);
	return normal.normalize();
}

void MyNode::triangulate(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles) {
	//make a copy of the face so we don't modify the original
	unsigned short n = face.size(), i;
	std::vector<unsigned short> copy(n), inds(n);
	for(i = 0; i < n; i++) {
		copy[i] = face[i];
		inds[i] = i;
	}
	triangulateHelper(copy, inds, triangles, getNormal(face));
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
			v1.set(data->worldVertices[face[i-1]] - data->worldVertices[face[i%n]]);
			v2.set(data->worldVertices[face[(i+1)%n]] - data->worldVertices[face[i%n]]);
			Vector3::cross(v2, v1, &v3);
			if(v3.dot(normal) < 0) continue;
			m.set(v1.x, v2.x, v3.x, 0, v1.y, v2.y, v3.y, 0, v1.z, v2.z, v3.z, 0, 0, 0, 0, 1);
			m.invert();
			//get barycentric coords of all other vertices of this face in the proposed triangle
			valid = true;
			for(j = (i+2)%n; j != i-1; j = (j+1)%n) {
				m.transformVector(data->worldVertices[face[j]] - data->worldVertices[face[i%n]], &coords);
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

MyNode::nodeData* MyNode::getData() {
	return data;
}

void MyNode::setData(MyNode::nodeData *newData) {
	nodeData *curData = data;
	data = NULL;
	if(curData != NULL) free(curData);
	data = newData;
}

MyNode::nodeData* MyNode::copyData() {
	nodeData *copy = new nodeData();
	copy->type = data->type;
	copy->vertices = data->vertices;
	copy->worldVertices = data->worldVertices;
	copy->edges = data->edges;
	copy->edgeInd = data->edgeInd;
	copy->faces = data->faces;
	copy->triangles = data->triangles;
	copy->faceNeighbors = data->faceNeighbors;
	copy->normals = data->normals;
	copy->worldNormals = data->worldNormals;
	copy->objType = data->objType;
	copy->mass = data->mass;
	copy->staticObj = data->staticObj;
	copy->hulls = data->hulls;
	copy->constraints = data->constraints;
	copy->rotation = data->rotation;
	copy->translation = data->translation;
	copy->scale = data->scale;
	copy->initTrans = data->initTrans;
	copy->typeCount = data->typeCount;
	return copy;
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

void MyNode::loadData(const char *file)
{
	std::string filename = resolveFilename(file);
	std::auto_ptr<Stream> stream(FileSystem::open(filename.c_str()));
	if (stream.get() == NULL)
	{
		GP_ERROR("Failed to open file '%s'.", filename.c_str());
		return;
	}
	stream->rewind();
	
	data->typeCount = 0;

	char *str, line[2048];
    float x, y, z, w;
	std::istringstream in;
	str = stream->readLine(line, 2048);
	in.str(str);
	std::string type;
	in >> type;
	data->type = type;
	if(type.compare("root") != 0) { //this is a physical node, not just a root node
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> x >> y >> z >> w;
		data->rotation.set(Vector3(x, y, z), (float)(w*M_PI/180.0));
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> x >> y >> z;
		data->translation.set(x, y, z);
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> x >> y >> z;
		data->scale.set(x, y, z);
		str = stream->readLine(line, 2048);
		int nv = atoi(str), v = 0;
		for(int i = 0; i < nv; i++) {
			str = stream->readLine(line, 2048);
			in.str(str);
			in >> x >> y >> z;
			data->vertices.push_back(Vector3(x, y, z));
			data->worldVertices.push_back(Vector3(x, y, z));
		}
		str = stream->readLine(line, 2048);
		int ne = atoi(str), e = 0;
		unsigned short v1, v2, v3;
		std::vector<unsigned short> edge(2);
		for(int i = 0; i < ne; i++) {
			str = stream->readLine(line, 2048);
			in.str(str);
			in >> v1 >> v2;
			edge[0] = v1; edge[1] = v2;
			data->edges.push_back(edge);
			data->edgeInd[v1][v2] = data->edges.size()-1;
			data->edgeInd[v2][v1] = data->edges.size()-1;
		}
		str = stream->readLine(line, 2048);
		int nf = atoi(str), faceSize, numTriangles, numNeighbors;
		std::vector<unsigned short> triangle(3), face;
		data->triangles.resize(nf);
		data->faceNeighbors.resize(nf);
		Vector3 vec1, vec2, normal;
		//faces, along with their constituent triangles and neighboring faces (sharing an edge)
		for(int i = 0; i < nf; i++) {
			str = stream->readLine(line, 2048);
			faceSize = atoi(str);
			face.clear();
			str = stream->readLine(line, 2048);
			in.str(str);
			for(int j = 0; j < faceSize; j++) {
				in >> v1;
				face.push_back(v1);
			}
			data->faces.push_back(face);
			//triangles
			str = stream->readLine(line, 2048);
			numTriangles = atoi(str);
			for(int j = 0; j < numTriangles; j++) {
				str = stream->readLine(line, 2048);
				in.str(str);
				for(int k = 0; k < 3; k++) {
					in >> v1;
					triangle[k] = v1;
				}
				data->triangles[i].push_back(triangle);
			}
			//neighbors
	/*    	str = stream->readLine(line, 2048);
			numNeighbors = atoi(str);
			str = stream->readLine(line, 2048);
			in.str(str);
			for(int j = 0; j < numNeighbors; j++) {
				in >> v1;
				data->faceNeighbors[i].push_back(v1);
			}//*/
		}
		//set the vertices according to the initial rotation, translation, & scale
		data->initTrans = Matrix::identity();
		data->initTrans.rotate(data->rotation);
		data->initTrans.scale(data->scale);
		data->initTrans.translate(data->translation);
		for(int i = 0; i < data->vertices.size(); i++) {
			data->initTrans.transformPoint(data->vertices[i], &data->worldVertices[i]);
		}
		setNormals();
		//physics
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> type;
		data->objType = type;
		str = stream->readLine(line, 2048);
		int nh = atoi(str), hullSize;
		data->hulls.resize(nh);
		for(int i = 0; i < nh; i++) {
			str = stream->readLine(line, 2048);
			hullSize = atoi(str);
			str = stream->readLine(line, 2048);
			in.str(str);
			for(int j = 0; j < hullSize; j++) {
				in >> v1;
				data->hulls[i].push_back(v1);
			}
		}
		str = stream->readLine(line, 2048);
		int nc = atoi(str);
		data->constraints.resize(nc);
		std::string word;
		for(int i = 0; i < nc; i++) {
			data->constraints[i] = new nodeConstraint();
			str = stream->readLine(line, 2048);
			in.str(str);
			in >> word;
			data->constraints[i]->type = word.c_str();
			in >> word;
			data->constraints[i]->other = word.c_str();
			in >> x >> y >> z >> w;
			data->constraints[i]->rotation.set(x, y, z, w);
			in >> x >> y >> z;
			data->constraints[i]->translation.set(x, y, z);
			data->constraints[i]->id = -1;
		}
		str = stream->readLine(line, 2048);
		data->mass = atof(str);
		str = stream->readLine(line, 2048);
		data->staticObj = atoi(str) > 0;
	}
	//see if this node has any children
	str = stream->readLine(line, 2048);
	int numChildren = atoi(str);
	std::string childId;
	for(int i = 0; i < numChildren; i++) {
		str = stream->readLine(line, 2048);
		in.str(str);
		in >> childId;
		MyNode *child = MyNode::create(childId.c_str());
		child->loadData();
		addChild(child);
	}
    stream->close();
}

void MyNode::writeData(const char *file) {
	std::string filename = resolveFilename(file);
	std::auto_ptr<Stream> stream(FileSystem::open(filename.c_str(), FileSystem::WRITE));
	if (stream.get() == NULL)
	{
		GP_ERROR("Failed to open file '%s'.", filename.c_str());
		return;
	}
	std::string line;
	std::ostringstream os;
	os << data->type << endl;
	line = os.str();
	stream->write(line.c_str(), sizeof(char), line.length());
	if(data->type.compare("root") != 0) {
		data->rotation.set(getRotation());
		data->translation.set(getTranslationWorld());
		data->scale.set(getScale());
		os.str("");
		Vector3 axis;
		float angle = data->rotation.toAxisAngle(&axis) * 180.0f/M_PI;
		os << axis.x << "\t" << axis.y << "\t" << axis.z << "\t" << angle << endl;
		os << data->translation.x << "\t" << data->translation.y << "\t" << data->translation.z << endl;
		os << data->scale.x << "\t" << data->scale.y << "\t" << data->scale.z << endl;
		os << data->vertices.size() << endl;
		for(int i = 0; i < data->vertices.size(); i++) {
			for(int j = 0; j < 3; j++) os << gv(&data->vertices[i],j) << "\t";
			os << endl;
		}
		line = os.str();
		stream->write(line.c_str(), sizeof(char), line.length());

		updateEdges();
		os.str("");
		os << data->edges.size() << endl;
		for(int i = 0; i < data->edges.size(); i++) {
			for(int j = 0; j < 2; j++) os << data->edges[i][j] << "\t";
			os << endl;
		}
		line = os.str();
		stream->write(line.c_str(), sizeof(char), line.length());
		os.str("");
		os << data->faces.size() << endl;
		for(int i = 0; i < data->faces.size(); i++) {
			os << data->faces[i].size() << endl;
			for(int j = 0; j < data->faces[i].size(); j++) os << data->faces[i][j] << "\t";
			os << endl << data->triangles[i].size() << endl;
			for(int j = 0; j < data->triangles[i].size(); j++) {
				for(int k = 0; k < 3; k++) os << data->triangles[i][j][k] << "\t";
				os << endl;
			}
			//os << data->faceNeighbors[i].size() << endl;
			//for(int j = 0; j < data->faceNeighbors[i].size(); j++) os << data->faceNeighbors[i][j] << "\t";
			//os << endl;
		}
		line = os.str();
		stream->write(line.c_str(), sizeof(char), line.length());
		os.str("");
		os << data->objType << endl;
		os << data->hulls.size() << endl;
		for(int i = 0; i < data->hulls.size(); i++) {
			os << data->hulls[i].size() << endl;
			for(int j = 0; j < data->hulls[i].size(); j++) os << data->hulls[i][j] << "\t";
			os << endl;
		}
		line = os.str();
		stream->write(line.c_str(), sizeof(char), line.length());
		os.str("");
		os << data->constraints.size() << endl;
		for(int i = 0; i < data->constraints.size(); i++) {
			os << data->constraints[i]->type << "\t" << data->constraints[i]->other << "\t";
			os << data->constraints[i]->rotation.x << "\t";
			os << data->constraints[i]->rotation.y << "\t";
			os << data->constraints[i]->rotation.z << "\t";
			os << data->constraints[i]->rotation.w << "\t";
			os << data->constraints[i]->translation.x << "\t";
			os << data->constraints[i]->translation.y << "\t";
			os << data->constraints[i]->translation.z << endl;
		}
		float mass = (getCollisionObject() != NULL) ? getCollisionObject()->asRigidBody()->getMass() : data->mass;
		os << mass << endl;
		os << (data->staticObj ? 1 : 0) << endl;
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
	for(int i = 0; i < children.size(); i++) os << children[i]->getId() << endl;
	line = os.str();
	stream->write(line.c_str(), sizeof(char), line.length());
	stream->close();
	for(int i = 0; i < children.size(); i++) children[i]->writeData();
}

void MyNode::updateData() {
	if(data == NULL) return;
	Matrix world = getWorldMatrix();
	unsigned short i, nv = data->vertices.size();
	Vector3 min, max;
	for(i = 0; i < nv; i++) {
		world.transformPoint(data->vertices[i], &data->worldVertices[i]);
	}
	setNormals();
}

void MyNode::updateModelFromData(bool doPhysics) {
	if(data->type.compare("root") != 0) {
		//reset the physics and transformation while loading the data
		removePhysics();
		setRotation(Quaternion::identity());
		setTranslation(Vector3::zero());
		setScale(Vector3(1,1,1));	
		std::string materialFile = concat(2, "res/common/models.material#", data->type.c_str());
		//update the mesh to contain the new coordinates
		float *vertices, radius = 0;
		unsigned short *triangles, i, j, k;
		int numVertices = 0, numTriangles = 0, v = 0, t = 0, f = 0;
		Vector3 min(1000,1000,1000), max(-1000,-1000,-1000);
		for(int i = 0; i < data->faces.size(); i++) {
			numVertices += data->faces[i].size();
			numTriangles += data->triangles[i].size();
		}
		vertices = new float[numVertices*6];
		triangles = new unsigned short[numTriangles*3];
		for(i = 0; i < data->faces.size(); i++) {
			for(j = 0; j < data->faces[i].size(); j++) {
				for(k = 0; k < 3; k++) {
					vertices[v++] = gv(&data->vertices[data->faces[i][j]],k);
					radius = fmaxf(radius, data->vertices[data->faces[i][j]].length());
					if(vertices[v-1] < gv(&min, k)) sv(&min, k, vertices[v-1]);
					if(vertices[v-1] > gv(&max, k)) sv(&max, k, vertices[v-1]);
				}
				for(k = 0; k < 3; k++) vertices[v++] = gv(&data->normals[i],k);
			}
			for(j = 0; j < data->triangles[i].size(); j++) {
				for(k = 0; k < 3; k++) triangles[t++] = f + data->triangles[i][j][k];
			}
			f += data->faces[i].size();
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
		//detach from my parent long enough to set the initial transformation and add physics
		Node *parent = getParent();
		if(parent != NULL) {
			addRef();
			parent->removeChild(this);
		}
		setRotation(data->rotation);
		setTranslation(data->translation);
		setScale(data->scale);
		updateData();
		if(doPhysics) addPhysics(false);
		if(parent != NULL) {
			parent->addChild(this);
			release();
		}
	}
	for(MyNode *child = dynamic_cast<MyNode*>(getFirstChild()); child; child = dynamic_cast<MyNode*>(child->getNextSibling())) {
		child->updateModelFromData(doPhysics);
	}
}

bool MyNode::isStatic() {
	return data->staticObj;
}

void MyNode::setStatic(bool stat) {
	data->staticObj = stat;
}

void MyNode::setNormals() {
	unsigned short nf = data->faces.size(), i, j, n;
	data->normals.resize(nf);
	data->worldNormals.resize(nf);
	Vector3 v1, v2, normal;
	Matrix invRot;
	getRotation(&invRot);
	invRot.invert();
	for(i = 0; i < nf; i++) {
		n = data->faces[i].size();
		normal.set(0, 0, 0);
		//first calculate the world normal
		for(j = 0; j < n; j++) {
			v1.set(data->worldVertices[data->faces[i][j]]);
			v2.set(data->worldVertices[data->faces[i][(j+1)%n]]);
			normal.x += (v1.y - v2.y) * (v1.z + v2.z);
			normal.y += (v1.z - v2.z) * (v1.x + v2.x);
			normal.z += (v1.x - v2.x) * (v1.y + v2.y);
		}
		normal.normalize(&normal);
		data->worldNormals[i].set(normal);
		//then undo this node's rotation to get the post-scaling model space normal
		invRot.transformVector(data->worldNormals[i], &data->normals[i]);
	}
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

/*********** OVERRIDES ***********/

void MyNode::myTranslate(const Vector3& delta) {
	for(MyNode *child = dynamic_cast<MyNode*>(getFirstChild()); child; child = dynamic_cast<MyNode*>(child->getNextSibling())) {
		child->myTranslate(delta);
	}
	cout << "moving " << getId() << " by " << app->printVector(delta) << endl;
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
	cout << "rotating by " << angle << " about " << app->printVector(axis) << " [" << delta.x << "," << delta.y << "," << delta.z << "," << delta.w << "]" << endl;
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
	if(data->type.compare("root") == 0) return;
	PhysicsRigidBody::Parameters params;
	params.mass = data->staticObj ? 0.0f : data->mass;
	if(data->objType.compare("mesh") == 0) {
		Mesh *mesh = getModel()->getMesh();
		mesh->vertices = &data->vertices;
		mesh->hulls = &data->hulls;
		setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::mesh(mesh), &params);
	} else if(data->objType.compare("box") == 0) {
		setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &params);
	} else if(data->objType.compare("sphere") == 0) {
		setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::sphere(), &params);
	} else if(data->objType.compare("capsule") == 0) {
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
	for(short i = 0; i < data->constraints.size(); i++) {
		if(data->constraints[i]->other.compare(other->getId()) == 0) return data->constraints[i];
	}
	return NULL;
}

