#ifndef MYNODE_H_
#define MYNODE_H_

#include "gameplay.h"
#include "FileSystem.h"
#include <cstdarg>

using namespace gameplay;

class T4TApp;
class Meshy;
class MyNode;

//polygon on mesh surface - may have holes inside
class Face {
	public:
	Meshy *_mesh;
	unsigned short _index; //my index in mesh's face list
	std::vector<unsigned short> _border; //outer boundary
	std::vector<std::vector<unsigned short> > _triangles, _holes;
	std::map<unsigned short, unsigned short> _next; //directed edge's opposite endpoint from each vertex
	Plane _plane, _worldPlane;
	
	typedef std::map<unsigned short, unsigned short>::iterator boundary_iterator;

	Face();	
	Face(Meshy *mesh);
	unsigned short size();
	unsigned short nv();
	unsigned short nh();
	unsigned short nt();
	boundary_iterator vbegin();
	boundary_iterator vend();
	unsigned short holeSize(unsigned short h);
	unsigned short hole(unsigned short h, unsigned short ind);
	unsigned short triangle(unsigned short t, unsigned short ind);
	bool hasHoles();
	void clear();
	void push_back(unsigned short vertex);
	void set(const std::vector<unsigned short> &boundary);
	void resize(unsigned short size);
	void addHole(const std::vector<unsigned short> &hole);
	unsigned short& operator[](unsigned short index);
	unsigned short front();
	unsigned short back();
	void addEdge(unsigned short e1, unsigned short e2, bool boundary = false);
	void updateEdges();
	void setTransform();
	void updateTransform();
	Plane getPlane(bool modelSpace = false);
	Vector3 getNormal(bool modelSpace = false);
	float getDistance(bool modelSpace = false);

	//triangulation of faces via GLU Tesselator
	void triangulate();
	
	static GLUtesselator *_tess;
	static Face *_tessFace;
	static GLenum _tessType;
	//vertices is what tesselator returns to us, buffer is the list of vertices we have fed to tesselator
	static std::vector<unsigned short> _tessVertices, _tessBuffer;
	static short _tessBufferInd;
	static void initTess();
	static void tessBegin(GLenum type);
	static void tessEnd();
	static void tessVertex(unsigned short *vertex);
	static void tessCombine(GLdouble coords[3], unsigned short *vertex[4], GLfloat weight[4], unsigned short **dataOut);
	static void tessError(GLenum errno);	
};

//generic wrapper for keeping track of mesh's data - node or convex hull
class Meshy {
public:
	Node *_node;
	Matrix _worldMatrix, _normalMatrix;
	std::vector<Vector3> _vertices, _worldVertices;
	std::vector<Face> _faces;
	std::vector<std::vector<unsigned short> > _edges;
	std::map<unsigned short, std::map<unsigned short, short> > _edgeInd;
	
	std::vector<std::string> _vInfo; //any info about the history of this vertex, for debugging
	
	Meshy();
	short nv();
	short nf();
	short ne();
	void addVertex(const Vector3 &v);
	void addVertex(float x, float y, float z);
	void setVInfo(unsigned short v, const char *info);
	void printVertex(unsigned short v);
	void addFace(Face &face);
	void addFace(std::vector<unsigned short> &face, bool reverse = false);
	void addFace(short n, ...);
	void addFace(std::vector<unsigned short> &face, std::vector<std::vector<unsigned short> > &triangles);
	void printFace(std::vector<unsigned short> &face);
	void printFace(unsigned short n);
	void printFaces();
	void printTriangles(short face = -1);
	void addEdge(unsigned short e1, unsigned short e2, short faceInd = -1);
	void update();
	virtual void updateTransform();
	virtual void updateEdges();
	virtual void setNormals();
	Vector3 getNormal(std::vector<unsigned short> &face, bool modelSpace = false);
	static Vector3 getNormal(std::vector<Vector3> &face);
	virtual void copyMesh(Meshy *mesh);
	virtual void clearMesh();
};

class MyNode : public Node, public Meshy {

public:

	T4TApp *app;

	class ConvexHull : public Meshy {
		public:
		ConvexHull(Node *node);
	};

    struct nodeConstraint {
    	int id; //global ID in simulation for this constraint
    	std::string other; //id of the node to which this one is constrained
    	std::string type; //one of: hinge, spring, fixed, socket
    	Vector3 translation; //offset of the constraint point from my origin
    	Quaternion rotation; //rotation offset of the constraint point
    };

	std::string _type; //which item this is from the catalog
	int _typeCount; //number of clones of this model currently in the simulation
	bool _wireframe, //drawing option for debugging
		_chain, _loop; //whether node is treated as vertex chain or triangle mesh
	float _lineWidth; //OpenGL line width if wireframe
	Vector3 _color;

	//physics
	std::string _objType; //mesh, box, sphere, capsule
	float _mass;
	bool _staticObj;
	std::vector<ConvexHull*> _hulls;
	std::vector<nodeConstraint*> _constraints;
	MyNode *_constraintParent;
	//location and axis of constraint joint with parent in parent's model space
	Vector3 _parentOffset, _parentAxis;

	MyNode(const char *id);
	static MyNode* create(const char *id = NULL);
	void init();
	static MyNode* cloneNode(Node *node);
	
	std::string resolveFilename(const char *filename = NULL);
	void loadData(const char *filename = NULL, bool doPhysics = true);
	void writeData(const char *filename = NULL);
	void updateTransform();
	void updateEdges();
	void setNormals();
	void updateModel(bool doPhysics = true);
	void calculateHulls();
	void setColor(float r, float g, float b);

	//transform
	Matrix getRotTrans();
	Matrix getInverseRotTrans();
	Matrix getInverseWorldMatrix();
	Vector3 getScaleVertex(short v);
	Vector3 getScaleNormal(short f);
	BoundingBox getWorldBox();
	void set(const Matrix& trans);
	void set(Node *other);
	void myTranslate(const Vector3& delta);
	void setMyTranslation(const Vector3& translation);
	void myRotate(const Quaternion& delta);
	void setMyRotation(const Quaternion& rotation);
	void myScale(const Vector3& scale);
	void setMyScale(const Vector3& scale);
	
	short pt2Face(Vector3 point, Vector3 viewer = Vector3::zero());
	Plane facePlane(unsigned short f, bool modelSpace = false);
	Vector3 faceCenter(unsigned short f, bool modelSpace = false);
	void rotateFaceToPlane(unsigned short f, Plane p);
	void rotateFaceToFace(unsigned short f, MyNode *other, unsigned short g);

	//topology
	void triangulate(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles);
	void triangulateHelper(std::vector<unsigned short>& face, std::vector<unsigned short>& inds,
	  std::vector<std::vector<unsigned short> >& triangles, Vector3 normal);
	void setWireframe(bool wireframe);
	void copyMesh(Meshy *mesh);
	void clearMesh();

	//physics
	void addHullFace(ConvexHull *hull, short f);
	void setOneHull();
	bool isStatic();
	void setStatic(bool stat);
	void addCollisionObject();
	void addPhysics(bool recur = true);
	void removePhysics(bool recur = true);
	void enablePhysics(bool enable = true, bool recur = true);
	bool physicsEnabled();
	nodeConstraint* getNodeConstraint(MyNode *other);
	MyNode *getConstraintNode(nodeConstraint *constraint);

	//general purpose
	static Quaternion getVectorRotation(Vector3 v1, Vector3 v2);
	static float gv(Vector3 &v, int dim);
	static void sv(Vector3 &v, int dim, float val);
	static Vector3 unitV(short axis);
	
	static char* concat(int n, ...);
	static float inf();
};

#include "T4TApp.h"

#endif

