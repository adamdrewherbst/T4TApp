#ifndef MYNODE_H_
#define MYNODE_H_

#include "Modes.h"

class MyNode : public Node, public Meshy {

public:

	T4TApp *app;

	class ConvexHull : public Meshy {
		public:
		ConvexHull(Node *node);
	};

	std::string _type; //which item this is from the catalog
	int _typeCount; //number of clones of this model currently in the simulation
	bool _wireframe, //drawing option for debugging
		_chain, _loop; //whether node is treated as vertex chain or triangle mesh
	float _lineWidth; //OpenGL line width if wireframe
	Vector3 _color;

	//physics
	std::string _objType; //mesh, box, sphere, capsule
	float _mass, _radius;
	bool _staticObj;
	std::vector<ConvexHull*> _hulls;
	std::vector<nodeConstraint*> _constraints;
	MyNode *_constraintParent;
	//location and axis of constraint joint with parent in parent's model space
	Vector3 _parentOffset, _parentAxis;
	
	Project::Element *_element; //if we are part of a project

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

#endif

