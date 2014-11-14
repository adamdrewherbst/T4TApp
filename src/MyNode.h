#ifndef MYNODE_H_
#define MYNODE_H_

#include "Project.h"

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
	bool _staticObj, _visible;
	std::vector<ConvexHull*> _hulls;
	std::vector<nodeConstraint*> _constraints;
	MyNode *_constraintParent;
	//for a compound object, store a rest position for each node so we have a rest configuration for the object
	Matrix _restPosition;
	//location and axis of constraint joint with parent in parent's model space
	Vector3 _parentOffset, _parentAxis;
	//when moving the node by dragging
	Vector3 _baseTranslation, _baseScale;
	Quaternion _baseRotation;
	
	Project::Element *_element; //if we are part of a project
	
	//animation
	AnimationClip *_currentClip;

	MyNode(const char *id);
	static MyNode* create(const char *id = NULL);
	void init();
	static MyNode* cloneNode(Node *node);
	
	std::string resolveFilename(const char *filename = NULL);
	void loadData(const char *filename = NULL, bool doPhysics = true);
	void writeData(const char *filename = NULL);
	void loadAnimation(const char *filename, const char *id);
	void playAnimation(const char *id, bool repeat = false, float speed = 1.0f);
	void stopAnimation();
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
	BoundingBox getBoundingBox(bool modelSpace = false);
	void set(const Matrix& trans);
	void set(Node *other);
	void myTranslate(const Vector3& delta);
	void setMyTranslation(const Vector3& translation);
	void myRotate(const Quaternion& delta);
	void setMyRotation(const Quaternion& rotation);
	void myScale(const Vector3& scale);
	void setMyScale(const Vector3& scale);
	void shiftModel(float x, float y, float z);
	void setBase();
	void baseTranslate(const Vector3& delta);
	void baseRotate(const Quaternion& delta);
	void baseScale(const Vector3& delta);
	void setRest();
	void placeRest();
	
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
	std::vector<MyNode*> getAllNodes();

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
	void setVisible(bool visible);
	void setActivation(int state);
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

