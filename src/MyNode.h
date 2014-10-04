#ifndef MYNODE_H_
#define MYNODE_H_

#include "gameplay.h"
#include "FileSystem.h"
#include <cstdarg>

using namespace gameplay;

class T4TApp;

class MyNode : public Node {

public:

	T4TApp *app;
	MyNode *_constraintParent;
	//location and axis of constraint joint with parent in parent's model space
	Vector3 parentOffset, parentAxis;

	MyNode(const char *id);
	static MyNode* create(const char *id = NULL);
	void init();
	static MyNode* cloneNode(Node *node);

    struct nodeConstraint {
    	int id; //global ID in simulation for this constraint
    	std::string other; //id of the node to which this one is constrained
    	std::string type; //one of: hinge, spring, fixed, socket
    	Vector3 translation; //offset of the constraint point from my origin
    	Quaternion rotation; //rotation offset of the constraint point
    };

   	//any data associated with a node
	struct nodeData {
		//model
		Quaternion rotation;
		Vector3 translation;
		Vector3 scale;
		Matrix initTrans; //combination of inital rotation, translation, and scaling
		std::vector<Vector3> vertices, worldVertices, //model space and world space coords
			normals, worldNormals; //model space (after scaling) and world space normal for each face
		std::vector<std::vector<unsigned short> > edges; //vertex index pairs
		std::map<unsigned short, std::map<unsigned short, unsigned short> > edgeInd; //index into edge list by vertex pair
		std::vector<std::vector<unsigned short> > faces; //vertex indices of polygons (not triangles)
		std::vector<std::vector<std::vector<unsigned short> > > triangles; //triangulation of each polygon
		std::vector<std::vector<unsigned short> > faceNeighbors;
		std::string type;
		int typeCount; //number of clones of this model currently in the simulation
		//physics
		std::string objType; //mesh, box, sphere, capsule
		float mass;
		bool staticObj;
		std::vector<std::vector<unsigned short> > hulls; //vertex indices of convex hulls
		std::vector<nodeConstraint*> constraints;
	};
	nodeData *data;

	nodeData* getData();
	void setData(nodeData *newData);
	nodeData* copyData();
	const char* getFilename();
	void loadData(const char *filename = NULL);
	void writeData(const char *filename = NULL);
	void updateData();
	void updateModelFromData(bool addPhysics = true);
	void setNormals();

	Matrix getRotTrans();
	Matrix getInverseRotTrans();
	Matrix getInverseWorldMatrix();
	BoundingBox getWorldBox();
	void myTranslate(const Vector3& delta);
	void setMyTranslation(const Vector3& translation);
	void myRotate(const Quaternion& delta);
	void setMyRotation(const Quaternion& rotation);
	void myScale(const Vector3& scale);
	void setMyScale(const Vector3& scale);

	short pt2Face(Vector3 point);
	Plane facePlane(unsigned short f, bool modelSpace = false);
	Vector3 faceCenter(unsigned short f, bool modelSpace = false);
	void rotateFaceToPlane(unsigned short f, Plane p);
	void rotateFaceToFace(unsigned short f, MyNode *other, unsigned short g);

	void addEdge(unsigned short e1, unsigned short e2);
	void updateEdges();
	void addFace(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles, bool reverse = false);
	void addFaceHelper(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles);
	Vector3 getNormal(std::vector<unsigned short>& face, bool modelSpace = false);
	void triangulate(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles);
	void triangulateHelper(std::vector<unsigned short>& face, std::vector<unsigned short>& inds,
	  std::vector<std::vector<unsigned short> >& triangles, Vector3 normal);
	
	bool isStatic();
	void setStatic(bool stat);
	void addCollisionObject();
	void addPhysics(bool recur = true);
	void removePhysics(bool recur = true);
	void enablePhysics(bool enable = true, bool recur = true);
	bool physicsEnabled();
	nodeConstraint* getNodeConstraint(MyNode *other);
	
	static Quaternion getVectorRotation(Vector3 v1, Vector3 v2);
	static float gv(Vector3 *v, int dim);
	static void sv(Vector3 *v, int dim, float val);
	
	static char* concat(int n, ...);
};

#endif

#include "T4TApp.h"

