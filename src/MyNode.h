#include "T4TApp.h"
#include "FileSystem.h"

using namespace gameplay;

class MyNode : public Node {

public:

	MyNode(const char *id);
	static MyNode* create(const char *id = NULL);
	void init();
	static MyNode* cloneNode(Node *node);

    struct nodeConstraint {
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
	static nodeData* readData(const char *filename);
	static void writeData(nodeData *data, const char *filename, Node *node = NULL);
	void writeMyData(const char *filename = NULL);
	void loadData(const char *filename = NULL);
	void updateData();
	void reloadFromData(const char *filename, bool addPhysics = true);
	void setNormals();
	Matrix getInverseRotTrans();
	short pt2Face(Vector3 point);
	Plane facePlane(unsigned short f, bool modelSpace = false);
	Vector3 faceCenter(unsigned short f, bool modelSpace = false);
	void rotateFaceToPlane(unsigned short f, Plane p);
	void rotateFaceToFace(unsigned short f, Node *other, unsigned short g);
	void addEdge(unsigned short e1, unsigned short e2);
	void addFace(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles);
	void addFaceHelper(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles);
	Vector3 getNormal(std::vector<unsigned short>& face, bool modelSpace = false);
	void triangulate(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles);
	void triangulateHelper(std::vector<unsigned short>& face, std::vector<unsigned short>& inds,
	  std::vector<std::vector<unsigned short> >& triangles, Vector3 normal);
	bool isStatic();
	void setStatic(bool stat);
	
	static Quaternion getVectorRotation(Vector3 v1, Vector3 v2);
	static float gv(Vector3 *v, int dim);
	static void sv(Vector3 *v, int dim, float val);
}

