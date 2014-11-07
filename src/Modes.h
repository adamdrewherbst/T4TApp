#ifndef MODES_H_
#define MODES_H_

#include "Meshy.h"

class MyNode;
class T4TApp;

class Mode : public Button, public Control::Listener
{
public:
	T4TApp *app;
	Scene *_scene;
	Camera *_camera;

	std::vector<std::string> _subModes;
	short _subMode;
	Container *_container, *_controls, *_subModePanel;
	bool _active, _touching, _doSelect;
	std::ostringstream os;

	int _x, _y; //mouse position wrt upper left of entire window, not just my button
	TouchPoint _touchPt;
	MyNode *_selectedNode, *_touchNode;
	Vector3 _selectPoint, _touchPoint, _mousePoint;
	Vector2 _touchPix, _mousePix;
	Plane _plane;
	Ray _ray;
	//Base members remember the value from the time of the last TOUCH_PRESS event
	Camera *_cameraBase;
	cameraState *_cameraStateBase;
	Rectangle _viewportBase;
	
	Mode(const char* id);
	
	virtual bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	virtual void controlEvent(Control *control, EventType evt);
	virtual void setActive(bool active);
	virtual bool setSubMode(short mode);
	virtual bool setSelectedNode(MyNode *node, Vector3 point = Vector3::zero());
	virtual bool isSelecting();
	virtual void placeCamera();
	virtual void update();
	virtual void draw();
	
};

class NavigateMode : public Mode
{
public:
	NavigateMode();
	void setActive(bool active);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
};

//for altering a model using a tool such as a knife or drill
class ToolMode : public Mode
{
public:
	static ToolMode *_instance;
	
	struct Tool : public Meshy {
		std::string id;
		short type; //0 = saw, 1 = drill
		float *fparam; //any parameters this tool type requires
		int *iparam;
		Model *model; //representation of the tool
	};
	std::vector<std::vector<Tool*> > _tools; //my toolbox - each inner vector holds all bits for a single tool type
	short _currentTool;
	//to display the tool to the user
	MyNode *_tool;
	Matrix _toolWorld, _toolInv, _toolNorm;
	//tool translation in xy-plane and rotation about z-axis in its local frame
	Vector2 _toolTrans;
	float _toolRot;
	
	Container *_moveMenu, *_bitMenu;
	short _moveMode;

	MyNode *_node, *_newNode; //a model to hold the modified node data
	Meshy *_mesh, *_newMesh; //whatever node/hull we are currently working on
	
	short usageCount;

	ToolMode();
	void createBit(short type, ...);
	void setActive(bool active);
	bool setSelectedNode(MyNode *node, Vector3 point = Vector3::zero());
	bool setSubMode(short mode);
	void setMoveMode(short mode);
	void setTool(short n);
	Tool *getTool();
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
	void placeCamera();
	void placeTool();
	bool toolNode();
	void setMesh(Meshy *mesh);
	
	//for sawing
	bool sawNode();
	bool getEdgeSawInt(unsigned short *e, short *lineInd, float *distance);
	
	//for drilling
	bool drillNode();
	bool drillCGAL();
	bool getEdgeDrillInt(unsigned short *e, short *lineInd, float *distance);
	bool getHullSliceInt(unsigned short *e, short *planeInd, float *distance);
	bool drillKeep(unsigned short n);
	
	//general purpose
	Ray _axis; //geometry of the tool
	std::vector<Ray> lines;
	std::vector<Plane> planes;
	Vector3 axis, right, up;
	std::vector<Vector3> toolVertices; //coords of model vertices in tool frame
	std::vector<Plane> toolPlanes; //model face planes in tool frame
	std::vector<short> keep; //-1 if discarding the vertex, otherwise its index in the new model's vertex list
	//edgeInt[edge vertex 1][edge vertex 2] = (tool plane number, index of intersection point in new model's vertex list)
	std::map<unsigned short, std::map<unsigned short, std::pair<unsigned short, unsigned short> > > edgeInt;
	std::pair<unsigned short, unsigned short> _tempInt;
	//toolInt[tool line #][model face #] = index of line-face intersection in new model's vertex list
	std::map<unsigned short, std::map<unsigned short, unsigned short> > toolInt;
	//new edges in tool planes
	std::map<unsigned short, std::map<unsigned short, unsigned short> > segmentEdges;
	short _lastInter; //for building edges on the tool surface
	short _faceNum, _hullNum, _hullSlice; //which convex hull segment we are working on
	std::map<unsigned short, unsigned short> _next; //new edges for the face currently being tooled
	
	void getEdgeInt(bool (ToolMode::*getInt)(unsigned short*, short*, float*));
	bool checkEdgeInt(unsigned short v1, unsigned short v2);
	void addToolEdge(unsigned short v1, unsigned short v2, unsigned short lineNum);
	void getNewFaces(std::map<unsigned short, unsigned short> &edges, Vector3 normal);
	short addToolInt(Vector3 &v, unsigned short line, unsigned short face, short segment = -1);
	void addToolFaces();
	
	//debugging
	void showFace(Meshy *mesh, std::vector<unsigned short> &face, bool world);
};

class PositionMode : public Mode
{
public:
	std::vector<std::string> _axisNames;
	short _axis;
	MyNode *_parentNode;
	float _positionValue;
    Quaternion _baseRotation;
    Vector3 _baseTranslation, _baseScale, _basePoint, _transDir, _normal, _planeCenter;
    Plane _plane;
    Vector2 _dragOffset;
    short _groundFace;
    
	Button *_axisButton;
    Slider *_gridSlider, *_valueSlider;
    CheckBox *_gridCheckbox, *_staticCheckbox;

	PositionMode();
	void setActive(bool active);
	bool setSubMode(short mode);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
	bool setSelectedNode(MyNode *node, Vector3 point);
	void setAxis(short axis);
	void setPosition(float value, bool finalize = false);
	void updateSlider();
};

class ConstraintMode : public Mode {
public:
	std::vector<std::string> _constraintTypes;
	MyNode *_nodes[2];
	short _faces[2], _currentNode;
	Quaternion _rot[2];
	Vector3 _trans[2];
	
	ConstraintMode();
	void setActive(bool active);
	bool setSubMode(short mode);
	
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
};

class TestMode : public Mode
{
public:
	TestMode();
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
};

class TouchMode : public Mode
{
public:
	MyNode *_face, *_vertex;
	CheckBox *_hullCheckbox;
	
	TouchMode();
	void setActive(bool active);
	bool setSubMode(short mode);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
};

class StringMode : public Mode
{
public:
	class NodeData {
		public:
		StringMode *_mode;
		MyNode *_node;
		Vector3 _point;
		std::vector<Vector3> _planeVertices;
		Vector3 _planeCenter; //node's center in string plane coords
		std::vector<Vector2> _outline, _hull;
		
		NodeData(MyNode *node, StringMode *mode);
		void updateVertices();
		bool getOutline();
		bool getHull();
	};
	std::vector<NodeData*> _nodes;
	Plane _stringPlane;
	Vector3 _axis, _up, _normal, _origin;
	std::vector<Vector2> _path;
	MyNode *_linkTemplate, *_stringTemplate; //static templates to copy each time
	MyNode *_pathNode, *_stringNode; //visual rep of path, and actual string
	std::vector<MyNode*> _links;
	float _linkLength, _linkWidth;
	bool _enabled;
	
	StringMode();
	void setActive(bool active);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
	void addNode(MyNode *node, Vector3 point);
	bool getPlane();
	bool getPath();
	void clearPath();
	void makeString();
	void enableString(bool enable);
	Vector3 plane2World(Vector2 &v);
	Vector3 plane2Vec(Vector2 &v);
};

class Project : public Mode
{
public:
	//component is divided into elements, eg. a lever has a base and arm
	class Element {
		public:
		T4TApp *app;
		Project *_project;
		std::string _id, _name;
		bool _static, _multiple, _movable[3], _rotable[3];
		float _limits[3][2];
		short _moveRef, _numNodes;
		std::vector<std::shared_ptr<MyNode> > _nodes;
		const char *_currentNodeId;
		Element *_parent;
		Plane _plane;
		TouchPoint _parentTouch, _planeTouch;
		std::vector<Element*> _children;
		
		Element(Project *project, const char *id, const char *name = NULL, Element *parent = NULL);
		void setMovable(bool x, bool y, bool z, short ref = -1);
		void setRotable(bool x, bool y, bool z);
		void setLimits(short axis, float lower, float upper);
		void setPlane(const Plane &plane);
		void applyLimits(Vector3 &translation);
		void setParent(Element *parent);
		void addChild(Element *child);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		MyNode* getNode(short n = 0);
		void setNode(const char *id);
		void addNode(const Vector3 &position);
		virtual void placeNode(const Vector3 &position, short n = 0);
		virtual void addPhysics(short n = -1);
	};
	std::vector<std::shared_ptr<Element> > _elements;

	short _currentElement, _typeCount, _moveMode, _moveAxis;
	Quaternion _baseRotation;
	Vector3 _baseTranslation;

	//blank scene onto which to build the component
	std::string _sceneFile, _nodeId;
	MyNode *_rootNode; //parent node for this component

	Container *_elementContainer;

	bool _inSequence; //true during the first run-through to add all the elements
	bool _testing; //when in Test mode

	Project(const char* id);

	virtual void setupMenu();
	void setActive(bool active);
	Element* addElement(Element *element);
	Element* getEl(short n = -1);
	MyNode* getNode(short n = -1);
	void controlEvent(Control *control, EventType evt);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void promptNextElement();
	virtual void finish();
	virtual void addPhysics();
	virtual void test();
};

class Buggy : public Project {
public:
	class Body : public Project::Element {
		public:
		Body(Project *project);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	};

	class Axle : public Project::Element {
		public:
		Axle(Project *project, const char *id, const char *name, Element *parent);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void placeNode(const Vector3 &position, short n);
		void addPhysics(short n);
	};

	class Wheels : public Project::Element {
		public:
		Wheels(Project *project, const char *id, const char *name, Element *parent);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void placeNode(const Vector3 &position, short n);
		void addPhysics(short n);
	};
	
	Element *_body, *_frontAxle, *_rearAxle, *_frontWheels, *_rearWheels;
	MyNode *_ramp;
	float _rampSlope;
	bool _launched;
	Button *_launchButton;

	Buggy();
	void setupMenu();
	void setActive(bool active);
	void controlEvent(Control *control, EventType evt);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void test();
	void setRampHeight(float scale);
};

class Lever : public Project
{
public:
	PhysicsHingeConstraint *_armConstraint;

	Lever();
	bool baseTouch(Touch::TouchEvent evt, int x, int y);
	bool armTouch(Touch::TouchEvent evt, int x, int y);
	void placeElement();
	void finishElement();
	void finishComponent();
};

class Pulley : public Project
{
public:
	float _radius, _linkLength, _linkWidth;
	unsigned short _wheelLinks, _dropLinks, _numLinks;
	std::vector<PhysicsSocketConstraint*> _constraints;
	
	Pulley();
	bool baseTouch(Touch::TouchEvent evt, int x, int y);
	bool wheelTouch(Touch::TouchEvent evt, int x, int y);
	bool bucketTouch(Touch::TouchEvent evt, int x, int y);
	void placeElement();
	void finishElement();
	void finishComponent();
};

class Rocket : public Project
{
public:
	//menu of balloons
	Container *_balloonMenu;
	std::vector<std::shared_ptr<MyNode> > _balloons;
	short _balloonType;
	//balloons in the simulation
	class Balloon {
		public:
		Rocket *_rocket;
		std::shared_ptr<MyNode> _balloonNode, _anchorNode;
		float _balloonRadius, _anchorRadius, _scale = 1;
		
		Balloon();
		Balloon(Rocket *rocket);
		void setScale(float scale);
	};
	std::vector<Balloon> _balloonNodes;
	float _strawRadius, _strawLength, _originalStrawLength;
	MyNode *_strawNode;
	PhysicsConstraint *_strawConstraint;
	bool _launching;
	
	Rocket();
	void setActive(bool active);
	bool setSubMode(short mode);
	void setStrawConstraint();
	void update();
	void controlEvent(Control *control, EventType evt);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
};

#endif


