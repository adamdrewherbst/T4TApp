#ifndef MODES_H_
#define MODES_H_

#include "gameplay.h"
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <stdlib.h>
#include <limits>
#include "T4TApp.h"

using std::cout;
using std::endl;

using namespace gameplay;

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

	int _x, _y; //mouse position wrt upper left of entire window, not just my button		
	MyNode *_selectedNode, *_touchNode;
	Vector3 _selectPoint, _touchPoint, _mousePoint;
	Vector2 _touchPix, _mousePix;
	Plane _plane;
	Ray _ray;
	//Base members remember the value from the time of the last TOUCH_PRESS event
	Camera *_cameraBase;
	T4TApp::cameraState *_cameraStateBase;
	Rectangle _viewportBase;
	
	Mode(const char* id);
	
	virtual bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	virtual void controlEvent(Control *control, EventType evt);
	virtual void setActive(bool active);
	virtual bool setSubMode(short mode);
	virtual bool setSelectedNode(MyNode *node, Vector3 point = Vector3::zero());
	virtual bool isSelecting();
	virtual void placeCamera();
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
	std::vector<short> keep; //-1 if discarding the vertex, otherwise its index in the new model's vertex list
	//edgeInt[edge vertex 1][edge vertex 2] = (tool plane number, index of intersection point in new model's vertex list)
	std::map<unsigned short, std::map<unsigned short, std::pair<unsigned short, unsigned short> > > edgeInt;
	std::pair<unsigned short, unsigned short> _tempInt;
	//toolInt[tool line #][model face #] = index of line-face intersection in new model's vertex list
	std::map<unsigned short, std::map<unsigned short, unsigned short> > toolInt;
	//new edges in tool planes
	std::map<unsigned short, std::map<unsigned short, unsigned short> > segmentEdges;
	short _hullSlice; //which convex hull segment we are working on
	
	void getEdgeInt(bool (ToolMode::*getInt)(unsigned short*, short*, float*));
	bool checkEdgeInt(unsigned short v1, unsigned short v2);
	void addToolEdge(unsigned short v1, unsigned short v2, unsigned short lineNum);
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
		StringMode *_mode;
		MyNode *_node;
		std::vector<unsigned short> _faces;
		std::vector<MyNode*> _outlines;
		
		NodeData(MyNode *node, StringMode *mode);
		void addFace(unsigned short face);
		void getOutlines();
	};
	std::vector<NodeData*> _nodes;
	Plane _stringPlane;
	
	StringMode();
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, Control::Listener::EventType evt);
	void makeString();
};

class ProjectComponent : public Mode
{
public:
	//each element of this component is positioned/finalized via touches => has its own touch callback
	typedef bool (ProjectComponent::*TouchCallback)(Touch::TouchEvent, int, int);

	//component is divided into elements, eg. a lever has a base and arm
	class Element {
		public:
		ProjectComponent *comp;
		std::string _name;
		bool _isStatic, _movable[3], _rotable[3];
		float _limits[3][2];
		short _moveRef;
		TouchCallback _callback;
		MyNode *_node;
		
		Element(ProjectComponent *comp_);
		void setMovable(bool x, bool y, bool z, short ref = -1);
		void setRotable(bool x, bool y, bool z);
		void setLimits(short axis, float lower, float upper);
		void applyLimits(Vector3 &translation);
	};
	std::vector<Element*> _elements;

	short _currentElement, _typeCount, _moveMode, _moveAxis;
	Quaternion _baseRotation;
	Vector3 _baseTranslation;

	//blank scene onto which to build the component
	std::string _sceneFile, _nodeId;
	MyNode *_rootNode; //parent node for this component

	ProjectComponent(const char* id);

	void setActive(bool active);
	void loadScene();
	virtual void releaseScene();
	void addElement(const char *name, TouchCallback callback, bool isStatic = false);
	Element* getEl();
	MyNode* getNode(short n = -1);
	void controlEvent(Control *control, EventType evt);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	virtual void placeElement() = 0; //position the element in space before it has physics attached
	virtual void finishElement() = 0; //post processing once the collision object is attached
	virtual void finishComponent();
};

class Lever : public ProjectComponent 
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

class Pulley : public ProjectComponent 
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

#endif

