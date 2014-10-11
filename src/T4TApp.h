#ifndef TEMPLATEGAME_H_
#define TEMPLATEGAME_H_

#include "gameplay.h"
#include "MyNode.h"
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <stdlib.h>

using namespace gameplay;

using std::cout;
using std::endl;

class T4TApp: public Game, Control::Listener, PhysicsCollisionObject::CollisionListener
{
public:

    T4TApp();
    
    T4TApp* getInstance();

	void generateModels();
	void generateModel(const char *type, ...);
	
	MyNode* loadNode(const char* id);    
    MyNode* duplicateModelNode(const char* type, bool isStatic = false);
    MyNode* createWireframe(std::vector<float>& vertices, const char *id=NULL);
    bool printNode(Node *node);
    bool prepareNode(MyNode *node);
    void translateNode(MyNode *node, Vector3 trans);
    PhysicsConstraint* addConstraint(MyNode *n1, MyNode *n2, int id, const char *type, ...);
    //misc functions
    const std::string pv(const Vector3& v);
    const std::string pv2(const Vector2& v);
    const std::string printQuat(Quaternion& q);

    void initScene();
    void setSceneName(const char *name);
	void loadScene(const char *scene = NULL);
	std::string getSceneDir();
	void clearScene();
	bool removeNode(Node *node);
	bool auxNode(Node *node);
	void saveScene(const char *scene = NULL);
	bool saveNode(Node *n);
	void releaseScene();
	void hideScene();
	bool hideNode(Node *node);
	void showScene();
	bool showNode(Node *node);
    void setActiveScene(Scene *scene);
    std::string _sceneName;

    Camera* getCamera();
    Node* getCameraNode();
    void setCameraEye(float radius, float theta, float phi);
    void setCameraZoom(float radius);
    void setCameraTarget(Vector3 target);
    void resetCamera();

    void addConstraints(MyNode *node);
    void removeConstraints(MyNode *node);
    void enableConstraints(MyNode *node, bool enable = true);
    void reloadConstraint(MyNode *node, MyNode::nodeConstraint *constraint);
     
    bool mouseEvent(Mouse::MouseEvent evt, int x, int y, int wheelDelta);
	void keyEvent(Keyboard::KeyEvent evt, int key);
    void touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
    void controlEvent(Control* control, Control::Listener::EventType evt);
    void collisionEvent(PhysicsCollisionObject::CollisionListener::EventType type,
    	const PhysicsCollisionObject::CollisionPair& pair, const Vector3& pointA, const Vector3& pointB);
	void debugTrigger();

    //add/remove listener to/from entire control tree
    void addListener(Control *control, Control::Listener *listener, int evtFlags = Control::Listener::CLICK);
    void removeListener(Control *control, Control::Listener *listener);
    void enableListener(bool enable, Control *control, Control::Listener *listener, int evtFlags = Control::Listener::CLICK);

    void initialize();
    void finalize();
    void update(float elapsedTime);
    void render(float elapsedTime);
    bool drawScene(Node* node);
    void placeNode(MyNode *node, float x, float y);
    void setMode(short mode);

    //see if the current touch coordinates intersect a given model in the scene
    bool checkTouchModel(Node* node);
    MyNode* getMouseNode(int x, int y, Vector3 *touch = NULL);
    
    //UI factory functions
    Form* addMenu(const char *name, Container *parent = NULL, const char *buttonText = NULL,
      Layout::Type layout = Layout::LAYOUT_VERTICAL);
    Form* addPanel(const char *name, Container *parent = NULL);
    template <class ButtonType> ButtonType* addButton(Container *parent, const char *name, const char *text = NULL);
    template <class ControlType> ControlType* addControl(Container *parent, const char *name, Theme::Style *style, const char *text = NULL);
    //other UI
    void promptComponent();
    void getText(const char *prompt, const char *type, void (T4TApp::*callback)(const char*));
    void doConfirm(const char *message, void (T4TApp::*callback)(bool));
    void showDialog(Container *dialog, bool show = true);
    void confirmDelete(bool yes);

	//scene setup
    Scene* _scene;
    Node* _lightNode;
    Light* _light;
    
    //T4T objects for modeling
    Scene *_models;
    std::vector<std::string> _modelNames;
    PhysicsVehicle *_carVehicle;
    float _steering, _braking, _driving;
    
    //for placing objects
    Node *_intersectModel;
    std::vector<MyNode*> _intersectNodeGroup;
    BoundingBox _intersectBox;
    Vector3 _intersectPoint;
    Plane _groundPlane;
    
    //each constraint in the simulation will have an integer ID for lookup
    std::map<int, PhysicsConstraint*> _constraints;
    int _constraintCount;
    
    //current state
    short _activeMode;
    bool _drawDebug;
    int _running;
    Scene *_activeScene;
    Vector3 _cameraCenter;

    //user interface
    Form *_mainMenu;
    Container *_sideMenu, *_stage, *_sceneMenu, *_componentMenu, *_machineMenu, *_modePanel,
      *_textDialog, *_confirmDialog, *_overlay;
    Label *_textPrompt, *_confirmMessage;
    TextBox *_textName;
    Button *_textSubmit, *_textCancel, *_confirmYes, *_confirmNo;
    std::vector<Container*> _submenus; //submenus
    CheckBox *_drawDebugCheckbox;
    std::vector<std::string> _modeNames, _machineNames;
    Theme *_theme;
    Theme::Style *_formStyle, *_buttonStyle, *_titleStyle, *_hiddenStyle;
    Font *_font;
    //callbacks
    void (T4TApp::*_textCallback)(const char*), (T4TApp::*_confirmCallback)(bool);
	
	class Mode : public Button, public Control::Listener
	{
public:
		T4TApp *app;
		Scene *_scene;
		
		std::vector<std::string> _subModes;
		short _subMode, _cameraMode;
		Container *_container, *_controls, *_subModePanel;
		Button *_cameraModeButton;
		bool _active, _touching, _doSelect;

		int _x, _y; //mouse position wrt upper left of entire window, not just my button		
		MyNode *_selectedNode, *_touchNode;
		Vector3 _selectPoint, _touchPoint, _mousePoint;
		Vector2 _touchPix, _mousePix;
		Plane _plane;
		Ray _ray;
		//Base members remember the value from the time of the last TOUCH_PRESS event
		Camera *_camera, *_cameraBase;
		Vector3 _cameraCenterBase;
		Rectangle _viewportBase;
		
		Mode(const char* id);
		
		virtual bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		virtual void controlEvent(Control *control, EventType evt);
		virtual void setActive(bool active);
		virtual void setSubMode(short mode);
		virtual void setCameraMode(short mode);
		virtual void setSelectedNode(MyNode *node, Vector3 point = Vector3::zero());
		virtual void placeCamera();
		virtual void draw();
	};
	std::vector<Mode*> _modes;
	
	class NavigateMode : public Mode
	{
public:
		NavigateMode();
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
		void setSubMode(short mode);
	};
	
	//generic mode for altering a model using a tool such as a knife or drill
	class ToolMode : public Mode
	{
public:
		struct Tool {
			std::string id;
			short type; //0 = saw, 1 = drill
			float *fparam; //any parameters this tool type requires
			int *iparam;
			Model *model; //representation of the tool
		};
		std::vector<std::vector<Tool*> > _tools; //my toolbox - each inner vector holds all bits for a single tool type
		short _currentTool;
		MyNode *_tool; //to display the tool to the user
		Container *_bitMenu;

		MyNode *_newNode; //a model to hold the modified node data
		MyNode::nodeData *data, *newData;
		Plane _viewPlane;
		Vector3 _touchStart, _touchPoint, _viewPlaneOrigin;
		Quaternion _toolBaseRotation;
		
		short usageCount;

		ToolMode();
		void createBit(short type, ...);
		void setActive(bool active);
		void setSelectedNode(MyNode *node);
		void setSubMode(short mode);
		void setTool(short n);
		Tool *getTool();
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
		void setAxis(int axis);
		void setView();
		bool toolNode();
		
		//for sawing
		bool sawNode();
		
		//for drilling
		bool drillNode();
		void addDrillEdge(unsigned short v1, unsigned short v2, unsigned short lineNum);
		std::map<unsigned short, std::map<unsigned short, unsigned short> > segmentEdges;
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
		void setSubMode(short mode);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
		void setSelectedNode(MyNode *node, Vector3 point);
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
		void setSubMode(short mode);
		
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
		MyNode *_face;
		
		TouchMode();
		void setActive(bool active);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
	};

	class ProjectComponent : public Mode
	{
public:
		//each element of this component is positioned/finalized via touches => has its own touch callback
		typedef bool (T4TApp::ProjectComponent::*TouchCallback)(Touch::TouchEvent, int, int);

    	//component is divided into elements, eg. a lever has a base and arm
    	struct Element {
    		std::string _name;
    		bool _isStatic;
    		TouchCallback _callback;
    		MyNode *_node;
    	};
    	std::vector<Element*> _elements;

    	int _currentElement, _typeCount;
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
		virtual void placeElement() = 0; //position the element in space before it has physics attached
		virtual void finishElement() = 0; //post processing once the collision object is attached
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void finishComponent();
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
	};
};

#endif
