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

/**
 * Main game class.
 */
class T4TApp: public Game, Control::Listener, PhysicsCollisionObject::CollisionListener
{
public:

    /**
     * Constructor.
     */
    T4TApp();
    
    T4TApp* getInstance();

	MyNode* loadNode(const char* id);    
    MyNode* duplicateModelNode(const char* type, bool isStatic = false);
    MyNode* createWireframe(std::vector<float>& vertices, char *id=NULL);
    bool printNode(Node *node);
    bool prepareNode(MyNode *node);
    void translateNode(MyNode *node, Vector3 trans);
    PhysicsConstraint* addConstraint(MyNode *n1, MyNode *n2, int id, const char *type, ...);
    //misc functions
    const std::string printVector(const Vector3& v);
    const std::string printVector2(const Vector2& v);
    const std::string printQuat(Quaternion& q);
	void loadScene();
	void releaseScene();
	void hideScene();
	void showScene();
	bool hideNode(Node *node);
	bool showNode(Node *node);
    void setActiveScene(Scene *scene);
    void addConstraints(MyNode *node);
    void removeConstraints(MyNode *node);
    void enableConstraints(MyNode *node, bool enable = true);
    void reloadConstraint(MyNode *node, MyNode::nodeConstraint *constraint);
    
    /**
     * @see Game::keyEvent
     */
     
    bool mouseEvent(Mouse::MouseEvent evt, int x, int y, int wheelDelta);
	void keyEvent(Keyboard::KeyEvent evt, int key);
	void debugTrigger();
    
    /**
     * @see Game::touchEvent
     */
    void touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
    
    void controlEvent(Control* control, Control::Listener::EventType evt);
    
    void enableScriptCamera(bool enable);

    void collisionEvent(PhysicsCollisionObject::CollisionListener::EventType type,
    	const PhysicsCollisionObject::CollisionPair& pair, const Vector3& pointA, const Vector3& pointB);

//protected:

    /**
     * @see Game::initialize
     */
    void initialize();

    /**
     * @see Game::finalize
     */
    void finalize();

    /**
     * @see Game::update
     */
    void update(float elapsedTime);

    /**
     * @see Game::render
     */
    void render(float elapsedTime);

//private:

    /**
     * Draws the scene each frame.
     */
    bool drawScene(Node* node);
    void placeNode(MyNode *node, float x, float y);
    void setMode(const char *mode);

    //see if the current touch coordinates intersect a given model in the scene
    bool checkTouchModel(Node* node);
    BoundingBox getWorldBox(Node *node);
    MyNode* getMouseNode(int x, int y, Vector3 *touch = NULL);
    
    //UI factory functions
    Form* addMenu(Form *parent, const char *name);
    Form* addPanel(Form *parent, const char *name);
    template <class ButtonType> ButtonType* addButton(Form *menu, const char *name, const char *text = NULL);
    template <class ControlType> ControlType* addControl(Form *parent, const char *name, Theme::Style *style, const char *text = NULL);
    
    //standard projects
    void promptComponent();

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
    std::string _mode;
    bool _drawDebug;
    int _running;
    Scene *_activeScene;

    //user interface
    Form *_mainMenu, *_sideMenu, *_itemContainer, *_machineContainer, *_modeContainer, *_modePanel, *_componentMenu;
    std::vector<Form*> _submenus; //submenus
    Button *_itemButton, *_modeButton, *_machineButton; //submenu handles
    std::map<std::string, Form*> _modeOptions;
    CheckBox *_drawDebugCheckbox;
    Slider *_zoomSlider;
    std::vector<std::string> _modeNames, _machineNames;
    Theme *_theme;
    Theme::Style *_formStyle, *_buttonStyle, *_titleStyle, *_hiddenStyle;
	
	class ProjectComponent : public Button, Control::Listener
	{
public:
		T4TApp *app; //containing app
    	Form *_container; //wrapper for the full-screen button that handles touch events for this component
    	//component is divided into elements, eg. a lever has a base and arm
    	std::vector<std::string> _elementNames;
    	int _currentElement, _typeCount;
    	std::vector<bool> _isStatic; //whether this element should be immovable
    	//blank scene onto which to build the component
    	std::string _sceneFile;
    	Scene *_scene;
    	MyNode *_node; //parent node for this component
    	std::vector<MyNode*> _allNodes;

		//each element of this component is positioned/finalized via touches => has its own touch callback
		typedef bool (T4TApp::ProjectComponent::*TouchCallback)(Touch::TouchEvent, int, int);
    	std::vector<T4TApp::ProjectComponent::TouchCallback> _elementCallbacks;

    	ProjectComponent(T4TApp *app_, const char* filename, const char* id, Theme::Style* buttonStyle, Theme::Style* formStyle);

		void setActive(bool active);
		void loadScene();
		virtual void releaseScene();
		void addElement(const char *name, bool (T4TApp::ProjectComponent::*)(Touch::TouchEvent evt, int x, int y),
				bool isStatic = false);
		void controlEvent(Control *control, EventType evt);
		virtual void placeElement(MyNode *node) = 0; //position the element in space before it has physics attached
		virtual void finishElement(MyNode *node) = 0; //post processing once the collision object is attached
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void finishComponent();
	};
	
	class Lever : public ProjectComponent 
	{
public:
		PhysicsHingeConstraint *_armConstraint;

		Lever(T4TApp *app_, Theme::Style* buttonStyle, Theme::Style* formStyle);
		bool baseTouch(Touch::TouchEvent evt, int x, int y);
		bool armTouch(Touch::TouchEvent evt, int x, int y);
		void placeElement(MyNode *node);
		void finishElement(MyNode *node);
		void releaseScene();
	};

	class Pulley : public ProjectComponent 
	{
public:
		float _radius, _linkLength, _linkWidth;
		unsigned short _wheelLinks, _dropLinks, _numLinks;
		std::vector<PhysicsSocketConstraint*> _constraints;
		
		Pulley(T4TApp *app_, Theme::Style* buttonStyle, Theme::Style* formStyle);
		bool baseTouch(Touch::TouchEvent evt, int x, int y);
		bool wheelTouch(Touch::TouchEvent evt, int x, int y);
		bool bucketTouch(Touch::TouchEvent evt, int x, int y);
		void placeElement(MyNode *node);
		void finishElement(MyNode *node);
		void releaseScene();
	};

	std::vector<ProjectComponent*> _machines;
	
	class Mode : public Button, Control::Listener
	{
public:
		T4TApp *app;
		Form *_container, *_controls; //the click overlay and control panel
		std::vector<Control*> _subControls; //the actual buttons etc.
		bool _active, _touching;
		
		Mode(T4TApp *app_, const char* id, const char* filename = NULL);
		
		virtual bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		virtual void controlEvent(Control *control, EventType evt) = 0;
		virtual void setActive(bool active);
		virtual void draw();
	};
	std::vector<Mode*> _modes;
	
	//generic mode for altering a model using a tool such as a knife or drill
	class ToolMode : public Mode
	{
public:
		std::string _toolType;
		MyNode *_node, *_newNode; //the model to be altered and a model to hold the modified node data
		MyNode::nodeData *data, *newData;
		Plane _viewPlane;
		Vector3 _touchStart, _touchPoint, _viewPlaneOrigin;
		MyNode *_tool; //to display the tool to the user
		Quaternion _toolBaseRotation;
		int _subMode; //0 = rotate, 1 = translate
		bool _touching, _rotate;
		
		short usageCount;

		ToolMode(T4TApp *app_, const char* id, const char* filename = NULL);
		virtual void setActive(bool active);
		void setNode(MyNode *node);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		virtual void controlEvent(Control *control, Control::Listener::EventType evt);
		virtual void setAxis(int axis);
		void setView();
		virtual bool toolNode();
	};

	class SliceMode : public ToolMode
	{
public:
		Plane _slicePlane;

		SliceMode(T4TApp *app_);
		void setAxis(int axis);
		bool toolNode();
	};

	class DrillMode : public ToolMode
	{
public:
		Ray _axis;
		float _radius;
		int _segments;
		//store all the lines and planes of the drill bit
		std::vector<Ray> lines;
		std::vector<Plane> planes;
		//contiguous patches of the surface based on alignment of normal with drill
		std::vector<Vector3> drillVertices; //coords of model vertices wrt drill axis
		//edgeInt[edge vertex 1][edge vertex 2] = (drill ray number, index of intersection point in new model's vertex list)
		//drillInt[drill ray number][face index in old model] = index of intersection point in new model's vertex list
		std::map<unsigned short, std::map<unsigned short, unsigned short> > drillInt;
		std::map<unsigned short, std::map<unsigned short, std::pair<unsigned short, unsigned short> > > edgeInt;
		//for each edge in a drill plane, which drill plane it is in
		std::map<unsigned short, std::map<unsigned short, unsigned short> > edgeLine;
		//for each edge in a drill plane, whether the interior of the object is to the left of the edge
		std::map<unsigned short, std::map<unsigned short, bool> > leftEdge;
		//for each drill line intersection point, whether the interior of the object is in the forward drill axis direction
		std::map<unsigned short, bool> enterInt;
		//all the directed edges in each segment of the drill
		std::map<unsigned short, std::map<unsigned short, unsigned short> > segmentEdges;
		std::vector<unsigned short> newEdge;

		//GUI
		Form *_bitMenu;
		void controlEvent(Control *control, Control::Listener::EventType evt);
		void setActive(bool active);
		void draw();

		DrillMode(T4TApp *app_);
		void setAxis(int axis);
		bool toolNode();
		void partitionNode();
		void buildPatch(unsigned short face);
		short occlusion(unsigned short f1, unsigned short f2);
		void addEdge(unsigned short e1, unsigned short e2);
		void addDrillEdge(unsigned short v1, unsigned short v2, unsigned short lineNum);
		void addFace(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles);
		void addFaceHelper(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles);
		Vector3 getNormal(std::vector<unsigned short>& face);
		void triangulate(std::vector<unsigned short>& face, std::vector<std::vector<unsigned short> >& triangles);
		void triangulateHelper(std::vector<unsigned short>& face, std::vector<unsigned short>& inds, 
		  std::vector<std::vector<unsigned short> >& triangles, Vector3 normal);
		unsigned short vertexClass(std::vector<unsigned short>& face, Vector3& normal, unsigned short index,
		  std::vector<std::pair<unsigned short, unsigned short> >& classes);
		unsigned short vertexClass(std::vector<unsigned short>& face, int i,
		  std::vector<std::pair<unsigned short, unsigned short> > classes);
		void drawFace(int face);
	};
	
	class RotateMode : public Mode
	{
public:
		bool _rotate;
		
		RotateMode(T4TApp *app_);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
	};
	
	class PositionMode : public Mode
	{
public:
		std::vector<std::string> _subModes, _axisNames;
		unsigned short _subMode;
		short _axis;
		MyNode *_selectedNode, *_parentNode;
		float _positionValue;
	    Quaternion _baseRotation;
	    Vector3 _baseTranslation, _baseScale, _basePoint, _transDir, _normal, _planeCenter;
	    Plane _plane;
	    Vector2 _dragOffset;
	    short _groundFace;
	    
		Button *_subModeButton, *_axisButton;
	    Slider *_gridSlider, *_valueSlider;
	    CheckBox *_gridCheckbox, *_staticCheckbox;
	    bool _dragging;

		PositionMode(T4TApp *app_);
		void setActive(bool active);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
		void setSelectedNode(MyNode *node);
		void setSubMode(unsigned short mode);
		void setAxis(short axis);
		void setPosition(float value, bool finalize = false);
		void updateSlider();
	};
	
	class ConstraintMode : public Mode {
public:
		std::vector<std::string> _subModes, _constraintTypes;
		unsigned short _subMode;
		MyNode *_nodes[2];
		short _faces[2], _currentNode;
		Quaternion _rot[2];
		Vector3 _trans[2];
		
		ConstraintMode(T4TApp *app_);
		void setActive(bool active);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
	};
	
	class TestMode : public Mode
	{
public:
		TestMode(T4TApp *app_);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
	};
	
	class TouchMode : public Mode
	{
public:
		MyNode *_face;
		
		TouchMode(T4TApp *app_);
		void setActive(bool active);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
	};
		
    Vector2 _mousePoint, _touchPoint;
    std::string _mouseString;
    Font* _font;
    
    //debugging variables
    bool _physicsStopped;
};

#endif
