#ifndef TEMPLATEGAME_H_
#define TEMPLATEGAME_H_

#include "gameplay.h"
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
    
    Node* duplicateModelNode(const char* type, bool isStatic = false);
    void addCollisionObject(Node *node);
	Node* loadNodeFromData(const char *nodeID);
    void changeNodeModel(Node *node, const char* type);
    bool printNode(Node *node);
    bool prepareNode(Node *node);
    void translateNode(Node *node, Vector3 trans);
    PhysicsConstraint* addConstraint(Node *n1, Node *n2, const char *type, ...);
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
    void removeNode(Node *node, const char *newID = NULL);
    void addConstraints(Node *node);
    void removeConstraints(Node *node);
    
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
    void setSelected(Node* node);
    void placeNode(Node *node, float x, float y);
    void setMode(const char *mode);

    //see if the current touch coordinates intersect a given model in the scene
    bool checkTouchModel(Node* node);
    bool checkTouchEdge(Node* node);
    Node* getMouseNode(int x, int y, Vector3 *touch = NULL);
    
    //UI factory functions
    Form* addMenu(Form *parent, const char *name);
    Form* addPanel(Form *parent, const char *name);
    template <class ButtonType> ButtonType* addButton(Form *menu, const char *name, const char *text = NULL);
    template <class ControlType> ControlType* addControl(Form *parent, const char *name, Theme::Style *style, const char *text = NULL);
    
    //standard projects
    void buildVehicle();
    Node* promptComponent();

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
    Node *_lastNode, *_intersectNode, *_intersectModel;
    const BoundingBox *_intersectBox;
    Vector3 _intersectPoint;
    Plane _groundPlane;
    
    //for creating physical constraints between objects
    Node* _constraintNodes[2];
    unsigned short _constraintEdges[2];
    std::map<Node*, std::vector<PhysicsConstraint*> > _constraints;
    
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
    Button *_vehicleButton;
    CheckBox *_drawDebugCheckbox;
    Slider *_zoomSlider;
    std::vector<std::string> _modeNames, _machineNames;
    Theme *_theme;
    Theme::Style *_formStyle, *_buttonStyle, *_titleStyle, *_hiddenStyle;
    
    //use Control::Listener instances and enumerated lists of required components 
    //for progressing through standard project scripts
    class VehicleProject : public Button, Control::Listener {
public:
    	T4TApp *app;
    	Form *container;
    	std::vector<std::string> componentName;
    	Scene *_scene;
    	Node *_chassisNode, *_frontWheels[2], *_backWheels[2];
    	Node *_allNodes[5], *_carNode;

		void controlEvent(Control *control, EventType evt);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	    bool disableObject(Node* node);
		bool hideNode(Node* node);
		bool showNode(Node* node);
		void loadScene();
		void releaseScene();
		
    	VehicleProject(T4TApp *app_, const char* id, Theme::Style* buttonStyle, Theme::Style* formStyle);
    	
		typedef enum {
			CHASSIS,
			FRONT_WHEELS,
			BACK_WHEELS,
			COMPLETE
		} VehicleComponent;

		VehicleComponent _currentComponent;
		
		void advanceComponent() {
			switch(_currentComponent) {
				case CHASSIS: _currentComponent = FRONT_WHEELS; break;
				case FRONT_WHEELS: _currentComponent = BACK_WHEELS; break;
				case BACK_WHEELS: _currentComponent = COMPLETE; break;
				default: break;
			}
		}
		void setActive(bool active);
	};
	VehicleProject *_vehicleProject;
	
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
    	std::vector<Node*> _allNodes;

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
		virtual void placeElement(Node *node) = 0; //position the element in space before it has physics attached
		virtual void finishElement(Node *node) = 0; //post processing once the collision object is attached
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
		void placeElement(Node *node);
		void finishElement(Node *node);
		void releaseScene();
	};
	std::vector<ProjectComponent*> _machines;
	
	class Mode : public Button, Control::Listener
	{
public:
		T4TApp *app;
		Form *_container, *_controls;
		bool _active;
		
		Mode(T4TApp *app_, const char* id, const char* filename = NULL);
		
		virtual bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) = 0;
		virtual void controlEvent(Control *control, EventType evt) = 0;
		virtual void setActive(bool active);
	};
	std::vector<Mode*> _modes;
	
	class SliceMode : public Mode
	{
public:
		Node *_node; //the object to be sliced
		Plane _slicePlane, _viewPlane;
		Vector3 _touchStart, _touchPoint, _viewPlaneOrigin;
		Node *_knife; //to display the slice plane to the user
		Quaternion _knifeBaseRotation;
		int _subMode; //0 = rotate, 1 = translate
		bool _touching;

		SliceMode(T4TApp *app_);
		void setActive(bool active);
		void setNode(Node *node);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
		void setAxis(int axis);
		void setView();
		bool sliceNode();
	};
	
	class RotateMode : public Mode
	{
public:
		bool _rotate;
		
		RotateMode(T4TApp *app_);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
	};
	
	class SelectMode : public Mode
	{
public:
		Node *_selectedNode;
		std::vector<Node*> _nodeGroup;
		std::vector<Vector3> _groupOffset;
	    Vector2 _dragOffset;
	    Slider *_gridSlider;
	    CheckBox *_gridCheckbox;

		SelectMode(T4TApp *app_);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		void controlEvent(Control *control, Control::Listener::EventType evt);
	};
	
    class TouchPoint
    {
    public:
        unsigned int _id;
        Vector2 _coord;
        bool _isStale;
    };

    std::list<TouchPoint> _touchPoints;
    Vector2 _mousePoint, _touchPoint;
    std::string _mouseString;
    Font* _font;
    
    //debugging variables
    bool _physicsStopped;
    PhysicsRigidBody *_lastBody;
	
};

#endif
