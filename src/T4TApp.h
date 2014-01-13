#ifndef TEMPLATEGAME_H_
#define TEMPLATEGAME_H_

#include "gameplay.h"
#include <cstring>
#include <cstdio>
#include <cstdarg>

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
    void changeNodeModel(Node *node, const char* type);
    bool printNode(Node *node);
    bool prepareNode(Node *node);
    //misc functions
    const std::string printVector(Vector3& v);
    const std::string printVector2(Vector2& v);
    void setActiveScene(Scene *scene);
    
    /**
     * @see Game::keyEvent
     */
     
    bool mouseEvent(Mouse::MouseEvent evt, int x, int y, int wheelDelta);

	void keyEvent(Keyboard::KeyEvent evt, int key);
    
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
    void placeSelected(float x, float y);
    void setMode(const char *mode);

    //see if the current touch coordinates intersect a given model in the scene
    bool checkTouchModel(Node* node);
    bool checkTouchEdge(Node* node);
    
    //model factory functions
    void loadNodeData(Node *node, const char *type);
    char* concat(int n, ...);
    
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
    PhysicsVehicle *_carVehicle;
    float _steering, _braking, _driving;
    
    //for placing objects
    Node *_selectedNode, *_lastNode, *_intersectModel;
    const BoundingBox* _selectedBox;
    Plane _groundPlane;
    Vector3 _intersectPoint;
    Vector2 _dragOffset;
    
    //for creating physical constraints between objects
    Node* _constraintNodes[2];
    unsigned short _constraintEdges[2];
    
    //current state
    std::string _mode;
    bool _drawDebug;
    Scene *_activeScene;

    //user interface
    Form *_mainMenu, *_sideMenu, *_itemContainer, *_modeContainer, *_componentMenu;
    std::vector<Form*> *_submenus; //submenus
    std::map<std::string, Form*> _modeOptions;
    Button *_itemButton, *_modeButton; //submenu handles
    Button *_vehicleButton;
    CheckBox *_gridCheckbox, *_drawDebugCheckbox;
    Slider *_gridSlider, *_zoomSlider;
    std::vector<std::string> *_modeNames;
    std::vector<RadioButton*> *_modeButtons;
    Theme *_theme;
    Theme::Style *_formStyle, *_buttonStyle, *_titleStyle, *_hiddenStyle;
    Button *_clickOverlay;
    Form *_clickOverlayContainer;
    
    //use Control::Listener instances and enumerated lists of required components 
    //for progressing through standard project scripts
    class VehicleProject : public Button, Control::Listener {
    
public:
    	T4TApp *app;
    	Form *container;
    	std::vector<std::string> componentName;
    	Scene *_scene;
    	Node *_chassisNode, *_frontWheels[2], *_backWheels[2];

    	VehicleProject(T4TApp *app_, const char* id, Theme::Style* buttonStyle, Theme::Style* formStyle) : app(app_) {
    	
    		_currentComponent = CHASSIS;
    		componentName.push_back("Chassis");
    		componentName.push_back("Front Wheels");
    		componentName.push_back("Back Wheels");
    		componentName.push_back("Complete");

			//create the form to hold this button
    		container = Form::create(app->concat(2, "container_", id), formStyle, Layout::LAYOUT_VERTICAL);
    		container->setPosition(app->_sideMenu->getX(), 0.0f);
    		container->setWidth(app->getWidth() - container->getX());
    		container->setAutoHeight(true);
    		container->setScroll(Container::SCROLL_VERTICAL);
			container->setConsumeInputEvents(true);
    		container->setVisible(false);
    		
    		_id = id;
    		_style = buttonStyle;
    		setAutoWidth(true);
    		setAutoHeight(true);
    		setConsumeInputEvents(true);
    		container->addControl(this);
    		app->_mainMenu->addControl(container);
    		
    		_scene = Scene::load("res/common/vehicle.scene");
    		_scene->setId("vehicle");
   		    _scene->getActiveCamera()->setAspectRatio(app->getAspectRatio());
    		//scene is inactive until called upon to build a vehicle project
    		for(Node *node = _scene->getFirstNode(); node != NULL; node = node->getNextSibling()) {
    			PhysicsCollisionObject *obj = node->getCollisionObject();
    			if(obj) obj->setEnabled(false);
    		}
    		_chassisNode = _scene->findNode("carbody");
    		_frontWheels[0] = _scene->findNode("wheelFrontLeft");
    		_frontWheels[1] = _scene->findNode("wheelFrontRight");
    		_backWheels[0] = _scene->findNode("wheelBackLeft");
    		_backWheels[1] = _scene->findNode("wheelBackRight");
    		
    		//camera will always be pointing down the z axis from 20 units away
    		Node *cameraNode = _scene->getActiveCamera()->getNode();
    		Matrix lookAt;
    		Vector3 scale, translation;
    		Quaternion rotation;
    		Matrix::createLookAt(Vector3(0.0f,2.0f,20.0f), Vector3(0.0f,0.0f,0.0f), Vector3(0.0f,1.0f,0.0f), &lookAt);
    		lookAt.invert();
    		lookAt.decompose(&scale, &rotation, &translation);
    		cameraNode->setScale(scale);
    		cameraNode->setRotation(rotation);
    		cameraNode->setTranslation(translation);
    		cout << "set vehicle cam scale = " << app->printVector(scale) << ", translation = " << app->printVector(translation);
    		cout << ", rotation = " << rotation.x << "," << rotation.y << "," << rotation.z << "," << rotation.w << endl;
    	}
    	
    	static VehicleProject *create(T4TApp *app_, const char* id, Theme::Style* buttonStyle, Theme::Style* formStyle) {

    		VehicleProject *v = new VehicleProject(app_, id, buttonStyle, formStyle);
    		return v;
    	}

		void controlEvent(Control *control, EventType evt);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);

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
	
/*	class OverlayPad : public Button
	{
public:
		T4TApp *app;
		OverlayPad::OverlayPad() {}
		void setApp(T4TApp *app_) { app = app_; }
		void touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	};//*/
    
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
