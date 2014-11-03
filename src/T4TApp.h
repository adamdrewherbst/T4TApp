#ifndef TEMPLATEGAME_H_
#define TEMPLATEGAME_H_

#include "gameplay.h"
#include "MyNode.h"

using namespace gameplay;

class Mode;

class T4TApp: public Game, public Control::Listener, public PhysicsCollisionObject::CollisionListener
{
public:

	//scene setup
    Scene* _scene;
    Node* _lightNode;
    Light* _light;

	//the functionality of the various interactive modes    
	std::vector<Mode*> _modes;

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
    short _navMode; //-1 = inactive, 0 = rotate, 1 = translate, 2 = zoom - overrides currently active mode when active
    bool _drawDebug;
    int _running;
    Scene *_activeScene;

    //history
    struct cameraState {
    	MyNode *node;
    	float radius, theta, phi;
    	Vector3 target;
    };
    cameraState *_cameraState;
    std::vector<cameraState*> _cameraHistory;
    //undo/redo
    struct Action {
    	std::string type;
    	//the node(s) that were acted upon and a reference for each one, holding any info needed to revert the action
    	std::vector<MyNode*> nodes, refNodes;
    };
    Action *_action; //the current, uncommitted action
    std::vector<Action*> _history, _undone; //queue of committed actions and those undone since the last commit
    MyNode *_tmpNode; //for swapping info between nodes
    int _tmpCount; //number of nodes whose info has been temporarily saved to disk

    //user interface
    Form *_mainMenu;
    Container *_sideMenu, *_stage, *_sceneMenu, *_componentMenu, *_machineMenu, *_modePanel,
      *_textDialog, *_confirmDialog, *_overlay, *_cameraMenu;
    Label *_textPrompt, *_confirmMessage;
    TextBox *_textName;
    Button *_textSubmit, *_textCancel, *_confirmYes, *_confirmNo, *_undo, *_redo;
    std::vector<Container*> _submenus; //submenus
    CheckBox *_drawDebugCheckbox;
    std::vector<std::string> _modeNames, _machineNames;
    Theme *_theme;
    Theme::Style *_formStyle, *_buttonStyle, *_titleStyle, *_hiddenStyle;
    Font *_font;
    //callbacks
    void (T4TApp::*_textCallback)(const char*), (T4TApp::*_confirmCallback)(bool);
    
    //debugging
    Meshy *_debugMesh;
    std::vector<unsigned short> _debugFace;
    short _debugEdge, _debugVertex; //debugEdge is index within debugFace, debugVertex is index in debugMesh vertex list
    bool _debugWorld; //using world space or model space coords?
   	MyNode *_face, *_edge, *_vertex;

    T4TApp();
    T4TApp* getInstance();
	void generateModels();
	void generateModel(const char *id, const char *type, ...);
	
	MyNode* loadNode(const char* id);
    MyNode* duplicateModelNode(const char* type, bool isStatic = false);
    MyNode* addModelNode(const char *type);
    Model* createModel(std::vector<float> &vertices, bool wireframe = false, const char *material = "red", Node *node = NULL);
    MyNode* createWireframe(std::vector<float>& vertices, const char *id=NULL);
	MyNode* dropBall(Vector3 point);
	void showFace(Meshy *mesh, std::vector<unsigned short> &face, bool world = false);
	void showFace(Meshy *mesh, std::vector<Vector3> &face);
	void showEdge(short e);
	void showVertex(short v);
	void showEye(float radius, float theta, float phi);
    bool printNode(Node *node);
    bool prepareNode(MyNode *node);
    void translateNode(MyNode *node, Vector3 trans);
    PhysicsConstraint* addConstraint(MyNode *n1, MyNode *n2, int id, const char *type,
      Quaternion &rot1, Vector3 &trans1, Quaternion &rot2, Vector3 &trans2, bool parentChild = false);
    PhysicsConstraint* addConstraint(MyNode *n1, MyNode *n2, int id, const char *type,
      const Vector3 &joint, const Vector3 &direction, bool parentChild = false);
    //misc functions
    const std::string pv(const Vector3& v);
    const std::string pv2(const Vector2& v);
    const std::string pq(const Quaternion& q);
    const std::string pcam(cameraState *state);

    void initScene();
    void setSceneName(const char *name);
	void loadScene(const char *scene = NULL);
	std::string getSceneDir();
	void clearScene();
	void removeNode(MyNode *node, bool erase = true);
	bool auxNode(Node *node);
	void saveScene(const char *scene = NULL);
	bool saveNode(Node *n);
	void releaseScene();
	bool hideNode(Node *node);
	void showScene();
	bool showNode(Node *node);
    void setActiveScene(Scene *scene);
    std::string _sceneName;

    Camera* getCamera();
    Node* getCameraNode();
    void placeCamera();
    void setCameraEye(float radius, float theta, float phi);
    void setCameraZoom(float radius);
    void setCameraTarget(Vector3 target);
    void setCameraNode(MyNode *node);
    void resetCamera();
    void cameraPush();
    void cameraPop();
    cameraState* copyCameraState(cameraState *state, cameraState *dst = NULL);
    
    void setAction(const char *type, ...); //set the current action parameters but don't commit
    void commitAction();
    void undoLastAction();
    void redoLastAction();
    void swapTransform(MyNode *n1, MyNode *n2);
    void swapMesh(MyNode *n1, MyNode *n2);

    void addConstraints(MyNode *node);
    void removeConstraints(MyNode *node);
    void enableConstraints(MyNode *node, bool enable = true);
    void reloadConstraint(MyNode *node, MyNode::nodeConstraint *constraint);
     
    bool mouseEvent(Mouse::MouseEvent evt, int x, int y, int wheelDelta);
	void keyEvent(Keyboard::KeyEvent evt, int key);
    void touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
    void controlEvent(Control* control, Control::Listener::EventType evt);
    void inactivateControls(Container *container = NULL);
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
    bool drawNode(Node* node);
    void placeNode(MyNode *node, float x, float y);
    void setMode(short mode);
    void setNavMode(short mode);

    //see if the current touch coordinates intersect a given model in the scene
    bool checkTouchModel(Node* node);
    MyNode* getMouseNode(int x, int y, Vector3 *touch = NULL);
    
    //UI factory functions
    Form* addMenu(const char *name, Container *parent = NULL, const char *buttonText = NULL,
      Layout::Type layout = Layout::LAYOUT_VERTICAL);
    Form* addPanel(const char *name, Container *parent = NULL);
    template <class ButtonType> ButtonType* addButton(Container *parent, const char *name, const char *text = NULL,
      Theme::Style *style = NULL);
    template <class ControlType> ControlType* addControl(Container *parent, const char *name, const char *text = NULL,
      Theme::Style *style = NULL);
    //other UI
    void promptComponent();
    void getText(const char *prompt, const char *type, void (T4TApp::*callback)(const char*));
    void doConfirm(const char *message, void (T4TApp::*callback)(bool));
    void showDialog(Container *dialog, bool show = true);
    void confirmDelete(bool yes);
    
    //miscellaneous
    template <class T> T* popBack(std::vector<T*> &vec);
    template <class T> void delBack(std::vector<T*> &vec);
    template <class T> void delAll(std::vector<T*> &vec);

	class HitFilter : public PhysicsController::HitFilter {
		public:
		T4TApp *app;
		HitFilter(T4TApp *app_);
		bool filter(PhysicsCollisionObject *object);
		//bool hit(const HitResult &hit);
	};
	HitFilter *_hitFilter;
};


#include "Modes.h"

#endif

