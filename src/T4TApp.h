#ifndef TEMPLATEGAME_H_
#define TEMPLATEGAME_H_

#include "gameplay.h"
#include <cstring>

using namespace gameplay;

/**
 * Main game class.
 */
class T4TApp: public Game, Control::Listener
{
public:

    /**
     * Constructor.
     */
    T4TApp();
    
    T4TApp* getInstance();
    
    bool printNode(Node *node);

    /**
     * @see Game::keyEvent
     */
     
    bool mouseEvent(Mouse::MouseEvent evt, int x, int y, int wheelDelta);

	void keyEvent(Keyboard::KeyEvent evt, int key);
    
    /**
     * @see Game::touchEvent
     */
    void touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
    
    void controlEvent(Control* control, EventType evt);
    
    void enableScriptCamera(bool enable);

protected:

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

private:

    /**
     * Draws the scene each frame.
     */
    bool drawScene(Node* node);
    
    void setSelected(Node* node);
    void setMode(const char *mode);

    //see if the current touch coordinates intersect a given model in the scene
    bool checkTouchModel(Node* node);
    bool checkTouchEdge(Node* node);
    
    //misc functions
    const std::string printVector(Vector3& v);
    
    //model factory functions
    Node* createBoxNode(float width, float height, float depth);
    
    //UI factory functions
    Form* addMenu(Form *parent, const char *name);
    template <class ButtonType> ButtonType* addButton(Form *menu, const char *name, const char *text = NULL);

	//scene setup
    Scene* _scene;
    Node* _lightNode;
    Light* _light;
    
    //T4T objects for modeling
    std::vector<Node*>* _catalog;
    std::vector<std::string>* _itemNames;
    std::vector<int>* _itemCount;
    
    //for placing objects
    Node *_selectedNode, *_lastNode;
    const BoundingBox* _selectedBox;
    Plane _groundPlane;
    Vector3 _intersectPoint;
    Node* _intersectModel;
    float _intersectHeight;
    
    //for creating physical constraints between objects
    Node* _constraintNodes[2];
    unsigned short _constraintEdges[2];
    
    //current state
    std::string _mode;
    
    //user interface
    Form *_mainMenu, *_sideMenu, *_itemContainer, *_modeContainer; //main menu
    std::vector<Form*> *_submenus; //submenus
    Button *_itemButton, *_modeButton; //submenu handles
    CheckBox *_gridCheckbox, *_debugCheckbox;
    Slider *_gridSlider, *_zoomSlider;
    std::vector<std::string> *_modeNames;
    std::vector<RadioButton*> *_modeButtons;
    Theme::Style *_formStyle, *_buttonStyle, *_titleStyle;
    
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
