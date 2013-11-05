#ifndef TEMPLATEGAME_H_
#define TEMPLATEGAME_H_

#include "gameplay.h"

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

    //see if the current touch coordinates intersect a given model in the scene
    bool checkTouchModel(Node* node);
    bool checkTouchEdge(Node* node);
    
    //model factory functions
    Node* createBoxNode(float width, float height, float depth);

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
    Form* _itemSelectForm;
    Plane _groundPlane;
    Vector3 _intersectPoint;
    Node* _intersectModel;
    float _intersectHeight;
    
    //for creating physical constraints between objects
    Node* _constraintNodes[2];
    unsigned short _constraintEdges[2];
    
    //user interface
    CheckBox* _snapToGridCheckbox;
    Slider *_gridSpacingSlider, *_cameraZoomSlider;
    RadioButton *_selectMode, *_rotateMode, *_constraintMode;
    
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
    
    //debugging flags
    bool _physicsStopped;
	
};

#endif
