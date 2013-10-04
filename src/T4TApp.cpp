#include "T4TApp.h"
#include "Grid.h"
#include <cstdio>
#include <cmath>

#define PI 3.1415926535

using std::cout;
using std::cin;
using std::endl;

// Declare our game instance
static T4TApp* __t4tInstance = NULL;
T4TApp game;
bool paused = false;

T4TApp::T4TApp()
    : _scene(NULL)
{
	__t4tInstance = this;
}

T4TApp* T4TApp::getInstance() {
	return __t4tInstance;	
}

bool T4TApp::printNode(Node *node) {
	Node *parent = node->getParent();
	if(parent != NULL) {
		cout << parent->getId() << ";" << parent->getType() << " => ";
	}
	cout << node->getId() << ";" << node->getType() << endl;
	return true;
}
void T4TApp::initialize()
{
    // Load font
    _font = Font::create("res/common/arial18.gpb");
    assert(_font);
    
    cout << "sin(30) = " << sin(30.0f*PI/180) << endl;

    // Generate game scene
    _scene = Scene::load("res/box.gpb");
    _scene->visit(this, &T4TApp::printNode);
    _selectedNode = NULL;
    
    // Set the aspect ratio for the scene's camera to match the current resolution
    _scene->getActiveCamera()->setAspectRatio(getAspectRatio());
    
    // Get light node
    _lightNode = _scene->findNode("directionalLight");
    _light = _lightNode->getLight();

    // Load camera script
    getScriptController()->loadScript("res/common/camera.lua");
    enableScriptCamera(true);
    //and initialize camera position by triggering a touch event
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", Touch::TOUCH_PRESS, 0, 0, 0);
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", Touch::TOUCH_MOVE, 0, 0, 0);
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", Touch::TOUCH_RELEASE, 0, 0, 0);

    // populate catalog of items
    size_t ind = 0, numItems = 3;
    _catalog = new std::vector<Mesh*>();
    _itemNames = new std::vector<std::string>();
    _catalog->push_back(createBoxMesh(1, 2, 3));
    //_catalog->push_back(box);
    _itemNames->push_back("Box");
    _itemNames->push_back("Sphere");
    _itemNames->push_back("Cone");
//*/

	//create the grid on which to place objects
    Model* gridModel = createGridModel();
    assert(gridModel);
    gridModel->setMaterial("res/common/grid.material");
    Node* node = _scene->addNode("grid");
    node->setModel(gridModel);
    gridModel->release();//*/

    gridModel = createBoxModel(1.0f, 2.0f, 3.0f);
    assert(gridModel);
    Material* material = gridModel->setMaterial("res/common/box.material");
    material->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
    material->getParameter("u_lightColor")->setValue(_light->getColor());
    material->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());//*/
    node = _scene->addNode("grid");
    node->setModel(gridModel);
    gridModel->release();//*/

    // Initialize box model
    Node* boxNode = _scene->findNode("box");
    //Model* boxModel = createBoxModel(0.5f, 0.5f, 0.5f);
    //Material* boxMaterial = boxModel->setMaterial("res/common/box.material");
    //boxNode->setModel(boxModel);
    Vector3 pos = boxNode->getTranslation();
    cout << "initial box at " << pos.x << ", " << pos.y << ", " << pos.z << endl;
    //_catalog->push_back(boxNode->getModel()->getMesh());
    _scene->removeNode(boxNode);
    /*boxMaterial->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
    boxMaterial->getParameter("u_lightColor")->setValue(_light->getColor());
    boxMaterial->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());//*/
    
    //create the form for selecting catalog items
    Theme* theme = Theme::create("res/common/default.theme");
    Theme::Style* formStyle = theme->getStyle("basicContainer");
    Theme::Style* buttonStyle = theme->getStyle("buttonStyle");
    Theme::Style* titleStyle = theme->getStyle("title");

    // Note: this calls addRef() on formStyle's Theme, which we created above.
    _itemSelectForm = Form::create("itemSelect", formStyle, Layout::LAYOUT_VERTICAL);
    theme->release();   // So we can release it once we're done creating forms with it.

    _itemSelectForm->setAutoHeight(true);
    _itemSelectForm->setWidth(200.0f);
    _itemSelectForm->setScroll(Container::SCROLL_VERTICAL);
    _itemSelectForm->setConsumeInputEvents(true);

	//size_t numItems = _itemNames->size();
	for(size_t i = 0; i < numItems; i++) {
		cout << "adding button for " << (*_itemNames)[i] << endl;
		Button* itemButton = Button::create((*_itemNames)[i].c_str(), buttonStyle);
		itemButton->setText((*_itemNames)[i].c_str());
		itemButton->setAutoWidth(true);
		itemButton->setHeight(60);
		itemButton->setConsumeInputEvents(false);
		itemButton->addListener(this, Control::Listener::CLICK);
		_itemSelectForm->addControl(itemButton);
		itemButton->release();
	}
    _itemSelectForm->setState(Control::FOCUS);
//*/
}

void T4TApp::finalize()
{
    SAFE_RELEASE(_scene);
    SAFE_RELEASE(_itemSelectForm);
}

void T4TApp::update(float elapsedTime)
{
    // Rotate model
    //if(!paused) _scene->findNode("box")->rotateY(MATH_DEG_TO_RAD((float)elapsedTime / 1000.0f * 180.0f));
    _itemSelectForm->update(elapsedTime);
    getScriptController()->executeFunction<void>("camera_update", "f", elapsedTime);
}

void T4TApp::render(float elapsedTime)
{
    // Clear the color and depth buffers
    clear(CLEAR_COLOR_DEPTH, Vector4::zero(), 1.0f, 0);

    // Visit all the nodes in the scene for drawing
    _scene->visit(this, &T4TApp::drawScene);

    // Draw text
    Vector4 fontColor(1.0f, 1.0f, 1.0f, 1.0f);
    unsigned int width, height;
    char buffer[50];

    _font->start();

    // Mouse
    sprintf(buffer, "M(%d,%d)", (int)_mousePoint.x, (int)_mousePoint.y);
    _font->measureText(buffer, _font->getSize(), &width, &height);
    int x = _mousePoint.x - (int)(width>>1);
    int y = _mousePoint.y - (int)(height>>1);
    //cout << "drawing " << buffer << " at " << x << ", " << y << endl;
    _font->drawText(buffer, x, y, fontColor, _font->getSize());
    if (_mouseString.length() > 0)
    {
        int y = getHeight() - _font->getSize();
        _font->drawText(_mouseString.c_str(), 0, y, fontColor, _font->getSize());
    }
    _font->finish();

    _itemSelectForm->draw();
}

bool T4TApp::drawScene(Node* node)
{
    // If the node visited contains a model, draw it
    Model* model = node->getModel(); 
    if (model)
    {
        model->draw();
        //cout << "drawing model " << node->getId() << endl;
    }
    return true;
}

bool T4TApp::mouseEvent(Mouse::MouseEvent evt, int x, int y, int wheelDelta)
{
	return false;
/*    _mousePoint.set(x, y);
    _mouseString.clear();
    //cout << "mouse event at " << x << ", " << y << endl;

    switch (evt)
    {
    case Mouse::MOUSE_PRESS_LEFT_BUTTON:
        _mouseString.append("MOUSE_PRESS_LEFT_BUTTON");
        break;
    case Mouse::MOUSE_RELEASE_LEFT_BUTTON:
        _mouseString.append("MOUSE_RELEASE_LEFT_BUTTON");
        break;
    case Mouse::MOUSE_PRESS_MIDDLE_BUTTON:
        _mouseString.append("MOUSE_PRESS_MIDDLE_BUTTON");
        break;
    case Mouse::MOUSE_RELEASE_MIDDLE_BUTTON:
        _mouseString.append("MOUSE_RELEASE_MIDDLE_BUTTON");
        break;
    case Mouse::MOUSE_PRESS_RIGHT_BUTTON:
        _mouseString.append("MOUSE_PRESS_RIGHT_BUTTON");
        break;
    case Mouse::MOUSE_RELEASE_RIGHT_BUTTON:
        _mouseString.append("MOUSE_RELEASE_RIGHT_BUTTON");
        break;
    case Mouse::MOUSE_MOVE:
        _mouseString.append("MOUSE_MOVE");
        break;
    case Mouse::MOUSE_WHEEL:
        _mouseString.append("MOUSE_WHEEL");
        break;
    }
    return true;//*/
}

void T4TApp::keyEvent(Keyboard::KeyEvent evt, int key)
{
    if (evt == Keyboard::KEY_PRESS)
    {
        switch (key)
        {
        case Keyboard::KEY_ESCAPE:
            exit();
            break;
        }
    }
}

void T4TApp::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	//cout << "touch event at " << x << ", " << y << endl;
    switch (evt)
    {
    case Touch::TOUCH_PRESS:
    	paused = true;
        break;
    case Touch::TOUCH_RELEASE:
    	_selectedNode = NULL;
    	enableScriptCamera(true);
    	paused = false;
        break;
    case Touch::TOUCH_MOVE:
    	_mousePoint.set(x, y);
    	_mouseString.clear();
    	_mouseString.append("TOUCH_MOVE");
        break;
    };
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", evt, x, y, contactIndex);
    Node* cameraNode = _scene->getActiveCamera()->getNode();
    Quaternion rotation = cameraNode->getRotation();
    Vector3 translation = cameraNode->getTranslation();
    //cout << "camera rotation now " << rotation.x << ", " << rotation.y << ", " << rotation.z << ", " << rotation.w << endl;
    //cout << "camera translation now " << translation.x << ", " << translation.y << ", " << translation.z << endl;
}

void T4TApp::controlEvent(Control* control, EventType evt)
{
	cout << "CLICKED " << control->getId() << endl;
	const size_t catalogSize = _itemNames->size();
	for(size_t i = 0; i < catalogSize; i++)
	{
		if((*_itemNames)[i].compare(control->getId()) == 0)
		{
			cout << "selected " << (*_itemNames)[i] << endl;
			Mesh* mesh = (*_catalog)[i];
		    Model* model = Model::create(mesh);
			Material* material = model->setMaterial("res/box.material");
			material->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
			material->getParameter("u_lightColor")->setValue(_light->getColor());
			material->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());
			Node* node = _scene->addNode("hello");
			node->setModel(model);
			model->release();
			Vector3 pos = node->getTranslation();
			cout << "new node is at " << pos.x << ", " << pos.y << ", " << pos.z << endl;
			_selectedNode = node;
			enableScriptCamera(false);
		}
	}
}

void T4TApp::enableScriptCamera(bool enable)
{
	getScriptController()->executeFunction<void>("camera_setActive", "b", enable);
}

