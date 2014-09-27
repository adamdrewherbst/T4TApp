#include "T4TApp.h"
#include "Grid.h"
#include <cmath>
#include <sstream>

using std::istringstream;

// Declare our game instance
static T4TApp* __t4tInstance = NULL;
static bool _debugFlag = false;
T4TApp game;

T4TApp::T4TApp()
    : _scene(NULL)
{
	__t4tInstance = this;
	_physicsStopped = false;
}

void T4TApp::debugTrigger()
{
	int x = 0;
}

T4TApp* T4TApp::getInstance() {
	return __t4tInstance;
}

void T4TApp::initialize()
{
    // Load font
    _font = Font::create("res/common/arial18.gpb");
    assert(_font);
    
    // Load camera script
    getScriptController()->loadScript("res/common/camera.lua");

    loadScene();
    
	getPhysicsController()->setGravity(Vector3(0.0f, -10.0f, 0.0f));

	/*Node* carNode = _vehicle->findNode("carbody");
	if (carNode && carNode->getCollisionObject() && carNode->getCollisionObject()->getType() == PhysicsCollisionObject::VEHICLE)
	{
		_carVehicle = static_cast<PhysicsVehicle*>(carNode->getCollisionObject());
	}//*/
	_steering = _braking = _driving = 0.0f;

	//as a test, add two boxes with a hinge constraint between them
/*    Node *node = addCatalogItem(1);
    node->setTranslation(0.0, 0.5, 0.0);
    PhysicsRigidBody *body1 = node->getCollisionObject()->asRigidBody();
    body1->setEnabled(false);
    
    node = addCatalogItem(1);
    node->setTranslation(1.0, 0.5, 0.0);
   	PhysicsRigidBody *body2 = node->getCollisionObject()->asRigidBody();
   	body2->setEnabled(false);

	PhysicsHingeConstraint* constraint = getPhysicsController()->createHingeConstraint(
		body1,
		Quaternion(0.0f, 0.0f, 0.0f, 1.0f),
		Vector3(0.5f, 0.5f, 0.0f),
		body2,
		Quaternion(0.0f, 0.0f, 0.0f, 1.0f),
		Vector3(-0.5f, 0.5f, 0.0f)
	);
	constraint->setLimits(0.0f, PI, 1.0f);
	
	node->translate(0.0f, 1.0f, 0.0f);
	node->rotate(0.0f, 0.0f, sin(45.0f*PI/180), cos(45.0f*PI/180));
	body1->setEnabled(true); body2->setEnabled(true);//*/
    
    //create the form for selecting catalog items
    _theme = Theme::create("res/common/default.theme");
    _formStyle = _theme->getStyle("basicContainer");
    _hiddenStyle = _theme->getStyle("hiddenContainer");
    _buttonStyle = _theme->getStyle("buttonStyle");
    _titleStyle = _theme->getStyle("title");

	/*********************** GUI SETUP ***********************/
	
	//root menu node for finding controls by ID
	_mainMenu = Form::create("mainMenu", _formStyle, Layout::LAYOUT_ABSOLUTE);
	_mainMenu->setWidth(getWidth());
	_mainMenu->setHeight(getHeight());
	_mainMenu->setVisible(false);
	//main menu on left side [Note: this calls addRef() on formStyle's Theme, which we created above]
    _sideMenu = (Form*) addMenu(NULL, "sideMenu");
    //submenu for adding simple machines
    _machineContainer = addMenu(_sideMenu, "machineContainer");
    //submenu holding catalog of T4T items
    _itemContainer = addMenu(_sideMenu, "itemContainer");
    //submenu for current interaction mode
    _modeContainer = addMenu(_sideMenu, "modeContainer");
    
    //popup menu to allow user to select a component
    _componentMenu = Form::create("componentMenu", _formStyle, Layout::LAYOUT_FLOW);
    _componentMenu->setPosition(_sideMenu->getX() + _sideMenu->getWidth() + 25.0f, 25.0f);
    _componentMenu->setWidth(getWidth() - _componentMenu->getX() - 25.0f);
    _componentMenu->setHeight(getHeight() - 50.0f);
    _componentMenu->setVisible(false);
    _componentMenu->setScroll(Container::SCROLL_VERTICAL);
    _componentMenu->setConsumeInputEvents(true);
    _mainMenu->addControl(_componentMenu);

    _theme->release();   // So we can release it once we're done creating forms with it.
    
	// populate catalog of items
	//_models = Scene::load("res/common/models.scene");
	//_models->setId("models");
	_models = Scene::create("models");
	_models->addNode("sphere");
	_models->addNode("cylinder");
	_models->addNode("halfpipe");
	_models->addNode("box");
	_models->addNode("gear");
	_models->addNode("tube");
	_models->addNode("vase");
	_models->addNode("disc");
	_models->addNode("cone");
	_models->addNode("heart");
	_models->addNode("gear_thin");

	//populate item submenu
	_itemButton = addButton <Button> (_sideMenu, "itemButton", "Add Object >>"); //button to open object catalog

    Node *modelNode = _models->getFirstNode();
    while(modelNode) {
    	if(strstr(modelNode->getId(), "_part") == NULL) {
			cout << "adding button for " << modelNode->getId() << endl;
			//modelNode->loadData(concat(3, "res/common/", modelNode->getId(), ".node"));
			modelNode->reloadFromData(concat(3, "res/common/", modelNode->getId(), ".node"), false);
			Button* itemButton = addButton <Button> (_itemContainer, modelNode->getId());
			ImageControl* itemImage = addButton <ImageControl> (_componentMenu, concat(2, "comp_", modelNode->getId()));
			itemImage->setImage("res/png/cowboys-helmet-nobkg.png");
			itemImage->setWidth(150.0f);
			itemImage->setHeight(150.0f);
			Node::nodeData *data = modelNode->getData();
			data->type = modelNode->getId();
		}
		modelNode->setTranslation(Vector3(1000.0f,0.0f,0.0f));
		modelNode = modelNode->getNextSibling();
	}

    //populate simple machine submenu
    _machines.push_back(new Lever(this, _buttonStyle, _formStyle));
    _machines.push_back(new Pulley(this, _buttonStyle, _formStyle));
    _machineNames.push_back("Lever");
    _machineNames.push_back("Pulley");
    _machineButton = addButton <Button> (_sideMenu, "parent_machineContainer", "Add Machine >>");
    for(size_t i = 0; i < _machineNames.size(); i++) {
    	Button *machineButton = addButton <Button> (_machineContainer, _machineNames[i].c_str());
    }

	_modes.push_back(new RotateMode(this));
	_modes.push_back(new PositionMode(this));
	_modes.push_back(new ConstraintMode(this));
	_modes.push_back(new SliceMode(this));
	_modes.push_back(new DrillMode(this));
	_modes.push_back(new TestMode(this));
	_modes.push_back(new TouchMode(this));
	_modePanel = addPanel(_sideMenu, "container_modes");
	for(size_t i = 0; i < _modes.size(); i++) {
		const char *id = _modes[i]->getId();
		Button *modeButton = addControl <Button> (_modePanel, id, _buttonStyle, id+5);
		modeButton->setHeight(40.0f);
	}
	_modePanel->setHeight(_modes.size() * 50.0f);
	_modePanel->setVisible(true);

	_vehicleButton = addControl <Button> (_sideMenu, "buildVehicle", _buttonStyle, "Build Vehicle");
	
	_drawDebugCheckbox = addControl <CheckBox> (_sideMenu, "drawDebug", _buttonStyle, "Draw Debug");
	Button *debugButton = addControl <Button> (_sideMenu, "debugButton", _buttonStyle, "Debug");
	
	//create an instance of each project template - will be activated as needed
	_vehicleProject = new VehicleProject(this, "vehicleProject", _buttonStyle, _formStyle);

	_drawDebug = true;	
	setMode("Rotate");
    _sideMenu->setState(Control::FOCUS);
	
	_running = 0;
}

void T4TApp::loadScene()
{
    // Generate game scene
    _scene = Scene::load("res/common/game.scene");
    _scene->setId("scene");
    _scene->visit(this, &T4TApp::printNode);
    setActiveScene(_scene);
    
    // Set the aspect ratio for the scene's camera to match the current resolution
    _scene->getActiveCamera()->setAspectRatio(getAspectRatio());
    
    // Get light node
    _lightNode = _scene->findNode("lightNode");
    _light = _lightNode->getLight();

	//create the grid on which to place objects
    Node* node = _scene->addNode("grid");
    Model* gridModel = createGridModel();
    gridModel->setMaterial("res/common/grid.material");
    node->setModel(gridModel);
    gridModel->release();
	PhysicsRigidBody::Parameters gridParams;
	gridParams.mass = 0.0f;
	node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY,
		PhysicsCollisionShape::box(),
		&gridParams);
	PhysicsRigidBody* body = node->getCollisionObject()->asRigidBody();
	body->setEnabled(true);
    //store the plane representing the grid, for calculating intersections
    _groundPlane = Plane(Vector3(0, 1, 0), 0);
    
    //create lines for the positive axes
    std::vector<float> vertices;
    vertices.resize(36,0);
    for(int i = 0; i < 6; i++) {
    	vertices[i*6+1] = 5.0f;
    	if(i%2 == 1) vertices[i*6+i/2] += 5.0f;
    	vertices[i*6+4] = 1.0f; //color green
    }
    _scene->addNode(createWireframe(vertices));

    enableScriptCamera(true);
    //and initialize camera position by triggering a touch event
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", Touch::TOUCH_PRESS, 0, 0, 0);
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", Touch::TOUCH_MOVE, 0, 0, 0);
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", Touch::TOUCH_RELEASE, 0, 0, 0);
}

void T4TApp::releaseScene()
{
	if(_activeScene == _scene) _activeScene = NULL;
	_carVehicle = NULL;
	//enableScriptCamera(false);
	SAFE_RELEASE(_scene);
}

void T4TApp::hideScene() {
	_scene->visit(this, &T4TApp::hideNode);
}

void T4TApp::showScene() {
	_scene->visit(this, &T4TApp::showNode);
	_activeScene = _scene;
}

bool T4TApp::hideNode(Node *node) {
	PhysicsCollisionObject *obj = node->getCollisionObject();
	if(obj)	obj->setEnabled(false);
	return true;
}

bool T4TApp::showNode(Node *node) {
	PhysicsCollisionObject *obj = node->getCollisionObject();
	if(obj) obj->setEnabled(true);
	return true;
}

template <class ButtonType> ButtonType* T4TApp::addButton(Form *menu, const char *id, const char *text)
{
	ButtonType* button = ButtonType::create(id, _buttonStyle);
	if(text == NULL) button->setText(id);
	else button->setText(text);
	button->setAutoWidth(true);
	button->setHeight(50);
	button->setConsumeInputEvents(false);
	button->addListener(this, Control::Listener::CLICK);
	menu->addControl(button);
	return button;
}

template <class ControlType> ControlType* T4TApp::addControl(Form *parent, const char *id, Theme::Style *style, const char *text)
{
	ControlType* control = ControlType::create(id, style);
	if(text == NULL) control->setText(id);
	else control->setText(text);
	control->setHeight(50);
	control->setWidth(150);
	control->setConsumeInputEvents(false);
	control->addListener(this, Control::Listener::CLICK);
	parent->addControl(control);
	return control;
}

Form* T4TApp::addMenu(Form *parent, const char *name)
{
    Form *container = Form::create(name, _formStyle, Layout::LAYOUT_VERTICAL);
    if(parent == NULL) {
    	container->setAutoHeight(true);
    }
    else {
	    container->setHeight(300.0f);
	    container->setVisible(false);
	    _submenus.push_back(container);
	}
    container->setWidth(200.0f);
    container->setScroll(Container::SCROLL_VERTICAL);
    container->setConsumeInputEvents(true);
    _mainMenu->addControl(container);
    return container;
}

Form* T4TApp::addPanel(Form *parent, const char *name)
{
	Form *container = Form::create(name, _formStyle, Layout::LAYOUT_VERTICAL);
	container->setHeight(250);
	container->setWidth(175);
	container->setScroll(Container::SCROLL_NONE);
    container->setConsumeInputEvents(true);
    _mainMenu->addControl(container);
    parent->addControl(container);
    return container;
}

void T4TApp::finalize()
{
    SAFE_RELEASE(_scene);
    SAFE_RELEASE(_mainMenu);
}

int updateCount = 0;
void T4TApp::update(float elapsedTime)
{
    _mainMenu->update(elapsedTime);
	if(_carVehicle) _carVehicle->update(elapsedTime, _steering, _braking, _driving);
/*	cout << "updating car [" << elapsedTime << "]: " << _driving << ", " << _braking << ", " << _steering << endl;
	cout << "\tspeed now " << _carVehicle->getSpeedKph();
	Vector3 vel = _carVehicle->getRigidBody()->getLinearVelocity();
	cout << ", vel = " << printVector(vel) << endl;//*/
	getScriptController()->executeFunction<void>("camera_update", "f", elapsedTime);

    if(_state == Game::RUNNING) {
    	//cout << "Still running... " << updateCount++ << endl;
    }else if(!_physicsStopped) {
    	_physicsStopped = true;
    	cout << "Physics stopped" << endl;
    }
}

void T4TApp::render(float elapsedTime)
{
    // Clear the color and depth buffers
    clear(CLEAR_COLOR_DEPTH, Vector4::zero(), 1.0f, 0);

    // Visit all the nodes in the scene for drawing
    if(_activeScene != NULL) {
    	_activeScene->visit(this, &T4TApp::drawScene);
	    //if(_activeScene != _vehicleProject->_scene) _vehicleProject->_scene->visit(this, &T4TApp::drawScene);
    	if(_drawDebug) getPhysicsController()->drawDebug(_activeScene->getActiveCamera()->getViewProjectionMatrix());
    }

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

    _sideMenu->draw();
    for(size_t i = 0; i < _submenus.size(); i++)
    	if(_submenus[i]->isVisible()) _submenus[i]->draw();
    if(_componentMenu->isVisible()) _componentMenu->draw();
	if(_vehicleProject->container->isVisible()) _vehicleProject->container->draw();
	for(size_t i = 0; i < _modes.size(); i++) {
		if(_modes[i]->_active) _modes[i]->draw();
	}
}

bool T4TApp::prepareNode(Node* node)
{
	PhysicsCollisionObject* obj = node->getCollisionObject();
	if(obj && obj->asRigidBody()) {
		cout << "adding collision listener to " << node->getId() << endl;
		obj->asRigidBody()->addCollisionListener(this);
	}
}
bool T4TApp::printNode(Node *node) {
	Node *parent = node->getParent();
	if(parent != NULL) {
		cout << parent->getId() << ";" << parent->getType() << " => ";
	}
	cout << node->getId() << ";" << node->getType() << endl;
	return true;
}
bool T4TApp::drawScene(Node* node)
{
//	if(strcmp(node->getScene()->getId(), "models") == 0) return true;
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
    	    case Keyboard::KEY_UP_ARROW:
    	    	_braking = 0.0f;
    	    	_driving += 0.1f;
    	    	if(_driving > 1.0f) _driving = 1.0f;
    	    	break;
    	    case Keyboard::KEY_DOWN_ARROW:
    	    	_driving = 0.0f;
    	    	_braking += 0.1f;
    	    	if(_braking > 1.0f) _braking = 1.0f;
    	    	break;
    	    case Keyboard::KEY_LEFT_ARROW:
    	    	_steering -= 0.1f;
    	    	if(_steering < -1.0f) _steering = -1.0f;
    	    	break;
    	    case Keyboard::KEY_RIGHT_ARROW:
    	    	_steering += 0.1f;
    	    	if(_steering > 1.0f) _steering = 1.0f;
    	    	break;
    	    default: break;
        }
    }
}

void T4TApp::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	_touchPoint.set(x, y);
    switch (evt)
    {
		case Touch::TOUCH_PRESS:
			_debugFlag = false;
			break;
		case Touch::TOUCH_MOVE:
			break;
		case Touch::TOUCH_RELEASE:
			_debugFlag = true;
			enableScriptCamera(true);
			_physicsStopped = false;
			updateCount = 0;
		    break;
    };
}

void T4TApp::controlEvent(Control* control, Control::Listener::EventType evt)
{
	const char *controlID = control->getId();
	cout << "CLICKED " << controlID << endl;
	//const size_t catalogSize = _itemNames->size();

	//if a submenu handle is clicked, toggle whether the submenu is expanded
	if(strncmp(controlID, "parent_", 7) == 0) {
		const char *subName = controlID+7;
		cout << "Looking for submenu " << subName << endl;
		Container *subMenu = (Container*)_mainMenu->getControl(subName);
		if(subMenu != NULL) {
			bool visible = subMenu->isVisible();
			for(size_t i = 0; i < _submenus.size(); i++)
				_submenus[i]->setVisible(false);
			cout << "\ttoggling menu to " << !visible << endl;
			subMenu->setVisible(!visible);
			if(!visible) { //if expanding the submenu, position it next to its handle
				subMenu->setPosition(control->getX() + control->getWidth(), control->getY());
			}
		} else {
			cout << "No control with ID " << subName << endl;
		}
	}
	//if selected a mode, update the mode flag
	else if(_modeContainer->getControl(controlID) == control) {
		setMode(controlID);
		_modeContainer->setVisible(false);
	}
	else if(_machineContainer->getControl(controlID) == control) {
		cout << "clicked machine " << controlID << endl;
		_machineContainer->setVisible(false);
		for(size_t i = 0; i < _machines.size(); i++) {
			if(strcmp(_machines[i]->getId(), controlID) == 0) {
				_machines[i]->setActive(true);
			}
		}
	}
	else if(_modePanel->getControl(controlID) == control) {
		for(size_t i = 0; i < _modes.size(); i++) {
			if(_modes[i]->_active) {
				cout << "setting " << _modes[i]->getId() << " inactive" << endl;
				_modes[i]->setActive(false);
			}
		}
		for(size_t i = 0; i < _modes.size(); i++) {
			if(strcmp(_modes[i]->getId(), controlID) == 0) {
				cout << "setting " << _modes[i]->getId() << " active" << endl;
				_modes[i]->setActive(true);
			}
		}
	}
	//handle opening of the item catalog/adding an item
	else if(control == _itemButton) {
		_componentMenu->addListener(this, Control::Listener::CLICK);
		_componentMenu->setVisible(true);
	}
	else if(_componentMenu->getControl(controlID) == control && strncmp(controlID, "comp_", 5) == 0) {
		Node *node = duplicateModelNode(controlID+5);
		addCollisionObject(node);
		_scene->addNode(node);
		placeNode(node, 0.0f, 0.0f);
		_componentMenu->setVisible(false);
		_componentMenu->removeListener(this);
	}
	else if(strcmp(controlID, "debugButton") == 0) {
		debugTrigger();
	}
	else if(_itemContainer->getControl(controlID) == control) {
		Node *node = duplicateModelNode(controlID);
		addCollisionObject(node);
		_scene->addNode(node);
		placeNode(node, 0.0f, 0.0f);
		_itemContainer->setVisible(false);
	}
	else if(control == _zoomSlider) {
	    getScriptController()->executeFunction<void>("camera_setRadius", "f", _zoomSlider->getValue());
	}
	else if(control == _vehicleButton) {
		buildVehicle();
	}
	else if(control == _drawDebugCheckbox) {
		_drawDebug = _drawDebugCheckbox->isChecked();
	}
	else if(strcmp(control->getId(), "DebugButton") == 0) {
		if(_lastNode) {
			PhysicsRigidBody *body = _lastNode->getCollisionObject()->asRigidBody();
			cout << "last node " << _lastNode->getId() << " is " << (body->isEnabled() ? "" : "NOT") << " enabled" << endl;
		}
	}
}

Node* T4TApp::getMouseNode(int x, int y, Vector3 *touch) {
	Camera* camera = _scene->getActiveCamera();
	Ray ray;
	camera->pickRay(getViewport(), x, y, &ray);
	PhysicsController::HitResult hitResult;
	if(!getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) return NULL;
	if(touch) touch->set(hitResult.point);
	Node *node = hitResult.object->getNode();
	if(node == NULL || strcmp(node->getId(), "grid") == 0) return NULL;
	return node;
}

//see if the current touch location intersects the bottom face of the given object
bool T4TApp::checkTouchModel(Node* node)
{
	if(strcmp(node->getScene()->getId(), "models") == 0) return true;
	if(strcmp(node->getId(), "knife") == 0) return true;
	if(strcmp(node->getId(), "drill") == 0) return true;
	for(int i = 0; i < _intersectNodeGroup.size(); i++)
		if(node == _intersectNodeGroup[i]) return true;
	Vector3 pos = node->getTranslation();
	BoundingBox bbox = getWorldBox(node);
	if(bbox.isEmpty()) return true;
	float halfX = (_intersectBox.max.x - _intersectBox.min.x) / 2.0f,
		halfY = (_intersectBox.max.y - _intersectBox.min.y) / 2.0f,
		halfZ = (_intersectBox.max.z - _intersectBox.min.z) / 2.0f;
	if(_intersectPoint.x + halfX > pos.x + bbox.min.x && _intersectPoint.x - halfX < pos.x + bbox.max.x
		&& _intersectPoint.z + halfZ > pos.z + bbox.min.z && _intersectPoint.z - halfZ < pos.z + bbox.max.z)
	{
		if(_intersectModel == NULL || halfY + pos.y + bbox.max.y > _intersectPoint.y)
		{
			_intersectModel = node;
			_intersectPoint.y = pos.y + bbox.max.y + halfY;
		}
	}
	return true;
}

//seems bounding box is not corrected for scale - do that here
BoundingBox T4TApp::getWorldBox(Node *node) {
	Model *model = node->getModel();
	if(model == NULL) return BoundingBox::empty();
	BoundingBox box = model->getMesh()->getBoundingBox();
	Vector3 scale = node->getScale(), center = box.getCenter(), min = box.min - center, max = box.max - center;
	min.x *= scale.x;
	min.y *= scale.y;
	min.z *= scale.z;
	max.x *= scale.x;
	max.y *= scale.y;
	max.z *= scale.z;
	return BoundingBox(center + min, center + max);
}

//find the closest edge on this model to the touch point in 3D space - if it is the closest so far, store edge and distance
bool T4TApp::checkTouchEdge(Node* node)
{
	Node::nodeData* data = node->getData();
	for(int i = 0; i < data->edges.size(); i++) {
		//get starting point and direction vector of camera sight, and edge
	}
	return false;
}

void T4TApp::changeNodeModel(Node *node, const char* type)
{
	Node *modelNode = _models->findNode(type);
	if(modelNode == NULL) return;
	Node *clone = modelNode->clone();
	Model *model = clone->getModel();
	cout << "setting " << node->getId() << " model to " << model << endl;
	node->setModel(model);
//	node->setScale(modelNode->getScale());
//	node->setRotation(modelNode->getRotation());
	Node::nodeData *data = modelNode->getData();
	node->data = data;
	clone->release();
}

Node* T4TApp::duplicateModelNode(const char* type, bool isStatic)
{
	Node *modelNode = _models->findNode(type);
	if(modelNode == NULL) return NULL;
	Node *node = modelNode->clone();
	BoundingBox box = node->getModel()->getMesh()->getBoundingBox();
	node->setTranslation(Vector3(0.0f, (box.max.y - box.min.y)/2.0f, 0.0f));
	Node::nodeData *data = modelNode->getData();
	const char count[2] = {(char)(++data->typeCount + 48), '\0'};
	node->setId(concat(2, modelNode->getId(), count));
	node->loadData(concat(3, "res/common/", type, ".node"));
	node->updateData();
	data = node->getData();
	if(isStatic) data->mass = 0;

	/*PhysicsRigidBody::Parameters params;
	params.mass = isStatic ? 0.0f : 10.0f;
	node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::mesh(node->getModel()->getMesh()), &params);
//*/
	/*if(isStatic) node->setCollisionObject(concat(2, "res/common/models.physics#static", modelNode->getId()));
	else node->setCollisionObject(concat(2, "res/common/models.physics#", modelNode->getId()));
	PhysicsRigidBody* body = node->getCollisionObject()->asRigidBody();
	body->addCollisionListener(this);
	body->_body->setSleepingThresholds(0.1f, 0.1f);
	body->setActivation(ACTIVE_TAG);//*/
	return node;
}

Node* T4TApp::createWireframe(std::vector<float>& vertices, char *id) {
	int numVertices = vertices.size()/6;
	VertexFormat::Element elements[] = {
		VertexFormat::Element(VertexFormat::POSITION, 3),
		VertexFormat::Element(VertexFormat::COLOR, 3)
	};
	Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), numVertices, false);
	mesh->setPrimitiveType(Mesh::LINES);
	mesh->setVertexData(&vertices[0], 0, numVertices);
	Model *model = Model::create(mesh);
	mesh->release();
	model->setMaterial("res/common/grid.material");
	if(id == NULL) {
		char newID[40];
		for(int i = 0; i < 40; i++) newID[i] = '\0';
		int n = 0;
		do {
			n++;
			sprintf(newID, "wireframe%d", n);
		} while(_scene->findNode(newID) != NULL);
		id = newID;
	}
	Node *node = Node::create(id);
	node->setModel(model);
	model->release();
	return node;
}

//place at the given xz-coords and set its y-coord so it is on top of any objects it would otherwise intersect
void T4TApp::placeNode(Node *node, float x, float z)
{
	_intersectNodeGroup.clear();
	_intersectNodeGroup.push_back(node);
	_intersectBox = getWorldBox(node);
	float minY = _intersectBox.min.y;
	node->setTranslation(x, -minY, z); //put the bounding box bottom on the ground
	_intersectPoint.set(x, -minY, z);
	_intersectModel = NULL;
	_scene->visit(this, &T4TApp::checkTouchModel); //will change _intersectPoint.y to be above any intersecting models
	node->setTranslation(_intersectPoint);
}

void T4TApp::setMode(const char *mode)
{
	//make sure the radio button corresponding to this mode is selected
	RadioButton *oldButton, *newButton;
	oldButton = (RadioButton*)(_modeContainer->getControl(_mode.c_str()));
	if(oldButton != NULL) oldButton->setSelected(false);
	newButton = (RadioButton*)(_modeContainer->getControl(mode));
	if(newButton == NULL) {
		cout << "No button for mode " << mode << " - aborting" << endl;
		return;
	}
	newButton->setSelected(true);
	//add the options panel corresponding to this mode if any
	cout << "looking for " << (std::string("options_") + mode).c_str() << endl;
	Control *oldOptions = _modeOptions[_mode], //_mainMenu->getControl((std::string("options_") + _mode).c_str()),
		*newOptions = _modeOptions[mode]; //_mainMenu->getControl((std::string("options_") + mode).c_str());
	if(oldOptions) {
		cout << "setting old " << oldOptions->getId() << " invisible" << endl;
		_sideMenu->removeControl(oldOptions);
	}
	if(newOptions) {
		cout << "setting new " << newOptions->getId() << " visible" << endl;
		cout << "\t(has " << ((Form*)newOptions)->getControls().size() << " controls inside)" << endl;
		_sideMenu->addControl(newOptions);
	}
	_mode = mode;
}

void T4TApp::setActiveScene(Scene *scene)
{
	_activeScene = scene;
}

void T4TApp::enableScriptCamera(bool enable)
{
	getScriptController()->executeFunction<void>("camera_setActive", "b", enable);
}

const std::string T4TApp::printVector(const Vector3& v) {
	std::ostringstream os;
	os << "<" << v.x << "," << v.y << "," << v.z << ">";
	return os.str();
}

const std::string T4TApp::printVector2(const Vector2& v) {
	std::ostringstream os;
	os << "<" << v.x << "," << v.y << ">";
	return os.str();
}

const std::string T4TApp::printQuat(Quaternion& q) {
	std::ostringstream os;
	Vector3 axis;
	float ang = q.toAxisAngle(&axis);
	os << (int)(ang*180/M_PI) << " degrees about " << "<" << axis.x << "," << axis.y << "," << axis.z << ">";
	return os.str();
}

//go through the interactive steps to add all required components to a vehicle
void T4TApp::buildVehicle() {
	cout << "starting vehicle" << endl;
	_vehicleProject->setActive(true);
}

Node* T4TApp::promptComponent() {
	_componentMenu->setVisible(true);
}

void T4TApp::collisionEvent(PhysicsCollisionObject::CollisionListener::EventType type, 
                            const PhysicsCollisionObject::CollisionPair& pair, 
                            const Vector3& pointA, const Vector3& pointB)
{
    GP_WARN("Collision between rigid bodies %s (at point (%f, %f, %f)) "
            "and %s (at point (%f, %f, %f)).",
            pair.objectA->getNode()->getId(), pointA.x, pointA.y, pointA.z, 
            pair.objectB->getNode()->getId(), pointB.x, pointB.y, pointB.z);
}

void T4TApp::translateNode(Node *node, Vector3 trans) {
	//determine the set of all nodes constrained to this one - translate all of them
	Node::nodeData *data = node->getData();
	std::vector<Node*> nodes;
	nodes.push_back(node);
	for(int i = 0; i < data->constraints.size(); i++) {
		Node *other = _scene->findNode(data->constraints[i]->other.c_str());
		if(other != NULL) nodes.push_back(other);
	}
	for(int i = 0; i < nodes.size(); i++) {
		Node *n = nodes[i];
		n->setCollisionObject(PhysicsCollisionObject::NONE);
	}
}

Node* T4TApp::loadNodeFromData(const char *nodeID) {
	Node *node = _scene->addNode(nodeID);
	char *filename = concat(3, "res/common/", nodeID, ".node");
	node->reloadFromData(filename, true);
	addConstraints(node);
}

void T4TApp::removeNode(Node *node, const char *newID) {
	//if we are planning to reload this node under a new ID, update all constraints to use the new ID
	if(newID != NULL) {
		Node::nodeData *data = node->getData();
		for(int i = 0; i < data->constraints.size(); i++) {
			Node *other = _scene->findNode(data->constraints[i]->other.c_str());
			if(other == NULL) continue;
			Node::nodeData *otherData = other->getData();
			for(int j = 0; j < otherData->constraints.size(); j++) {
				if(otherData->constraints[j]->other.compare(node->getId()) == 0) {
					otherData->constraints[j]->other = newID;
				}
			}
		}
	}
	//remove the node and its constraints
	PhysicsCollisionObject *obj = node->getCollisionObject();
	if(obj) getPhysicsController()->removeCollisionObject(obj, true);
	_scene->removeNode(node);
	removeConstraints(node);
}

/********** PHYSICS ***********/

void T4TApp::addCollisionObject(Node *node) {
	Node::nodeData *data = node->getData();
	PhysicsRigidBody::Parameters params;
	params.mass = data->staticObj ? 0.0f : data->mass;
	if(data->objType.compare("mesh") == 0) {
		node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::mesh(node->getModel()->getMesh()), &params);
	} else if(data->objType.compare("box") == 0) {
		node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &params);
	} else if(data->objType.compare("sphere") == 0) {
		node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::sphere(), &params);
	}
}

PhysicsConstraint* T4TApp::addConstraint(Node *n1, Node *n2, const char *type, ...) {
	va_list arguments;
	va_start(arguments, type);
	Node *node[2];
	PhysicsRigidBody *body[2];
	Vector3 trans[2];
	Quaternion rot[2];
	PhysicsConstraint *ret;
	unsigned short i, j;
	for(i = 0; i < 2; i++) {
		node[i] = i == 0 ? n1 : n2;
		body[i] = node[i]->getCollisionObject()->asRigidBody();
	}
	if(strcmp(type, "hinge") == 0) {
		for(i = 0; i < 2; i++) {
			rot[i] = *((Quaternion*) va_arg(arguments, Quaternion*));
			trans[i] = *((Vector3*) va_arg(arguments, Vector3*));
		}
		ret = getPhysicsController()->createHingeConstraint(body[0], rot[0], trans[0], body[1], rot[1], trans[1]);
	} else if(strcmp(type, "socket") == 0) {
		for(i = 0; i < 2; i++) {
			trans[i] = *((Vector3*) va_arg(arguments, Vector3*));
		}
		ret = getPhysicsController()->createSocketConstraint(body[0], trans[0], body[1], trans[1]);
	} else if(strcmp(type, "fixed") == 0) {
		ret = getPhysicsController()->createFixedConstraint(body[0], body[1]);
	}
	va_end(arguments);
	for(i = 0; i < 2; i++) {
		bool exists = false; //see if this constraint is already in the node data
		Node::nodeData *data = node[i]->getData();
		unsigned short ind = data->constraints.size();
		for(j = 0; j < ind; j++) {
			if(data->constraints[j]->other.compare(node[(i+1)%2]->getId()) == 0
			  && data->constraints[j]->type.compare(type) == 0) {
				exists = true;
				break;
			}
		}
		if(!exists) {
			data->constraints.push_back(new Node::nodeConstraint());
			data->constraints[ind]->other = node[(i+1)%2]->getId();
			data->constraints[ind]->type = type;
			data->constraints[ind]->rotation = rot[i];
			data->constraints[ind]->translation = trans[i];
		}
		_constraints[node[i]].push_back(ret);
	}
	return ret;
}

void T4TApp::addConstraints(Node *node) {
	Node::nodeData *data = node->getData();
	for(int i = 0; i < data->constraints.size(); i++) {
		Node *other = _scene->findNode(data->constraints[i]->other.c_str());
		if(other == NULL) continue;
		std::string type = data->constraints[i]->type;
		Node::nodeData *otherData = other->getData();
		for(int j = 0; j < otherData->constraints.size(); j++) {
			if(otherData->constraints[j]->other.compare(node->getId()) == 0
				&& otherData->constraints[j]->type.compare(type) == 0) {
				addConstraint(node, other, type.c_str(), &data->constraints[i]->rotation, &data->constraints[i]->translation,
					&otherData->constraints[j]->rotation, &otherData->constraints[j]->translation);
			}
		}
	}
}

void T4TApp::removeConstraints(Node *node) {
	PhysicsController *controller = getPhysicsController();
	if(_constraints.find(node) == _constraints.end() || _constraints[node].size() == 0) return;
	for(PhysicsConstraint *constraint = _constraints[node].back(); constraint != NULL; constraint = _constraints[node].back()) {
		controller->removeConstraint(constraint);
		_constraints[node].pop_back();
	}
}

void T4TApp::enablePhysics(Node *node, bool enable) {
	PhysicsCollisionObject *obj = node->getCollisionObject();
	if(enable) {
		if(obj == NULL) {
			addPhysics(node);
		} else {
			PhysicsRigidBody *body = obj->asRigidBody();
			body->setActivation(ACTIVE_TAG);
			body->setEnabled(true);
		}
	} else {
		if(obj->isStatic()) {
			removePhysics(node);
		} else {
			obj->asRigidBody()->setEnabled(false);
		}
	}
}

void T4TApp::removePhysics(Node *node) {
	removeConstraints(node);
	node->setCollisionObject(PhysicsCollisionObject::NONE);
}

void T4TApp::addPhysics(Node *node) {
	addCollisionObject(node);
	addConstraints(node);
}


