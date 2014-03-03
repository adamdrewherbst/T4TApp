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
	_models->addNode("box");//*/

	//populate item submenu
	_itemButton = addButton <Button> (_sideMenu, "parent_itemContainer", "Add Object >>"); //dropdown menu for object catalog

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
			Node::nodeData *data = (Node::nodeData*)modelNode->getUserPointer();
			data->type = modelNode->getId();
		}
		modelNode->setTranslation(Vector3(1000.0f,0.0f,0.0f));
		modelNode = modelNode->getNextSibling();
	}

    //populate simple machine submenu
    _machines.push_back(new Lever(this, _buttonStyle, _formStyle));
    _machineNames.push_back("Lever");
    _machineNames.push_back("Lever");
    _machineButton = addButton <Button> (_sideMenu, "parent_machineContainer", "Add Machine >>");
    for(size_t i = 0; i < _machineNames.size(); i++) {
    	Button *machineButton = addButton <Button> (_machineContainer, _machineNames[i].c_str());
    }

	//populate mode submenu
/*	_modeButton = addButton <Button> (_sideMenu, "parent_modeContainer", "Set Mode >>");
    _modeNames.push_back("Rotate");
    _modeNames.push_back("Select");
    _modeNames.push_back("Constraint");
    _modeNames.push_back("Ball Drop");
	for(size_t i = 0; i < _modeNames.size(); i++) {
		RadioButton* modeButton = addButton <RadioButton> (_modeContainer, _modeNames[i].c_str());
		modeButton->setGroupId("interactionMode");
		modeButton->setSelected(false);
		//each mode (Rotate/Select/Constraint) has an associated option panel that only displays when that mode is selected
		Form *options = addPanel(_sideMenu, concat(2, "options_", _modeNames[i].c_str()));
		switch(i) {
			case 0: //Rotate
				break;
			case 1: //Select
				break;
			case 2: //Constraint
				break;
			default: break;
		}
		options->setHeight(250.0f);
		options->setVisible(true);
		_sideMenu->removeControl(options);
		_modeOptions[_modeNames[i]] = options;
	}
//*/
	_modes.push_back(new RotateMode(this));
	_modes.push_back(new SelectMode(this));	
	_modes.push_back(new SliceMode(this));
	_modes.push_back(new DrillMode(this));
	_modes.push_back(new TestMode(this));
	_modePanel = addPanel(_sideMenu, "container_modes");
	for(size_t i = 0; i < _modes.size(); i++) {
		const char *id = _modes[i]->getId();
		Button *modeButton = addControl <Button> (_modePanel, id, _buttonStyle, id+5);
		modeButton->setHeight(40.0f);
	}
	_modePanel->setHeight(250.0f);
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
		if(_modes[i]->_active && _modes[i]->_controls != NULL) {
			_modes[i]->_controls->draw();
		}
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
	bool rotate = false;
	//cout << "touch event at " << x << ", " << y << endl;
    switch (evt)
    {
    case Touch::TOUCH_PRESS:
		if(_mode.compare("Constraint") == 0) {
			//see if the touch point intersects an edge
			//_scene->visit(this, &T4TApp::checkTouchEdge);
		    Camera* camera = _scene->getActiveCamera();

		    // Get a pick ray
		    Ray ray;
		    camera->pickRay(getViewport(), x, y, &ray);

		    // Cast a ray into the physics world to test for hits
		    PhysicsController::HitResult hitResult;
		    if (getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult))
		    {
		        // Get the node we touched
		        Node* node = hitResult.object->getNode();
		        if(node != NULL && strcmp(node->getId(), "grid") == 0) break;
		        // Get the exact point touched, in world space
		        Vector3 p = hitResult.point;
		        cout << "hit " << node->getId() << " at " << p.x << "," << p.y << "," << p.z << endl;
		        //the vertex data is in a GL buffer, so to get it, we have to transform the model vertices
		        Node::nodeData *data = (Node::nodeData*)node->getUserPointer();
		        if(data != NULL) {
			        node->updateData();
		        	cout << "finding closest edge" << endl;
				    //get the closest edge to the contact point by using the vertex coords obtained above
				    Vector3 v, w, projection;
				    float t, distance, minDistance = 999999;
				    int minEdge;
				    for(int i = 0; i < data->edges.size(); i++) {
				    	//cout << "testing " << i << ": " << data->edges[i*2] << "-" << data->edges[i*2+1] << endl;
				    	v = data->worldVertices[data->edges[i][0]];
				    	w = data->worldVertices[data->edges[i][1]];
					    const float l2 = (v - w).lengthSquared();
						const float t = (p - v).dot(w - v) / l2;
						if (t < 0.0) distance = p.distance(v);       // Beyond the 'v' end of the segment
						else if (t > 1.0) distance = p.distance(w);  // Beyond the 'w' end of the segment
						else {
							projection = v + ((w - v) * t);  // Projection falls on the segment
							distance = p.distance(projection);
						}
						if(distance < minDistance) {
							minDistance = distance;
							minEdge = i;
						}
				    }
				    cout << "closest edge at distance " << minDistance << ": (" << printVector(data->worldVertices[data->edges[minEdge][0]]) << ", " << printVector(data->worldVertices[data->edges[minEdge][1]]) << ")" << endl;
				    
				    if(_constraintNodes[0] == NULL || _constraintNodes[0] == node) { //if this is the first edge, highlight it
				    	_constraintNodes[0] = node;
				    	_constraintEdges[0] = minEdge;
				    } else { //otherwise, create the constraint
				    	_constraintNodes[1] = node;
				    	_constraintEdges[1] = minEdge;

				    	//node/edge data
				    	Node::nodeData *dataArr[2];
				    	dataArr[0] = (Node::nodeData*)_constraintNodes[0]->getUserPointer();
				    	dataArr[1] = data;
				    	Vector3 vert[2][2], edgeDir[2], edgeMid[2], bodyCenter[2];
				    	//for calculations
				    	Vector3 cross, rotAxis;
				    	float rotAngle;
				    	
				    	//find the rotation and translation from each body's local z-axis to the hinge edge axis
				    	//mainly to be used when creating the constraint
				    	Quaternion localRot[2];
				    	Vector3 localTrans[2], localEdge[2], localEdgeDir[2], localZ = Vector3(0.0f, 0.0f, 1.0f);
				    	for(int i = 0; i < 2; i++) {
				    		for(int j = 0; j < 2; j++) {
				    			int v = dataArr[i]->edges[_constraintEdges[i]][j];
				    			localEdge[j] = dataArr[i]->vertices[v];
				    		}
				    		cout << "edge on " << _constraintNodes[i]->getId() << ": (" << printVector(localEdge[0]) << ", " << printVector(localEdge[1]) << ")" << endl;
				    		localTrans[i] = (localEdge[0] + localEdge[1]) / 2.0f;
				    		localEdgeDir[i] = localEdge[1] - localEdge[0];
				    		Vector3::cross(localZ, localEdgeDir[i], &cross);
				    		localRot[i] = Quaternion(cross.x, cross.y, cross.z,
				    			sqrt(localEdgeDir[i].lengthSquared()) + Vector3::dot(localZ, localEdgeDir[i]));
				    		localRot[i].normalize();
				    		
				    		rotAngle = localRot[i].toAxisAngle(&rotAxis);
				    		cout << "rotation by " << rotAngle << " about " << printVector(rotAxis) << endl;
				    		cout << "translation by " << printVector(localTrans[i]) << endl;
				    	}

				    	//transform the edges' vertices to world space
				    	for(int i = 0; i < 2; i++) {
							int v = dataArr[0]->edges[_constraintEdges[0]][i];
							_constraintNodes[0]->getWorldMatrix().transformVector(dataArr[0]->vertices[v], &vert[0][i]);
				    		vert[1][i] = data->worldVertices[data->edges[minEdge][i]];
				    	}
				    	for(int i = 0; i < 2; i++) edgeDir[i] = vert[i][1] - vert[i][0];

				    	//find the rotation quaternion between the 2 edges to be joined
				    	Vector3::cross(edgeDir[1], edgeDir[0], &cross);
				    	Quaternion rot(cross.x, cross.y, cross.z,
				    		sqrt(edgeDir[0].lengthSquared() * edgeDir[1].lengthSquared()) + Vector3::dot(edgeDir[0], edgeDir[1]));
				    	rot.normalize();
				    	
				    	rotAngle = rot.toAxisAngle(&rotAxis);
				    	cout << "need to rotate " << _constraintNodes[1]->getId() << " by " << rotAngle << " about " << printVector(rotAxis) << endl;
				    	
				    	//perform the rotation
				    	Quaternion curRot = _constraintNodes[1]->getRotation();
				    	_constraintNodes[1]->setRotation(rot);
				    	_constraintNodes[1]->rotate(curRot);
				    	
				    	//recalculate the edge coords
				    	for(int i = 0; i < 2; i++) {
				    		int v = dataArr[1]->edges[_constraintEdges[1]][i];
				    		_constraintNodes[1]->getWorldMatrix().transformVector(dataArr[1]->vertices[v], &vert[1][i]);
				    	}
				    	for(int i = 0; i < 2; i++) {
				    		edgeMid[i] = (vert[i][0] + vert[i][1]) / 2.0f;
				    		edgeDir[i] = vert[i][1] - vert[i][0];
				    		bodyCenter[i] = _constraintNodes[i]->getTranslation();
				    	}
				    	
				    	//rotate the 2nd body about the hinge axis to be opposite the 1st body
				    	Vector3 bodyToEdge[2], bodyToEdgePerp[2];
				    	for(int i = 0; i < 2; i++) {
				    		bodyToEdge[i] = edgeMid[i] - bodyCenter[i];
				    		bodyToEdgePerp[i] = bodyToEdge[i] - (edgeDir[i] * (Vector3::dot(bodyToEdge[i], edgeDir[i]) / edgeDir[i].lengthSquared()));
				    	}
				    	Vector3::cross(bodyToEdgePerp[1], bodyToEdgePerp[0] * -1.0f, &cross);
				    	rot.set(cross.x, cross.y, cross.z,
				    		sqrt(bodyToEdgePerp[0].lengthSquared() * bodyToEdgePerp[1].lengthSquared()) + Vector3::dot(bodyToEdgePerp[0], bodyToEdgePerp[1]));
				    	rot.normalize();
				    	rotAngle = rot.toAxisAngle(&rotAxis);
				    	_constraintNodes[1]->rotate(localEdgeDir[1], rotAngle);
				    	
				    	//recalculate the edge coords
				    	for(int i = 0; i < 2; i++) {
				    		int v = dataArr[1]->edges[_constraintEdges[1]][i];
				    		_constraintNodes[1]->getWorldMatrix().transformVector(dataArr[1]->vertices[v], &vert[1][i]);
				    	}
				    	edgeMid[1] = (vert[1][0] + vert[1][1]) / 2.0f;

				    	//translate the 2nd body so their edge midpoints touch
				    	_constraintNodes[1]->translate(edgeMid[0] - edgeMid[1]);
				    	
				    	//create the hinge constraint
				    	getPhysicsController()->createHingeConstraint(
				    		_constraintNodes[0]->getCollisionObject()->asRigidBody(),
				    		localRot[0],
				    		localTrans[0],
				    		_constraintNodes[1]->getCollisionObject()->asRigidBody(),
				    		localRot[1],
				    		localTrans[1]
				    	);
				    	
				    	//update the collision object for the 2nd body
				    	PhysicsRigidBody *body = _constraintNodes[1]->getCollisionObject()->asRigidBody();
				    	body->setEnabled(false); body->setEnabled(true); body->setActivation(ACTIVE_TAG);
				    	_constraintNodes[0] = NULL; //and reset for next time
				    }
				}
		    }
		}
		else if(_mode.compare("Ball Drop") == 0) {
			Ray ray;
			Plane vertical(Vector3(0, 0, 1), 0);
			_scene->getActiveCamera()->pickRay(getViewport(), x, y, &ray);
			float distance = ray.intersects(vertical);
			if(distance != Ray::INTERSECTS_NONE) {
				float worldX = ray.getOrigin().x + ray.getDirection().x * distance;
				Node *node = duplicateModelNode("sphere");
				addCollisionObject(node);
				PhysicsRigidBody *body = node->getCollisionObject()->asRigidBody();
				body->setEnabled(false);
				node->setTranslation(worldX, 10.0f, 0.0f);
				body->setEnabled(true);
				_scene->addNode(node);
			}
		}
	    _debugFlag = false;
	    break;
    case Touch::TOUCH_MOVE:
   	{
    }
    case Touch::TOUCH_RELEASE:
    	_debugFlag = true;
    	enableScriptCamera(true);
    	_physicsStopped = false;
    	updateCount = 0;
        break;
    };
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
	for(int i = 0; i < _intersectNodeGroup.size(); i++)
		if(node == _intersectNodeGroup[i]) return true;
	Vector3 pos = node->getTranslation();
	Model* model = node->getModel();
	if(model == NULL) return true;
	BoundingBox bbox = model->getMesh()->getBoundingBox();
	float halfX = (_intersectBox->max.x - _intersectBox->min.x) / 2.0f,
		halfY = (_intersectBox->max.y - _intersectBox->min.y) / 2.0f,
		halfZ = (_intersectBox->max.z - _intersectBox->min.z) / 2.0f;
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

//find the closest edge on this model to the touch point in 3D space - if it is the closest so far, store edge and distance
bool T4TApp::checkTouchEdge(Node* node)
{
	Node::nodeData* data = (Node::nodeData*)node->getUserPointer();
	for(int i = 0; i < data->edges.size(); i++) {
		//get starting point and direction vector of camera sight, and edge
	}
	return false;
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
	//user clicked on an item in the catalog
	else if(_componentMenu->getControl(controlID) == control) {
		//now handled by individual project components
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
	Node::nodeData *data = (Node::nodeData*) modelNode->getUserPointer();
	node->setUserPointer(data);
	clone->release();
}

Node* T4TApp::duplicateModelNode(const char* type, bool isStatic)
{
	Node *modelNode = _models->findNode(type);
	if(modelNode == NULL) return NULL;
	Node *node = modelNode->clone();
	BoundingBox box = node->getModel()->getMesh()->getBoundingBox();
	node->setTranslation(Vector3(0.0f, (box.max.y - box.min.y)/2.0f, 0.0f));
	Node::nodeData *data = (Node::nodeData*) modelNode->getUserPointer();
	const char count[2] = {(char)(++data->typeCount + 48), '\0'};
	node->setId(concat(2, modelNode->getId(), count));
	node->loadData(concat(3, "res/common/", type, ".node"));
	node->updateData();
	data = (Node::nodeData*)node->getUserPointer();
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

void T4TApp::addCollisionObject(Node *node) {
	Node::nodeData *data = (Node::nodeData*)node->getUserPointer();
	PhysicsRigidBody::Parameters params;
	params.mass = data->mass;
	if(data->objType.compare("mesh") == 0) {
		node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::mesh(node->getModel()->getMesh()), &params);
	} else if(data->objType.compare("box") == 0) {
		node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &params);
	} else if(data->objType.compare("sphere") == 0) {
		node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::sphere(), &params);
	}
}

//place at the given xz-coords and set its y-coord so it is on top of any objects it would otherwise intersect
void T4TApp::placeNode(Node *node, float x, float z)
{
	_intersectNodeGroup.clear();
	_intersectNodeGroup.push_back(node);
	_intersectBox = &node->getModel()->getMesh()->getBoundingBox();
	float minY = _intersectBox->min.y;
	node->setTranslation(x, -minY, z); //put the bounding box bottom on the ground
	_intersectPoint.set(x, -minY, z);
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
	Node::nodeData *data = (Node::nodeData*)node->getUserPointer();
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

PhysicsConstraint* T4TApp::addConstraint(Node *n1, Node *n2, const char *type, ...) {
	va_list arguments;
	va_start(arguments, type);
	Node *node[2];
	PhysicsConstraint *ret;
	for(int i = 0; i < 2; i++) node[i] = i == 0 ? n1 : n2;
	if(strcmp(type, "hinge") == 0) {
		Quaternion rot[2];
		Vector3 trans[2];
		PhysicsRigidBody *body[2];
		for(int i = 0; i < 2; i++) {
			rot[i] = *((Quaternion*) va_arg(arguments, Quaternion*));
			trans[i] = *((Vector3*) va_arg(arguments, Vector3*));
			body[i] = node[i]->getCollisionObject()->asRigidBody();
			Node::nodeData *data = (Node::nodeData*)node[i]->getUserPointer();
			int ind = data->constraints.size();
			bool exists = false; //see if this constraint is already in the node data
			for(int j = 0; j < ind; j++) {
				if(data->constraints[j]->other.compare(node[(i+1)%2]->getId()) == 0
					&& data->constraints[j]->type.compare(type) == 0) exists = true;
			}
			if(!exists) {
				data->constraints.push_back(new Node::nodeConstraint());
				data->constraints[ind]->other = node[(i+1)%2]->getId();
				data->constraints[ind]->type = type;
				data->constraints[ind]->rotation = rot[i];
				data->constraints[ind]->translation = trans[i];
			}
		}
		ret = getPhysicsController()->createHingeConstraint(body[0], rot[0], trans[0], body[1], rot[1], trans[1]);
	}
	va_end(arguments);
	for(int i = 0; i < 2; i++) _constraints[node[i]].push_back(ret);
	return ret;
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
		Node::nodeData *data = (Node::nodeData*)node->getUserPointer();
		for(int i = 0; i < data->constraints.size(); i++) {
			Node *other = _scene->findNode(data->constraints[i]->other.c_str());
			if(other == NULL) continue;
			Node::nodeData *otherData = (Node::nodeData*)other->getUserPointer();
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

void T4TApp::addConstraints(Node *node) {
	Node::nodeData *data = (Node::nodeData*)node->getUserPointer();
	for(int i = 0; i < data->constraints.size(); i++) {
		Node *other = _scene->findNode(data->constraints[i]->other.c_str());
		if(other == NULL) continue;
		std::string type = data->constraints[i]->type;
		Node::nodeData *otherData = (Node::nodeData*)other->getUserPointer();
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


