#include "T4TApp.h"
#include "Grid.h"
#include <cmath>
#include <stdlib.h>
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
    _scene = Scene::load("res/common/game.scene");
    _scene->visit(this, &T4TApp::printNode);
    setSelected(NULL);
    
    // Set the aspect ratio for the scene's camera to match the current resolution
    _scene->getActiveCamera()->setAspectRatio(getAspectRatio());
    
    // Get light node
    _lightNode = _scene->findNode("lightNode");
    _light = _lightNode->getLight();

	getPhysicsController()->setGravity(Vector3(0.0f, -10.0f, 0.0f));

    // Load camera script
    getScriptController()->loadScript("res/common/camera.lua");
    enableScriptCamera(true);
    //and initialize camera position by triggering a touch event
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", Touch::TOUCH_PRESS, 0, 0, 0);
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", Touch::TOUCH_MOVE, 0, 0, 0);
    getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", Touch::TOUCH_RELEASE, 0, 0, 0);

    // populate catalog of items
    _models = Scene::load("res/common/models.scene");
    
    _vehicle = Scene::load("res/common/vehicle.scene");
    _scene->addNode(_vehicle->findNode("car"));
	Node* carNode = _scene->findNode("carbody");
	if (carNode && carNode->getCollisionObject()->getType() == PhysicsCollisionObject::VEHICLE)
	{
		_carVehicle = static_cast<PhysicsVehicle*>(carNode->getCollisionObject());
	}
	_steering = _braking = _driving = 0.0f;

    //set available interaction modes
    _modeNames = new std::vector<std::string>();
    _modeNames->push_back("Rotate");
    _modeNames->push_back("Select");
    _modeNames->push_back("Constraint");
    
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
	body->setLinearVelocity(Vector3(1.0f, 0.0f, 0.0f));//*/
    //store the plane representing the grid, for calculating intersections
    _groundPlane = Plane(Vector3(0, 1, 0), 0);

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
	_submenus = new std::vector<Form*>();
	//main menu on left side [Note: this calls addRef() on formStyle's Theme, which we created above]
    _sideMenu = (Form*) addMenu(NULL, "sideMenu");
    //submenu holding catalog of T4T items
    _itemContainer = addMenu(_sideMenu, "itemContainer");
    //submenu for current interaction mode
    _modeContainer = addMenu(_sideMenu, "modeContainer");
    
    //popup menu to allow user to select a component
    _componentMenu = Form::create("componentMenu", _formStyle, Layout::LAYOUT_VERTICAL);
    _componentMenu->setPosition(_sideMenu->getX() + _sideMenu->getWidth() + 25.0f, 25.0f);
    _componentMenu->setWidth(getWidth() - _componentMenu->getX() - 25.0f);
    _componentMenu->setHeight(getHeight() - 50.0f);
    _componentMenu->setVisible(false);
    _componentMenu->setScroll(Container::SCROLL_VERTICAL);
    _componentMenu->setConsumeInputEvents(true);
    _mainMenu->addControl(_componentMenu);

    _theme->release();   // So we can release it once we're done creating forms with it.

	//populate item submenu
	_itemButton = addButton <Button> (_sideMenu, "parent_itemContainer", "Add Object >>"); //dropdown menu for object catalog

    Node *modelNode = _models->getFirstNode();
    while(modelNode) {
//	for(size_t i = 0; i < _itemNames->size(); i++) { //first, a button for each possible object
		cout << "adding button for " << modelNode->getId() << endl;
		modelNode->getCollisionObject()->setEnabled(false);
		loadNodeData(modelNode, modelNode->getId());
		Button* itemButton = addButton <Button> (_itemContainer, concat(2, "item_", modelNode->getId()));
		ImageControl* itemImage = addButton <ImageControl> (_componentMenu, concat(2, "comp_", modelNode->getId()));
		itemImage->setImage("res/png/cowboys-helmet-nobkg.png");
		itemImage->setWidth(150.0f);
		itemImage->setHeight(150.0f);
		modelNode = modelNode->getNextSibling();
	}
	//populate mode submenu
	_modeButton = addButton <Button> (_sideMenu, "parent_modeContainer", "Set Mode >>");
	for(size_t i = 0; i < _modeNames->size(); i++) {
		RadioButton* modeButton = addButton <RadioButton> (_modeContainer, (*_modeNames)[i].c_str());
		modeButton->setGroupId("interactionMode");
		modeButton->setSelected(false);
		//each mode (Rotate/Select/Constraint) has an associated option panel that only displays when that mode is selected
		Form *options = addPanel(_sideMenu, (std::string("options_") + (*_modeNames)[i]).c_str());
		switch(i) {
			case 0: //Rotate
				break;
			case 1: //Select
				//snap-to-grid checkbox
				_gridCheckbox = addControl <CheckBox> (options, "snapGrid", _buttonStyle, "Snap to grid");
				_gridCheckbox->setChecked(false);
				//slider to set grid spacing
				_gridSlider = addControl <Slider> (options, "gridSpacing", _buttonStyle, "Grid spacing");
				_gridSlider->setMin(0.1f);
				_gridSlider->setMax(2.0f);
				_gridSlider->setStep(0.1f);
				_gridSlider->setValue(1.0f);
				_gridSlider->setValueTextVisible(true);
				_gridSlider->setValueTextPrecision(1);
				_gridSlider->setEnabled(false);//*/
				break;
			case 2: //Constraint
				break;
			default: break;
		}
		options->setHeight(250.0f);
		options->setVisible(true);
		_modeOptions[(*_modeNames)[i]] = options;
	}
	
	_drawDebugCheckbox = addControl <CheckBox> (_sideMenu, "drawDebug", _buttonStyle, "Draw Debug");
	
	//create an invisible overlay the size of the screen, on which specialized touch listeners can be temporarily added
/*	_clickOverlayContainer = Form::create("clickOverlayContainer", _hiddenStyle, Layout::LAYOUT_VERTICAL);
    _clickOverlayContainer->setPosition(_sideMenu->getX() + _sideMenu->getWidth(), 0.0f);
    _clickOverlayContainer->setWidth(getWidth() - _clickOverlayContainer->getX());
    _clickOverlayContainer->setHeight(getHeight());
    _clickOverlayContainer->setScroll(Container::SCROLL_VERTICAL);
    _clickOverlayContainer->setConsumeInputEvents(true);
    _clickOverlayContainer->setVisible(false);
    _mainMenu->addControl(_clickOverlayContainer);//*/

	//create an instance of each project template - will be activated as needed
	_vehicleProject = new VehicleProject(this, "vehicleProject", _buttonStyle, _formStyle);

	_drawDebug = true;	
	setMode("Rotate");
    _sideMenu->setState(Control::FOCUS);
	//buildVehicle();

	//camera zoom slider
/*	_zoomSlider = Slider::create("gridSpacing", buttonStyle);
	_zoomSlider->setMin(1.0f);
	_zoomSlider->setMax(100.0f);
	_zoomSlider->setStep(1.0f);
	_zoomSlider->setValue(10.0f);
	_zoomSlider->setValueTextVisible(true);
	_zoomSlider->setValueTextPrecision(1);
	_zoomSlider->setText("Zoom");
	_zoomSlider->setAutoWidth(true);
	_zoomSlider->setHeight(60);
	_zoomSlider->setConsumeInputEvents(false);
	_zoomSlider->addListener(this, Control::Listener::CLICK);
	_sideMenu->addControl(_zoomSlider);
	//checkbox for whether to just print object info when clicking on it
	_debugCheckbox = CheckBox::create("snapGrid", buttonStyle);
	_debugCheckbox->setText("Debug-only");
	_debugCheckbox->setAutoWidth(true);
	_debugCheckbox->setHeight(60);
	_debugCheckbox->setConsumeInputEvents(false);
	_debugCheckbox->addListener(this, Control::Listener::CLICK);
	_debugCheckbox->setChecked(false);
	_sideMenu->addControl(_debugCheckbox);

	//button to print out some predefined debug info
	Button* debugButton = Button::create("DebugButton", buttonStyle);
	debugButton->setText("Debug");
	debugButton->setAutoWidth(true);
	debugButton->setHeight(60);
	debugButton->setConsumeInputEvents(false);
	debugButton->addListener(this, Control::Listener::CLICK);
	_sideMenu->addControl(debugButton);
	debugButton->release();
//*/
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
	    _submenus->push_back(container);
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
    parent->addControl(container);
    _mainMenu->addControl(container);
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
	_carVehicle->update(elapsedTime, _steering, _braking, _driving);
	cout << "updating car [" << elapsedTime << "]: " << _driving << ", " << _braking << ", " << _steering << endl;
	getScriptController()->executeFunction<void>("camera_update", "f", elapsedTime);

    if(_state == Game::RUNNING) {
    	//cout << "Still running... " << updateCount++ << endl;
    }else if(!_physicsStopped) {
    	_physicsStopped = true;
    	cout << "Physics stopped" << endl;
    }
    if(_lastBody) {
    	int activation = _lastBody->getActivation();
    	//cout << "updating last body: " << activation << endl;
    	if(activation == 2) _lastBody = NULL;
    }
    //cout << "update " << updateCount++ << endl;
    //usleep(500000);
}

void T4TApp::render(float elapsedTime)
{
    // Clear the color and depth buffers
    clear(CLEAR_COLOR_DEPTH, Vector4::zero(), 1.0f, 0);

    // Visit all the nodes in the scene for drawing
    _scene->visit(this, &T4TApp::drawScene);
    _vehicle->visit(this, &T4TApp::drawScene);
    if(_drawDebug) getPhysicsController()->drawDebug(_scene->getActiveCamera()->getViewProjectionMatrix());

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
    if(_componentMenu->isVisible()) _componentMenu->draw();
    if(_itemContainer->isVisible()) _itemContainer->draw();
    if(_modeContainer->isVisible()) _modeContainer->draw();
	if(_vehicleProject->container->isVisible()) _vehicleProject->container->draw();

    if(_lastBody) {
    	int activation = _lastBody->getActivation();
    	//cout << "rendering last body: " << activation << endl;
    	if(activation == 2) _lastBody = NULL;
    }
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
		if(_mode.compare("Select") == 0) {
		    Camera* camera = _scene->getActiveCamera();
		    // Get a pick ray
		    Ray ray;
		    camera->pickRay(getViewport(), x, y, &ray);
		    // Cast a ray into the physics world to test for hits
		    PhysicsController::HitResult hitResult;
		    if (getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) {
		    	Node *node = hitResult.object->getNode();
		    	if(node->getCollisionObject() != NULL) {
		    		if(node != NULL && strcmp(node->getId(), "grid") == 0) break;
					cout << "selected: " << node->getId() << endl;
					//see if this object is constrained to any others
					PhysicsRigidBody* body = node->getCollisionObject()->asRigidBody();
					cout << "\tenabled: " << body->isEnabled() << endl;
					cout << "\tkinematic: " << body->isKinematic() << endl;
					cout << "\tstatic: " << body->isStatic() << endl;
					cout << "\tactivation = " << body->getActivation() << endl;
					if(body->_constraints != NULL) for(int i = 0, size = body->_constraints->size(); i < size; i++) {
						PhysicsConstraint* constraint = (*(body->_constraints))[i];
						PhysicsRigidBody* other = constraint->_a == body ? constraint->_b : constraint->_a;
						cout << "\tconstrained to " << other->getNode()->getId() << endl;
						btHingeConstraint *_constraint = (btHingeConstraint*) constraint->_constraint;
						btTransform transform = constraint->_a == body ? _constraint->getFrameOffsetA() : _constraint->getFrameOffsetB();
						//determine the axis of rotation in world coords
						btQuaternion rotation = transform.getRotation();
						btScalar angle = rotation.getAngle();
						btVector3 axis = rotation.getAxis(), origin = transform.getOrigin();
						btTransform rot = btTransform(rotation);
						axis = rot * btVector3(0, 0, 1);
						//determine the compensatory translation to maintain the hinge alignment
						float ang = 45.0f*PI/180;
						btTransform t1 = btTransform(btQuaternion(axis, ang));
						btVector3 test = t1 * origin - origin;
						cout << "\thinge = " << angle << " about " << axis.x() << "," << axis.y() << "," << axis.z()
							<< " with origin " << origin.x() << "," << origin.y() << "," << origin.z() << endl;
						//as a test, rotate the object about the hinge by 45 degrees
						cout << "\tbefore: activation = " << body->getActivation() << endl;
						body->setEnabled(false);
						node->rotate(Vector3(axis.x(), axis.y(), axis.z()), ang);
						node->translate(-test.x(), -test.y(), -test.z());
						body->setActivation(ACTIVE_TAG);
						body->setEnabled(true);
						cout << "\tafter: activation = " << body->getActivation() << endl;
						_lastBody = body;
					}
					else {
						cout << "\tbefore: activation = " << body->getActivation() << endl;
						setSelected(node);
						//treat it as if the user clicked on the point on the grid directly below this object's center
						_dragOffset.set(0.0f, 0.0f);
						Vector3 center = node->getTranslation(), hitPoint = hitResult.point;
						Vector2 centerPix, hitPix;
						center.y = 0;
						Matrix viewProj = _scene->getActiveCamera()->getViewProjectionMatrix();
						_scene->getActiveCamera()->project(getViewport(), hitPoint, &hitPix);
						cout << "hit at " << printVector(hitPoint) << " => " << printVector2(hitPix) << endl;
						_scene->getActiveCamera()->project(getViewport(), center, &centerPix);
						_dragOffset.set(centerPix.x - x, centerPix.y - y);
						cout << "dragging " << node->getId() << " with offset " << printVector2(_dragOffset) << endl;
						body->setEnabled(false);
						_lastBody = body;
					}
				}
			}
		}
		else if(_mode.compare("Constraint") == 0) {
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
		        nodeData *data = (nodeData*)node->getUserPointer();
		        if(data != NULL) {
		        	cout << "finding closest edge" << endl;
				    Matrix world = node->getWorldMatrix();
				    Vector3 *vertices = new Vector3[data->numVertices];
				    for(int i = 0; i < data->numVertices; i++)
				    	world.transformVector(data->vertices[i*3], data->vertices[i*3+1], data->vertices[i*3+2], 1, &vertices[i]);
				    //get the closest edge to the contact point by using the vertex coords obtained above
				    Vector3 v, w, projection;
				    float t, distance, minDistance = 999999;
				    int minEdge;
				    for(int i = 0; i < data->numEdges; i++) {
				    	//cout << "testing " << i << ": " << data->edges[i*2] << "-" << data->edges[i*2+1] << endl;
				    	v = vertices[data->edges[i*2]];
				    	w = vertices[data->edges[i*2+1]];
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
				    cout << "closest edge at distance " << minDistance << ": (<" << vertices[data->edges[minEdge*2]].x << "," << vertices[data->edges[minEdge*2]].y << "," << vertices[data->edges[minEdge*2]].z << ">, <" << vertices[data->edges[minEdge*2+1]].x << "," << vertices[data->edges[minEdge*2+1]].y << "," << vertices[data->edges[minEdge*2+1]].z << ">)" << endl;
				    
				    if(_constraintNodes[0] == NULL || _constraintNodes[0] == node) { //if this is the first edge, highlight it
				    	_constraintNodes[0] = node;
				    	_constraintEdges[0] = minEdge;
				    } else { //otherwise, create the constraint
				    	_constraintNodes[1] = node;
				    	_constraintEdges[1] = minEdge;

				    	//node/edge data
				    	nodeData *dataArr[2];
				    	dataArr[0] = (nodeData*)_constraintNodes[0]->getUserPointer();
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
				    			int v = dataArr[i]->edges[_constraintEdges[i]*2 + j];
				    			localEdge[j] = Vector3(dataArr[i]->vertices[v*3], dataArr[i]->vertices[v*3+1], dataArr[i]->vertices[v*3+2]);
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
							int v = dataArr[0]->edges[_constraintEdges[0]*2+i];
							_constraintNodes[0]->getWorldMatrix().transformVector(dataArr[0]->vertices[v*3], dataArr[0]->vertices[v*3+1], dataArr[0]->vertices[v*3+2], 1, &vert[0][i]);
				    		vert[1][i] = vertices[data->edges[minEdge*2 + i]];
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
				    		int v = dataArr[1]->edges[_constraintEdges[1]*2+i];
				    		_constraintNodes[1]->getWorldMatrix().transformVector(dataArr[1]->vertices[v*3], dataArr[1]->vertices[v*3+1], dataArr[1]->vertices[v*3+2], 1, &vert[1][i]);
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
				    		int v = dataArr[1]->edges[_constraintEdges[1]*2+i];
				    		_constraintNodes[1]->getWorldMatrix().transformVector(dataArr[1]->vertices[v*3], dataArr[1]->vertices[v*3+1], dataArr[1]->vertices[v*3+2], 1, &vert[1][i]);
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
		else if(_mode.compare("Rotate") == 0) {
			rotate = true;
			break;
		}
	    _debugFlag = false;
	    break;
    case Touch::TOUCH_MOVE:
   	{
   		//if an object is currently selected, move it to the touch position (projected onto the ground plane)
    	if(_selectedNode != NULL) {
			Ray ray;
			_scene->getActiveCamera()->pickRay(getViewport(), x + _dragOffset.x, y + _dragOffset.y, &ray);
			float distance = ray.intersects(_groundPlane);
			if(distance == Ray::INTERSECTS_NONE) break;
			_intersectPoint = ray.getOrigin() + ray.getDirection()*distance;
			_intersectPoint.y = (_selectedBox->max.y - _selectedBox->min.y) / 2.0f;
			//snap object to grid if desired
			if(_gridCheckbox->isChecked()) {
				float spacing = _gridSlider->getValue();
				_intersectPoint.x = round(_intersectPoint.x / spacing) * spacing;
				_intersectPoint.z = round(_intersectPoint.z / spacing) * spacing;
			}
			//if would intersect another object, place it on top of that object instead
			_intersectModel = NULL;
			_scene->visit(this, &T4TApp::checkTouchModel);
			_selectedNode->setTranslation(_intersectPoint);
			PhysicsRigidBody* body = _selectedNode->getCollisionObject()->asRigidBody();
			body->setEnabled(true); body->setEnabled(false);
		    break;
		}
		else if(_mode.compare("Rotate") == 0) {
			rotate = true;
			break;
		}
    }
    case Touch::TOUCH_RELEASE:
    	_debugFlag = true;
    	if(_selectedNode != NULL) {
    		PhysicsRigidBody *body = _selectedNode->getCollisionObject()->asRigidBody();
    		body->setEnabled(true);
			body->setActivation(ACTIVE_TAG);
    	}
    	setSelected(NULL);
    	enableScriptCamera(true);
    	_physicsStopped = false;
    	updateCount = 0;
        break;
    };
    if(rotate) getScriptController()->executeFunction<void>("camera_touchEvent", "[Touch::TouchEvent]iiui", evt, x, y, contactIndex);
}

//see if the current touch location intersects the bottom face of the given object
bool T4TApp::checkTouchModel(Node* node)
{
	if(strcmp(node->getScene()->getId(), "models") == 0) return true;
	if(node == _selectedNode) return true;
	Vector3 pos = node->getTranslation();
	Model* model = node->getModel();
	if(model == NULL) return true;
	BoundingBox bbox = model->getMesh()->getBoundingBox();
	float halfX = (_selectedBox->max.x - _selectedBox->min.x) / 2.0f,
		halfY = (_selectedBox->max.y - _selectedBox->min.y) / 2.0f,
		halfZ = (_selectedBox->max.z - _selectedBox->min.z) / 2.0f;
	if(_intersectPoint.x + halfX > pos.x + bbox.min.x && _intersectPoint.x - halfX < pos.x + bbox.max.x
		&& _intersectPoint.z + halfZ > pos.z + bbox.min.z && _intersectPoint.z - halfZ < pos.z + bbox.max.z)
	{
		/*float distance = sqrt(pow((_intersectPoint.x - (bbox.max.x + bbox.min.x)/2.0f), 2)
			+ pow((_intersectPoint.z - (bbox.max.z + bbox.min.z)/2.0f), 2));
		float diagonal = sqrt(pow((bbox.max.x - bbox.min.x)/2.0f, 2) + pow((bbox.max.z - bbox.min.z)/2.0f, 2));//*/
		
		if(_intersectModel == NULL || halfY + pos.y + bbox.max.y > _intersectPoint.y)
		{
			_intersectModel = node;
			_intersectPoint.y = pos.y + bbox.max.y + halfY;
			//cout << "intersects " << node->getId() << " [" << pos.y << ", " << bbox.min.y << "-" << bbox.max.y << "]" << endl;
			//cout << "\tmoving to " << halfY << ": " << _intersectPoint.x << ", " << _intersectPoint.y << ", " << _intersectPoint.z << endl;
		}
	}
	return true;
}

//find the closest edge on this model to the touch point in 3D space - if it is the closest so far, store edge and distance
bool T4TApp::checkTouchEdge(Node* node)
{
	nodeData* data = (nodeData*)node->getUserPointer();
	unsigned short* edges = data->edges;
	for(int i = 0; i < data->numEdges; i++) {
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
		cout << "Looking for submenu " << subName << endl;;
		Container *subMenu = (Container*)_mainMenu->getControl(subName);
		if(subMenu != NULL) {
			bool visible = subMenu->isVisible();
			for(size_t i = 0; i < _submenus->size(); i++)
				(*_submenus)[i]->setVisible(false);
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
	//user clicked on an item in the catalog
	else if(_componentMenu->getControl(controlID) == control) {
		
	}
	else if(_itemContainer->getControl(controlID) == control) {
		Node *node = duplicateModelNode(controlID+5);
		_itemContainer->setVisible(false);
	}
	else if(control == _gridCheckbox) {
		//cout << "checkbox is now " << (_gridCheckbox->isChecked() ? "" : "NOT ") << "checked" << endl;
		_gridSlider->setEnabled(_gridCheckbox->isChecked());
	}
	else if(control == _zoomSlider) {
	    getScriptController()->executeFunction<void>("camera_setRadius", "f", _zoomSlider->getValue());
	}
	else if(strcmp(control->getId(), "DebugButton") == 0) {
		if(_lastNode) {
			PhysicsRigidBody *body = _lastNode->getCollisionObject()->asRigidBody();
			cout << "last node " << _lastNode->getId() << " is " << (body->isEnabled() ? "" : "NOT") << " enabled" << endl;
		}
	}
	else if(control == _drawDebugCheckbox) {
		_drawDebug = _drawDebugCheckbox->isChecked();
	}
}

Node* T4TApp::duplicateModelNode(const char* type, bool isStatic)
{
	Node *modelNode = _models->findNode(type);
	if(modelNode == NULL) return NULL;
	Node *node = modelNode->clone();
	nodeData *data = (nodeData*) modelNode->getUserPointer();
	const char count[2] = {(char)(++data->typeCount + 48), '\0'};
	node->setId(concat(2, modelNode->getId(), count));
	node->setUserPointer(data);
	if(isStatic) node->setCollisionObject(concat(2, "res/common/models.physics#static", modelNode->getId()));
	else node->setCollisionObject(concat(2, "res/common/models.physics#", modelNode->getId()));
	PhysicsRigidBody* body = node->getCollisionObject()->asRigidBody();
	body->addCollisionListener(this);
	body->_body->setSleepingThresholds(0.1f, 0.1f);
	body->setActivation(ACTIVE_TAG);
	_scene->addNode(node);
	Node *curSelected = _selectedNode;
	setSelected(node);
	placeSelected(0.0f, 0.0f);
	setSelected(curSelected);
	cout << "ADDED NODE " << node->getId() << endl;
	return node;
}

void T4TApp::loadNodeData(Node *node, const char *type)
{
	char *filename = concat(3, "res/common/", type, ".node");
	cout << "reading " << node->getId() << " from file " << filename << endl;
	std::auto_ptr<Stream> stream(FileSystem::open(filename));
	if (stream.get() == NULL)
	{
		GP_ERROR("Failed to open file '%s'.", filename);
		return;
	}
	
	nodeData* data = new nodeData();
	data->typeCount = 0;

	char *str, line[2048];
	istringstream in;
    str = stream->readLine(line, 2048);
    int nv = atoi(str), v = 0;
    cout << nv << " vertices" << endl;
    data->numVertices = nv;
    data->vertices = (float*) malloc(3 * nv * sizeof(float));
    for(int i = 0; i < nv; i++) {
    	str = stream->readLine(line, 2048);
    	in.str(str);
    	in >> data->vertices[v++] >> data->vertices[v++] >> data->vertices[v++];
    }
    str = stream->readLine(line, 2048);
    int ne = atoi(str), e = 0;
    cout << ne << " edges" << endl;
    data->numEdges = ne;
    data->edges = (unsigned short*) malloc(2* ne * sizeof(unsigned short));
    for(int i = 0; i < ne; i++) {
    	str = stream->readLine(line, 2048);
    	in.str(str);
    	in >> data->edges[e++] >> data->edges[e++];
    }
	node->setUserPointer(data);
    stream->close();//*/
}

//place the selected object at the given xz-coords and set its y-coord so it is on top of any objects it would otherwise intersect
void T4TApp::placeSelected(float x, float z)
{
	float minY = _selectedNode->getModel()->getMesh()->getBoundingBox().min.y;
	_selectedNode->setTranslation(x, -minY, z); //put the bounding box bottom on the ground
	_intersectPoint.set(x, -minY, z);
	_scene->visit(this, &T4TApp::checkTouchModel); //will change _intersectPoint.y to be above any intersecting models
	_selectedNode->setTranslation(_intersectPoint);
}

void T4TApp::setSelected(Node* node)
{
	if(node != NULL)
	{
		if(strcmp(node->getId(), "grid") == 0) return;
		if(node->getCollisionObject() == NULL) return; //shouldn't select a non-physical object (like the floor grid)
		cout << "selecting " << node->getId() << endl;
		PhysicsRigidBody* body = node->getCollisionObject()->asRigidBody();
		body->setEnabled(false); //turn off physics on this body while dragging it around
		_selectedBox = &(node->getModel()->getMesh()->getBoundingBox());
	}
	else
	{
		cout << "selecting NULL" << endl;
		if(_selectedNode) {
			PhysicsRigidBody* body = _selectedNode->getCollisionObject()->asRigidBody();
			body->setEnabled(true); //turn off physics on this body while dragging it around
		}
		_selectedBox = NULL;
	}
	_selectedNode = node;
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

void T4TApp::enableScriptCamera(bool enable)
{
	getScriptController()->executeFunction<void>("camera_setActive", "b", enable);
}

const std::string T4TApp::printVector(Vector3& v) {
	std::ostringstream os;
	os << "<" << v.x << "," << v.y << "," << v.z << ">";
	return os.str();
}

const std::string T4TApp::printVector2(Vector2& v) {
	std::ostringstream os;
	os << "<" << v.x << "," << v.y << ">";
	return os.str();
}

//go through the interactive steps to add all required components to a vehicle
void T4TApp::buildVehicle() {
	cout << "starting vehicle" << endl;
	_vehicleProject->setActive(true);
}

void T4TApp::VehicleProject::setActive(bool active) {
	cout << "adding vehicle listener" << endl;
	//app->_componentMenu->addListener(_vehicleListener, Control::Listener::CLICK);
	app->_componentMenu->setVisible(active);
	if(active) {
		app->_componentMenu->setState(Control::FOCUS);
		app->_mainMenu->addListener(this, Control::Listener::CLICK);
		app->_componentMenu->addListener(this, Control::Listener::CLICK);
	}else {
		app->_componentMenu->setState(Control::NORMAL);
		app->_mainMenu->removeListener(this);
		app->_componentMenu->removeListener(this);
	}
	const std::vector<Control*> buttons = app->_componentMenu->getControls();
	for(std::vector<Control*>::size_type i = 0; i != buttons.size(); i++) {
		if(active) {
			buttons[i]->addListener(this, Control::Listener::CLICK);
			cout << "added listener to " << buttons[i]->getId() << endl;
		}
		else {
			buttons[i]->removeListener(this);
			cout << "removed listener from " << buttons[i]->getId() << endl;
		}
	}
}

Node* T4TApp::promptComponent() {
	_componentMenu->setVisible(true);
}

void T4TApp::VehicleProject::controlEvent(Control* control, EventType evt) {
	const char *controlID = control->getId();
	cout << "CLICKED " << controlID << endl;
	if(strncmp(controlID, "comp_", 5) == 0) {
		Node *node = app->duplicateModelNode(controlID+5, true);
		if(node != NULL) {
			bool found = true;
			//node->getCollisionObject()->setEnabled(false);
			switch(_currentComponent) {
				case CHASSIS:
					_chassisNode = node;
					break;
				case FRONT_WHEELS:
					break;
				case BACK_WHEELS:
					break;
				default: found = false; break;
			}
			if(found) {
				if(_currentComponent != COMPLETE) {
					addListener(this, Control::Listener::CLICK);
					//app->_clickOverlayContainer->setVisible(true);
					container->setVisible(true);
				}
			}
			app->_componentMenu->setVisible(false);
		}
	}
}

bool T4TApp::VehicleProject::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
	bool componentDone = false;
	switch(evt)
	{
		case Touch::TOUCH_PRESS:
			cout << "clicked overlay at " << x << "," << y << endl;
			//place the vehicle part at the chosen location
			switch(_currentComponent) {
				case CHASSIS:
					componentDone = true;
					break;
				//for wheels, make sure they are transverse to the chassis' long axis which is Z
				case FRONT_WHEELS:
				case BACK_WHEELS: {
					Camera* camera = app->_scene->getActiveCamera();
					Ray ray;
					camera->pickRay(app->getViewport(), x, y, &ray);
					Vector3 origin = ray.getOrigin(), direction = ray.getDirection(),
						camPos = camera->getNode()->getTranslation(), chassisPos = _chassisNode->getTranslation();
					cout << "camera: " << app->printVector(camPos) << endl;
					cout << "chassis: " << app->printVector(chassisPos) << endl;
					cout << "ray: " << app->printVector(origin) << " => " << app->printVector(direction) << endl;
					PhysicsController::HitResult hitResult;
					if (!app->getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) {
						cout << "didn't hit object" << endl;
						break;
					}
					Node *node = hitResult.object->getNode();
					if(node != _chassisNode) {
						break;
						cout << "hit " << node->getId() << ", not " << _chassisNode->getId() << endl;
					}
					Vector3 pt = hitResult.point;
					componentDone = true;
					} break;
				default: break;
			}
			if(componentDone) {
				advanceComponent();
				cout << "Moving on to " << componentName[_currentComponent] << endl;
				removeListener(this);
				//app->_clickOverlayContainer->setVisible(false);
				container->setVisible(false);
				if(_currentComponent != COMPLETE) {
					//bring up the menu to select the next vehicle part
					app->_componentMenu->setVisible(true);
				}
			}
			break;
		case Touch::TOUCH_RELEASE:
			break;
		default: break;
	}
	return true;
}

char* T4TApp::concat(int n, ...)
{
	const char** strings = new const char*[n];
	int length = 0;
	va_list arguments;
	va_start(arguments, n);
	for(int i = 0; i < n; i++) {
		strings[i] = (const char*) va_arg(arguments, const char*);
		length += strlen(strings[i]);
	}
	char* dest = new char[length+1];
	dest[0] = '\0';
	for(int i = 0; i < n; i++) strcat(dest, strings[i]);
	dest[length] = '\0';
	return dest;
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

