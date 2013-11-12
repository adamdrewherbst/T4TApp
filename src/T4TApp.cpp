#include "T4TApp.h"
#include "Grid.h"
#include <cstdio>
#include <cmath>
#include <stdlib.h>
#include <sstream>

#define PI 3.1415926535

using std::cout;
using std::cin;
using std::endl;

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
    _scene = Scene::load("res/box.gpb");
    _scene->visit(this, &T4TApp::printNode);
    setSelected(NULL);
    
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
    _catalog = new std::vector<Node*>();
    _itemNames = new std::vector<std::string>();
    _itemNames->push_back("Box");
    _itemNames->push_back("Sphere");
    _itemNames->push_back("Cone");
    _itemCount = new std::vector<int>();
    _itemCount->resize(_itemNames->size());
    for(int i = 0; i < _itemNames->size(); i++) (*_itemCount)[i] = 0;

	//create the grid on which to place objects
    Node* node = _scene->addNode("grid");
    Model* gridModel = createGridModel();
    gridModel->setMaterial("res/common/grid.material");
    node->setModel(gridModel);
    gridModel->release();//*/
	PhysicsRigidBody::Parameters gridParams;
	gridParams.mass = 0.0f;
	node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY,
		PhysicsCollisionShape::box(),
		&gridParams);
	PhysicsRigidBody* body = (PhysicsRigidBody*)node->getCollisionObject();
	body->setEnabled(true);
	body->setLinearVelocity(Vector3(1.0f, 0.0f, 0.0f));
    //store the plane representing the grid, for calculating intersections
    _groundPlane = Plane(Vector3(0, 1, 0), 0);

    node = _scene->addNode("bigbox");
    gridModel = createBoxModel(1.0f, 2.0f, 3.0f, node);
    assert(gridModel);
    Mesh* gridMesh = gridModel->getMesh();
    Material* material = gridModel->setMaterial("res/common/sample.material#cube");
    material->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
    material->getParameter("u_lightColor")->setValue(_light->getColor());
    material->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());
    gridModel->release();
    node->translate(0.0f, 1.0f, 0.0f);
	PhysicsRigidBody::Parameters boxParams;
	boxParams.mass = 0.0f;
	node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &boxParams);
	body = ((PhysicsRigidBody*)node->getCollisionObject());
	body->setLinearVelocity(Vector3(0.0f, 0.0f, 0.0f));
	body->setRestitution(1.0f);
	body->_body->setSleepingThresholds(0.1f, 0.1f);
    cout << "mybox primitive " << gridModel->getMesh()->getPrimitiveType() << endl;
    const BoundingSphere* sphere = &gridModel->getMesh()->getBoundingSphere();
    cout << "mybox sphere " << sphere->radius << " from (" << sphere->center.x << "," << sphere->center.y << "," << sphere->center.z << ")" << endl;
	PhysicsRigidBody *body1 = body;
	body1->setEnabled(false);
	
    node = _scene->addNode("bigbox2");
    gridModel = createBoxModel(1.0f, 2.0f, 3.0f, node);
    assert(gridModel);
    gridMesh = gridModel->getMesh();
    material = gridModel->setMaterial("res/common/sample.material#cube");
    material->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
    material->getParameter("u_lightColor")->setValue(_light->getColor());
    material->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());
    //node->rotate(0.0f, 0.0f, sin(30.0f*PI/180), cos(30.0f*PI/180));
    gridModel->release();
    node->setModel(gridModel);
    node->translate(1.0f, 1.0f, 0.0f);
    boxParams.mass = 10.0f;
	node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &boxParams);
	body = ((PhysicsRigidBody*)node->getCollisionObject());
	body->setLinearVelocity(Vector3(-0.0f, 0.0f, 0.0f));
	body->setRestitution(1.0f);
	body->_body->setSleepingThresholds(0.1f, 0.1f);
	PhysicsRigidBody *body2 = body;
	body2->setEnabled(false);
	
	PhysicsHingeConstraint* constraint = getPhysicsController()->createHingeConstraint(
		body1,
		Quaternion(0.0f, 0.0f, 0.0f, 1.0f),
		Vector3(0.5f, 1.0f, 0.0f),
		body2,
		Quaternion(0.0f, 0.0f, 0.0f, 1.0f),
		Vector3(-0.5f, 1.0f, 0.0f)
	);
	constraint->setLimits(0.0f, PI, 1.0f);
	
	node->translate(0.5f, 1.5f, 0.0f);
	node->rotate(0.0f, 0.0f, sin(45.0f*PI/180), cos(45.0f*PI/180));
	body1->setEnabled(true); body2->setEnabled(true);//*/
	
    // Initialize box model
    Node* boxNode = _scene->findNode("box");
    _scene->removeNode(boxNode);
    
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

	//set up the UI for placing new objects
	for(size_t i = 0; i < _itemNames->size(); i++) { //first, a button for each possible object
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
	//checkbox for whether to snap to grid when placing/moving
	_snapToGridCheckbox = CheckBox::create("snapGrid", buttonStyle);
	_snapToGridCheckbox->setText("Snap to grid");
	_snapToGridCheckbox->setAutoWidth(true);
	_snapToGridCheckbox->setHeight(60);
	_snapToGridCheckbox->setConsumeInputEvents(false);
	_snapToGridCheckbox->addListener(this, Control::Listener::CLICK);
	_snapToGridCheckbox->setChecked(false);
	_itemSelectForm->addControl(_snapToGridCheckbox);
	//slider to set grid spacing when snap-to-grid is on
	_gridSpacingSlider = Slider::create("gridSpacing", buttonStyle);
	_gridSpacingSlider->setMin(0.1f);
	_gridSpacingSlider->setMax(2.0f);
	_gridSpacingSlider->setStep(0.1f);
	_gridSpacingSlider->setValue(1.0f);
	_gridSpacingSlider->setValueTextVisible(true);
	_gridSpacingSlider->setValueTextPrecision(1);
	_gridSpacingSlider->setText("Grid spacing");
	_gridSpacingSlider->setAutoWidth(true);
	_gridSpacingSlider->setHeight(60);
	_gridSpacingSlider->setConsumeInputEvents(false);
	_gridSpacingSlider->addListener(this, Control::Listener::CLICK);
	_gridSpacingSlider->setEnabled(false);
	_itemSelectForm->addControl(_gridSpacingSlider);
	//camera zoom slider
	_cameraZoomSlider = Slider::create("gridSpacing", buttonStyle);
	_cameraZoomSlider->setMin(1.0f);
	_cameraZoomSlider->setMax(100.0f);
	_cameraZoomSlider->setStep(1.0f);
	_cameraZoomSlider->setValue(10.0f);
	_cameraZoomSlider->setValueTextVisible(true);
	_cameraZoomSlider->setValueTextPrecision(1);
	_cameraZoomSlider->setText("Zoom");
	_cameraZoomSlider->setAutoWidth(true);
	_cameraZoomSlider->setHeight(60);
	_cameraZoomSlider->setConsumeInputEvents(false);
	_cameraZoomSlider->addListener(this, Control::Listener::CLICK);
	_itemSelectForm->addControl(_cameraZoomSlider);
	//pointer mode: select & move, rotate, constraint
	_selectMode = RadioButton::create("selectMode", buttonStyle);
	_selectMode->setGroupId("pointerMode");
	_selectMode->addListener(this, Control::Listener::CLICK);
	_selectMode->setText("Select Mode");
	_selectMode->setAutoWidth(true);
	_selectMode->setHeight(60);
	_selectMode->setConsumeInputEvents(false);
	_itemSelectForm->addControl(_selectMode);
	_rotateMode = RadioButton::create("rotateMode", buttonStyle);
	_rotateMode->setGroupId("pointerMode");
	_rotateMode->addListener(this, Control::Listener::CLICK);
	_rotateMode->setText("Rotate Mode");
	_rotateMode->setAutoWidth(true);
	_rotateMode->setHeight(60);
	_rotateMode->setConsumeInputEvents(false);
	_rotateMode->setSelected(true);
	_itemSelectForm->addControl(_rotateMode);
	_constraintMode = RadioButton::create("constraintMode", buttonStyle);
	_constraintMode->setGroupId("pointerMode");
	_constraintMode->addListener(this, Control::Listener::CLICK);
	_constraintMode->setText("Constraint Mode");
	_constraintMode->setAutoWidth(true);
	_constraintMode->setHeight(60);
	_constraintMode->setConsumeInputEvents(false);
	_itemSelectForm->addControl(_constraintMode);
	//checkbox for whether to just print object info when clicking on it
	_debugCheckbox = CheckBox::create("snapGrid", buttonStyle);
	_debugCheckbox->setText("Debug-only");
	_debugCheckbox->setAutoWidth(true);
	_debugCheckbox->setHeight(60);
	_debugCheckbox->setConsumeInputEvents(false);
	_debugCheckbox->addListener(this, Control::Listener::CLICK);
	_debugCheckbox->setChecked(false);
	_itemSelectForm->addControl(_debugCheckbox);

	//button to print out some predefined debug info
	Button* debugButton = Button::create("DebugButton", buttonStyle);
	debugButton->setText("Debug");
	debugButton->setAutoWidth(true);
	debugButton->setHeight(60);
	debugButton->setConsumeInputEvents(false);
	debugButton->addListener(this, Control::Listener::CLICK);
	_itemSelectForm->addControl(debugButton);
	debugButton->release();

	//give the form focus
    _itemSelectForm->setState(Control::FOCUS);
}

Node* T4TApp::createBoxNode(float width, float height, float depth)
{
	Model* model = createBoxModel(width, height, depth);
	Node* node = Node::create();
	node->setModel(model);
	Material* material = model->setMaterial("res/common/sample.material#cube");
	material->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
	material->getParameter("u_lightColor")->setValue(_light->getColor());
	material->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());
	PhysicsRigidBody::Parameters params;
	params.mass = 10.0f;
	node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &params);
	PhysicsRigidBody* body = (PhysicsRigidBody*) node->getCollisionObject();
	body->setEnabled(false);
	body->setRestitution(1.0f);
	return node;
}

void T4TApp::finalize()
{
    SAFE_RELEASE(_scene);
    SAFE_RELEASE(_itemSelectForm);
}

int updateCount = 0;
void T4TApp::update(float elapsedTime)
{
    // Rotate model
    //if(!paused) _scene->findNode("box")->rotateY(MATH_DEG_TO_RAD((float)elapsedTime / 1000.0f * 180.0f));
    _itemSelectForm->update(elapsedTime);
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
    getPhysicsController()->drawDebug(_scene->getActiveCamera()->getViewProjectionMatrix());

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

    if(_lastBody) {
    	int activation = _lastBody->getActivation();
    	//cout << "rendering last body: " << activation << endl;
    	if(activation == 2) _lastBody = NULL;
    }
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
	_touchPoint.set(x, y);
	bool rotate = false;
	//cout << "touch event at " << x << ", " << y << endl;
    switch (evt)
    {
    case Touch::TOUCH_PRESS:
		if(_selectMode->isSelected()) {
		    Camera* camera = _scene->getActiveCamera();
		    // Get a pick ray
		    Ray ray;
		    camera->pickRay(getViewport(), x, y, &ray);
		    // Cast a ray into the physics world to test for hits
		    PhysicsController::HitResult hitResult;
		    if (getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) {
		    	Node *node = hitResult.object->getNode();
		    	if(node->getCollisionObject() != NULL) {
					cout << "selected: " << node->getId() << endl;
					//see if this object is constrained to any others
					PhysicsRigidBody* body = node->getCollisionObject()->asRigidBody();
					cout << "\tenabled: " << body->isEnabled() << endl;
					cout << "\tkinematic: " << body->isKinematic() << endl;
					cout << "\tstatic: " << body->isStatic() << endl;
			    	if(_debugCheckbox->isChecked()) {
						cout << "\tactivation = " << body->getActivation() << endl;
			    	}
					else if(body->_constraints != NULL) for(int i = 0, size = body->_constraints->size(); i < size; i++) {
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
						body->setEnabled(false);
						node->translate(0.0f, 1.0f, 0.0f);
						body->setActivation(ACTIVE_TAG);
						body->setEnabled(true);
						cout << "\tafter: activation = " << body->getActivation() << endl;
						_lastBody = body;
					}
				}
			}
		}
		else if(_constraintMode->isSelected()) {
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
				    
				    if(_constraintNodes[0] == NULL) { //if this is the first edge, highlight it
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
				    	
				    	
				    	//rotate the second node so that the edges to be joined are aligned with each other
				    	//-first, get the xy-plane orientations of the two line segments
/*				    	nodeData *data2 = (nodeData*)_constraintNodes[0]->getUserPointer();
				    	Vector3 vert[2][2];
				    	Vector3 edgeMid[2], bodyCenter[2];
				    	for(int i = 0; i < 2; i++) {
							int v = data2->edges[_constraintEdges[0]*2+i];
							_constraintNodes[0]->getWorldMatrix().transformVector(data2->vertices[v*3], data2->vertices[v*3+1], data2->vertices[v*3+2], 1, &vert[0][i]);
				    		vert[1][i] = vertices[data->edges[minEdge*2 + i]];
				    	}
				    	float ang[2];
				    	for(int i = 0; i < 2; i++) {
				    		ang[i] = atan2(vert[i][1].z - vert[i][0].z, vert[i][1].x - vert[i][0].x);
				    		edgeMid[i] = (vert[i][0] + vert[i][1]) / 2.0f;
				    	}
				    	cout << "angles: " << _constraintNodes[0]->getId() << " at " << ang[0] << ", " << _constraintNodes[1]->getId() << " at " << ang[1] << endl;
				    	cout << "start edges: " << printVector(edgeMid[0]) << ", " << printVector(edgeMid[1]) << endl;
				    	
				    	//rotate the 2nd body around the y-axis such that the hinge edges are xz-aligned, and update its edge coords
				    	_constraintNodes[1]->rotate(Vector3(0.0f, 1.0f, 0.0f), ang[0] - ang[1]);
				    	for(int i = 0; i < 2; i++) {
							int v = data->edges[_constraintEdges[1]*2+i];
							_constraintNodes[1]->getWorldMatrix().transformVector(data->vertices[v*3], data->vertices[v*3+1], data->vertices[v*3+2], 1, &vert[1][i]);
						}
						edgeMid[1] = (vert[1][0] + vert[1][1]) / 2.0f;
				    	cout << "rotated edges: " << printVector(edgeMid[0]) << ", " << printVector(edgeMid[1]) << endl;
				    	
				    	//make sure the body's centers are on opposite sides of the hinge edge - if not, rotate the 2nd body 180 degrees
				    	Vector3 edgeDir = vert[0][1] - vert[0][0], diff, cross;
				    	edgeDir.y = 0;
				    	short sign[2];
				    	for(int i = 0; i < 2; i++) {
				    		bodyCenter[i] = _constraintNodes[i]->getTranslation();
				    		edgeMid[i] = (vert[i][0] + vert[i][1]) / 2.0f;
				    		diff = bodyCenter[i] - edgeMid[i];
				    		diff.y = 0;
				    		Vector3::cross(edgeDir, diff, &cross);
				    		sign[i] = cross.y > 0 ? 1 : -1;
				    	}
				    	if(sign[0] == sign[1]) { //body centers are on same side of edge - rotate the 2nd body 180 degrees around y-axis
				    		_constraintNodes[1]->rotate(Vector3(0.0f, 1.0f, 0.0f), PI);
				    	}
				    	
				    	//update the edge coords on the 2nd body, then translate the 2nd body to properly align the edges
				    	for(int i = 0; i < 2; i++) {
							int v = data->edges[_constraintEdges[1]*2+i];
							_constraintNodes[1]->getWorldMatrix().transformVector(data->vertices[v*3], data->vertices[v*3+1], data->vertices[v*3+2], 1, &vert[1][i]);
						}
				    	edgeMid[1] = (vert[1][0] + vert[1][1]) / 2.0f;
				    	bodyCenter[1] = _constraintNodes[1]->getTranslation();
				    	cout << "flipped edges: " << printVector(edgeMid[0]) << ", " << printVector(edgeMid[1]) << endl;
				    	_constraintNodes[1]->translate(edgeMid[0] - edgeMid[1]);
				    	
				    	//create the constraint
				    	PhysicsRigidBody *body = _constraintNodes[1]->getCollisionObject()->asRigidBody();
				    	body->setEnabled(false); body->setEnabled(true);
				    	getPhysicsController()->createHingeConstraint(
				    		(PhysicsRigidBody*)_constraintNodes[0]->getCollisionObject(),
							Quaternion(0.0f, 0.0f, 0.0f, 1.0f),
							edgeMid[0] - bodyCenter[0],
				    		(PhysicsRigidBody*)_constraintNodes[1]->getCollisionObject(),
							Quaternion(0.0f, 0.0f, 0.0f, 1.0f),
							edgeMid[1] - bodyCenter[1]
				    	);//*/
				    	_constraintNodes[0] = NULL; //and reset for next time
				    }
				}
		    }
		}
		else if(_rotateMode->isSelected()) {
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
			_scene->getActiveCamera()->pickRay(getViewport(), x, y, &ray);
			float distance = ray.intersects(_groundPlane);
			if(distance == Ray::INTERSECTS_NONE) break;
			_intersectPoint = ray.getOrigin() + ray.getDirection()*distance;
			_intersectPoint.y = (_selectedBox->max.y - _selectedBox->min.y) / 2.0f;
			//snap object to grid if desired
			if(_snapToGridCheckbox->isChecked()) {
				float spacing = _gridSpacingSlider->getValue();
				_intersectPoint.x = round(_intersectPoint.x / spacing) * spacing;
				_intersectPoint.z = round(_intersectPoint.z / spacing) * spacing;
			}
			//if would intersect another object, place it on top of that object instead
			_intersectModel = NULL;
			_scene->visit(this, &T4TApp::checkTouchModel);
			_selectedNode->setTranslation(_intersectPoint);
			PhysicsRigidBody* obj = _selectedNode->getCollisionObject()->asRigidBody();
		    break;
		}
		else if(_rotateMode->isSelected()) {
			rotate = true;
			break;
		}
    }
    case Touch::TOUCH_RELEASE:
    	_debugFlag = true;
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
			_intersectPoint.y = halfY + pos.y + bbox.max.y;
			//cout << "intersects " << node->getId() << "; moving to " << _intersectPoint.x << ", " << _intersectPoint.y << ", " << _intersectPoint.z << endl;
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

void T4TApp::controlEvent(Control* control, EventType evt)
{
	cout << "CLICKED " << control->getId() << endl;
	const size_t catalogSize = _itemNames->size();
	for(size_t i = 0; i < catalogSize; i++)
	{
		if((*_itemNames)[i].compare(control->getId()) == 0)
		{
			char shapeName[20], number[8];
			strcpy(shapeName, (*_itemNames)[i].c_str());
			cout << "selected " << shapeName << endl;
			sprintf(number, "%d", ++(*_itemCount)[i]);
			strcat(shapeName, number);
			Node* node = _scene->addNode(shapeName);
			Model* model = NULL;
			switch(i) {
				case 0: //box
					model = createBoxModel(1, 1, 1, node);
					break;
				case 1: //sphere
					model = createBoxModel(1, 1, 2, node);
					break;
				case 2: //cone
					model = createBoxModel(1, 0.5, 1, node);
					break;
				default: break;
			}
			if(model == NULL) break;
			//node->setModel(model);
			Material* material = model->setMaterial("res/common/sample.material#cube");
			material->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
			material->getParameter("u_lightColor")->setValue(_light->getColor());
			material->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());
			model->release();
			PhysicsRigidBody::Parameters params;
			params.mass = 10.0f;
			node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &params);
			PhysicsRigidBody* body = (PhysicsRigidBody*) node->getCollisionObject();
			body->setEnabled(false);
			//body->setKinematic(true);
			body->setRestitution(0.5f);
			Vector3 pos = node->getTranslation();
			PhysicsCollisionObject* obj = node->getCollisionObject();
			obj->asRigidBody()->_body->setSleepingThresholds(0.1f, 0.1f);
			cout << node->getId() << " is at " << pos.x << ", " << pos.y << ", " << pos.z << " and has shape type " << obj->getShapeType() << endl;
			setSelected(node);
			_lastNode = node;
			enableScriptCamera(false);
		}
	}
	if(control == _snapToGridCheckbox) {
		//cout << "checkbox is now " << (_snapToGridCheckbox->isChecked() ? "" : "NOT ") << "checked" << endl;
		_gridSpacingSlider->setEnabled(_snapToGridCheckbox->isChecked());
	}
	if(control == _cameraZoomSlider) {
	    getScriptController()->executeFunction<void>("camera_setRadius", "f", _cameraZoomSlider->getValue());
	}
	if(strcmp(control->getId(), "DebugButton") == 0) {
		if(_lastNode) {
			PhysicsRigidBody *body = _lastNode->getCollisionObject()->asRigidBody();
			cout << "last node " << _lastNode->getId() << " is " << (body->isEnabled() ? "" : "NOT") << " enabled" << endl;
		}
	}
}

void T4TApp::setSelected(Node* node)
{
	cout << "selecting " << (node == NULL ? "NULL" : node->getId()) << endl;
	if(node != NULL)
	{
		if(node->getCollisionObject() == NULL) return; //shouldn't select a non-physical object (like the floor grid)
		PhysicsRigidBody* body = node->getCollisionObject()->asRigidBody();
		body->setEnabled(false); //turn off physics on this body while dragging it around
		_selectedBox = &(node->getModel()->getMesh()->getBoundingBox());
	}
	else
	{
		if(_selectedNode) {
			PhysicsRigidBody* body = _selectedNode->getCollisionObject()->asRigidBody();
			body->setEnabled(true); //turn off physics on this body while dragging it around
		}
		_selectedBox = NULL;
	}
	_selectedNode = node;
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

