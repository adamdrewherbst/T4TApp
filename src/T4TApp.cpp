#include "T4TApp.h"
#include "Grid.h"
#include <cstdio>
#include <cmath>
#include <stdlib.h>

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
    Model* gridModel = createGridModel();
    assert(gridModel);
    gridModel->setMaterial("res/common/grid.material");
    Node* node = _scene->addNode("grid");
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

    gridModel = createBoxModel(1.0f, 2.0f, 3.0f);
    Mesh* gridMesh = gridModel->getMesh();
    assert(gridModel);
    Material* material = gridModel->setMaterial("res/common/sample.material#cube");
    material->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
    material->getParameter("u_lightColor")->setValue(_light->getColor());
    material->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());
    node = _scene->addNode("bigbox");
    node->setModel(gridModel);
    node->translate(0.0f, 1.0f, 0.0f);
    gridModel->release();
	PhysicsRigidBody::Parameters boxParams;
	boxParams.mass = 0.0f;
	node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &boxParams);
	body = ((PhysicsRigidBody*)node->getCollisionObject());
	body->setLinearVelocity(Vector3(0.0f, 0.0f, 0.0f));
	body->setRestitution(1.0f);
    cout << "mybox primitive " << gridModel->getMesh()->getPrimitiveType() << endl;
    const BoundingSphere* sphere = &gridModel->getMesh()->getBoundingSphere();
    cout << "mybox sphere " << sphere->radius << " from (" << sphere->center.x << "," << sphere->center.y << "," << sphere->center.z << ")" << endl;
	PhysicsRigidBody *body1 = body;
	body1->setEnabled(false);
	
    gridModel = createBoxModel(1.0f, 2.0f, 3.0f);
    gridMesh = gridModel->getMesh();
    assert(gridModel);
    material = gridModel->setMaterial("res/common/sample.material#cube");
    material->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
    material->getParameter("u_lightColor")->setValue(_light->getColor());
    material->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());
    node = _scene->addNode("bigbox2");
    node->setModel(gridModel);
    node->translate(1.0f, 1.0f, 0.0f);
    //node->rotate(0.0f, 0.0f, sin(30.0f*PI/180), cos(30.0f*PI/180));
    gridModel->release();
    boxParams.mass = 10.0f;
	node->setCollisionObject(PhysicsCollisionObject::RIGID_BODY, PhysicsCollisionShape::box(), &boxParams);
	body = ((PhysicsRigidBody*)node->getCollisionObject());
	body->setLinearVelocity(Vector3(-0.0f, 0.0f, 0.0f));
	body->setRestitution(1.0f);//*/
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
	getPhysicsController()->setGravity(Vector3(0.0f, -0.5f, 0.0f));//*/
	body1->setEnabled(true); body2->setEnabled(true);
	//body1->setEnabled(false); body2->setEnabled(false);
	
    // Initialize box model
    Node* boxNode = _scene->findNode("box");
    _scene->removeNode(boxNode);
    /*Model* boxModel = boxNode->getModel(); //createBoxModel(0.5f, 0.5f, 0.5f);
    Mesh* boxMesh = boxModel->getMesh();
    Material* boxMaterial = boxModel->setMaterial("res/common/box.material");
    //boxNode->setModel(boxModel);
    //_catalog->push_back(boxNode->getModel()->getMesh());
    boxMaterial->getParameter("u_ambientColor")->setValue(_scene->getAmbientColor());
    boxMaterial->getParameter("u_lightColor")->setValue(_light->getColor());
    boxMaterial->getParameter("u_lightDirection")->setValue(_lightNode->getForwardVectorView());
    cout << "hisbox primitive " << boxModel->getMesh()->getPrimitiveType() << endl;
    sphere = &boxModel->getMesh()->getBoundingSphere();
    cout << "hisbox sphere " << sphere->radius << " from (" << sphere->center.x << "," << sphere->center.y << "," << sphere->center.z << ")" << endl;//*/
    
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

void T4TApp::update(float elapsedTime)
{
    // Rotate model
    //if(!paused) _scene->findNode("box")->rotateY(MATH_DEG_TO_RAD((float)elapsedTime / 1000.0f * 180.0f));
    _itemSelectForm->update(elapsedTime);
    getScriptController()->executeFunction<void>("camera_update", "f", elapsedTime);
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
					for(int i = 0, size = body->_constraints->size(); i < size; i++) {
						PhysicsConstraint* constraint = (*(body->_constraints))[i];
						PhysicsRigidBody* other = constraint->_a == body ? constraint->_b : constraint->_a;
						cout << "\tconstrained to " << other->getNode()->getId() << endl;
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
				    Matrix world = node->getWorldMatrix();
				    Vector3 *vertices = new Vector3[data->numVertices];
				    for(int i = 0; i < data->numVertices; i++)
				    	world.transformVector(data->vertices[i*3], data->vertices[i*3+1], data->vertices[i*3+2], 1, &vertices[i]);
				    //get the closest edge to the contact point by using the vertex coords obtained above
				    Vector3 v, w, projection;
				    float t, distance, minDistance = 999999;
				    int minEdge;
				    for(int i = 0; i < data->numEdges; i++) {
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
				    	/*getPhysicsController()->createHingeConstraint(
				    		(PhysicsRigidBody*)_constraintNodes[0]->getCollisionObject(),//*/
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
    	if(_selectedNode != NULL) {
    		PhysicsRigidBody* obj = _selectedNode->getCollisionObject()->asRigidBody();
    		obj->setEnabled(true);
    		//obj->setKinematic(false);
    	}
    	_debugFlag = true;
    	setSelected(NULL);
    	enableScriptCamera(true);
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
			cout << node->getId() << " is at " << pos.x << ", " << pos.y << ", " << pos.z << " and has shape type " << obj->getShapeType() << endl;
			setSelected(node);
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
}

void T4TApp::setSelected(Node* node)
{
	if(node != NULL)
	{
		if(node->getCollisionObject() == NULL) return; //shouldn't select a non-physical object (like the floor grid)
		PhysicsRigidBody* body = node->getCollisionObject()->asRigidBody();
		body->setEnabled(false); //turn off physics on this body while dragging it around
		_selectedBox = &(node->getModel()->getMesh()->getBoundingBox());
	}
	else
	{
		_selectedBox = NULL;
	}
	_selectedNode = node;
}

void T4TApp::enableScriptCamera(bool enable)
{
	getScriptController()->executeFunction<void>("camera_setActive", "b", enable);
}

