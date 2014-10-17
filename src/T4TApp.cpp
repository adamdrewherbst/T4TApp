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
	//generateModels();
	
    // Load font
    _font = Font::create("res/common/arial18.gpb");
    assert(_font);
    
    initScene();
    
    cout << "cam at: " << pcam(_cameraState) << endl;
    
	getPhysicsController()->setGravity(Vector3(0.0f, -10.0f, 0.0f));

	_steering = _braking = _driving = 0.0f;
    
    //create the form for selecting catalog items
    _theme = Theme::create("res/common/default.theme");
    _formStyle = _theme->getStyle("basicContainer");
    _hiddenStyle = _theme->getStyle("hiddenContainer");
    _buttonStyle = _theme->getStyle("buttonStyle");
    _titleStyle = _theme->getStyle("title");

	/*********************** GUI SETUP ***********************/
	
	//root menu node for finding controls by ID
	_mainMenu = Form::create("res/common/main.form");
	_sideMenu = (Container*)_mainMenu->getControl("sideMenu");
	_stage = (Container*)_mainMenu->getControl("stage");
	_sceneMenu = (Container*)_mainMenu->getControl("submenu_sceneMenu");
	_componentMenu = (Container*)_mainMenu->getControl("submenu_componentMenu");
    _componentMenu->setPosition(_sideMenu->getX() + _sideMenu->getWidth() + 25.0f, 25.0f);
    _componentMenu->setWidth(getWidth() - 2 * _componentMenu->getX());
    _componentMenu->setHeight(getHeight() - 2 * _componentMenu->getY());
	_machineMenu = (Container*)_mainMenu->getControl("submenu_machineMenu");
	_modePanel = (Container*)_sideMenu->getControl("modePanel");
	_cameraMenu = (Container*)_stage->getControl("camera");
	
	//dialogs
	_textDialog = (Container*)_mainMenu->getControl("textDialog");
	_textPrompt = (Label*)_textDialog->getControl("textPrompt");
	_textName = (TextBox*)_textDialog->getControl("textName");
	_textSubmit = (Button*)_textDialog->getControl("textSubmit");
	_textCancel = (Button*)_textDialog->getControl("textCancel");
	_textDialog->setVisible(false);
	_confirmDialog = (Container*)_mainMenu->getControl("confirmDialog");
	_confirmMessage = (Label*)_confirmDialog->getControl("confirmMessage");
	_confirmYes = (Button*)_confirmDialog->getControl("confirmYes");
	_confirmNo = (Button*)_confirmDialog->getControl("confirmNo");
	_confirmDialog->setVisible(false);
	_overlay = (Container*)_mainMenu->getControl("overlay");
	_overlay->setVisible(false);
    
	//identify all submenus so we can close any open ones when another is clicked
	std::vector<Control*> controls = _mainMenu->getControls();
	Container *container;
	for(std::vector<Control*>::iterator it = controls.begin(); it != controls.end(); it++) {
		container = dynamic_cast<Container*>(*it);
		if(container && strncmp(container->getId(), "submenu_", 8) == 0) {
			_submenus.push_back(container);
			container->setVisible(false);
		}
	}
	
    _theme->release();   // So we can release it once we're done creating forms with it.
    
	// populate catalog of items
	_models = Scene::create("models");
	_models->addNode(new MyNode("sphere"));
	_models->addNode(new MyNode("cylinder"));
	_models->addNode(new MyNode("halfpipe"));
	_models->addNode(new MyNode("box"));
	_models->addNode(new MyNode("gear_basic"));
/*	_models->addNode(new MyNode("gear"));
	_models->addNode(new MyNode("tube"));
	_models->addNode(new MyNode("vase"));
	_models->addNode(new MyNode("disc"));
	_models->addNode(new MyNode("cone"));
	_models->addNode(new MyNode("heart"));
	_models->addNode(new MyNode("gear_thin"));//*/

    MyNode *modelNode = dynamic_cast<MyNode*>(_models->getFirstNode());
    while(modelNode) {
    	if(strstr(modelNode->getId(), "_part") == NULL) {
			modelNode->loadData("res/common/", false);
			modelNode->_type = modelNode->getId();
			ImageControl* itemImage = addButton <ImageControl> (_componentMenu, concat(2, "comp_", modelNode->getId()));
			itemImage->setZIndex(_componentMenu->getZIndex());
			itemImage->setImage("res/png/cowboys-helmet-nobkg.png");
			itemImage->setWidth(150.0f);
			itemImage->setHeight(150.0f);
		}
		modelNode->setTranslation(Vector3(1000.0f,0.0f,0.0f));
		modelNode = dynamic_cast<MyNode*>(modelNode->getNextSibling());
	}

	//interactive modes
	_modes.push_back(new NavigateMode());
	_modes.push_back(new PositionMode());
	_modes.push_back(new ConstraintMode());
	_modes.push_back(new ToolMode());
	_modes.push_back(new TestMode());

	//simple machines
    _modes.push_back(new Lever());
    _modes.push_back(new Pulley());
    
	_activeMode = -1;
	setMode(0);

	//_drawDebugCheckbox = addControl <CheckBox> (_sideMenu, "drawDebug", _buttonStyle, "Draw Debug");
	//Button *debugButton = addControl <Button> (_sideMenu, "debugButton", _buttonStyle, "Debug");
	
	_drawDebug = true;	
    _sideMenu->setState(Control::FOCUS);
    
    addListener(_mainMenu, this);
    addListener(_textName, this, Control::Listener::TEXT_CHANGED);
	
	_running = 0;
	_constraintCount = 0;
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

/*    _font->start();

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
//*/
	_mainMenu->draw();
}

void T4TApp::setMode(short mode) {
	mode %= _modes.size();
	if(_activeMode == mode) return;
	if(_activeMode >= 0) _modes[_activeMode]->setActive(false);
	_activeMode = mode;
	if(_activeMode >= 0) _modes[_activeMode]->setActive(true);
}

void T4TApp::setNavMode(short mode) {
	_navMode = mode;
}

void T4TApp::controlEvent(Control* control, Control::Listener::EventType evt)
{
	const char *id = control->getId();
	Container *parent = control->getParent();
	cout << "CLICKED " << id << endl;

	//scene operations
	if(_sceneMenu->getControl(id) == control) {
		if(strcmp(id, "new") == 0) {
			clearScene();
			setSceneName("test");
		}
		else if(strcmp(id, "load") == 0) {
			getText("Enter scene name: ", "Load", &T4TApp::loadScene);
		}
		else if(strcmp(id, "save") == 0) {
			saveScene();
		}
		else if(strcmp(id, "saveAs") == 0) {
			getText("Enter scene name:", "Save", &T4TApp::saveScene);
		}
	}	
	//callbacks for modal dialogs
	else if((control == _textName && evt == TEXT_CHANGED &&
	  (_textName->getLastKeypress() == 10 || _textName->getLastKeypress() == 13))
	  || control == _textSubmit) {
		if(_textCallback != NULL) (this->*_textCallback)(_textName->getText());
		showDialog(_textDialog, false);
	}
	else if(control == _textCancel) {
		_textCallback = NULL;
		showDialog(_textDialog, false);
	}
	else if(control == _confirmYes || control == _confirmNo) {
		if(_confirmCallback) (this->*_confirmCallback)(control == _confirmYes);
		showDialog(_confirmDialog, false);
	}

	//if a submenu handle is clicked, toggle whether the submenu is expanded
	else if(strncmp(id, "parent_", 7) == 0) {
		const char *subName = MyNode::concat(2, "submenu_", id+7);
		cout << "Looking for submenu " << subName << endl;
		Container *subMenu = dynamic_cast<Container*>(_mainMenu->getControl(subName));
		if(subMenu) {
			bool visible = subMenu->isVisible();
			for(size_t i = 0; i < _submenus.size(); i++)
				_submenus[i]->setVisible(false);
			cout << "\ttoggling menu to " << !visible << endl;
			subMenu->setVisible(!visible);
			if(!visible) { //if expanding the submenu, position it next to its handle
				if(subMenu != _componentMenu) {
					float x = control->getX() + control->getWidth(), y = control->getY();
					subMenu->setPosition(x, y);
					cout << "\tpositioned at " << x << ", " << y << endl;
				}
			}
		} else {
			cout << "No control with ID " << subName << endl;
		}
	}

	//misc submenu funcionality
	else if(_cameraMenu->getControl(id) == control) {
		if(strcmp(id, "eye") == 0) {
			if(_navMode >= 0) setNavMode(-1);
			else setNavMode(0);
		} else if(strcmp(id, "rotate") == 0) {
			setNavMode(0);
		} else if(strcmp(id, "translate") == 0) {
			setNavMode(1);
		} else if(strcmp(id, "zoom") == 0) {
			setNavMode(2);
		} else if(strcmp(id, "reset") == 0) {
			resetCamera();
		}
	}
	else if(_modePanel->getControl(id) == control || _machineMenu->getControl(id) == control) {
		//simple machines and interactive modes are functionally equivalent, just in different submenus
		for(short i = 0; i < _modes.size(); i++) {
			if(strcmp(_modes[i]->getId(), id) == 0) setMode(i);
		}
	}
	else if(_componentMenu->getControl(id) == control && strncmp(id, "comp_", 5) == 0) {
		MyNode *node = duplicateModelNode(id+5);
		node->addPhysics();
		node->enablePhysics();
		_scene->addNode(node);
		placeNode(node, 0.0f, 0.0f);
	}
	else if(strcmp(id, "debugButton") == 0) {
		debugTrigger();
	}
	else if(control == _drawDebugCheckbox) {
		_drawDebug = _drawDebugCheckbox->isChecked();
	}

	//close a submenu when one of its items is clicked
	Container *next = parent;
	while(next != NULL && strncmp(next->getId(), "submenu_", 8) == 0) {
		next->setVisible(false);
		next = next->getParent();
	}
}

void T4TApp::keyEvent(Keyboard::KeyEvent evt, int key) {}
void T4TApp::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {}
bool T4TApp::mouseEvent(Mouse::MouseEvent evt, int x, int y, int wheelDelta) {}

void T4TApp::initScene()
{
    // Generate game scene
    _scene = Scene::load("res/common/game.scene");
    _scene->setId("scene");
    setSceneName("test");
    _scene->visit(this, &T4TApp::printNode);
    setActiveScene(_scene);
    
    // Set the aspect ratio for the scene's camera to match the current resolution
    _scene->getActiveCamera()->setAspectRatio(getAspectRatio());
    _cameraState = new cameraState();
    resetCamera();
    
    // Get light node
    _lightNode = _scene->findNode("lightNode");
    _light = _lightNode->getLight();

	//create the grid on which to place objects
    MyNode* node = MyNode::create("grid");
    Model* gridModel = createGridModel();
    gridModel->setMaterial("res/common/grid.material");
    node->setModel(gridModel);
    gridModel->release();
    node->_objType = "box";
    node->setStatic(true);
    node->addPhysics();
    _scene->addNode(node);
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
    _scene->addNode(createWireframe(vertices, "axes"));
}

void T4TApp::setSceneName(const char *name) {
	_sceneName = name;
}

std::string T4TApp::getSceneDir() {
	return "res/scenes/" + _sceneName + "_";
}

void T4TApp::loadScene(const char *scene) {
	std::string oldName = _sceneName;
	if(scene != NULL) setSceneName(scene);
	std::string listFile = getSceneDir() + "scene.list", id;
	std::auto_ptr<Stream> stream(FileSystem::open(listFile.c_str()));
	if(stream.get() == NULL) {
		cout << "Failed to open file" << listFile << endl;
		setSceneName(oldName.c_str());
		return;
	}
	clearScene();
	char line[256], *str;
	std::istringstream ss;
	while(!stream->eof()) {
		str = stream->readLine(line, 256);
		ss.str(str);
		ss >> id;
		loadNode(id.c_str());
	}
}

MyNode* T4TApp::loadNode(const char *id) {
	MyNode *node = MyNode::create(id);
	_scene->addNode(node);
	node->loadData();
	return node;
}

void T4TApp::saveScene(const char *scene) {
	if(scene != NULL) setSceneName(scene);
	//create a file that lists all root nodes in the scene, and save each one to its own file
	std::string listFile = getSceneDir() + "scene.list", line;
	std::auto_ptr<Stream> stream(FileSystem::open(listFile.c_str(), FileSystem::WRITE));
	if(stream.get() == NULL) {
		GP_ERROR("Failed to open file '%s'", listFile.c_str());
		return;
	}
	MyNode *node;
	std::ostringstream os;
	for(Node *n = _scene->getFirstNode(); n != NULL; n = n->getNextSibling()) {
		if(n->getParent() != NULL || auxNode(n)) continue;
		node = dynamic_cast<MyNode*>(n);
		if(node) {
			os.str("");
			os << node->getId() << endl;
			line = os.str();
			stream->write(line.c_str(), sizeof(char), line.length());
			node->writeData();
		}
	}
	stream->close();
}

bool T4TApp::saveNode(Node *n) {
	MyNode *node = dynamic_cast<MyNode*>(n);
	if(node && !node->getParent() && !auxNode(node)) node->writeData();
	return true;
}

void T4TApp::clearScene() {
	std::vector<MyNode*> nodes;
	for(Node *n = _scene->getFirstNode(); n != NULL; n = n->getNextSibling()) {
		if(n->getParent() != NULL || auxNode(n)) continue;
		MyNode *node = dynamic_cast<MyNode*>(n);
		if(node) nodes.push_back(node);
	}
	for(short i = 0; i < nodes.size(); i++) {
		nodes[i]->removePhysics();
		_scene->removeNode(nodes[i]);
	}
}

bool T4TApp::removeNode(Node *n) {
	MyNode *node = dynamic_cast<MyNode*>(n);
	if(node && !node->getParent() && !auxNode(node)) {
		node->removePhysics();
		_scene->removeNode(node);
	}
	return true;
}

bool T4TApp::auxNode(Node *node) {
	const char *id = node->getId();
	return strcmp(id, "grid") == 0 || strcmp(id, "axes") == 0;
}

void T4TApp::releaseScene()
{
	if(_activeScene == _scene) _activeScene = NULL;
	_carVehicle = NULL;
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
	MyNode *n = dynamic_cast<MyNode*>(node);
	if(n) n->enablePhysics(false);
	return true;
}

bool T4TApp::showNode(Node *node) {
	MyNode *n = dynamic_cast<MyNode*>(node);
	if(n) n->enablePhysics(true);
	return true;
}

template <class ButtonType> ButtonType* T4TApp::addButton(Container *parent, const char *id, const char *text)
{
	ButtonType* button = ButtonType::create(id, _buttonStyle);
	if(text == NULL) button->setText(id);
	else button->setText(text);
	button->setAutoWidth(true);
	button->setHeight(50);
	button->setConsumeInputEvents(true);
	parent->addControl(button);
	return button;
}

template <class ControlType> ControlType* T4TApp::addControl(Container *parent, const char *id, Theme::Style *style, const char *text)
{
	ControlType* control = ControlType::create(id, style);
	if(text == NULL) control->setText(id);
	else control->setText(text);
	control->setHeight(50);
	control->setWidth(150);
	control->setConsumeInputEvents(true);
	parent->addControl(control);
	return control;
}

Form* T4TApp::addMenu(const char *name, Container *parent, const char *buttonText, Layout::Type layout)
{
	bool isSubmenu = parent != NULL && buttonText != NULL;
	const char *id = name;
	if(isSubmenu) id = MyNode::concat(2, "submenu_", name);
    Form *container = Form::create(id, _formStyle, layout);
    if(parent == NULL || parent == _mainMenu) container->setAutoHeight(true);
	else container->setHeight(300.0f);
    container->setWidth(200.0f);
    container->setScroll(Container::SCROLL_VERTICAL);
    container->setConsumeInputEvents(true);
    if(isSubmenu) {
    	container->setZIndex(parent->getZIndex() + 10);
		container->setVisible(false);
		_mainMenu->addControl(container);
		_submenus.push_back(container);
    	Button *toggle = addButton<Button>(parent, MyNode::concat(2, "parent_", name), buttonText);
    }
    else if(parent != NULL) parent->addControl(container);
    return container;
}

Form* T4TApp::addPanel(const char *name, Container *parent)
{
	Form *container = Form::create(name, _formStyle, Layout::LAYOUT_VERTICAL);
	container->setHeight(250);
	container->setWidth(175);
	container->setScroll(Container::SCROLL_NONE);
    container->setConsumeInputEvents(true);
    if(parent != NULL) parent->addControl(container);
    return container;
}

void T4TApp::addListener(Control *control, Control::Listener *listener, int evtFlags) {
	enableListener(true, control, listener, evtFlags);
}

void T4TApp::removeListener(Control *control, Control::Listener *listener) {
	enableListener(false, control, listener);
}

void T4TApp::enableListener(bool enable, Control *control, Control::Listener *listener, int evtFlags) {
	if(enable) control->addListener(listener, evtFlags);
	else control->removeListener(listener);
	Container *container = dynamic_cast<Container*>(control), *submenu;
	if(container) {
		std::vector<Control*> controls = container->getControls();
		for(int i = 0; i < controls.size(); i++) {
			const char *id = controls[i]->getId(), *submenuID;
			if(strncmp(id, "submenu_", 8) != 0) {
				enableListener(enable, controls[i], listener, evtFlags);
			}
			if(strncmp(id, "parent_", 7) == 0) {
				submenuID = MyNode::concat(2, "submenu_", id+7);
				submenu = dynamic_cast<Container*>(_mainMenu->getControl(submenuID));
				cout << "enabling on submenu " << submenuID << " = " << submenu << endl;
				if(submenu) enableListener(enable, submenu, listener, evtFlags);
			}
		}
	}
}

void T4TApp::getText(const char *prompt, const char *type, void (T4TApp::*callback)(const char*)) {
	_textCallback = callback;
	_textPrompt->setText(prompt);
	_textSubmit->setText(type);
	_textName->setText("");
	showDialog(_textDialog);
	_textName->setState(Control::ACTIVE);
}

void T4TApp::doConfirm(const char *message, void (T4TApp::*callback)(bool)) {
	_confirmCallback = callback;
	_confirmMessage->setText(message);
	showDialog(_confirmDialog);
}

void T4TApp::showDialog(Container *dialog, bool show) {
	_overlay->setVisible(show);
	dialog->setVisible(show);
	if(show) dialog->setState(Control::FOCUS);
}

void T4TApp::confirmDelete(bool yes) {
	if(!yes) return;
	if(_activeMode < 0) return;
	MyNode *node = _modes[_activeMode]->_selectedNode;
	if(node != NULL) {
		node->removePhysics();
		_scene->removeNode(node);
	}
}

bool T4TApp::prepareNode(MyNode* node)
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

MyNode* T4TApp::getMouseNode(int x, int y, Vector3 *touch) {
	Camera* camera = _scene->getActiveCamera();
	Ray ray;
	camera->pickRay(getViewport(), x, y, &ray);
	PhysicsController::HitResult hitResult;
	if(!getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) return NULL;
	if(touch) touch->set(hitResult.point);
	MyNode *node = dynamic_cast<MyNode*>(hitResult.object->getNode());
	if(!node || strcmp(node->getId(), "grid") == 0) return NULL;
	return node;
}

//see if the current touch location intersects the bottom face of the given object
bool T4TApp::checkTouchModel(Node* n)
{
	MyNode *node = dynamic_cast<MyNode*>(n);
	if(!node) return true;
	if(strcmp(node->getScene()->getId(), "models") == 0) return true;
	if(strcmp(node->getId(), "knife") == 0) return true;
	if(strcmp(node->getId(), "drill") == 0) return true;
	for(int i = 0; i < _intersectNodeGroup.size(); i++)
		if(node == _intersectNodeGroup[i]) return true;
	Vector3 pos = node->getTranslation();
	BoundingBox bbox = node->getWorldBox();
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

MyNode* T4TApp::duplicateModelNode(const char* type, bool isStatic)
{
	MyNode *modelNode = dynamic_cast<MyNode*>(_models->findNode(type));
	if(!modelNode) return NULL;
	MyNode *node = MyNode::cloneNode(modelNode);
	BoundingBox box = node->getModel()->getMesh()->getBoundingBox();
	node->loadData(concat(3, "res/common/", type, ".node"));
	const char count[2] = {(char)(++modelNode->_typeCount + 48), '\0'};
	node->setId(concat(2, modelNode->getId(), count));
	node->setTranslation(Vector3(0.0f, (box.max.y - box.min.y)/2.0f, 0.0f));
	node->updateTransform();
	node->setStatic(isStatic);
	return node;
}

MyNode* T4TApp::createWireframe(std::vector<float>& vertices, const char *id) {
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
	if(id == NULL) id = "wireframe1";
	MyNode *node = MyNode::create(id);
	node->setModel(model);
	model->release();
	return node;
}

//place at the given xz-coords and set its y-coord so it is on top of any objects it would otherwise intersect
void T4TApp::placeNode(MyNode *node, float x, float z)
{
	_intersectNodeGroup.clear();
	_intersectNodeGroup.push_back(node);
	_intersectBox = node->getWorldBox();
	float minY = _intersectBox.min.y;
	node->setTranslation(x, -minY, z); //put the bounding box bottom on the ground
	_intersectPoint.set(x, -minY, z);
	_intersectModel = NULL;
	_scene->visit(this, &T4TApp::checkTouchModel); //will change _intersectPoint.y to be above any intersecting models
	node->setTranslation(_intersectPoint);
}

void T4TApp::setActiveScene(Scene *scene)
{
	_activeScene = scene;
}

Camera* T4TApp::getCamera() {
	return _scene->getActiveCamera();
}

Node* T4TApp::getCameraNode() {
	return getCamera()->getNode();
}

void T4TApp::placeCamera() {
	float radius = _cameraState->radius, theta = _cameraState->theta, phi = _cameraState->phi;
	Vector3 eye(radius * cos(theta) * cos(phi), radius * sin(phi), radius * sin(theta) * cos(phi));
	eye += _cameraState->target;
	Vector3 up(-cos(theta) * sin(phi), cos(phi), -sin(theta) * sin(phi));
	Matrix cam;
	Matrix::createLookAt(eye, _cameraState->target, up, &cam);
	cam.invert();
	if(_cameraState->node != NULL) {
		Matrix node = _cameraState->node->getRotTrans(), camCopy = cam;
		Matrix::multiply(node, camCopy, &cam);
	}
	Vector3 scale, translation; Quaternion rotation;
	cam.decompose(&scale, &rotation, &translation);
	getCameraNode()->set(scale, rotation, translation);
}

void T4TApp::setCameraEye(float radius, float theta, float phi) {
	_cameraState->radius = radius;
	_cameraState->theta = theta;
	_cameraState->phi = phi;
	placeCamera();
}

void T4TApp::setCameraZoom(float radius) {
	_cameraState->radius = radius;
	placeCamera();
}

void T4TApp::setCameraTarget(Vector3 target) {
	_cameraState->target = target;
	placeCamera();
}

void T4TApp::setCameraNode(MyNode *node) {
	_cameraState->node = node;
	_cameraMenu->getControl("translate")->setEnabled(node == NULL);
	if(node != NULL) {
		cameraPush();
		resetCamera();
		if(_navMode == 1) setNavMode(0);
	} else {
		cameraPop();
	}
}

void T4TApp::resetCamera() {
	_cameraState->target.set(0, 0, 0);
	if(_cameraState->node == NULL) {
		_cameraState->radius = 30;
		_cameraState->theta = M_PI / 2;
		_cameraState->phi = M_PI / 12;
	} else {
		_cameraState->radius = 20;
		_cameraState->theta = -M_PI/2;
		_cameraState->phi = 0;
	}
	placeCamera();
}

T4TApp::cameraState* T4TApp::copyCameraState(T4TApp::cameraState *state, T4TApp::cameraState *dst) {
	if(dst == NULL) dst = new cameraState();
	dst->node = state->node;
	dst->radius = state->radius;
	dst->theta = state->theta;
	dst->phi = state->phi;
	dst->target = state->target;
	return dst;
}

void T4TApp::cameraPush() {
	cameraState *state = copyCameraState(_cameraState);
	_cameraHistory.push_back(state);
}

void T4TApp::cameraPop() {
	cameraState *state = _cameraHistory.back();
	copyCameraState(state, _cameraState);
	_cameraHistory.pop_back();
	placeCamera();
}

const std::string T4TApp::pv(const Vector3& v) {
	std::ostringstream os;
	os << "<" << v.x << "," << v.y << "," << v.z << ">";
	return os.str();
}

const std::string T4TApp::pv2(const Vector2& v) {
	std::ostringstream os;
	os << "<" << v.x << "," << v.y << ">";
	return os.str();
}

const std::string T4TApp::pq(const Quaternion& q) {
	std::ostringstream os;
	Vector3 axis;
	float ang = q.toAxisAngle(&axis);
	os << "<" << axis.x << "," << axis.y << "," << axis.z << " ; " << (int)(ang*180/M_PI) << ">";
	return os.str();
}

const std::string T4TApp::pcam(T4TApp::cameraState *state) {
	std::ostringstream os;
	os << state->radius << "," << state->theta << "," << state->phi << " => " << pv(state->target);
	if(state->node != NULL) os << " [node " << state->node->getId() << "]";
	return os.str();
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

/********** PHYSICS ***********/

PhysicsConstraint* T4TApp::addConstraint(MyNode *n1, MyNode *n2, int id, const char *type, ...) {
	va_list arguments;
	va_start(arguments, type);
	MyNode *node[2];
	PhysicsRigidBody *body[2];
	Vector3 trans[2];
	Quaternion rot[2];
	PhysicsConstraint *ret;
	unsigned short i, j;
	for(i = 0; i < 2; i++) {
		node[i] = i == 0 ? n1 : n2;
		body[i] = node[i]->getCollisionObject()->asRigidBody();
		rot[i] = *((Quaternion*) va_arg(arguments, Quaternion*));
		trans[i] = *((Vector3*) va_arg(arguments, Vector3*));
		if(strcmp(type, "hinge") == 0) {
			body[i]->setEnabled(false);
			body[i]->setFriction(0.01f);
			body[i]->setEnabled(true);
		}
	}
	va_end(arguments);
	if(strcmp(type, "hinge") == 0) {
		ret = getPhysicsController()->createHingeConstraint(body[0], rot[0], trans[0], body[1], rot[1], trans[1]);
	} else if(strcmp(type, "spring") == 0) {
		ret = getPhysicsController()->createSpringConstraint(body[0], rot[0], trans[0], body[1], rot[1], trans[1]);
	} else if(strcmp(type, "socket") == 0) {
		ret = getPhysicsController()->createSocketConstraint(body[0], trans[0], body[1], trans[1]);
	} else if(strcmp(type, "fixed") == 0) {
		ret = getPhysicsController()->createFixedConstraint(body[0], body[1]);
	}
	if(id < 0) {
		id = _constraintCount++;
		for(i = 0; i < 2; i++) {
			j = node[i]->_constraints.size();
			node[i]->_constraints.push_back(new MyNode::nodeConstraint());
			node[i]->_constraints[j]->other = node[(i+1)%2]->getId();
			node[i]->_constraints[j]->type = type;
			node[i]->_constraints[j]->rotation = rot[i];
			node[i]->_constraints[j]->translation = trans[i];
			node[i]->_constraints[j]->id = id;
		}
	}
	_constraints[id] = ret;
	return ret;
}

void T4TApp::addConstraints(MyNode *node) {
	MyNode::nodeConstraint *c1, *c2;
	unsigned short i, j;
	for(i = 0; i < node->_constraints.size(); i++) {
		c1 = node->_constraints[i];
		if(c1->id >= 0) continue;
		MyNode *other = dynamic_cast<MyNode*>(_scene->findNode(c1->other.c_str()));
		if(!other || !other->getCollisionObject()) continue;
		for(j = 0; j < other->_constraints.size(); j++) {
			c2 = other->_constraints[j];
			if(c2->other.compare(node->getId()) == 0 && c2->type.compare(c1->type) == 0 && c2->id < 0) {
				c1->id = _constraintCount;
				c2->id = _constraintCount++;
				addConstraint(node, other, c1->id, c1->type.c_str(), &c1->rotation, &c1->translation,
					&c2->rotation, &c2->translation);
			}
		}
	}
}

void T4TApp::removeConstraints(MyNode *node) {
	PhysicsController *controller = getPhysicsController();
	MyNode::nodeConstraint *c1, *c2;
	unsigned short i, j;
	for(i = 0; i < node->_constraints.size(); i++) {
		c1 = node->_constraints[i];
		if(c1->id < 0) continue;
		MyNode *other = dynamic_cast<MyNode*>(_scene->findNode(c1->other.c_str()));
		if(!other) continue;
		for(j = 0; j < other->_constraints.size(); j++) {
			c2 = other->_constraints[j];
			if(c2->other.compare(node->getId()) == 0 && c2->type.compare(c1->type) == 0 && c2->id == c1->id) {
				controller->removeConstraint(_constraints[c1->id]);
				_constraints.erase(c1->id);
				c1->id = -1;
				c2->id = -1;
			}
		}
	}
}

void T4TApp::enableConstraints(MyNode *node, bool enable) {	
	int id;
	for(short i = 0; i < node->_constraints.size(); i++) {
		id = node->_constraints[i]->id;
		if(id < 0 || _constraints.find(id) == _constraints.end()) continue;
		_constraints[id]->setEnabled(enable);
	}
}

void T4TApp::reloadConstraint(MyNode *node, MyNode::nodeConstraint *constraint) {
	int id = constraint->id;
	if(id >= 0 && _constraints.find(id) != _constraints.end()) {
		getPhysicsController()->removeConstraint(_constraints[id]);
		_constraints.erase(id);
	}
	MyNode *other = dynamic_cast<MyNode*>(_scene->findNode(constraint->other.c_str()));
	if(!other) return;
	MyNode::nodeConstraint *otherConstraint;
	for(short i = 0; i < other->_constraints.size(); i++) {
		otherConstraint = other->_constraints[i];
		if(otherConstraint->id == id) {
			addConstraint(node, other, id, constraint->type.c_str(), &constraint->rotation, &constraint->translation,
			  &otherConstraint->rotation, &otherConstraint->translation);
			break;
		}
	}
}


