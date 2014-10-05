#include "T4TApp.h"

T4TApp::ToolMode::ToolMode(const char* id, const char* filename) 
  : T4TApp::Mode::Mode(id, filename) {
	_toolType = id+5;
	_subMode = 0;
	_touching = false;
	_newNode = MyNode::create(app->concat(2, "newNode_", _toolType.c_str()));
	newData = _newNode->getData();
}

void T4TApp::ToolMode::setActive(bool active) {
	Mode::setActive(active);
}

void T4TApp::ToolMode::setSelectedNode(MyNode *node) {
	Mode::setSelectedNode(node);
	app->getScriptController()->executeFunction<void>("camera_setNode", "s", node != NULL ? node->getId() : NULL);
	if(node != NULL) {
		setAxis(0);
		app->_scene->addNode(_tool);
	} else {
		app->_scene->removeNode(_tool);
	}
}

void T4TApp::ToolMode::setSubMode(short mode) {
	Mode::setSubMode(mode);
	setCameraMode(-1);
}

void T4TApp::ToolMode::setAxis(int axis) {
	//translate the camera to look at the center of the node
	//and face along the <axis> direction in its model space
	float yaw = 0, pitch = 0;
	Vector3 sliceNormal, viewNormal, translation(_selectedNode->getTranslationWorld());
	Matrix rotation;
	_selectedNode->getRotation(&rotation);
	_tool->setRotation(rotation);
	switch(axis) {
		case 0: //x
			yaw = 0;
			pitch = 0;
			break;
		case 1: //y
			yaw = 0;
			pitch = M_PI/2;
			break;
		case 2: //z
			yaw = M_PI/2;
			pitch = 0;
			break;
	}
	_tool->setTranslation(translation);
	_toolBaseRotation = _tool->getRotation();
	app->getScriptController()->executeFunction<void>("camera_rotateTo", "ff", yaw, pitch);
	setView();
}

void T4TApp::ToolMode::setView() {
	//align the tool to the viewing axis
	Node *camNode = app->_scene->getActiveCamera()->getNode();
	Quaternion rot = camNode->getRotation(), offset;
	Quaternion::createFromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), M_PI/2, &offset);
	rot.multiply(offset);
	_tool->setRotation(rot);
	//make the view plane orthogonal to the viewing axis
	Vector3 cam(camNode->getTranslationWorld()), camToNode(_selectedNode->getTranslationWorld() - cam);
	_viewPlane.setNormal(camToNode);
}

bool T4TApp::ToolMode::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Mode::touchEvent(evt, x, y, contactIndex);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			break;
		}
		case Touch::TOUCH_MOVE: {
			break;
		}
		case Touch::TOUCH_RELEASE:
			break;
	}
}

void T4TApp::ToolMode::controlEvent(Control *control, Control::Listener::EventType evt) {
	
	const char *controlID = control->getId();
	cout << "tool mode prelim clicked " << controlID << endl;
	if(strncmp(_toolType.c_str(), controlID, _toolType.length()) != 0) return;
	cout << "tool mode clicked " << controlID << endl;
	controlID = controlID + _toolType.length()+1;

	if(strcmp(controlID, "axis") == 0 && _selectedNode != NULL) {
		const char *_axes[3] = {"X", "Y", "Z"};
		for(int i = 0; i < 3; i++) {
			if(strcmp(((Button*)control)->getText(), _axes[i]) == 0) {
				setAxis(i);
				((Button*)control)->setText(_axes[(i+1)%3]);
				cout << "set axis to " << _axes[i] << endl;
				break;
			}
		}
	}
	else if(strcmp(controlID, "doTool") == 0) {
		toolNode();
		setActive(false);
	}
}

bool T4TApp::ToolMode::toolNode() {
	_selectedNode->updateData();
	data = _selectedNode->getData();
	
	usageCount++;

	//reset the data on the altered node
	_newNode->data = NULL;
	free(newData);
	newData = new MyNode::nodeData();
	_newNode->data = newData;
	newData->type = data->type;
	newData->objType = "mesh"; //can't keep sphere/box collision object once it has been warped!
	newData->mass = data->mass;
	newData->rotation = data->rotation;
	newData->translation = data->translation;
	newData->scale = data->scale;
	newData->constraints = data->constraints;
}

