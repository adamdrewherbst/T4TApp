#ifndef MODES_H_
#define MODES_H_

#include "Meshy.h"

class MyNode;
class T4TApp;

class Mode : public Button, public Control::Listener
{
public:
	T4TApp *app;
	Scene *_scene;
	Camera *_camera;

	std::vector<std::string> _subModes;
	short _subMode;
	Container *_container, *_controls, *_subModePanel;
	bool _active, _touching, _doSelect;
	std::ostringstream os;

	int _x, _y; //mouse position wrt upper left of entire window, not just my button
	TouchPoint _touchPt;
	MyNode *_selectedNode, *_touchNode;
	Vector3 _selectPoint, _touchPoint, _mousePoint;
	Vector2 _touchPix, _mousePix;
	Plane _plane;
	Ray _ray;
	//Base members remember the value from the time of the last TOUCH_PRESS event
	Camera *_cameraBase;
	cameraState *_cameraStateBase;
	Rectangle _viewportBase;
	
	Mode(const char* id);
	
	virtual bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	virtual void controlEvent(Control *control, EventType evt);
	virtual void setActive(bool active);
	virtual bool setSubMode(short mode);
	virtual bool setSelectedNode(MyNode *node, Vector3 point = Vector3::zero());
	virtual bool isSelecting();
	virtual void placeCamera();
	virtual void update();
	virtual void draw();
};

#endif


