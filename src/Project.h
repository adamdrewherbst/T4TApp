#ifndef PROJECT_H_
#define PROJECT_H_

#include "Mode.h"

class Project : public Mode
{
public:
	//component is divided into elements, eg. a lever has a base and arm
	class Element {
		public:
		T4TApp *app;
		Project *_project;
		std::string _id, _name;
		bool _static, _multiple, _movable[3], _rotable[3];
		float _limits[3][2];
		short _moveRef, _numNodes;
		std::vector<std::shared_ptr<MyNode> > _nodes;
		const char *_currentNodeId, *_filter;
		Element *_parent;
		Plane _plane;
		TouchPoint _parentTouch, _planeTouch;
		std::vector<Element*> _children;
		
		Element(Project *project, Element *parent, const char *id, const char *name = NULL);
		void setMovable(bool x, bool y, bool z, short ref = -1);
		void setRotable(bool x, bool y, bool z);
		void setLimits(short axis, float lower, float upper);
		void setPlane(const Plane &plane);
		void applyLimits(Vector3 &translation);
		void setParent(Element *parent);
		void addChild(Element *child);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
		MyNode* getNode(short n = 0);
		bool setNode(const char *id);
		virtual void addNode(const Vector3 &position);
		virtual void placeNode(const Vector3 &position, short n = 0);
		virtual void addPhysics(short n = -1);
	};
	std::vector<std::shared_ptr<Element> > _elements;

	short _currentElement, _typeCount, _moveMode, _moveAxis;
	Quaternion _baseRotation;
	Vector3 _baseTranslation;

	//blank scene onto which to build the component
	std::string _sceneFile, _nodeId;
	MyNode *_rootNode; //parent node for this component

	Container *_elementContainer;

	bool _inSequence; //true during the first run-through to add all the elements

	Project(const char* id);

	virtual void setupMenu();
	void setActive(bool active);
	bool setSubMode(short mode);
	Element* addElement(Element *element);
	Element* getEl(short n = -1);
	MyNode* getNode(short n = -1);
	void controlEvent(Control *control, EventType evt);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void promptNextElement();
	virtual void finish();
	virtual void addPhysics();
};

#endif
