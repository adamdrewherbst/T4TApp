#include "T4TApp.h"
#include "Launcher.h"
#include "MyNode.h"

Launcher::Launcher() : Project::Project("launcher") {

	app->addItem("rubberBand1", 1, "launcherBand");
	app->addItem("rubberBand2", 1, "launcherBand");

	_rubberBand = (RubberBand*) addElement(new RubberBand(this));
	setupMenu();

	_table = MyNode::create("table1");
	_table->loadData("res/common/");
	BoundingBox tableBox = _table->getBoundingBox(true);
	Vector3 tablePos(0, -tableBox.min.y, -tableBox.max.z);
	_table->setMyTranslation(tablePos);
	_table->enablePhysics(true);
	for(short i = 0; i < 2; i++) {
		_clamps[i] = MyNode::create("clamp");
		_clamps[i]->loadData("res/common/");
		BoundingBox clampBox = _clamps[i]->getBoundingBox(true);
		Vector3 clampPos((2*i-1) * _clampWidth/2, (tableBox.max.y - tableBox.min.y) - clampBox.min.y, -clampBox.max.z);
		_anchorPoints[i] = clampPos;
		if(i == 0) clampPos.x -= clampBox.max.x;
		else clampPos.x -= clampBox.min.x;
		_clamps[i]->setMyTranslation(clampPos);
		_clamps[i]->enablePhysics(true);
		clampPos.y += clampBox.min.y;
		app->addConstraint(_table, _clamps[i], -1, "fixed", clampPos, Vector3::unitY(), true);
	}
	_table->enablePhysics(false);
	
	
}

bool Launcher::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Project::touchEvent(evt, x, y, contactIndex);
	if(_subMode == 1 && !_launched) {
	}
}

bool Launcher::setSubMode(short n) {
	bool changed = Project::setSubMode(n);
	switch(_subMode) {
		case 0: {
			break;
		} case 1: {
			//place the crew exploration vehicle in front of the rubber band
			_cev = app->getProjectNode("buggy");
			_cev->enablePhysics(false);
			_cev->placeRest();
			_cev->updateTransform();
			_scene->addNode(_table);
			_table->enablePhysics(true);
			break;
		}
	}
	return changed;
}

void Launcher::launch() {
	
}

Launcher::RubberBand::RubberBand(Project *project)
  : Project::Element::Element(project, NULL, "rubberBand", "Rubber Band"), _numNodes(30) {
	_filter = "launcherBand";
}

void Launcher::RubberBand::placeNode(short n) {
	//each node is one link - place it at the appropriate location in the chain
	float distance = _anchorPoints[0].x + (_anchorPoints[1].x - _anchorPoints[0].x) * (n+0.5f) / _numNodes;
	Vector3 pos(distance, 0, 0);
	_nodes[n]->setMyTranslation(pos);
}

void Launcher::RubberBand::addPhysics(short n) {
	Project::Element::addPhysics(n);
}


