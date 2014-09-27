#include "skp.h"
#include "Node.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <cstdlib>
#include <cstdarg>
#include <cmath>

using std::cout; using std::endl; using std::ifstream; using std::istringstream;

// Declare our game instance
skp game;

skp::skp()
    : _scene(NULL)
{
}

void skp::initialize()
{
	const char *filename = "res/app materials.obj";
	char *buf = (char*) malloc(4096), *label = (char*) malloc(100), *token = (char*) malloc(100),
		*nodeName = (char*) malloc(100);
	ifstream in(filename);
	int line = 0, ind;
	unsigned short i, j, n, e1, e2, vOffset = 0;
	cout << "Reading from: " << filename << endl;
	Node *node = NULL;
	Node::nodeData *data;
	float x, y, z;
	Vector3 vertex;
	std::vector<unsigned short> face, edge(2);
	std::vector<std::vector<unsigned short> > triangles;
	std::string outfile;
	while(!in.eof()) {
		line++;
		in.getline(buf, 4096);
		istringstream ss(buf);
		strcpy(label, "");
		ss >> label;
		//cout << line << ": " << buf << endl; 
		//cout << label << endl;
		if((strcmp(label, "g") == 0 || in.eof()) && node != NULL) { //end of current model
			//get the edge list from the face list
			for(i = 0; i < data->faces.size(); i++) {
				face = data->faces[i];
				n = face.size();
				for(j = 0; j < n; j++) {
					e1 = face[j];
					e2 = face[(j+1)%n];
					if(data->edgeInd.find(e1) == data->edgeInd.end() || data->edgeInd[e1].find(e2) == data->edgeInd[e1].end()) {
						data->edgeInd[e1][e2] = data->edges.size();
						data->edgeInd[e2][e1] = data->edges.size();
						edge[0] = e1;
						edge[1] = e2;
						data->edges.push_back(edge);
					}
				}
			}
			//put the center of mass at the origin
			vertex.set(0, 0, 0);
			n = data->vertices.size();
			for(i = 0; i < n; i++) vertex += data->vertices[i];
			vertex = vertex / (float)n;
			for(i = 0; i < n; i++) {
				data->vertices[i] -= vertex;
				data->worldVertices[i] -= vertex;
			}
			outfile = std::string("/home/aherbst/Documents/Programming/GamePlay/projects/t4tapp/build/bin/linux/res/common/")
				+ std::string(node->getId()) + std::string(".node");
			node->writeMyData(outfile.c_str());
			vOffset += n;
		}
		if(strcmp(label, "g") == 0) { //start of new model
			ss >> nodeName;
			node = Node::create(nodeName);
			data = node->getData();
			data->type = nodeName;
			data->objType = "mesh";
			data->mass = 10.0f;
			cout << endl << "Model: " << nodeName << endl;
		} else if(strcmp(label, "s") == 0) {
			ss >> x >> y >> z;
			node->setScale(x, y, z);
			cout << "Scale: " << x << ", " << y << ", " << z << endl;
		} else if(strcmp(label, "v") == 0) { //vertex
			ss >> x >> y >> z;
			vertex.set(x, y, z);
			data->vertices.push_back(vertex);
			data->worldVertices.push_back(vertex);
		} else if(strcmp(label, "f") == 0) { //face
			face.clear();
			ss >> token;
			while(!ss.fail()) {
				std::string str(token);
				size_t pos = str.find('/');
				if(pos > 0) str = str.substr(0, pos);
				ind = atoi(str.c_str());
				face.push_back(ind-1 - vOffset);
				ss >> token;
			}
			triangles.clear();
			node->addFace(face, triangles);
			data->hulls.push_back(face);
		}
	}
	exit();
}

void skp::finalize()
{
    SAFE_RELEASE(_scene);
}

void skp::update(float elapsedTime) {}

void skp::render(float elapsedTime) {}

bool skp::drawScene(Node* node) {}

void skp::keyEvent(Keyboard::KeyEvent evt, int key)
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

void skp::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
    switch (evt)
    {
    case Touch::TOUCH_PRESS:
        break;
    case Touch::TOUCH_RELEASE:
        break;
    case Touch::TOUCH_MOVE:
        break;
    };
}
