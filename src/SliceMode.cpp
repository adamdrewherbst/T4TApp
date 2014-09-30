#include "T4TApp.h"

T4TApp::SliceMode::SliceMode(T4TApp *app_) 
  : T4TApp::ToolMode::ToolMode(app_, "mode_Slice", "res/common/slice.form") {
	_slicePlane.set(Vector3(0, 0, 1), 0);
	//create the knife node
	float spacing = 0.5f, radius = 3.25f, distance, color[3] = {1.0f, 1.0f, 1.0f}, vec[2];
	int v = 0, numLines = (int)(radius/spacing), vertexCount = 4*(2*numLines+1)*6;
	std::vector<float> vertices(vertexCount);
	for(int i = -numLines; i <= numLines; i++) {
		for(int j = 0; j < 2; j++) {
			for(int k = 0; k < 2; k++) {
				vec[j] = i*spacing;
				vec[1-j] = (2*k-1) * sqrt(radius*radius - vec[j]*vec[j]);
				vertices[v++] = vec[0];
				vertices[v++] = vec[1];
				vertices[v++] = 0;
				for(int i = 0; i < 3; i++) vertices[v++] = color[i];
			}
		}
	}
	VertexFormat::Element elements[] = {
		VertexFormat::Element(VertexFormat::POSITION, 3),
		VertexFormat::Element(VertexFormat::COLOR, 3)
	};
	Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), v/6, false);
	mesh->setPrimitiveType(Mesh::LINES);
	mesh->setVertexData(&vertices[0], 0, v/6);
	Model *model = Model::create(mesh);
	mesh->release();
	model->setMaterial("res/common/grid.material");
	_tool = MyNode::create("knife");
	_tool->setModel(model);
	model->release();
}

void T4TApp::SliceMode::setAxis(int axis) {
	ToolMode::setAxis(axis);
	Vector3 sliceNormal;
	Matrix rotation;
	_node->getRotation(&rotation);
	switch(axis) {
		case 0: //x
			sliceNormal.set(1, 0, 0);
			break;
		case 1: //y
			sliceNormal.set(0, 1, 0);
			break;
		case 2: //z
			sliceNormal.set(0, 0, 1);
			break;
	}
	rotation.transformVector(&sliceNormal);
	_slicePlane.setNormal(sliceNormal);
}

bool T4TApp::SliceMode::toolNode() {
	ToolMode::toolNode();
	_slicePlane.set(Vector3(0, 0, 1), 0);
	Matrix trans;
	Matrix::createRotation(_tool->getRotation(), &trans);
	_slicePlane.transform(trans);
	_slicePlane.setDistance(-_tool->getTranslationWorld().dot(_slicePlane.getNormal()));
	cout << "slicing " << _node->getId() << " at " << app->printVector(_slicePlane.getNormal()) << " => " << _slicePlane.getDistance() << endl;
	MyNode::nodeData *data = _node->getData();
	unsigned short e1, e2, numKeep = 0;
	Vector3 v1, v2, planeOrigin = -_slicePlane.getDistance() * _slicePlane.getNormal();

	std::vector<short> keep(data->vertices.size()), //index in the new vertex list or -1 if discarding
		replace(data->vertices.size()); //index of the new vertex that is replacing this one
	for(int i = 0; i < data->vertices.size(); i++) {
		keep[i] = -1;
		replace[i] = -1;
		float dot = (data->worldVertices[i] - planeOrigin).dot(_slicePlane.getNormal());
		if(dot > 0) continue;
		keep[i] = numKeep++;
		newData->vertices.push_back(data->worldVertices[i]);
	}
	if(numKeep == 0) return false;

	//get all intersections between edges of this mesh and the slice plane
	Ray edge;
	Vector3 edgeVec, intersect, newEdgeVec;
	float distance, dot;
	std::map<unsigned short, std::map<unsigned short,unsigned short> > intersections;
	std::vector<unsigned short> newEdge(2);
	for(int i = 0; i < data->edges.size(); i++) {
		e1 = data->edges[i][0];
		e2 = data->edges[i][1];
		//if both endpoints are on the same side of the slice plane, decide whether to keep the edge and move on
		if(keep[e1] >= 0 && keep[e2] >= 0) {
			newEdge[0] = keep[e1];
			newEdge[1] = keep[e2];
			newData->edges.push_back(newEdge);
			continue;
		}
		if(keep[e1] < 0 && keep[e2] < 0) continue;
		//otherwise this edge must intersect the slice plane
		edgeVec.set(data->worldVertices[e2] - data->worldVertices[e1]);
		edge.set(data->worldVertices[e1], edgeVec);
		distance = edge.intersects(_slicePlane);
		if(distance != Ray::INTERSECTS_NONE) {
			intersect.set(edge.getOrigin() + edge.getDirection()*distance);
			newEdgeVec.set(intersect - data->worldVertices[e1]);
			dot = newEdgeVec.dot(edgeVec) / edgeVec.lengthSquared();
		}
		//limit the intersection point to the edge endpoints, in case computational geometry puts it outside the edge
		if(distance == Ray::INTERSECTS_NONE || dot <= 0 || dot >= 1) {
			//see if the entire edge is really on the keep side of the plane
			if(distance == Ray::INTERSECTS_NONE || (dot <= 0 && keep[e2] >= 0) || (dot >= 1 && keep[e1] >= 0))
				intersect.set(data->worldVertices[keep[e1] >= 0 ? e2 : e1]);
			//otherwise keep a tiny fraction of the edge so that we don't lose its direction vector
			else intersect.set(data->worldVertices[keep[e1] >= 0 ? e1 : e2] + (keep[e1] >= 0 ? 1 : -1)*0.01f*edgeVec);
		}
		//add the new vertex at the intersection point
		newData->vertices.push_back(intersect);
		unsigned short newInd = newData->vertices.size()-1,
			kept = keep[e1] >= 0 ? e1 : e2, discarded = keep[e1] >= 0 ? e2 : e1;
		replace[discarded] = newInd;
		//add the shortened edge to the new mesh
		newEdge[0] = keep[kept];
		newEdge[1] = newInd;
		newData->edges.push_back(newEdge);
		//add this intersection to the map
		intersections[e1][e2] = newInd;
		intersections[e2][e1] = newInd;
	}

	//modify each polygon according to its intersections
	std::vector<unsigned short> newFace, newTriangle(3);
	std::vector<std::vector<unsigned short> > newTriangles, newEdges;
	bool faceAltered, keepTriangle, intervening;
	short usedNew;
	for(int i = 0; i < data->faces.size(); i++) {
		newFace.clear();
		faceAltered = false;
		intervening = false;
		usedNew = -1;
		for(int j = 0; j < data->faces[i].size(); j++) { //for each old polygon...
			e1 = data->faces[i][j];
			e2 = data->faces[i][(j+1)%data->faces[i].size()];
			if(keep[e1] >= 0) {
				newFace.push_back(keep[e1]);
				if(usedNew >= 0) intervening = true;
			}
			else faceAltered = true;
			if(intersections.find(e1) == intersections.end() || intersections[e1].find(e2) == intersections[e1].end()) continue;
			newFace.push_back(intersections[e1][e2]);
			if(usedNew >= 0) { //we have a new edge formed by the 2 new vertices just added to this face
				//add them in reverse order so the new face on the slice plane has the right orientation
				newEdge[intervening ? 1 : 0] = intersections[e1][e2];
				newEdge[intervening ? 0 : 1] = usedNew;
				newData->edges.push_back(newEdge);
				newEdges.push_back(newEdge);
			}
			faceAltered = true;
			usedNew = intersections[e1][e2];
		}
		if(newFace.size() > 0) {
			newData->faces.push_back(newFace);
			if(faceAltered) { //retriangulate the polygon
				newTriangles.clear();
				/*for(int j = 0; j < data->triangles[i].size(); j++) { //keep as many of the old triangles as possible
					keepTriangle = true;
					for(int k = 0; k < 3; k++) if(!keep[data->triangles[i][j][k]) keepTriangle = false;
					if(keepTriangle) newTriangles.push_back(data->triangles[i][j]);
				}//*/
				for(int j = 1; j < newFace.size()-1; j++) {
					newTriangle[0] = 0;
					newTriangle[1] = j;
					newTriangle[2] = j+1;
					newTriangles.push_back(newTriangle);
				}
				newData->triangles.push_back(newTriangles);
			} else {
				newData->triangles.push_back(data->triangles[i]);
			}
		}
	}
	
	//add the brand new polygons formed by the slice plane
	unsigned short newFaceStart = newData->faces.size(); //note where the slice plane polygons begin
	newFace.clear();
	while(newEdges.size() > 0) {
		if(newFace.empty()) {
			newEdge = newEdges.back();
			newFace.push_back(newEdge[0]);
			newFace.push_back(newEdge[1]);
			newEdges.pop_back();
			continue;
		}
		unsigned short lastPoint = newFace[newFace.size()-1];
		bool found = false, done = false;
		//find the edge that connects to the endpoint of this one
		for(int i = 0; i < newEdges.size(); i++) {
			for(int j = 0; j < 2; j++) {
				if(newEdges[i][j] == lastPoint) {
					newEdge = newEdges[i];
					newEdges.erase(newEdges.begin()+i);
					found = true;
					done = newEdge[(j+1)%2] == newFace[0];
					if(!done) newFace.push_back(newEdge[(j+1)%2]);
					break;
				}
			}
			if(found) { //see if we have closed this polygon (note: may be more than one)
				if(done) {
					newData->faces.push_back(newFace);
					newTriangles.clear();
					for(int j = 1; j < newFace.size()-1; j++) {
						newTriangle[0] = 0;
						newTriangle[1] = j;
						newTriangle[2] = j+1;
						newTriangles.push_back(newTriangle);
					}
					newData->triangles.push_back(newTriangles);
					newFace.clear();
				}
				break;
			}
		}
		if(!found) GP_ERROR("Didn't find edge to continue new polygon");
	}

	//just add 1 convex hull for each convex face
	for(int i = 0; i < newData->faces.size(); i++)
		newData->hulls.push_back(newData->faces[i]);

	//update or discard all the old convex hulls
/*	std::vector<unsigned short> newHull;
	short *inHull = new short[newData->vertices.size()];
	for(int i = 0; i < newData->vertices.size(); i++) inHull[i] = false;
	for(int i = 0; i < data->hulls.size(); i++) {
		newHull.clear();
		for(int j = 0; j < data->hulls[i].size(); j++) {
			e1 = data->hulls[i][j];
			usedNew = keep[e1];
			if(usedNew < 0) usedNew = replace[e1];
			if(usedNew < 0) continue;
			newHull.push_back(usedNew);
			inHull[usedNew] = true;
		}
		if(newHull.size() > 0) newData->hulls.push_back(newHull);
	}
	//add 1 convex hull for each slice plane polygon if it has any vertices that aren't already in a hull
	for(int i = newFaceStart; i < newData->faces.size(); i++) {
		bool needHull = false;
		for(int j = 0; j < newData->faces[i].size() && !needHull; j++) needHull = !inHull[newData->faces[i][j]];
		if(needHull) newData->hulls.push_back(newData->faces[i]);
	}
//*/
	
	//transform the new vertices back to model space before saving the data
	Matrix worldModel;
	_node->getWorldMatrix().invert(&worldModel);
	Vector3 translation(_node->getTranslationWorld()), scale(_node->getScale());
	newData->translation.set(translation);
	newData->scale.set(scale);
	translation.x /= scale.x; translation.y /= scale.y; translation.z /= scale.z;
	for(int i = 0; i < newData->vertices.size(); i++) {
		worldModel.transformVector(&newData->vertices[i]);
		newData->vertices[i] -= translation;
	}
	
	_newNode->setData(NULL);
	_node->setData(newData);
	_node->updateModelFromData();
	translation.set(_node->getTranslationWorld());
	app->placeNode(_node, translation.x, translation.z);
	return true;
}


