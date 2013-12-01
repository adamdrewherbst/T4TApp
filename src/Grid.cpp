#include "Grid.h"
#include <cstdio>
using std::cout;
using std::cin;
using std::endl;

Mesh* createGridMesh(unsigned int lineCount)
{
    // There needs to be an odd number of lines
    lineCount |= 1;
    const unsigned int pointCount = lineCount * 4;
    const unsigned int verticesSize = pointCount * (3 + 3);

    std::vector<float> vertices;
    vertices.resize(verticesSize);

    const float gridLength = (float)(lineCount / 2);
    float value = -gridLength;
    for (unsigned int i = 0; i < verticesSize; ++i)
    {
        // Default line color is dark grey
        Vector4 color(0.3f, 0.3f, 0.3f, 1.0f);

        // Very 10th line is brighter grey
        if (((int)value) % 10 == 0)
        {
            color.set(0.45f, 0.45f, 0.45f, 1.0f);
        }

        // The Z axis is blue
        if (value == 0.0f)
        {
            color.set(0.15f, 0.15f, 0.7f, 1.0f);
        }

        // Build the lines
        vertices[i] = value;
        vertices[++i] = 0.0f;
        vertices[++i] = -gridLength;
        vertices[++i] = color.x;
        vertices[++i] = color.y;
        vertices[++i] = color.z;

        vertices[++i] = value;
        vertices[++i] = 0.0f;
        vertices[++i] = gridLength;
        vertices[++i] = color.x;
        vertices[++i] = color.y;
        vertices[++i] = color.z;

        // The X axis is red
        if (value == 0.0f)
        {
            color.set(0.7f, 0.15f, 0.15f, 1.0f);
        }
        vertices[++i] = -gridLength;
        vertices[++i] = 0.0f;
        vertices[++i] = value;
        vertices[++i] = color.x;
        vertices[++i] = color.y;
        vertices[++i] = color.z;

        vertices[++i] = gridLength;
        vertices[++i] = 0.0f;
        vertices[++i] = value;
        vertices[++i] = color.x;
        vertices[++i] = color.y;
        vertices[++i] = color.z;

        value += 1.0f;
    }
    VertexFormat::Element elements[] =
    {
        VertexFormat::Element(VertexFormat::POSITION, 3),
        VertexFormat::Element(VertexFormat::COLOR, 3)
    };
    Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), pointCount, false);
    if (mesh == NULL)
    {
        return NULL;
    }
    mesh->setPrimitiveType(Mesh::LINES);
    mesh->setVertexData(&vertices[0], 0, pointCount);
    unsigned int range = lineCount/2;
    mesh->setBoundingBox(BoundingBox(Vector3(-1.0f*range, -1.0f, -1.0f*range), Vector3(1.0f*range, 0.0f, 1.0f*range)));

    return mesh;
}

Model* createGridModel(unsigned int lineCount)
{
    Mesh* mesh = createGridMesh(lineCount);
    if (!mesh)
        return NULL;

    Model* model = Model::create(mesh);
    mesh->release();
    assert(model);
    return model;
}

Mesh* createBoxMesh(float width, float height, float depth, Node *node)
{
    const unsigned int pointCount = 24;
    const unsigned int verticesSize = pointCount * 6;

    std::vector<float> vertices;
    vertices.resize(verticesSize);
    float *justVertices = new float[8*3];
    const float dims[3] = {width, height, depth};

    Vector4 color(1.0f, 0.0f, 0.0f, 1.0f);
	unsigned int n, v = 0, f, fixed, fixedVal, ind;
    for (n = 0; n < pointCount; ++n)
    {
    	f = n/4; //face index
    	fixed = f/2;
    	fixedVal = f % 2;
    	ind = n % 4;
    	
    	vertices[v+fixed] = (2.0f*fixedVal - 1.0f) * dims[fixed]/2.0f;
    	fixed = (fixed+1) % 3;
    	vertices[v+fixed] = (2.0f*(ind % 2) - 1.0f) * dims[fixed]/2.0f;
    	fixed = (fixed+1) % 3;
    	vertices[v+fixed] = (2.0f*(ind / 2) - 1.0f) * dims[fixed]/2.0f;
    	fixed = (fixed+1) % 3;
    	if(n < 8) for(int i = 0; i < 3; i++) justVertices[n*3 + i] = vertices[v+i];
    	vertices[v+3] = 0;
    	vertices[v+4] = 0;
    	vertices[v+5] = 0;
    	vertices[v+3+fixed] = (fixedVal*2.0f - 1.0f);
    	v += 6;
    	//cout << "(" << vertices[v-6] << "," << vertices[v-5] << "," << vertices[v-4] << "): <" << vertices[v-3] << "," << vertices[v-2] << "," << vertices[v-1] << ">" << endl;
    }
    VertexFormat::Element elements[] =
    {
        VertexFormat::Element(VertexFormat::POSITION, 3),
        VertexFormat::Element(VertexFormat::NORMAL, 3)
    };
    Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), pointCount, false);
    if (mesh == NULL)
    {
        return NULL;
    }
    //mesh->setPrimitiveType(Mesh::LINES);
    mesh->setVertexData(&vertices[0], 0, pointCount);
    
    //build the faces
	short indices[3 * 6 * 2];
	for(n = 0; n < 6; n++)
	{
		if(n % 2 == 0) {
			indices[n*6] = n*4 + 0;
			indices[n*6+1] = n*4 + 2;
			indices[n*6+2] = n*4 + 1;
			indices[n*6+3] = n*4 + 1;
			indices[n*6+4] = n*4 + 2;
			indices[n*6+5] = n*4 + 3;
		} else {
			indices[n*6] = n*4 + 0;
			indices[n*6+1] = n*4 + 1;
			indices[n*6+2] = n*4 + 2;
			indices[n*6+3] = n*4 + 2;
			indices[n*6+4] = n*4 + 1;
			indices[n*6+5] = n*4 + 3;
		}
	}
	
	MeshPart *part = mesh->addPart(Mesh::TRIANGLES, Mesh::INDEX16, 36);
	part->setIndexData(indices, 0, 36);//*/
	
	//add bounding sphere to allow to work with culling materials
	mesh->setBoundingSphere(BoundingSphere(Vector3(0.0f, 0.0f, 0.0f), sqrt((width*width + height*height + depth*depth)/8.0f)));
	mesh->setBoundingBox(BoundingBox(Vector3(-width/2.0f, -height/2.0f, -depth/2.0f), Vector3(width/2.0f, height/2.0f, depth/2.0f)));
	
	if(node != NULL) { //store the vertex coords and edge indices to allow non-OpenGL calculations (eg. closest edge to touch)
		setEdges(node, "box");
		nodeData *data = (nodeData*)node->getUserPointer();
		data->numVertices = 8;
		data->vertices = justVertices;
	}
    return mesh;
}

Model* createBoxModel(float width, float height, float depth, Node* node)
{
    Mesh* mesh = createBoxMesh(width, height, depth, node);
    if (!mesh)
        return NULL;

    Model* model = Model::create(mesh);
    mesh->release();
    assert(model);
    if(node != NULL) {
		node->setModel(model);
	}
    return model;
}

Mesh* createCylinderMesh(float radius, float height, int segments, Node *node)
{
	if(segments < 0) segments = 100;
	//4 vertices for each tall side sliver, <segments> vertices for each end-cap, plus 1 point in the center of each end-cap for triangulation
    const unsigned int pointCount = segments * 4 + segments * 2 + 2;
    const unsigned int verticesSize = pointCount * 6;

    std::vector<float> vertices;
    vertices.resize(verticesSize);
    float *justVertices = new float[segments*2 * 3];

    Vector4 color(1.0f, 0.0f, 0.0f, 1.0f);
	int i, n, ind, v = 0;
	float angle, dAngle = 2*PI/segments;
	//first make vertical faces
    for (n = 0; n < segments; ++n)
    {
    	angle = n * 2*PI / segments;
    	for(ind = 0; ind < 4; ind++) {
    		//vertex coords
    		vertices[v++] = cos(angle + (ind/2)*dAngle) * radius;
    		vertices[v++] = (2 * (ind%2) - 1) * height/2;
    		vertices[v++] = sin(angle + (ind/2)*dAngle) * radius;
    		//normal components
    		vertices[v++] = cos(angle + dAngle/2);
    		vertices[v++] = 0;
    		vertices[v++] = sin(angle + dAngle/2);
    		if(ind < 2) for(i = 0; i < 3; i++) justVertices[(n*2 + ind) * 3 + i] = vertices[v-6 + i];
    	}
    }
    //then end-caps
    for(i = 0; i < 2; i++) {
	    for(n = 0; n < segments; n++) {
			angle = n * 2*PI / segments;
			//vertex
	    	vertices[v++] = cos(angle) * radius;
	    	vertices[v++] = (2 * (i%2) - 1) * height/2;
	    	vertices[v++] = sin(angle) * radius;
	    	//normal
	    	vertices[v++] = 0;
	    	vertices[v++] = 2 * (i%2) - 1;
	    	vertices[v++] = 0;
	    }
	}
	//then end-point centers
	for(i = 0; i < 2; i++) {
		//vertex
		vertices[v++] = 0;
		vertices[v++] = (2 * (i%2) - 1) * height/2;
		vertices[v++] = 0;
		//normal
		vertices[v++] = 0;
		vertices[v++] = 2 * (i%2) - 1;
		vertices[v++] = 0;
	}
	
	for(n = 0; n < pointCount; n++) {
		cout << "vertex " << n << ": ";
		for(i = 0; i < 3; i++) {
			cout << vertices[n*3 + i] << ",";
		}
		cout << endl;
	}
	
    VertexFormat::Element elements[] =
    {
        VertexFormat::Element(VertexFormat::POSITION, 3),
        VertexFormat::Element(VertexFormat::NORMAL, 3)
    };
    Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), pointCount, false);
    if (mesh == NULL)
    {
        return NULL;
    }
    //mesh->setPrimitiveType(Mesh::LINES);
    mesh->setVertexData(&vertices[0], 0, pointCount);
    
    //build the faces
	short *indices = new short[3 * segments*4];
	int f = 0;
	for(n = 0; n < segments; n++)
	{
		//vertical side
		indices[f++] = n*4;
		indices[f++] = n*4 + 1;
		indices[f++] = n*4 + 2;
		indices[f++] = n*4 + 2;
		indices[f++] = n*4 + 1;
		indices[f++] = n*4 + 3;
		//bottom wedge
		indices[f++] = segments*4 + n;
		indices[f++] = segments*4 + (n+1)%segments;
		indices[f++] = segments*6;
		//top wedge
		indices[f++] = segments*5 + n;
		indices[f++] = segments*6 + 1;
		indices[f++] = segments*5 + (n+1)%segments;
	}
	
	MeshPart *part = mesh->addPart(Mesh::TRIANGLES, Mesh::INDEX16, 12*segments);
	part->setIndexData(indices, 0, 12*segments);//*/
	
	//add bounding sphere to allow to work with culling materials
	mesh->setBoundingSphere(BoundingSphere(Vector3(0.0f, 0.0f, 0.0f), sqrt(radius*radius + height*height/4.0f)));
	mesh->setBoundingBox(BoundingBox(Vector3(-radius, -height/2.0f, -radius), Vector3(radius, height/2.0f, radius)));
	
	if(node != NULL) { //store the vertex coords and edge indices to allow non-OpenGL calculations (eg. closest edge to touch)
		setEdges(node, "cylinder", segments);
		nodeData *data = (nodeData*)node->getUserPointer();
		data->numVertices = segments*2;
		data->vertices = justVertices;
	}
    return mesh;
}

Model* createCylinderModel(float radius, float height, int segments, Node* node)
{
    Mesh* mesh = createCylinderMesh(radius, height, segments, node);
    if (!mesh)
        return NULL;

    Model* model = Model::create(mesh);
    mesh->release();
    assert(model);
    if(node != NULL) {
		node->setModel(model);
	}
    return model;
}

Mesh* createConeMesh(float radius, float height, int segments, Node *node)
{
	if(segments < 0) segments = 50;
	//each vertex on the base is used in 3 faces (base, and 2 slanting sides), plus we need the base center for triangulation, and the apex for each slanting face
    const unsigned int pointCount = segments * 4 + 1;
    const unsigned int verticesSize = pointCount * 6;

    std::vector<float> vertices;
    vertices.resize(verticesSize);
    float *justVertices = new float[3 * (segments + 1)];

    Vector4 color(1.0f, 0.0f, 0.0f, 1.0f);
	int i, n, ind, v = 0;
	float angle, dAngle = 2*PI/segments, inclineAngle = atan2(radius, height);
	//first make slanting sides
    for (n = 0; n < segments; ++n)
    {
    	angle = n * dAngle;
    	//2 vertices on base
    	for(ind = 0; ind < 2; ind++) {
    		//vertex coords
    		vertices[v++] = cos(angle + ind*dAngle) * radius;
    		vertices[v++] = -height/2;
    		vertices[v++] = sin(angle + ind*dAngle) * radius;
    		//normal components
    		vertices[v++] = cos(angle + dAngle/2) * cos(inclineAngle);
    		vertices[v++] = sin(inclineAngle);
    		vertices[v++] = sin(angle + dAngle/2) * cos(inclineAngle);
    		for(i = 0; i < 3; i++) justVertices[n*3 + i] = vertices[v-6 + i];
    	}
    	//apex
		//vertex coords
		vertices[v++] = 0;
		vertices[v++] = height/2;
		vertices[v++] = 0;
		//normal components
		vertices[v++] = cos(angle + dAngle/2) * cos(inclineAngle);
		vertices[v++] = sin(inclineAngle);
		vertices[v++] = sin(angle + dAngle/2) * cos(inclineAngle);
    }
    for(i = 0; i < 3; i++) justVertices[segments*3 + i] = vertices[2*6 + i];
    
    //then base
    for(n = 0; n < segments; n++) {
		angle = n * dAngle;
		//vertex
    	vertices[v++] = cos(angle) * radius;
    	vertices[v++] = -height/2;
    	vertices[v++] = sin(angle) * radius;
    	//normal
    	vertices[v++] = 0;
    	vertices[v++] = -1;
    	vertices[v++] = 0;
    }
	//base center
	//vertex
	vertices[v++] = 0;
	vertices[v++] = -height/2;
	vertices[v++] = 0;
	//normal
	vertices[v++] = 0;
	vertices[v++] = -1;
	vertices[v++] = 0;
	
	for(n = 0; n < pointCount; n++) {
		cout << "vertex " << n << ": ";
		for(i = 0; i < 3; i++) {
			cout << vertices[n*6 + i] << ",";
		}
		cout << endl;
	}
	
    VertexFormat::Element elements[] =
    {
        VertexFormat::Element(VertexFormat::POSITION, 3),
        VertexFormat::Element(VertexFormat::NORMAL, 3)
    };
    Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), pointCount, false);
    if (mesh == NULL)
    {
        return NULL;
    }
    //mesh->setPrimitiveType(Mesh::LINES);
    mesh->setVertexData(&vertices[0], 0, pointCount);
    
    //build the faces
	short *indices = new short[3 * segments*2];
	int f = 0;
	for(n = 0; n < segments; n++)
	{
		//slanting side
		indices[f++] = n*3;
		indices[f++] = n*3 + 2;
		indices[f++] = n*3 + 1;
		//bottom wedge
		indices[f++] = segments*3 + n;
		indices[f++] = segments*3 + (n+1)%segments;
		indices[f++] = segments*4;
	}
	
	MeshPart *part = mesh->addPart(Mesh::TRIANGLES, Mesh::INDEX16, 6*segments);
	part->setIndexData(indices, 0, 6*segments);//*/
	
	//add bounding sphere to allow to work with culling materials
	mesh->setBoundingSphere(BoundingSphere(Vector3(0.0f, 0.0f, 0.0f), sqrt(radius*radius + height*height/4.0f)));
	mesh->setBoundingBox(BoundingBox(Vector3(-radius, -height/2.0f, -radius), Vector3(radius, height/2.0f, radius)));
	
	if(node != NULL) { //store the vertex coords and edge indices to allow non-OpenGL calculations (eg. closest edge to touch)
		setEdges(node, "cone", segments);
		nodeData *data = (nodeData*)node->getUserPointer();
		data->numVertices = segments + 1;
		data->vertices = justVertices;
	}
    return mesh;
}

Model* createConeModel(float radius, float height, int segments, Node* node)
{
    Mesh* mesh = createConeMesh(radius, height, segments, node);
    if (!mesh)
        return NULL;

    Model* model = Model::create(mesh);
    mesh->release();
    assert(model);
    if(node != NULL) {
		node->setModel(model);
	}
    return model;
}

void setEdges(Node* node, const char* type, ...)
{
	nodeData* data = new nodeData();
	if(strcmp(type, "box") == 0) {
		data->numEdges = 12;
		int ind = 0;
		cout << "resizing edges vector" << endl;
		data->edges = (unsigned short*)malloc(2 * data->numEdges * sizeof(unsigned short));
		cout << "resized" << endl;
		for(int i = 0; i < 8; i++) { //for each vertex
			for(int j = 1; j < 5 && i+j < 8; j *= 2) { //add 1, 2, or 4 to get the x, y, or z neighbor
				if(i % (2*j) >= j) continue;
				//cout << "adding edges from " << ind << ": " << i << "-" << (i+j) << endl;
				data->edges[ind++] = i;
				data->edges[ind++] = i+j;
			}
		}
	}
	else if(strcmp(type, "cylinder") == 0) {
		va_list arguments;
		va_start(arguments, type);
		int segments = va_arg(arguments, int);
		data->numEdges = segments * 3;
		data->edges = (unsigned short*)malloc(2 * data->numEdges * sizeof(unsigned short));
		int ind = 0;
		for(int i = 0; i < segments; i++) {
			//vertical
			data->edges[ind++] = i*2;
			data->edges[ind++] = i*2 + 1;
			//bottom
			data->edges[ind++] = i*2;
			data->edges[ind++] = (i*2 + 2) % (segments*2);
			//top
			data->edges[ind++] = i*2 + 1;
			data->edges[ind++] = (i*2 + 3) % (segments*2);
		}
	}
	else if(strcmp(type, "cone") == 0) {
		va_list arguments;
		va_start(arguments, type);
		int segments = va_arg(arguments, int);
		data->numEdges = segments * 2;
		data->edges = (unsigned short*)malloc(2 * data->numEdges * sizeof(unsigned short));
		int ind =  0;
		for(int i = 0; i < segments; i++) {
			//slanting edge
			data->edges[ind++] = i;
			data->edges[ind++] = segments;
			//base edge
			data->edges[ind++] = i;
			data->edges[ind++] = (i+1) % segments;
		}
	}
	node->setUserPointer(data);
}

