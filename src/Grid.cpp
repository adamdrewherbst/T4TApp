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

Mesh* createBoxMesh(float width, float height, float depth)
{
    const unsigned int pointCount = 24;
    const unsigned int verticesSize = pointCount * 6;

    std::vector<float> vertices;
    const float dims[3] = {width, height, depth};
    vertices.resize(verticesSize);

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

    return mesh;
}

Model* createBoxModel(float width, float height, float depth)
{
    Mesh* mesh = createBoxMesh(width, height, depth);
    if (!mesh)
        return NULL;

    Model* model = Model::create(mesh);
    mesh->release();
    assert(model);
    return model;
}

