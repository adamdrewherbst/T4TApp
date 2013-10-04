#include "Grid.h"

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
    const unsigned int pointCount = 8;
    const unsigned int verticesSize = pointCount * 6;

    std::vector<float> vertices;
    vertices.resize(verticesSize);

    Vector4 color(1.0f, 0.0f, 0.0f, 1.0f);
	unsigned int i, j, k, v = 0;
    for (unsigned int n = 0; n < pointCount; ++n)
    {
    	i = n % 2;
    	j = (n % 4)/2;
    	k = n/4;
    	
    	vertices[v++] = i*width;
    	vertices[v++] = j*height;
    	vertices[v++] = k*depth;
    	vertices[v++] = color.x;
    	vertices[v++] = color.y;
    	vertices[v++] = color.z;
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
    //mesh->setPrimitiveType(Mesh::LINES);
    mesh->setVertexData(&vertices[0], 0, pointCount);
    
	short indices[3 * 12];
	indices[0] = 0;
	indices[1] = 2;
	indices[2] = 1;
	indices[3] = 1;
	indices[4] = 2;
	indices[5] = 3;
	indices[6] = 4;
	indices[7] = 5;
	indices[8] = 6;
	indices[9] = 5;
	indices[10] = 7;
	indices[11] = 6;
	indices[12] = 0;
	indices[13] = 1;
	indices[14] = 4;
	indices[15] = 1;
	indices[16] = 5;
	indices[17] = 4;
	indices[18] = 2;
	indices[19] = 6;
	indices[20] = 3;
	indices[21] = 3;
	indices[22] = 6;
	indices[23] = 7;
	indices[24] = 0;
	indices[25] = 4;
	indices[26] = 6;
	indices[27] = 0;
	indices[28] = 6;
	indices[29] = 2;
	indices[30] = 1;
	indices[31] = 7;
	indices[32] = 5;
	indices[33] = 1;
	indices[34] = 3;
	indices[35] = 7;
	
	MeshPart *part = mesh->addPart(Mesh::TRIANGLES, Mesh::INDEX16, 36);
	part->setIndexData(indices, 0, 36);//*/

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

