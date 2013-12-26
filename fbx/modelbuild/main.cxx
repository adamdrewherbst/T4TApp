/****************************************************************************************

   Copyright (C) 2013 Autodesk, Inc.
   All rights reserved.

   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.

****************************************************************************************/

/////////////////////////////////////////////////////////////////////////
//
// The scene created in this example is a cylinder linked to a skeleton
// made of 2 segments. Two animation stacks show the influence of the
// skeleton segments over the cylinder.
//
// The example illustrates how to:
//        1) create a patch
//        2) create a skeleton segment
//        3) create a link
//        4) store the bind pose
//        5) store one arbitrary rest pose
//        6) create multiple animation stacks
//        7) create meta-data and add a thumbnail
//        8) export a scene in a .FBX file (ASCII mode)
//
/////////////////////////////////////////////////////////////////////////

#include <fbxsdk.h>
#include <iostream>

#include "../samples/Common/Common.h"
#include "Thumbnail.h"

#define OUTPUT_DIRECTORY "/home/aherbst/Documents/Programming/GamePlay/tools/encoder/build/input/"
#define SCENE_FILENAME OUTPUT_DIRECTORY "scene.fbx"
#define MODEL_FILENAME OUTPUT_DIRECTORY "models.fbx"

#define PI 3.1415926535

using std::cin; using std::cout; using std::endl;

// Function prototypes.
bool CreateScene(FbxManager* pSdkManager, FbxScene* pScene);
bool CreateModels(FbxManager* pSdkManager, FbxScene* pScene);

FbxNode* CreateCylinder(FbxScene* pScene, const char* pName, float radius, float height, int segments);
FbxNode* CreateBox(FbxScene* pScene, const char* pName, float length, float width, float height);

int main(int argc, char** argv)
{
    FbxManager* lSdkManager = NULL;
    FbxScene *lScene = NULL, *lModels = NULL;
    bool lResult;

    // Prepare the FBX SDK.
    InitializeSdkObjects(lSdkManager, lScene);
	lScene = FbxScene::Create(lSdkManager, "scene");
	lModels = FbxScene::Create(lSdkManager, "models");
	if(!lScene || !lModels)
    {
        FBXSDK_printf("Error: Unable to create FBX scene!\n");
        exit(1);
    }
    
    // Create the scene.
    lResult = CreateScene(lSdkManager, lScene);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while creating the scene...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }

    // Create the models.
    lResult = CreateModels(lSdkManager, lModels);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while creating the models...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }

    // Save the scene and models.

    // The example can take an output file name as an argument.
	const char* lSampleFileName = NULL;
	for( int i = 1; i < argc; ++i )
	{
		if( FBXSDK_stricmp(argv[i], "-test") == 0 ) continue;
		else if( !lSampleFileName ) lSampleFileName = argv[i];
	}
	if( !lSampleFileName ) lSampleFileName = SCENE_FILENAME;
	
	lResult = SaveScene(lSdkManager, lScene, SCENE_FILENAME);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while saving the scene...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }

	lResult = SaveScene(lSdkManager, lModels, MODEL_FILENAME);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while saving the models...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }

    // Destroy all objects created by the FBX SDK.
    DestroySdkObjects(lSdkManager, lResult);

    return 0;
}

bool CreateScene(FbxManager *pSdkManager, FbxScene* pScene)
{
    // create scene info
    FbxDocumentInfo* sceneInfo = FbxDocumentInfo::Create(pSdkManager,"SceneInfo");
    sceneInfo->mTitle = "Scene";
    sceneInfo->mSubject = "Scene for Gameplay3D T4T App";
    sceneInfo->mAuthor = "modelbuild program.";
    sceneInfo->mRevision = "rev. 1.0";
    sceneInfo->mKeywords = "scene";
    sceneInfo->mComment = "no particular comments required.";

    pScene->SetSceneInfo(sceneInfo);
    FbxNode* lRootNode = pScene->GetRootNode();

	// Create a node for our camera in the scene.
	FbxNode* lCameraNode = FbxNode::Create(pScene, "cameraNode");
	FbxCamera* lCamera = FbxCamera::Create(pScene, "camera");
	lCameraNode->SetNodeAttribute(lCamera);
	lRootNode->AddChild(lCameraNode);
	pScene->GetGlobalSettings().SetDefaultCamera((char *) lCamera->GetName());

	// Create a node for our light in the scene.
	FbxNode* lLightNode = FbxNode::Create(pScene, "lightNode");
	FbxLight* lLight = FbxLight::Create(pScene, "light");
	lLightNode->SetNodeAttribute(lLight);
	lLight->LightType.Set(FbxLight::eDirectional);
	lLightNode->LclRotation.Set(FbxVector4(0.0, 0.0, -90.0));
	lRootNode->AddChild(lLightNode);
	
	pScene->GetGlobalSettings().SetAmbientColor(FbxColor(1.0, 1.0, 0.0));

    return true;
}

bool CreateModels(FbxManager *pSdkManager, FbxScene* pScene)
{
    // create scene info
    FbxDocumentInfo* sceneInfo = FbxDocumentInfo::Create(pSdkManager,"SceneInfo");
    sceneInfo->mTitle = "Models";
    sceneInfo->mSubject = "Models for Gameplay3D T4T App";
    sceneInfo->mAuthor = "modelbuild program.";
    sceneInfo->mRevision = "rev. 1.0";
    sceneInfo->mKeywords = "models";
    sceneInfo->mComment = "no particular comments required.";

    pScene->SetSceneInfo(sceneInfo);

    // Build the node tree.
    FbxNode* lRootNode = pScene->GetRootNode();
    FbxNode* lPatch = CreateCylinder(pScene, "cylinder", 1.0, 3.0, 20);
    lRootNode->AddChild(lPatch);
    lPatch = CreateBox(pScene, "box", 2.0, 2.0, 2.0);
    lRootNode->AddChild(lPatch);

    return true;
}

FbxNode* CreateCylinder(FbxScene* pScene, const char* pName, float radius, float height, int segments) {
	FbxMesh* mesh = FbxMesh::Create(pScene, pName);
	mesh->InitControlPoints(segments+1);
	int v = 0;
	for(int n = 0; n < 2; n++) {
		for(int i = 0; i < segments; i++) {
			mesh->SetControlPointAt(FbxVector4(radius * cos(2*PI*i/segments), (2*n-1) * height/2, radius * sin(2*PI*i/segments)), v++);
		}
		mesh->SetControlPointAt(FbxVector4(0.0, (2*n-1) * height/2, 0.0), v++);
	}
	for(int i = 0; i < segments; i++) {
		//bottom
		mesh->BeginPolygon();
		mesh->AddPolygon(i);
		mesh->AddPolygon((i+1) % segments);
		mesh->AddPolygon(segments);
		mesh->EndPolygon();
		//top
		mesh->BeginPolygon();
		mesh->AddPolygon(segments+1 + i);
		mesh->AddPolygon(segments+1 + segments);
		mesh->AddPolygon(segments+1 + (i+1) % segments);
		mesh->EndPolygon();
		//side
		mesh->BeginPolygon();
		mesh->AddPolygon(i);
		mesh->AddPolygon(segments+1 + i);
		mesh->AddPolygon((i+1) % segments);
		mesh->EndPolygon();

		mesh->BeginPolygon();
		mesh->AddPolygon((i+1) % segments);
		mesh->AddPolygon(segments+1 + i);
		mesh->AddPolygon(segments+1 + (i+1) % segments);
		mesh->EndPolygon();
	}
	
	//add normals
	mesh->GenerateNormals(true, false, false);
	
    FbxNode* lNode = FbxNode::Create(pScene,pName);

    FbxVector4 lR(0.0, 0.0, 0.0);
    lNode->LclRotation.Set(lR);
    lNode->SetNodeAttribute(mesh);
    return lNode;
}

FbxNode* CreateBox(FbxScene* pScene, const char* pName, float length, float width, float height) {
	FbxMesh* mesh = FbxMesh::Create(pScene, pName);
	//vertices
	mesh->InitControlPoints(8);
	int v = 0;
	for(int i = 0; i < 2; i++) {
		for(int j = 0; j < 2; j++) {
			for(int k = 0; k < 2; k++) {
				mesh->SetControlPointAt(FbxVector4((2*i-1) * width/2, (2*j-1) * height/2, (2*k-1) * length/2), v++);
			}
		}
	}
	//faces
	int next, prev, offset;
	for(int i = 1; i <= 4; i*=2) {
		for(int j = 0; j < 2; j++) {

			next = i*2; if(next > 4) next = 1;
			prev = next*2; if(prev > 4) prev = 1;
			offset = j * prev;
			
			mesh->BeginPolygon();
			mesh->AddPolygon(offset + 0);
			mesh->AddPolygon(offset + (j ? next : i));
			mesh->AddPolygon(offset + (j ? i : next));
			mesh->EndPolygon();

			mesh->BeginPolygon();
			mesh->AddPolygon(offset + next);
			mesh->AddPolygon(offset + (j ? i+next : i));
			mesh->AddPolygon(offset + (j ? i : i+next));
			mesh->EndPolygon();
		}
	}
	mesh->GenerateNormals(true, false, false);
	
    FbxNode* lNode = FbxNode::Create(pScene,pName);

    FbxVector4 lR(0.0, 0.0, 0.0);
    lNode->LclRotation.Set(lR);
    lNode->SetNodeAttribute(mesh);
    return lNode;
}

