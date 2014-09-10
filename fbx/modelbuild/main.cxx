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
#include <ios>
#include <ostream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <cmath>

#include "../samples/Common/Common.h"
#include "Thumbnail.h"

#define OUTPUT_DIRECTORY "/home/aherbst/Documents/Programming/GamePlay/tools/encoder/build/input/"
#define APP_RESOURCE_DIRECTORY "/home/aherbst/Documents/Programming/GamePlay/projects/t4tapp/build/bin/linux/res/common/"
#define SCENE_FILENAME OUTPUT_DIRECTORY "scene.fbx"
#define MODEL_FILENAME OUTPUT_DIRECTORY "models.fbx"
#define VEHICLE_FILENAME OUTPUT_DIRECTORY "vehicle.fbx"

#define PI 3.1415926535

using std::cin; using std::cout; using std::endl; using std::ostream; using std::ofstream; using std::ios;

// Function prototypes.
bool CreateScene(FbxManager* pSdkManager, FbxScene* pScene);
bool CreateModels(FbxManager* pSdkManager, FbxScene* pScene);
bool CreateVehicle(FbxManager* pSdkManager, FbxScene* pScene);

FbxNode** CreateNode(FbxScene* pScene, const char* pName, const char* type, ...);
FbxMesh** CreateCylinder(FbxScene* pScene, const char* pName, float radius, float height, int segments);
FbxMesh** CreateBox(FbxScene* pScene, const char* pName, float length, float width, float height);
FbxMesh** CreateHalfPipe(FbxScene* pScene, const char* pName, float radius, float height, float thickness, int segments);
FbxMesh** CreateSphere(FbxScene* pScene, const char* pName, float radius, int segments);

char* concat(int n, ...);

void printVector(ostream& of, FbxVector4 vec) {
	of << vec[0] << " " << vec[1] << " " << vec[2] << endl;
}
void printEdge(ostream& of, int v1, int v2) {
	of << v1 << " " << v2 << endl;
}

int main(int argc, char** argv)
{
    FbxManager* lSdkManager = NULL;
    FbxScene *lScene = NULL, *lModels = NULL, *lVehicle = NULL;
    bool lResult;

    // Prepare the FBX SDK.
    InitializeSdkObjects(lSdkManager, lScene);
	lScene = FbxScene::Create(lSdkManager, "scene");
	lModels = FbxScene::Create(lSdkManager, "models");
	lVehicle = FbxScene::Create(lSdkManager, "vehicle");
	if(!lScene || !lModels || !lVehicle)
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
    }//*/
    // Create the models.
    lResult = CreateModels(lSdkManager, lModels);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while creating the models...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }//*/
    // Create the vehicle.
    /*lResult = CreateVehicle(lSdkManager, lVehicle);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while creating the vehicle...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }//*/

    // Save the scene and models.

	lResult = SaveScene(lSdkManager, lScene, SCENE_FILENAME);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while saving the scene...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }//*/
	lResult = SaveScene(lSdkManager, lModels, MODEL_FILENAME);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while saving the models...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }
	/*lResult = SaveScene(lSdkManager, lVehicle, VEHICLE_FILENAME);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while saving the vehicle...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }//*/

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
    FbxNode** lPatch;
   	lPatch = CreateNode(pScene, "cylinder", "cylinder", 1.0, 3.0, 20);
    lPatch = CreateNode(pScene, "box", "box", 2.0, 2.0, 2.0);
    lPatch = CreateNode(pScene, "halfpipe", "halfpipe", 1.0, 3.0, 0.2, 20);
    lPatch = CreateNode(pScene, "sphere", "sphere", 1.0, 6);

    return true;
}

bool CreateVehicle(FbxManager *pSdkManager, FbxScene* pScene)
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
    FbxNode* lCarNode = FbxNode::Create(pScene,"car");
    FbxNode* lWheelsNode = FbxNode::Create(pScene,"wheels");
    lRootNode->AddChild(lCarNode);
    lCarNode->AddChild(lWheelsNode);
    FbxNode* lPatch;
    char id[] = "cylinder1";
    for(int i = 0; i < 4; i++) {
    	id[8] = (char)(48+i+1);
    	lPatch = CreateNode(pScene, id, "cylinder", 0.5, 0.5, 20)[0];
	   	lPatch->LclRotation.Set(FbxVector4(90.0, 0.0, 0.0));
    	lPatch->LclTranslation.Set(FbxVector4(i<2 ? 2.0 : -2.0, 0.5, i%2==0 ? -1.75 : 1.75));
	    lWheelsNode->AddChild(lPatch);
	}
    lPatch = CreateNode(pScene, "box", "box", 4.0, 6.0, 2.0)[0];
    lPatch->LclTranslation.Set(FbxVector4(0.0, 3.2, 0.0));
    lCarNode->AddChild(lPatch);

    lPatch = CreateNode(pScene, "ground", "box", 100.0, 100.0, 1.0)[0];
    lPatch->LclTranslation.Set(FbxVector4(0.0, -10.0, 0.0));
    lRootNode->AddChild(lPatch);
    
	// Create a node for our light in the scene.
	FbxNode* lLightNode = FbxNode::Create(pScene, "lightNode");
	FbxLight* lLight = FbxLight::Create(pScene, "light");
	lLightNode->SetNodeAttribute(lLight);
	lLight->LightType.Set(FbxLight::eDirectional);
	lLightNode->LclRotation.Set(FbxVector4(0.0, 0.0, -90.0));
	lRootNode->AddChild(lLightNode);
	
	// Create a node for our camera in the scene.
	FbxNode* lCameraNode = FbxNode::Create(pScene, "cameraNode");
	FbxCamera* lCamera = FbxCamera::Create(pScene, "camera");
	lCameraNode->SetNodeAttribute(lCamera);
	lRootNode->AddChild(lCameraNode);
	pScene->GetGlobalSettings().SetDefaultCamera((char *) lCamera->GetName());

	pScene->GetGlobalSettings().SetAmbientColor(FbxColor(1.0, 1.0, 0.0));

    return true;
}

FbxNode** CreateNode(FbxScene* pScene, const char* pName, const char* type, ...) {

	FbxMesh **arr, *mesh;

	va_list arguments;
	va_start(arguments, type);

    //write the vertices and edges to a file to be loaded by the app as reference data
    char* filename = new char[strlen(APP_RESOURCE_DIRECTORY) + strlen(pName) + 5];
    filename[0] = '\0';
    strcat(filename, APP_RESOURCE_DIRECTORY);
    strcat(filename, pName);
    strcat(filename, ".node");
    //why doesn't this line work the 2nd time around?
    //const char* filename = (std::string(APP_RESOURCE_DIRECTORY) + std::string(pName) + std::string(".node")).c_str();
    cout << "writing node file:\n" << filename << endl;
    ofstream out(filename, ios::out | ios::trunc);
    out << type << endl;
    int numParts = 2;

	//each mesh creation function, eg. CreateSphere, returns an array:
	// the entire mesh with its polygonal faces, where each face is followed by its constituent triangles and neighbors
	if(strcmp(type, "sphere") == 0) {
		float radius = (float)va_arg(arguments, double);
		int segments = va_arg(arguments, int);
		arr = CreateSphere(pScene, pName, radius, segments);
		mesh = arr[0];
		//rotation, translation, scale
		out << "1\t0\t0\t0" << endl;
		out << "0\t0\t0" << endl;
		out << "0.5\t0.5\t0.5" << endl;
		//write vertex/edge data
		out << segments*(segments-1)+2 << endl;
		for(int i = 0; i < segments*(segments-1)+2; i++) {
			printVector(out, mesh->GetControlPointAt(i));
		}
		out << segments*(2*segments-1) << endl;
		int start;
		for(int i = 0; i < segments-1; i++) {
			start = 1+i*segments;
			for(int j = 0; j < segments; j++) {
				printEdge(out, start+j, start+(j+1)%segments);
				if(i == 0) printEdge(out, start+j, 0);
				if(i < segments-2) printEdge(out, start+j, start+segments+j);
				else printEdge(out, start+j, segments*(segments-1)+1);
			}
		}
		//faces/triangles
		out << segments*segments << endl;
		for(int i = 0; i < segments; i++) {
			start = 1+(i-1)*segments;
			for(int j = 0; j < segments; j++) {
				if(i > 0 && i < segments-1) {
					out << 4 << endl; //face
					out << start+j << "\t" << start+(j+1)%segments << "\t" << start+segments+(j+1)%segments << "\t" << start+segments+j << endl;
					out << 2 << endl; //triangles
					out << 0 << "\t" << 1 << "\t" << 2 << endl;
					out << 0 << "\t" << 2 << "\t" << 3 << endl;
					out << 4 << endl; //neighbors
					out << (i-1)*segments + j << "\t" << i*segments + (j-1+segments)%segments << "\t"
						<< i*segments + (j+1)%segments << "\t" << (i+1)*segments + j << endl;
				} else if(i == 0) {
					out << 3 << endl; //face
					out << 0 << "\t" << 1+(j+1)%segments << "\t" << 1+j << endl;
					out << 1 << endl; //triangle
					out << 0 << "\t" << 1 << "\t" << 2 << endl;
					out << 3 << endl; //neighbors
					out << (j-1+segments)%segments << "\t" << (j+1)%segments << "\t" << segments+j << endl;
				} else if(i == segments-1) {
					out << 3 << endl; //face
					out << start+j << "\t" << start+(j+1)%segments << "\t" << 1+segments*(segments-1) << endl;
					out << 1 << endl; //triangle
					out << 0 << "\t" << 1 << "\t" << 2 << endl;
					out << 3 << endl; //neighbors
					out << (segments-1)*segments + (j-1+segments)%segments << "\t" << (segments-1)*segments + (j+1)%segments
						<< "\t" << (segments-2)*segments+j << endl;
				}
			}
		}
		//physics object
		out << "sphere" << endl;
		//convex hulls
		out << 1 << endl << segments*(segments-1)+2 << endl;
		for(int i = 0; i < segments*(segments-1)+2; i++) out << i << "\t";
		out << endl;
	}
	if(strcmp(type, "cylinder") == 0) {
		float radius = (float)va_arg(arguments, double);
		float height = (float)va_arg(arguments, double);
		int segments = va_arg(arguments, int);
		arr = CreateCylinder(pScene, pName, radius, height, segments);
		mesh = arr[0];
		//rotation, translation, scale
		out << "1\t0\t0\t0" << endl;
		out << "0\t0\t0" << endl;
		out << "1\t1\t1" << endl;
		//write vertex/edge data
		out << 2*segments+2 << endl;
		for(int i = 0; i < 2*segments+2; i++) {
			printVector(out, mesh->GetControlPointAt(i));
		}
		out << 3*segments << endl;
		for(int i = 0; i < segments; i++) {
			printEdge(out, i, (i+1)%segments);
			printEdge(out, segments+1 + i, segments+1 + (i+1)%segments);
			printEdge(out, i, segments+1 + i);
		}
		//faces/triangles
		out << segments+2 << endl;
		for(int i = 0; i < segments; i++) { //walls
			out << 4 << endl; //face
			out << i << "\t" << (i+1)%segments << "\t" << segments+1 + (i+1)%segments << "\t" << segments+1 + i << endl;
			out << 2 << endl; //triangles
			out << 0 << "\t" << 1 << "\t" << 2 << endl;
			out << 0 << "\t" << 2 << "\t" << 3 << endl;
			out << 4 << endl; //neighbors
			out << (i-1+segments)%segments << "\t" << (i+1)%segments << "\t" << segments << "\t" << segments+1 << endl;
		}
		for(int n = 0; n < 2; n++) { //end caps
			out << segments << endl; //face
			for(int i = 0; i < segments; i++) out << n*(segments+1) + (n == 0 ? segments-1-i : i) << "\t";
			out << endl;
			out << segments << endl; //triangles
			for(int i = 0; i < segments; i++) {
				out << i << "\t" << (i+1)%segments << "\t" << segments << endl;
			}
			out << segments << endl; //neighbors
			for(int i = 0; i < segments; i++) out << i << "\t";
			out << endl;
		}
		//physics object
		out << "mesh" << endl;
		//convex hulls
		out << 1 << endl << 2*segments+2 << endl;
		for(int i = 0; i < 2*segments+2; i++) out << i << "\t";
		out << endl;
	}
	else if(strcmp(type, "halfpipe") == 0) {
		float radius = (float)va_arg(arguments, double);
		float height = (float)va_arg(arguments, double);
		float thickness = (float)va_arg(arguments, double);
		int segments = va_arg(arguments, int);
		arr = CreateHalfPipe(pScene, pName, radius, height, thickness, segments);
		mesh = arr[0];
		numParts = 1+segments;
		//rotation, translation, scale
		out << "1\t0\t0\t180" << endl;
		out << "0\t0\t0" << endl;
		out << "4\t1\t1" << endl;
		//write vertex/edge data
		out << 2*segments+4 << endl;
		for(int i = 0; i < 2*segments+4; i++) {
			printVector(out, mesh->GetControlPointAt(i));
		}
		out << 3*segments+6 << endl;
		for(int i = 0; i < segments; i++) {
			printEdge(out, i, i+2);
			printEdge(out, segments+2 + i, segments+2 + i+2);
			printEdge(out, i, segments+2 + i);
		}
		printEdge(out, 0, 1);
		printEdge(out, segments+2, segments+2 + 1);
		printEdge(out, segments, segments + 1);
		printEdge(out, segments+2 + segments, segments+2 + segments + 1);
		printEdge(out, segments, segments+2 + segments);
		printEdge(out, segments+1, segments+2 + segments+1);
		//faces/triangles
		out << 2*segments+2 << endl;
		for(int i = 0; i < segments/2; i++) {
			for(int n = 0; n < 4; n++) {
				out << 4 << endl; //face
				switch(n) { //outside, inside, top, bottom
				case 0: out << 2*i << "\t" << 2*i+2 << "\t" << segments+2+2*i+2 << "\t" << segments+2+2*i << endl; break;
				case 1: out << 2*i+1 << "\t" << segments+2+2*i+1 << "\t" << segments+2+2*i+3 << "\t" << 2*i+3 << endl; break;
				case 2: out << segments+2+2*i << "\t" << segments+2+2*i+2 << "\t" << segments+2+2*i+3 << "\t" << segments+2+2*i+1 << endl; break;
				case 3: out << 2*i << "\t" << 2*i+1 << "\t" << 2*i+3 << "\t" << 2*i+2 << endl; break;
				}
				out << 2 << endl; //triangles
				out << 0 << "\t" << 1 << "\t" << 2 << endl;
				out << 0 << "\t" << 2 << "\t" << 3 << endl;
				out << 4 << endl; //neighbors
				out << 4*i + 2*(1 - n/2) << "\t" << 4*i + 2*(1 - n/2) + 1 << "\t";
				if(i == 0) out << 2*segments << "\t";
				else out << (i-1)*4 + n << "\t";
				if(i == segments/2 - 1) out << 2*segments + 1 << "\t";
				else out << (i+4)*4 + n << "\t";
				out << endl;
			}
		}
		for(int i = 0; i < 2; i++) {
			out << 4 << endl; //face
			if(i == 0) out << 0 << "\t" << segments+2 << "\t" << segments+2+1 << "\t" << 1 << endl;
			else out << segments+1 << "\t" << segments+2+segments+1 << "\t" << segments+2+segments << "\t" << segments << endl;
			out << 2 << endl; //triangles
			out << 0 << "\t" << 1 << "\t" << 2 << endl;
			out << 0 << "\t" << 2 << "\t" << 3 << endl;
			out << 4 << endl; //neighbors
			for(int j = 0; j < 4; j++) out << i*(segments/2 - 1)*4 + j << "\t";
			out << endl;
		}
		//physics object
		out << "mesh" << endl;
		//convex hulls
		out << 2*segments+2 << endl;
		for(int n = 0; n < 4; n++) {
			for(int i = 0; i < segments/2; i++) {
				out << 4 << endl;
				switch(n) { //outside, inside, top, bottom
				case 0: out << 2*i << "\t" << 2*i+2 << "\t" << segments+2+2*i+2 << "\t" << segments+2+2*i << endl; break;
				case 1: out << 2*i+1 << "\t" << segments+2+2*i+1 << "\t" << segments+2+2*i+3 << "\t" << 2*i+3 << endl; break;
				case 2: out << segments+2+2*i << "\t" << segments+2+2*i+2 << "\t" << segments+2+2*i+3 << "\t" << segments+2+2*i+1 << endl; break;
				case 3: out << 2*i << "\t" << 2*i+1 << "\t" << 2*i+3 << "\t" << 2*i+2 << endl; break;
				}
			}
		}
		out << 4 << endl;
		out << 0 << "\t" << 1 << "\t" << segments+2+1 << "\t" << segments+2 << endl;
		out << 4 << endl;
		out << segments << "\t" << segments+1 << "\t" << segments+2+segments+1 << "\t" << segments+2+segments << endl;
	}
	else if(strcmp(type, "box") == 0) {
		float length = (float)va_arg(arguments, double);
		float width = (float)va_arg(arguments, double);
		float height = (float)va_arg(arguments, double);
		arr = CreateBox(pScene, pName, length, width, height);
		mesh = arr[0];
		//rotation, translation, scale
		out << "1\t0\t0\t0" << endl;
		out << "0\t0\t0" << endl;
		out << "3\t1\t2" << endl;
		//write vertex/edge data
		out << 8 << endl;
		for(int i = 0; i < 8; i++) {
			printVector(out, mesh->GetControlPointAt(i));
		}
		out << 12 << endl;
		for(int i = 0; i < 7; i++) {
			for(int j = 1; j <= 4; j*=2) {
				if(i%(j*2) < j && i+j < 12) printEdge(out, i, i+j);
			}
		}
		//faces/triangles
		out << 6 << endl;
		int dir[] = {1,2,4}, d1, d2, d3, start;
		for(int i = 0; i < 6; i++) {
			out << 4 << endl; //face
			d1 = dir[(i/2 + 1 - (1-i%2)) % 3];
			d2 = dir[(i/2 + 1 - i%2) % 3];
			start = (i%2) * dir[(i/2 + 2) % 3];
			out << start << "\t" << start+d1 << "\t" << start+d1+d2 << "\t" << start+d2 << endl;
			out << 2 << endl; //triangles
			out << 0 << "\t" << 1 << "\t" << 2 << endl;
			out << 0 << "\t" << 2 << "\t" << 3 << endl;
			out << 4 << endl; //neighbors
			for(int j = 0; j < 6; j++) if(j != i && j != i + (1 - 2*(i%2))) out << j << "\t";
			out << endl;
		}
		//physics object
		out << "box" << endl;
		//convex hulls
		out << 6 << endl;
		for(int i = 0; i < 6; i++) {
			out << 4 << endl;
			d1 = dir[(i/2 + 1 - (1-i%2)) % 3];
			d2 = dir[(i/2 + 1 - i%2) % 3];
			start = (i%2) * dir[(i/2 + 2) % 3];
			out << start << "\t" << start+d1 << "\t" << start+d1+d2 << "\t" << start+d2 << endl;
		}
	}
	
	//no physics constraints when node is first written - may be added by app users
	out << 0 << endl;
	//mass is 10 by default
	out << 10 << endl;

	out.close();
	
    int n = 0;
    FbxNode** nodes = new FbxNode*[numParts];

	//create the node and its convex hull parts
    FbxNode *lNode = FbxNode::Create(pScene,pName), *lRootNode = pScene->GetRootNode();
    FbxVector4 lR(0.0, 0.0, 0.0);
    lNode->LclRotation.Set(lR);
    lNode->SetNodeAttribute(mesh);
    nodes[n++] = lNode;
    lRootNode->AddChild(lNode);
    //add the mesh parts as subnodes
    for(int i = 1; i < numParts; i++) {
    	std::stringstream ss;
    	ss << i;
    	lNode = FbxNode::Create(pScene,concat(3,pName,"_part",ss.str().c_str()));
    	lNode->SetNodeAttribute(arr[i]);
    	nodes[n++] = lNode;
    	lRootNode->AddChild(lNode);
    	FbxMesh* mesh = lNode->GetMesh();
    	cout << "POINTS FOR " << lNode->GetName() << endl;
    	mesh->BeginPolygon();
    	for(int j = 0; j < mesh->GetControlPointsCount(); j++) {
    		mesh->AddPolygon(j);
    		printVector(cout, mesh->GetControlPointAt(j));
    	}
    	mesh->EndPolygon();
    	cout << endl;
    }
    
    return nodes;
}

FbxMesh** CreateSphere(FbxScene* pScene, const char* pName, float radius, int segments) {
	FbxMesh **arr = new FbxMesh*[2], *mesh;
	arr[0] = FbxMesh::Create(pScene, pName);
	arr[1] = FbxMesh::Create(pScene, concat(2, pName, "_part1"));
	mesh = arr[0];
	mesh->InitControlPoints(segments*(segments-1)+2);
	int v = 0;
	float latitude, longitude;
	for(int i = 0; i <= segments; i++) {
		if(i == 0 || i == segments) {
			mesh->SetControlPointAt(FbxVector4(0.0, 0.0, i == 0 ? -radius : radius), v++);
			continue;
		}
		latitude = M_PI*(-0.5 + 1.0*i/segments);
		for(int j = 0; j < segments; j++) {
			longitude = 2*M_PI*j/segments;
			mesh->SetControlPointAt(FbxVector4(radius*cos(longitude)*cos(latitude), radius*sin(longitude)*cos(latitude), radius*sin(latitude)), v++);
		}
	}
	arr[1]->InitControlPoints(segments*(segments-1)+2);
	for(int i = 0; i < segments*(segments-1)+2; i++) {
		arr[1]->SetControlPointAt(mesh->GetControlPointAt(i), i);
	}
	
	int start;
	for(int i = 0; i < segments-2; i++) {
		start = i*segments+1;
		for(int j = 0; j < segments; j++) {
			mesh->BeginPolygon();
			mesh->AddPolygon(start+j);
			mesh->AddPolygon(start+(j+1)%segments);
			mesh->AddPolygon(start+segments+j);
			mesh->EndPolygon();

			mesh->BeginPolygon();
			mesh->AddPolygon(start+(j+1)%segments);
			mesh->AddPolygon(start+segments+(j+1)%segments);
			mesh->AddPolygon(start+segments+j);
			mesh->EndPolygon();
		}
	}
	for(int j = 0; j < segments; j++) {
		mesh->BeginPolygon();
		mesh->AddPolygon(0);
		mesh->AddPolygon(1+(j+1)%segments);
		mesh->AddPolygon(1+j);
		mesh->EndPolygon();

		mesh->BeginPolygon();
		mesh->AddPolygon(1+segments*(segments-2)+j);
		mesh->AddPolygon(1+segments*(segments-2)+(j+1)%segments);
		mesh->AddPolygon(1+segments*(segments-1));
		mesh->EndPolygon();
	}
	//add normals
	mesh->GenerateNormals(true, false, false);
	return arr;
}

FbxMesh** CreateCylinder(FbxScene* pScene, const char* pName, float radius, float height, int segments) {
	FbxMesh **arr = new FbxMesh*[2], *mesh;
	arr[0] = FbxMesh::Create(pScene, pName);
	arr[1] = FbxMesh::Create(pScene, concat(2,pName,"_part1"));
	mesh = arr[0];
	mesh->InitControlPoints(2*(segments+1));
	int v = 0;
	for(int n = 0; n < 2; n++) {
		for(int i = 0; i < segments; i++) {
			mesh->SetControlPointAt(FbxVector4((2*n-1) * height/2, radius * cos(2*PI*i/segments), radius * sin(2*PI*i/segments)), v++);
		}
		mesh->SetControlPointAt(FbxVector4((2*n-1) * height/2, 0.0, 0.0), v++);
	}
	//single convex hull
	arr[1]->InitControlPoints(2*segments);
	for(int i = 0; i < segments; i++) {
		arr[1]->SetControlPointAt(mesh->GetControlPointAt(i), 2*i);
		arr[1]->SetControlPointAt(mesh->GetControlPointAt(segments+1+i), 2*i+1);
	}

	for(int i = 0; i < segments; i++) {
		//bottom
		mesh->BeginPolygon();
		mesh->AddPolygon(i);
		mesh->AddPolygon(segments);
		mesh->AddPolygon((i+1) % segments);
		mesh->EndPolygon();
		//top
		mesh->BeginPolygon();
		mesh->AddPolygon(segments+1 + i);
		mesh->AddPolygon(segments+1 + (i+1) % segments);
		mesh->AddPolygon(segments+1 + segments);
		mesh->EndPolygon();
		//side
		mesh->BeginPolygon();
		mesh->AddPolygon(i);
		mesh->AddPolygon((i+1) % segments);
		mesh->AddPolygon(segments+1 + i);
		mesh->EndPolygon();

		mesh->BeginPolygon();
		mesh->AddPolygon((i+1) % segments);
		mesh->AddPolygon(segments+1 + (i+1) % segments);
		mesh->AddPolygon(segments+1 + i);
		mesh->EndPolygon();
	}
	
	//add normals
	mesh->GenerateNormals(true, false, false);
	return arr;
}

FbxMesh** CreateHalfPipe(FbxScene* pScene, const char* pName, float radius, float height, float thickness, int segments) {
	FbxMesh **arr = new FbxMesh*[1+segments+4], *mesh;
	arr[0] = FbxMesh::Create(pScene, pName);
	mesh = arr[0];
	//need 1 convex hull for each plane in the half-pipe, otherwise hull would exceed the object bounds
	mesh->InitControlPoints(2*segments+4);
	int v = 0;
	double ang = -M_PI/2;
	for(int n = 0; n < 2; n++) {
		for(int i = 0; i < segments/2+1; i++) {
			mesh->SetControlPointAt(FbxVector4((2*n-1) * height/2, radius * cos(ang + 2*M_PI*i/segments), radius * sin(ang + 2*M_PI*i/segments)), v++);
			mesh->SetControlPointAt(FbxVector4((2*n-1) * height/2, (radius-thickness) * cos(ang + 2*M_PI*i/segments), (radius - thickness) * sin(ang + 2*M_PI*i/segments)), v++);
		}
	}
	//hulls for inner & outer bowls
	int part = 1, start;
	for(int n = 0; n < 2; n++) {
		for(int i = 0; i < segments/2; i++) {
			v = 0; start = 2*i + n;
			std::stringstream ss; ss << part;
			arr[part] = FbxMesh::Create(pScene, concat(3,pName,"_part",ss.str().c_str()));
			arr[part]->InitControlPoints(4);
			arr[part]->SetControlPointAt(mesh->GetControlPointAt(start), v++);
			arr[part]->SetControlPointAt(mesh->GetControlPointAt(start+2), v++);
			arr[part]->SetControlPointAt(mesh->GetControlPointAt(start+2+segments+2), v++);
			arr[part]->SetControlPointAt(mesh->GetControlPointAt(start+segments+2), v++);
			part++;
		}
	}
	//hulls for left/right edges
/*	for(int n = 0; n < 2; n++) {
		v = 0; start = n*segments;
		std::stringstream ss; ss << part;
		arr[part] = FbxMesh::Create(pScene, concat(3,pName,"_part",ss.str().c_str()));
		arr[part]->InitControlPoints(4);
		arr[part]->SetControlPointAt(mesh->GetControlPointAt(start), v++);
		arr[part]->SetControlPointAt(mesh->GetControlPointAt(start+1), v++);
		arr[part]->SetControlPointAt(mesh->GetControlPointAt(start+1+segments+2), v++);
		arr[part]->SetControlPointAt(mesh->GetControlPointAt(start+segments+2), v++);
		part++;		
	}
	//hulls for semicircular ends
	for(int n = 0; n < 2; n++) {
		v = 0;
		std::stringstream ss; ss << part;
		arr[part] = FbxMesh::Create(pScene, concat(3,pName,"_part",ss.str().c_str()));
		arr[part]->InitControlPoints(segments+2);
		for(int i = 0; i < segments+2; i++) {
			arr[part]->SetControlPointAt(mesh->GetControlPointAt(i + n*(segments+2)), v++);
		}
		part++;		
	}
//*/
	int up = segments+2;
	for(int i = 0; i < segments; i++) {
		int next=i+2, across=i+1;
		bool outer = i%2==0;
		//bottom
		mesh->BeginPolygon();
		mesh->AddPolygon(i);
		mesh->AddPolygon(!outer ? next : across);
		mesh->AddPolygon(!outer ? across : next);
		mesh->EndPolygon();
		//top
		mesh->BeginPolygon();
		mesh->AddPolygon(up + i);
		mesh->AddPolygon(up + (!outer ? across : next));
		mesh->AddPolygon(up + (!outer ? next : across));
		mesh->EndPolygon();
		//side
		mesh->BeginPolygon();
		mesh->AddPolygon(i);
		mesh->AddPolygon(!outer ? up+i : next);
		mesh->AddPolygon(!outer ? next : up+i);
		mesh->EndPolygon();

		mesh->BeginPolygon();
		mesh->AddPolygon(next);
		mesh->AddPolygon(!outer ? up+i : up+i+2);
		mesh->AddPolygon(!outer ? up+i+2 : up+i);
		mesh->EndPolygon();
	}
	//side edges of half-pipe
	for(int i = 0; i < segments+1; i+=segments) {
		bool left = i == 0;
		mesh->BeginPolygon();
		mesh->AddPolygon(i);
		mesh->AddPolygon(!left ? i+1+up : i+up);
		mesh->AddPolygon(!left ? i+up : i+1+up);
		mesh->EndPolygon();

		mesh->BeginPolygon();
		mesh->AddPolygon(i);
		mesh->AddPolygon(!left ? i+1 : i+1+up);
		mesh->AddPolygon(!left ? i+1+up : i+1);
		mesh->EndPolygon();
	}
	
	//add normals
	mesh->GenerateNormals(true, false, false);
	return arr;
}

FbxMesh** CreateBox(FbxScene* pScene, const char* pName, float length, float width, float height) {
	FbxMesh **arr = new FbxMesh*[2], *mesh;
	arr[0] = FbxMesh::Create(pScene, pName);
	arr[1] = FbxMesh::Create(pScene, concat(2,pName,"_part1"));
	mesh = arr[0];
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
	arr[1]->InitControlPoints(8);
	for(int i = 0; i < 8; i++) arr[1]->SetControlPointAt(mesh->GetControlPointAt(i), i);
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
	return arr;
}

char* concat(int n, ...)
{
	const char** strings = new const char*[n];
	int length = 0;
	va_list arguments;
	va_start(arguments, n);
	for(int i = 0; i < n; i++) {
		strings[i] = (const char*) va_arg(arguments, const char*);
		length += strlen(strings[i]);
	}
	char* dest = new char[length+1];
	dest[0] = '\0';
	for(int i = 0; i < n; i++) strcat(dest, strings[i]);
	dest[length] = '\0';
	return dest;
}

