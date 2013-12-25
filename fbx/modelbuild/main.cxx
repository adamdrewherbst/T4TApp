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

#define OUTPUT_DIRECTORY "/home/aherbst/Documents/Programming/GamePlay/bin/linux/input/"
#define OUTPUT_FILENAME OUTPUT_DIRECTORY "models.fbx"

#define PI 3.1415926535

using std::cin; using std::cout; using std::endl;

// Function prototypes.
bool CreateScene(FbxManager* pSdkManager, FbxScene* pScene);

FbxNode* CreatePatch(FbxScene* pScene, const char* pName);
FbxNode* CreateCylinder(FbxScene* pScene, const char* pName, float radius, float height, int segments);

void StoreBindPose(FbxScene* pScene, FbxNode* pPatch);
void StoreRestPose(FbxScene* pScene, FbxNode* pSkeletonRoot);
void AddNodeRecursively(FbxArray<FbxNode*>& pNodeArray, FbxNode* pNode);

void SetXMatrix(FbxAMatrix& pXMatrix, const FbxMatrix& pMatrix);

int main(int argc, char** argv)
{
    FbxManager* lSdkManager = NULL;
    FbxScene* lScene = NULL;
    bool lResult;

    // Prepare the FBX SDK.
    InitializeSdkObjects(lSdkManager, lScene);

    // Create the scene.
    lResult = CreateScene(lSdkManager, lScene);

    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while creating the scene...\n");
        DestroySdkObjects(lSdkManager, lResult);
        return 0;
    }

    // Save the scene.

    // The example can take an output file name as an argument.
	const char* lSampleFileName = NULL;
	for( int i = 1; i < argc; ++i )
	{
		if( FBXSDK_stricmp(argv[i], "-test") == 0 ) continue;
		else if( !lSampleFileName ) lSampleFileName = argv[i];
	}
	if( !lSampleFileName ) lSampleFileName = OUTPUT_FILENAME;
	
	cout << "Saving to " << lSampleFileName << endl;

	lResult = SaveScene(lSdkManager, lScene, lSampleFileName);
    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while saving the scene...\n");
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
    sceneInfo->mTitle = "Models";
    sceneInfo->mSubject = "Models for Gameplay3D T4T App";
    sceneInfo->mAuthor = "modelbuild program.";
    sceneInfo->mRevision = "rev. 1.0";
    sceneInfo->mKeywords = "models";
    sceneInfo->mComment = "no particular comments required.";

    // we need to add the sceneInfo before calling AddThumbNailToScene because
    // that function is asking the scene for the sceneInfo.
    pScene->SetSceneInfo(sceneInfo);

    // Build the node tree.
    FbxNode* lPatch = CreateCylinder(pScene, "cylinder", 1.0, 3.0, 20);
    //FbxNode* lPatch = CreatePatch(pScene, "cylinder");
    FbxNode* lRootNode = pScene->GetRootNode();
    lRootNode->AddChild(lPatch);

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
	lRootNode->AddChild(lLightNode);

	// Store poses
    //StoreBindPose(pScene, lPatch);

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
	FbxVector4 normal;
	double *norm;
	cout << "normals: " << endl;
	for(int i = 0; i < mesh->GetPolygonCount(); i++) {
		bool b = mesh->GetPolygonVertexNormal(i, 0, normal);
		norm = normal;
		cout << i << ": " << norm[0] << "," << norm[1] << "," << norm[2] << endl;
	}
	
    FbxNode* lNode = FbxNode::Create(pScene,pName);

    // Rotate the cylinder along the X axis so the axis
    // of the cylinder is the same as the bone axis (Y axis)
    FbxVector4 lR(0.0, 0.0, 0.0);
    lNode->LclRotation.Set(lR);
    lNode->SetNodeAttribute(mesh);
    return lNode;
}

// Create a cylinder centered on the Z axis. 
FbxNode* CreatePatch(FbxScene* pScene, const char* pName)
{
    FbxPatch* lPatch = FbxPatch::Create(pScene,pName);

    // Set patch properties.
    lPatch->InitControlPoints(4, FbxPatch::eBSpline, 7, FbxPatch::eBSpline);
    lPatch->SetStep(4, 4);
    lPatch->SetClosed(true, false);

    FbxVector4* lVector4 = lPatch->GetControlPoints();
    int i;

    int nSegmentsY = 7, nSegmentsR = 16;
    double lRadius = 1.0;
    double lSegmentLength = 0.5, length = lSegmentLength * nSegmentsY;
    for (i = 0; i < nSegmentsY; i++) 
    {
        lVector4[4*i + 0].Set(lRadius, 0.0, (i-nSegmentsY/2)*lSegmentLength);
        lVector4[4*i + 1].Set(0.0, -lRadius, (i-nSegmentsY/2)*lSegmentLength);
        lVector4[4*i + 2].Set(-lRadius, 0.0, (i-nSegmentsY/2)*lSegmentLength);
        lVector4[4*i + 3].Set(0.0, lRadius, (i-nSegmentsY/2)*lSegmentLength);
    }
    
    FbxMesh *caps[2];
    for(int n = 0; n < 2; n++) {
    	caps[n] = FbxMesh::Create(pScene,pName);
    	caps[n]->InitControlPoints(nSegmentsR + 1);
		for(i = 0; i < nSegmentsR; i++) {
			caps[n]->SetControlPointAt(FbxVector4(lRadius * cos(2*PI/i), (2*n-1) * length/2, lRadius * sin(2*PI/i)), i);
		}
		caps[n]->SetControlPointAt(FbxVector4(0.0, (2*n-1) * length/2, 0.0), nSegmentsR);
		
		for(i = 0; i < nSegmentsR; i++) {
			caps[n]->BeginPolygon();
			caps[n]->AddPolygon(i);
			caps[n]->AddPolygon((i+1) % nSegmentsR);
			caps[n]->AddPolygon(nSegmentsR);
			caps[n]->EndPolygon();
		}
	}
	
    FbxNode* lNode = FbxNode::Create(pScene,pName);

    // Rotate the cylinder along the X axis so the axis
    // of the cylinder is the same as the bone axis (Y axis)
    FbxVector4 lR(0.0, 0.0, 0.0);
    lNode->LclRotation.Set(lR);
    lNode->SetNodeAttribute(lPatch);
    //lNode->AddNodeAttribute(caps[0]);
    //lNode->AddNodeAttribute(caps[1]);

    return lNode;
}

// Store the Bind Pose
void StoreBindPose(FbxScene* pScene, FbxNode* pPatch)
{
    // In the bind pose, we must store all the link's global matrix at the time of the bind.
    // Plus, we must store all the parent(s) global matrix of a link, even if they are not
    // themselves deforming any model.

    // In this example, since there is only one model deformed, we don't need walk through 
    // the scene
    //

    // Now list the all the link involve in the patch deformation
    FbxArray<FbxNode*> lClusteredFbxNodes;
    int                       i, j;

    if (pPatch && pPatch->GetNodeAttribute())
    {
        int lSkinCount=0;
        int lClusterCount=0;
        switch (pPatch->GetNodeAttribute()->GetAttributeType())
        {
	    default:
	        break;
        case FbxNodeAttribute::eMesh:
        case FbxNodeAttribute::eNurbs:
        case FbxNodeAttribute::ePatch:

            lSkinCount = ((FbxGeometry*)pPatch->GetNodeAttribute())->GetDeformerCount(FbxDeformer::eSkin);
            //Go through all the skins and count them
            //then go through each skin and get their cluster count
            for(i=0; i<lSkinCount; ++i)
            {
                FbxSkin *lSkin=(FbxSkin*)((FbxGeometry*)pPatch->GetNodeAttribute())->GetDeformer(i, FbxDeformer::eSkin);
                lClusterCount+=lSkin->GetClusterCount();
            }
            break;
        }
        //if we found some clusters we must add the node
        if (lClusterCount)
        {
            //Again, go through all the skins get each cluster link and add them
            for (i=0; i<lSkinCount; ++i)
            {
                FbxSkin *lSkin=(FbxSkin*)((FbxGeometry*)pPatch->GetNodeAttribute())->GetDeformer(i, FbxDeformer::eSkin);
                lClusterCount=lSkin->GetClusterCount();
                for (j=0; j<lClusterCount; ++j)
                {
                    FbxNode* lClusterNode = lSkin->GetCluster(j)->GetLink();
                    AddNodeRecursively(lClusteredFbxNodes, lClusterNode);
                }

            }

            // Add the patch to the pose
            lClusteredFbxNodes.Add(pPatch);
        }
    }

    // Now create a bind pose with the link list
    if (lClusteredFbxNodes.GetCount())
    {
        // A pose must be named. Arbitrarily use the name of the patch node.
        FbxPose* lPose = FbxPose::Create(pScene,pPatch->GetName());

        // default pose type is rest pose, so we need to set the type as bind pose
        lPose->SetIsBindPose(true);

        for (i=0; i<lClusteredFbxNodes.GetCount(); i++)
        {
            FbxNode*  lKFbxNode   = lClusteredFbxNodes.GetAt(i);
            FbxMatrix lBindMatrix = lKFbxNode->EvaluateGlobalTransform();

            lPose->Add(lKFbxNode, lBindMatrix);
        }

        // Add the pose to the scene
        pScene->AddPose(lPose);
    }
}

// Store a Rest Pose
void StoreRestPose(FbxScene* pScene, FbxNode* pSkeletonRoot)
{
    // This example show an arbitrary rest pose assignment.
    // This rest pose will set the bone rotation to the same value 
    // as time 1 second in the first stack of animation, but the 
    // position of the bone will be set elsewhere in the scene.
    FbxString     lNodeName;
    FbxNode*   lKFbxNode;
    FbxMatrix  lTransformMatrix;
    FbxVector4 lT,lR,lS(1.0, 1.0, 1.0);

    // Create the rest pose
    FbxPose* lPose = FbxPose::Create(pScene,"A Bind Pose");

    // Set the skeleton root node to the global position (10, 10, 10)
    // and global rotation of 45deg along the Z axis.
    lT.Set(10.0, 10.0, 10.0);
    lR.Set( 0.0,  0.0, 45.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton root node to the pose
    lKFbxNode = pSkeletonRoot;
    lPose->Add(lKFbxNode, lTransformMatrix, false /*it's a global matrix*/);

    // Set the lLimbNode1 node to the local position of (0, 40, 0)
    // and local rotation of -90deg along the Z axis. This show that
    // you can mix local and global coordinates in a rest pose.
    lT.Set(0.0, 40.0,   0.0);
    lR.Set(0.0,  0.0, -90.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode->GetChild(0);
    lPose->Add(lKFbxNode, lTransformMatrix, true /*it's a local matrix*/);

    // Set the lLimbNode2 node to the local position of (0, 40, 0)
    // and local rotation of 45deg along the Z axis.
    lT.Set(0.0, 40.0, 0.0);
    lR.Set(0.0,  0.0, 45.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode->GetChild(0);
    lNodeName = lKFbxNode->GetName();
    lPose->Add(lKFbxNode, lTransformMatrix, true /*it's a local matrix*/);

    // Now add the pose to the scene
    pScene->AddPose(lPose);
}

// Add the specified node to the node array. Also, add recursively
// all the parent node of the specified node to the array.
void AddNodeRecursively(FbxArray<FbxNode*>& pNodeArray, FbxNode* pNode)
{
    if (pNode)
    {
        AddNodeRecursively(pNodeArray, pNode->GetParent());

        if (pNodeArray.Find(pNode) == -1)
        {
            // Node not in the list, add it
            pNodeArray.Add(pNode);
        }
    }
}

void SetXMatrix(FbxAMatrix& pXMatrix, const FbxMatrix& pMatrix)
{
    memcpy((double*)pXMatrix, &pMatrix.mData[0][0], sizeof(pMatrix.mData));
}
