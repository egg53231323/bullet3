#include "FbxUtility.h"
#include <fbxsdk.h>

SkeletonNode::SkeletonNode() : idx(0), parentIdx(0)
{
	translation[0] = translation[1] = translation[2] = 0;
	rotation[0] = rotation[1] = rotation[2] = 0;
	scale[0] = scale[1] = scale[2] = 1;
}

namespace FbxUtility {

	bool importFileToScene(const char *fbxFilePath, FbxManager *sdkManager, FbxScene *scene)
	{
		if (NULL == fbxFilePath || NULL == sdkManager || NULL == scene)
		{
			return false;
		}

		FbxIOSettings * ioSettings = sdkManager->GetIOSettings();
		FbxImporter* importer = FbxImporter::Create(sdkManager, "");
		if (!importer->Initialize(fbxFilePath, -1, ioSettings))
		{
			importer->Destroy();
			return false;
		}

		bool result = importer->Import(scene);

		importer->Destroy();

		return result;
	}

	void visitSkeletonNode(FbxNode *node, int parentSkeletonNodeIdx, std::vector<SkeletonNode> &skeletonNodes)
	{
		FbxString name = node->GetName();

		skeletonNodes.push_back(SkeletonNode());
		SkeletonNode &skeletonNode = skeletonNodes.back();
		skeletonNode.idx = (int)(skeletonNodes.size() - 1);
		skeletonNode.parentIdx = parentSkeletonNodeIdx;
		FbxDouble3 translation = node->LclTranslation.Get();
		FbxDouble3 rotation = node->LclRotation.Get();
		FbxDouble3 scale = node->LclScaling.Get();
		for (int i = 0; i < 3; ++i)
		{
			skeletonNode.translation[i] = translation[i];
			skeletonNode.rotation[i] = rotation[i];
			skeletonNode.scale[i] = scale[i];
		}
	}

	void DFTSkeleton(FbxNode *node, int parentSkeletonNodeIdx, std::vector<SkeletonNode> &skeletonNodes)
	{
		FbxNodeAttribute *attributeNode = node->GetNodeAttribute();
		if (NULL == attributeNode || FbxNodeAttribute::eSkeleton != attributeNode->GetAttributeType())
		{
			return;
		}

		visitSkeletonNode(node, parentSkeletonNodeIdx, skeletonNodes);
		int nodeIdx = skeletonNodes.back().idx;

		int childCount = node->GetChildCount();
		for (int i = 0; i < childCount; i++)
		{
			DFTSkeleton(node->GetChild(i), nodeIdx, skeletonNodes);
		}
	}

	bool loadSceneData(FbxScene *scene, std::vector<SkeletonNode> &skeletonNodes)
	{
		if (NULL == scene)
		{
			return false;
		}

		FbxNode *rootNode = scene->GetRootNode();
		int childCount = rootNode->GetChildCount();
		for (int i = 0; i < childCount; i++)
		{
			FbxNode *childNode = rootNode->GetChild(i);
			FbxNodeAttribute *attributeNode = childNode->GetNodeAttribute();
			if (NULL == attributeNode)
			{
				continue;
			}
			FbxNodeAttribute::EType attributeType = attributeNode->GetAttributeType();
			if (FbxNodeAttribute::eSkeleton == attributeNode->GetAttributeType())
			{
				DFTSkeleton(childNode, -1, skeletonNodes);
				// only process one now
				break;
			}
		}

		return true;
	}

	bool FbxUtility::loadFbxFile(const char *fbxFilePath, std::vector<SkeletonNode> &skeletonNodes)
	{
		FbxManager *sdkManager = FbxManager::Create();

		FbxIOSettings * ioSettings = FbxIOSettings::Create(sdkManager, IOSROOT);
		sdkManager->SetIOSettings(ioSettings);

		FbxScene *scene = FbxScene::Create(sdkManager, "");

		bool result = false;
		if (importFileToScene(fbxFilePath, sdkManager, scene))
		{
			if (loadSceneData(scene, skeletonNodes))
			{
				result = true;
			}
		}

		scene->Destroy();
		sdkManager->Destroy();

		return result;
	}
};




