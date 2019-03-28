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

		FbxIOSettings *ioSettings = sdkManager->GetIOSettings();
		FbxImporter *importer = FbxImporter::Create(sdkManager, "");
		if (!importer->Initialize(fbxFilePath, -1, ioSettings))
		{
			importer->Destroy();
			return false;
		}

		bool result = importer->Import(scene);

		importer->Destroy();

		return result;
	}

	bool exportSceneToFile(const char *fbxFilePath, FbxManager *sdkManager, FbxScene *scene)
	{
		FbxExporter *exporter = FbxExporter::Create(sdkManager, "");
		if (!exporter->Initialize(fbxFilePath, 0, sdkManager->GetIOSettings()))
		{
			return false;
		}
		FbxIOSettings *ioSettings = sdkManager->GetIOSettings();
		ioSettings->SetBoolProp(EXP_FBX_ANIMATION, false);
		bool res = exporter->Export(scene);
		exporter->Destroy();
		return res;
	}

	void visitSkeletonNode(FbxNode *node, int parentSkeletonNodeIdx, std::vector<SkeletonNode> &skeletonNodes, const std::vector<SkeletonNode> *modifySkeletonNodes)
	{
		FbxString name = node->GetName();

		skeletonNodes.push_back(SkeletonNode());
		SkeletonNode &skeletonNode = skeletonNodes.back();
		skeletonNode.idx = (int)(skeletonNodes.size() - 1);
		skeletonNode.parentIdx = parentSkeletonNodeIdx;

		skeletonNode.name = node->GetName();

		FbxDouble3 translation = node->LclTranslation.Get();
		FbxDouble3 rotation = node->LclRotation.Get();
		FbxDouble3 scale = node->LclScaling.Get();
		for (int i = 0; i < 3; ++i)
		{
			skeletonNode.translation[i] = translation[i];
			skeletonNode.rotation[i] = rotation[i];
			skeletonNode.scale[i] = scale[i];
		}
		if (NULL != modifySkeletonNodes)
		{
			const SkeletonNode &targetNode = (*modifySkeletonNodes)[skeletonNode.idx];
			// node->LclTranslation.Set(FbxDouble3(targetNode.translation[0], targetNode.translation[1], targetNode.translation[2]));
			// node->LclRotation.Set(FbxDouble3(targetNode.rotation[0], targetNode.rotation[1], targetNode.rotation[2]));

			if (name == "Bip01 L Thigh")
			{
				node->LclTranslation.Set(FbxDouble3(skeletonNode.translation[0], skeletonNode.translation[1], skeletonNode.translation[2] / 2));
				node->LclRotation.Set(FbxDouble3(0, 0, 0));
			}
		}
	}

	void DFTSkeleton(FbxNode *node, int parentSkeletonNodeIdx, std::vector<SkeletonNode> &skeletonNodes, const std::vector<SkeletonNode> *modifySkeletonNodes)
	{
		FbxNodeAttribute *attributeNode = node->GetNodeAttribute();
		if (NULL == attributeNode || FbxNodeAttribute::eSkeleton != attributeNode->GetAttributeType())
		{
			return;
		}

		visitSkeletonNode(node, parentSkeletonNodeIdx, skeletonNodes, modifySkeletonNodes);
		int nodeIdx = skeletonNodes.back().idx;

		int childCount = node->GetChildCount();
		for (int i = 0; i < childCount; i++)
		{
			DFTSkeleton(node->GetChild(i), nodeIdx, skeletonNodes, modifySkeletonNodes);
		}
	}

	bool processSceneData(FbxScene *scene, std::vector<SkeletonNode> &skeletonNodes, const std::vector<SkeletonNode> *modifySkeletonNodes)
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
			if (FbxNodeAttribute::eSkeleton == attributeType)
			{
				DFTSkeleton(childNode, -1, skeletonNodes, modifySkeletonNodes);
				// only process one now
				break;
			}
			else
			{
				if (NULL != modifySkeletonNodes && FbxNodeAttribute::eMesh == attributeType)
				{
					scene->RemoveNode(childNode);
				}
			}
		}

		return true;
	}

	bool loadFbxFile(const char *fbxFilePath, std::vector<SkeletonNode> &skeletonNodes)
	{
		FbxManager *sdkManager = FbxManager::Create();

		FbxIOSettings * ioSettings = FbxIOSettings::Create(sdkManager, IOSROOT);
		sdkManager->SetIOSettings(ioSettings);

		FbxScene *scene = FbxScene::Create(sdkManager, "");

		bool result = false;
		if (importFileToScene(fbxFilePath, sdkManager, scene))
		{
			if (processSceneData(scene, skeletonNodes, NULL))
			{
				result = true;
			}
		}

		scene->Destroy();
		sdkManager->Destroy();

		return result;
	}

	bool transFbxFile(const char *fbxFilePath, const char *dstFbxFilePath, const std::vector<SkeletonNode> &skeletonNodes)
	{
		FbxManager *sdkManager = FbxManager::Create();

		FbxIOSettings * ioSettings = FbxIOSettings::Create(sdkManager, IOSROOT);
		sdkManager->SetIOSettings(ioSettings);

		FbxScene *scene = FbxScene::Create(sdkManager, "");

		bool result = false;
		if (importFileToScene(fbxFilePath, sdkManager, scene))
		{
			std::vector<SkeletonNode> tempNodes;
			if (processSceneData(scene, tempNodes, &skeletonNodes))
			{
				if (exportSceneToFile(dstFbxFilePath, sdkManager, scene))
				{
					result = true;
				}
			}
		}

		scene->Destroy();
		sdkManager->Destroy();

		return result;
	}
};




