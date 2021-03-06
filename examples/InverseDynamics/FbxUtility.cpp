#include "FbxUtility.h"
#include <fbxsdk.h>

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


	FbxAnimLayer* getAnimLayer(FbxScene *scene, int animStackIdx, int animLayerIdx)
	{
		if (NULL == scene)
		{
			return NULL;
		}

		FbxAnimLayer *animLayer = NULL;
		int numAnimStack = scene->GetSrcObjectCount<FbxAnimStack>();
		if (numAnimStack > 0 && animStackIdx < numAnimStack)
		{
			FbxAnimStack *animStack = scene->GetSrcObject<FbxAnimStack>(animStackIdx);
			int numAnimLayer = animStack->GetMemberCount<FbxAnimLayer>();
			if (numAnimLayer > 0 && animLayerIdx< numAnimLayer)
			{
				animLayer = animStack->GetMember<FbxAnimLayer>(animLayerIdx);
			}
		}
		return animLayer;
	}

	void getAnimData(FbxAnimCurve *animCurve, AnimationCurve<SkeletonNode::AnimValueType> &data)
	{
		if (NULL == animCurve)
		{
			return;
		}
		int count = animCurve->KeyGetCount();
		if (count <= 0)
		{
			return;
		}
		
		std::vector<AnimationKey<SkeletonNode::AnimValueType> > &dataKeys = data.keys;
		dataKeys.resize(count);
		for (int i = 0; i < count; i++)
		{
			FbxAnimCurveKey key = animCurve->KeyGet(i);
			AnimationKey<SkeletonNode::AnimValueType> &dstKey = dataKeys[i];
			dstKey.time = key.GetTime().GetMilliSeconds();
			dstKey.value = key.GetValue();
		}
	}

	void getAnimFromNode(FbxNode *node, SkeletonNode &skeletonNode)
	{
		FbxAnimLayer *animLayer = getAnimLayer(node->GetScene(), 0, 0);
		if (NULL == animLayer)
		{
			return;
		}
		getAnimData(node->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X), skeletonNode.animationT[0]);
		getAnimData(node->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y), skeletonNode.animationT[1]);
		getAnimData(node->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z), skeletonNode.animationT[2]);

		getAnimData(node->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X), skeletonNode.animationR[0]);
		getAnimData(node->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y), skeletonNode.animationR[1]);
		getAnimData(node->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z), skeletonNode.animationR[2]);

		getAnimData(node->LclScaling.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X), skeletonNode.animationS[0]);
		getAnimData(node->LclScaling.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y), skeletonNode.animationS[1]);
		getAnimData(node->LclScaling.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z), skeletonNode.animationS[2]);
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
		// todo WorldTransform = ParentWorldTransform * T * Roff * Rp * Rpre * R * Rpost -1 * Rp -1 * Soff * Sp * S * Sp -1
		// ȫ����һ�飿
		FbxDouble3 rotationPre = node->PreRotation.Get();
		for (int i = 0; i < 3; ++i)
		{
			skeletonNode.translation[i] = translation[i];
			skeletonNode.rotation[i] = rotation[i];
			skeletonNode.scale[i] = scale[i];

			skeletonNode.rotationPre[i] = rotationPre[i];
		}

		getAnimFromNode(node, skeletonNode);
		if (NULL != modifySkeletonNodes)
		{
			const SkeletonNode &targetNode = (*modifySkeletonNodes)[skeletonNode.idx];
			node->LclTranslation.Set(FbxDouble3(targetNode.translation[0], targetNode.translation[1], targetNode.translation[2]));
			node->LclRotation.Set(FbxDouble3(targetNode.rotation[0], targetNode.rotation[1], targetNode.rotation[2]));
			node->PreRotation.Set(FbxDouble3(targetNode.rotationPre[0], targetNode.rotationPre[1], targetNode.rotationPre[2]));
			node->UpdatePivotsAndLimitsFromProperties();
		}
	}

	void DFTSkeleton(FbxNode *node, int parentSkeletonNodeIdx, std::vector<SkeletonNode> &skeletonNodes, const std::vector<SkeletonNode> *modifySkeletonNodes)
	{
		FbxNodeAttribute *attributeNode = node->GetNodeAttribute();
		if (NULL == attributeNode || !(FbxNodeAttribute::eSkeleton == attributeNode->GetAttributeType() || std::string(node->GetName()) == std::string("Bip01")))
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
			else if (FbxNodeAttribute::eNull == attributeType)
			{
				if (std::string(childNode->GetName()) == std::string("Bip01"))
				{
					DFTSkeleton(childNode, -1, skeletonNodes, modifySkeletonNodes);
					// only process one now
					break;
				}
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




