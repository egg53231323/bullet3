#ifndef FBX_UTILITY_H
#define FBX_UTILITY_H

#include <vector>
#include "SkeletonNode.h"

namespace FbxUtility {
	bool loadFbxFile(const char *fbxFilePath, std::vector<SkeletonNode> &skeletonNodes);
	bool transFbxFile(const char *fbxFilePath, const char *dstFbxFilePath, const std::vector<SkeletonNode> &skeletonNodes);
};

#endif  //FBX_UTILITY_H
