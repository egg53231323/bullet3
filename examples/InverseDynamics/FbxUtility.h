#ifndef FBX_UTILITY_H
#define FBX_UTILITY_H

#include <vector>

class SkeletonNode
{
public:
	SkeletonNode();
	int idx;
	int parentIdx;
	double translation[3];
	double rotation[3];
	double scale[3];
};

namespace FbxUtility {
	bool loadFbxFile(const char *fbxFilePath, std::vector<SkeletonNode> &skeletonNodes);
};

#endif  //FBX_UTILITY_H
