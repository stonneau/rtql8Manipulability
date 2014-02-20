#include "kinematics\Skeleton.h"
#include "kinematics\BodyNode.h"
#include "rtql8Manipulability\DecomposedSkeleton.h"

using namespace rtql8::kinematics;

namespace
{
	void DecomposeRec(BodyNode* node, DecomposedSkeleton::T_BodyNodePtr& limbs)
	{
		int nbChilds = node->getNumChildJoints();
		if(nbChilds < 2)
		{
			limbs.push_back(node);
		}
		else
		{
			for(int i=0; i<nbChilds; ++i)
			{
				DecomposeRec(node->getChildNode(i), limbs);
			}
		}
	}

	DecomposedSkeleton::T_BodyNodePtr Decompose(Skeleton* skeleton)
	{
		DecomposedSkeleton::T_BodyNodePtr limbs;
		DecomposeRec(skeleton->getRoot(), limbs);
		return limbs;
	}
}

DecomposedSkeleton::DecomposedSkeleton(Skeleton* skeleton)
	: skeleton_(skeleton)
	, limbs_(Decompose(skeleton))
{
	// NOTHING
}

DecomposedSkeleton::~DecomposedSkeleton()
{
	// NOTHING
}
