#ifndef _CLASS_DECOMPOSEDSKELETON
#define _CLASS_DECOMPOSEDSKELETON

#include <vector>

namespace rtql8 {
namespace kinematics {

class Skeleton;
class BodyNode;

class DecomposedSkeleton {

public:
	typedef std::vector<BodyNode*> T_BodyNodePtr;
	typedef T_BodyNodePtr::const_iterator CIT_BodyNodePtr;

public:
	 DecomposedSkeleton(Skeleton* /*skeleton*/);
	~DecomposedSkeleton();

public:
    const Skeleton* skeleton_;
	const T_BodyNodePtr limbs_;

}; 
} // namespace kinematics
} //namespace rtql8

#endif //_CLASS_DECOMPOSEDSKELETON
