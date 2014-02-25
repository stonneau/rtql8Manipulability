
#ifndef _CLASS_SAMPLEGENERATORVISITOR_ABC
#define _CLASS_SAMPLEGENERATORVISITOR_ABC


namespace rtql8 {
namespace kinematics {

class Sample;
class DecomposedSkeleton;
class BodyNode;

class SampleGeneratorVisitor_ABC {

public:
	 SampleGeneratorVisitor_ABC();
	~SampleGeneratorVisitor_ABC();

public:
	virtual void Visit(BodyNode* /*limb*/, Sample& /*sample*/);	
private:
};
} // namespace kinematics
} //namespace rtql8
#endif //_CLASS_SAMPLEGENERATORVISITOR_ABC
