
#ifndef _CLASS_SAMPLEGENERATOR
#define _CLASS_SAMPLEGENERATOR

#include <memory>

namespace manip_core
{
class Obstacle;
}

namespace rtql8 {
namespace kinematics {
class DecomposedSkeleton;
class Skeleton;
class BodyNode;
class SampleGeneratorVisitor_ABC;
struct PImpl;

class SampleGenerator {

private:
	 SampleGenerator();
	~SampleGenerator();

	SampleGenerator(const SampleGenerator&); //Pas définie pour ne pas avoir a gérer les copies d'auto_ptr
	SampleGenerator& operator = (const SampleGenerator&); //idem

public:
	void GenerateSamples(const DecomposedSkeleton& /*character*/, int nbSamples = 1);
	void GenerateSamples(BodyNode* /*limb*/, int nbSamples = 1);
    void Request(BodyNode* /*limb*/, SampleGeneratorVisitor_ABC* /*visitor*/) const; // goes through all samples
    void RequestInContact(BodyNode* /*limb*/, SampleGeneratorVisitor_ABC* /*visitor*/) const; // goes through in contact samples

public:
    void AddObstacle(const manip_core::Obstacle* /*obstacle*/);

private:
    std::auto_ptr<PImpl> pImpl_;

private:
    static SampleGenerator* instance;

public:
	static SampleGenerator* GetInstance()
	{
		if(!instance)
		{
			instance = new SampleGenerator();
		}
		return instance;
	}
};
} // namespace kinematics
} //namespace rtql8
#endif //_CLASS_SAMPLEGENERATOR
