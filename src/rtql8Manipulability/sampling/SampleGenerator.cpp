
#include "rtql8Manipulability/sampling/SampleGenerator.h"
#include "rtql8Manipulability/sampling/SampleGeneratorVisitor_ABC.h"
#include "rtql8Manipulability/sampling/Sample.h"
#include "rtql8Manipulability/DecomposedSkeleton.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Joint.h"
#include "kinematics/Dof.h"

#include "utils/MatrixDefs.h"
#include "utils/MatrixDefsInternal.h"

#include <vector>
#include <map>
#include <time.h>

using namespace rtql8::kinematics;
using namespace Eigen;


namespace tree
{
	typedef Vector3 Vector3f;
	typedef size_t EntityId;
	typedef std::vector<EntityId> T_Id;
	typedef T_Id::const_iterator CIT_Id;

	const Vector3f minVals(-10.,-10.,-10.) ;
	const Vector3f maxVals(10.,10.,10.);
	const int pointsPerNodes = 10;

	const double distanceExtrusion = 0.02;

	struct Triangle3Df
	{
		Triangle3Df(const tree::Vector3f& a, const tree::Vector3f& b, const tree::Vector3f& c, const double d)
			: a_(a), b_(b), c_(c), d_(d) {}

		~Triangle3Df(){}

		const tree::Vector3f a_;
		const tree::Vector3f b_;
		const tree::Vector3f c_;
		const double d_;
	};

	struct RegularTree
	{
		RegularTree(const Vector3f& minVals, const Vector3f& maxVals, const int pointsPerNodes) {}
		~RegularTree(){}

		size_t insert(const Vector3f& point)
		{
			points_.push_back(point);
			return points_.size() -1;
		}

		/*T_Id select(const Obstacle& obstacle)
		{
			T_Id res; int id = 0;
			Vector3 foo_;
			for (std::vector<Vector3f>::const_iterator it = points_.begin(); it != points_.end(); ++ it, ++id)
			{
				if(obstacle.Distance(*it,foo_) < distanceExtrusion)
				{
					res.push_back(id);
				}
			}
			return res;
		}*/

	private:
		std::vector<Vector3f> points_;
	};

	typedef RegularTree RTree3f;

}


namespace rtql8
{
namespace kinematics
{
struct PImpl
{	
	typedef std::map<std::string, tree::RTree3f*> T_Trees;
	typedef std::map<tree::EntityId, Sample> T_IdMatches;
	typedef std::map<std::string, T_IdMatches> LLSamples;
	typedef T_IdMatches::iterator T_IdMatches_IT;
	typedef T_IdMatches::const_iterator T_IdMatches_CIT;

	PImpl()
	{
		// NOTHING
	}

	~PImpl()
	{
		for(T_Trees::iterator it = trees_.begin(); it!= trees_.end(); ++it)
		{
			delete (it->second);
		}
	}
	
	void GenerateSample(T_IdMatches& samples, tree::RTree3f * rtree, BodyNode* limb)
	{
		Sample sample(limb);
		tree::EntityId id = rtree->insert(sample.position_);
		T_IdMatches_IT it = samples.find(id);
		if(it == samples.end())
		{
			samples.insert(std::make_pair(id, sample));
		}
	}
	LLSamples allSamples_;
	T_Trees trees_;
};
} // namespace kinematics
} //namespace rtql8

SampleGenerator *SampleGenerator::instance = 0;

SampleGenerator::SampleGenerator()
	: pImpl_(new PImpl)
{
	srand((unsigned int)(time(0))); //Init Random generation
}

SampleGenerator::~SampleGenerator()
{
	// NOTHING
}

void SampleGenerator::GenerateSamples(BodyNode* limb, int nbSamples)
{
	assert(nbSamples > 0);
	const std::string name = limb->getName();
	if(pImpl_->allSamples_.find(name) == pImpl_->allSamples_.end())
	{	
		PImpl::T_IdMatches samples;
		pImpl_->allSamples_.insert(std::make_pair(name,samples));
		tree::RTree3f * rTree = new tree::RTree3f(tree::minVals, tree::maxVals, tree::pointsPerNodes);
		pImpl_->trees_.insert(std::make_pair(name,rTree));
		for(int i = 0; i < nbSamples; ++i)
		{
			pImpl_->GenerateSample(pImpl_->allSamples_[name], rTree, limb);
		}
	}
}

void SampleGenerator::GenerateSamples(const DecomposedSkeleton& character, int nbSamples)
{
	for(DecomposedSkeleton::CIT_BodyNodePtr cit = character.limbs_.begin(); cit!=character.limbs_.end(); ++cit)
	{
		GenerateSamples(*cit, nbSamples);
	}
}

void SampleGenerator::Request(BodyNode* limb, SampleGeneratorVisitor_ABC* visitor) const
{
	for(PImpl::T_IdMatches_IT it = pImpl_->allSamples_[limb->getName()].begin(); it != pImpl_->allSamples_[limb->getName()].end(); ++it)
	{
			visitor->Visit(limb, it->second);
	}
}



