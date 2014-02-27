
#include "rtql8Manipulability/sampling/Sample.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Joint.h"
#include "kinematics/Dof.h"

#include "utils/MatrixDefs.h"
#include "utils/MatrixDefsInternal.h"

using namespace rtql8::kinematics;
using namespace Eigen;

#include <iostream>

#include "kinematics/Skeleton.h"
namespace
{
    Eigen::Vector3d ComputeEffectorPosition(BodyNode* node)
    {
        // get effector
        BodyNode* effector = node;
		while(effector->getNumChildJoints() == 1)
        {
			effector = effector->getChildNode(0);
		}
        Vector3 zero(0,0,0);
        return Eigen::matrix4TimesVect3(node->getWorldInvTransform(), effector->evalWorldPos(zero));
	}

	void CollectAnglesRec(BodyNode* node, Sample::T_Angles& angles, bool randomize)
	{
        for(int i=0; i<node->getNumLocalDofs(); ++i)
		{
			if(node->getDof(i)->getJoint()->getJointType() != Joint::J_TRANS)
			{
                if(randomize)
                {
                    double minTheta, maxTheta;
                    minTheta = node->getDof(i)->getMin();
                    maxTheta = node->getDof(i)->getMax();
                    double value = (maxTheta - minTheta) * ((double)rand() / (double)RAND_MAX) + minTheta;
                    angles.push_back(value);
                    node->getDof(i)->setValue(value);
                    node->updateTransform();
				}
				else
                {
                    angles.push_back(node->getDof(i)->getValue());
				}
			}
			else
			{
				std::cout << "unconsidered joint " << node->getDof(i)->getJoint()->getJointType();
			}
		}
		int nbChilds = node->getNumChildJoints();
		assert(nbChilds <2); // if this is false this means we don't have a simple limb
		if(nbChilds>0)
		{
			CollectAnglesRec(node->getChildNode(0), angles, randomize);
		}
	}

	Sample::T_Angles CollectAngles(BodyNode* limbRoot, bool randomize)
	{
		Sample::T_Angles angles;
		CollectAnglesRec(limbRoot, angles, randomize);
		return angles;
	}
}

Sample::Sample(BodyNode* limbRoot, bool randomize)
	: angles_(CollectAngles(limbRoot, randomize))
    , position_(ComputeEffectorPosition(limbRoot))
{
	// Jacobian computation
	MatrixXd jacobian = limbRoot->getJacobianLinear();
	jacobianProd_ = jacobian * jacobian.transpose();
	jacobianProdInverse_ = jacobian * jacobian.transpose();
	Eigen::JacobiSVD<Matrix3d> svd = Eigen::JacobiSVD<Matrix3d>(jacobianProd_, Eigen::ComputeFullU | Eigen::ComputeFullV);
	PseudoInverseSVDDLS<>(jacobianProd_, svd, jacobianProdInverse_);
}

Sample::~Sample()
{
	// NOTHING
}


namespace
{
void LoadIntoLimbRec(BodyNode* node, bool updateTransform, Sample::CIT_Angles& cit, Sample::CIT_Angles& end)
{
    for(int i=0; i<node->getNumLocalDofs(); ++i)
    {
        if(node->getDof(i)->getJoint()->getJointType() != Joint::J_TRANS)
        {
            node->getDof(i)->setValue(*cit);
            ++cit;
            if(updateTransform)
                node->updateTransform();
        }
    }
    int nbChilds = node->getNumChildJoints();
    assert(nbChilds <2); // if this is false this means we don't have a simple limb
    if(nbChilds>0)
    {
        LoadIntoLimbRec(node->getChildNode(0),updateTransform, cit, end);
    }
}
}

void Sample::LoadIntoLimb(BodyNode* node, bool updateTransform) const
{
	CIT_Angles cit = angles_.begin();
    CIT_Angles end = angles_.end();
    LoadIntoLimbRec(node, updateTransform, cit, end);
}

double Sample::ForceManipulability (const Vector3d& direction) const
{
	double r = (direction.transpose()*jacobianProd_*direction);
	return 1/sqrt(r);
}
