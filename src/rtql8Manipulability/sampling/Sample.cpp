
#include "rtql8Manipulability/sampling/Sample.h"
#include "rtql8/kinematics/BodyNode.h"
#include "rtql8/kinematics/Joint.h"
#include "rtql8/kinematics/Dof.h"

#include "utils/MatrixDefs.h"
#include "utils/MatrixDefsInternal.h"

using namespace rtql8::kinematics;
using namespace Eigen;

#include <iostream>

namespace
{
	Eigen::Vector3d ComputeEffectorPositioon(BodyNode* node)
	{
		Matrix4d transform = Matrix4d::Identity();
		// get effector
		BodyNode* effector = node;
		while(effector->getNumChildJoints() == 1)
		{
			effector = effector->getChildNode(0);
		}
		Vector3 zero(0,0,0);
		return effector->evalWorldPos(zero) - node->evalWorldPos(zero) ;
	}

	void CollectAnglesRec(BodyNode* node, Sample::T_Angles& angles)
	{
		for(int i=0; i<node->getNumLocalDofs(); ++i)
		{
			if(node->getDof(i)->getJoint()->getJointType() != Joint::J_TRANS)
			{
				angles.push_back(node->getDof(i)->getValue());
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
			CollectAnglesRec(node->getChildNode(0), angles);
		}
	}

	Sample::T_Angles CollectAngles(BodyNode* limbRoot)
	{
		Sample::T_Angles angles;
		CollectAnglesRec(limbRoot, angles);
		return angles;
	}
}

Sample::Sample(BodyNode* limbRoot)
	: angles_(CollectAngles(limbRoot))
	, position_(ComputeEffectorPositioon(limbRoot))
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

void Sample::LoadIntoLimb(BodyNode* node, bool updateTransform) const
{
	CIT_Angles cit = angles_.begin();
	for(int i=0; i<node->getNumLocalDofs() && cit != angles_.end(); ++i)
	{
		if(node->getDof(i)->getJoint()->getJointType() != Joint::J_TRANS)
		{
			node->getDof(i)->setValue(*cit);
			++cit;
			if(updateTransform)
			{
				node->updateTransform();
			}
		}
		else
		{
			std::cout << "unconsidered joint " << node->getDof(i)->getJoint()->getJointType();
		}
	}
}

double Sample::ForceManipulability (const Vector3d& direction) const
{
	double r = (direction.transpose()*jacobianProd_*direction);
	return 1/sqrt(r);
}
