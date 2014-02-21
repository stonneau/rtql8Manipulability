
#ifndef _MATRIXDEFSINTERNAL
#define _MATRIXDEFSINTERNAL

#include "MatrixDefs.h"

#include <Eigen/Dense>

namespace Eigen
{
typedef MatrixXd MatrixX;
typedef VectorXd VectorX;

//REF: boulic et al An inverse kinematics architecture enforcing an arbitrary Numeric of strict priority levels
template<typename _Matrix_Type_>
void PseudoInverseDLS(_Matrix_Type_& pinvmat,  Numeric lambda)
{
	Eigen::JacobiSVD<_Matrix_Type_> svd(pinvmat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	VectorX m_sigma = svd.singularValues();

// temp computation foireuse pour lambda
// REF: must apply Numerical filtering for the operation of robotic manipulators through kinematically singular ...
	bool found = false; int i = m_sigma.rows() -1;
	Numeric val = 0;
	while (!found && i >= 0)
	{
		val = m_sigma(i);
		found = m_sigma(i) > 0;
		if (found) lambda = val  / 1.f;
		i--;
	}
//end tmp

	Numeric  pinvtoler= Numeric(lambda != 0 ? 0 : 1.e-6); // choose your tolerance widely!
	Numeric lambda2 = lambda * lambda;

	MatrixX m_sigma_inv = MatrixX::Zero(pinvmat.cols(),pinvmat.rows());
	for (int i=0; i<m_sigma.rows(); ++i)
	{
		if (m_sigma(i) > pinvtoler)
			m_sigma_inv(i,i)=m_sigma(i)/(m_sigma(i) * m_sigma(i) + lambda2);
	}
	pinvmat = (svd.matrixV()*m_sigma_inv*svd.matrixU().transpose());
}

template<typename _Matrix_Type_>
void PseudoInverseSVDDLS(_Matrix_Type_& pinvmat, Eigen::JacobiSVD<_Matrix_Type_>& svd, _Matrix_Type_& dest, Numeric lambda = 0.f)
{
	VectorX m_sigma = svd.singularValues();
		
	// temp computation foireuse pour lambda
	// REF: must apply Numerical filtering for the operation of robotic manipulators through kinematically singular ...
	bool found = false; int i = m_sigma.rows() -1;
	Numeric val = 0;
	while (!found && i >= 0)
	{
		val = m_sigma(i);
		found = m_sigma(i) > 0;
		if (found) lambda = val / 1.f;
		i--;
	}
	//end tmp
	Numeric  pinvtoler = Numeric(lambda != 0 ? 0 : 1.e-6); // choose your tolerance widely!
	Numeric lambda2 = lambda * lambda;
		
	MatrixX m_sigma_inv = MatrixX::Zero(pinvmat.cols(),pinvmat.rows());
	for (int i=0; i<m_sigma.rows(); ++i)
	{
		if (m_sigma(i) > pinvtoler)
			m_sigma_inv(i,i)=m_sigma(i)/(m_sigma(i) * m_sigma(i) + lambda2);
			//m_sigma_inv(i,i)=1.0/m_sigma(i);
	}
	dest= (svd.matrixV()*m_sigma_inv*svd.matrixU().transpose());
}

//REF: boulic et al An inverse kinematics architecture enforcing an arbitrary Numeric of strict priority levels
template<typename _Matrix_Type_>
void PseudoInverse(_Matrix_Type_& pinvmat)
{
	Eigen::JacobiSVD<_Matrix_Type_> svd(pinvmat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	VectorX m_sigma = svd.singularValues();

	Numeric  pinvtoler= 1.e-6; // choose your tolerance widely!

	MatrixX m_sigma_inv = MatrixX::Zero(pinvmat.cols(),pinvmat.rows());
	for (long i=0; i<m_sigma.rows(); ++i)
	{
		if (m_sigma(i) > pinvtoler)
			m_sigma_inv(i,i)=1.0/m_sigma(i);
	}
	pinvmat = (svd.matrixV()*m_sigma_inv*svd.matrixU().transpose());
}

}//namespace Eigen

#endif //_MATRIXDEFSINTERNAL
