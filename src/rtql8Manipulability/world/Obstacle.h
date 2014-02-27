
#ifndef _CLASS_OBSTACLE
#define _CLASS_OBSTACLE

#include "utils/MatrixDefs.h"

#include <vector>
namespace manip_core
{
class Obstacle {

public:
	typedef std::vector<Obstacle>		T_Obstacle;
	typedef T_Obstacle::const_iterator	CIT_Obstacle;

public:
	//make it clockwise from upper left
	 Obstacle(const Eigen::Vector3d& /*p1*/, const Eigen::Vector3d& /*p2*/, const Eigen::Vector3d& /*p3*/);
	~Obstacle();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// Minimal distance between the plan described by the obstacle and a point, that is, the distance btw point and its orthonormal projection on the plan
    double DistanceToPlan(const Eigen::Vector3d& /*point*/, Eigen::Vector3d& /*getCoordinates*/) const; // get coordinates of the projection
    double DistanceToPlan(const Eigen::Vector3d& /*point*/) const;
    double Distance(const Eigen::Vector3d& /*point*/) const;
    bool IsAbove(const Eigen::Vector3d& /*point*/) const; // true if a point is above the obstacle, considering the normal

	const Eigen::Vector3d  ProjectUp(const Eigen::Vector3d& /*point*/) const; // projects a points onto obstacle plan and rises it up a little
	
	const Eigen::Vector3d& Center  () const { return center_; }
	const Eigen::Matrix4d& Basis   () const { return basis_; }
	const Eigen::Matrix4d& BasisInv() const { return basisInverse_; }

	double GetD() const { return d_; }
	double GetW() const { return w_; }
	double GetH() const { return h_; }
	double GetA() const { return a_; }
	double GetB() const { return b_; }
	double GetC() const { return c_; }

public:
	const Eigen::Vector3d u_;
	const Eigen::Vector3d v_;
	const Eigen::Vector3d n_; // normal vector
	const Eigen::Vector3d p1_;
	const Eigen::Vector3d p2_;
	const Eigen::Vector3d p3_;

private:
	void InitObstacle();
  
	Eigen::Vector3d center_;

	Eigen::Matrix4d basis_; // transformation matrix to world basis (on p4)
	Eigen::Matrix4d basisInverse_; // transformation matrix to rectangle basis (on p4)

	double a_;
	double b_;
	double c_;
	double d_;
	double norm_;
	double normsquare_;
	double w_;
	double h_;
};
}

#endif //_CLASS_OBSTACLE
