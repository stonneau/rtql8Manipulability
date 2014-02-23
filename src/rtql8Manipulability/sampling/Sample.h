
#ifndef _CLASS_SAMPLE
#define _CLASS_SAMPLE

#include <Eigen/Dense>
#include <vector>

namespace rtql8 {
namespace kinematics {
class BodyNode;

class Sample {

public:
	typedef std::vector<double> T_Angles;
	typedef T_Angles::const_iterator CIT_Angles;

public:
	Sample(BodyNode* /*root*/);
	~Sample();

private:
	Sample& operator =(const Sample&);
	/*Sample(const Sample&);*/

public:
	void LoadIntoLimb(BodyNode* /*limbRoot*/, bool updateTransform=false) const;
	double ForceManipulability (const Eigen::Vector3d& /*direction*/) const ;

public:
	const T_Angles angles_; // filled using a depth first approach
	const Eigen::Vector3d position_; // Position in Coordinates of root node

private:
	Eigen::Matrix3d jacobianProd_;
	Eigen::Matrix3d jacobianProdInverse_;
};
} // namespace kinematics
} //namespace rtql8
#endif //_CLASS_SAMPLE
