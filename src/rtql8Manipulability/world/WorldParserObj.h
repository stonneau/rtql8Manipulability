
#ifndef _CLASS_WORLDPARSEROBJ
#define _CLASS_WORLDPARSEROBJ

#include <Eigen/Dense>

#include <string>
#include <vector>

struct WorldParserObj
{
public:
	 WorldParserObj();
	~WorldParserObj();

	void CreateWorld(const std::string& /*filename*/);

private:
	void CreateObstacle (const std::string& /*line*/);
	
private:
	typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > T_Vector3;

private:
	T_Vector3 points_;
	T_Vector3 normals_;
};

#endif //_CLASS_WORLDPARSEROBJ
