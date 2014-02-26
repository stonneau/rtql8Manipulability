
#include "rtql8Manipulability/world/Obstacle.h"
#include "utils/MatrixDefs.h"
#include "utils/MatrixDefsInternal.h"

#include <math.h>

using namespace manip_core;
using namespace Eigen;


Obstacle::Obstacle(const Vector3d& p1, const Vector3d& p2, const Vector3d& p3)
	: p1_(p1)
	, p2_(p2)
	, p3_(p3)
	, u_(p3-p1)
	, v_(p2-p1)
	, n_(u_.cross(v_))
{
	InitObstacle();
}


void Obstacle::InitObstacle()
{

	Vector3d p4_(p1_+ (p3_ - p2_));
	Vector3d normal = n_;
	a_ = (float)(normal.x());
	b_ = (float)(normal.y());
	c_ = (float)(normal.z());

	norm_ = (float)(normal.norm());
	normsquare_ = norm_ * norm_;
	d_ = (float)(-(a_ * p1_.x() + b_ * p1_.y() + c_ * p1_.z()));
	center_ = p1_ + ((p4_ - p1_) + (p2_ - p1_)) / 2 ;

	basis_ = Matrix4::Zero();
	Vector3d x = (p3_ - p4_); x.normalize();
	Vector3d y = (p1_ - p4_); y.normalize();
	normal.normalize();
	basis_.block(0,0,3,1) = x;
	basis_.block(0,1,3,1) = y;
	basis_.block(0,2,3,1) = normal;
	basis_.block(0,3,3,1) = p4_;
	basis_(3,3) = 1;
	basisInverse_ = basis_.inverse();

	w_ = (float)((p3_ - p4_).norm());
	h_ = (float)((p1_ - p4_).norm());
}


Obstacle::~Obstacle()
{
	// NOTHING
}

double Obstacle::Distance(const Vector3d& point, Vector3d& getCoordinates) const
{
	// http://fr.wikipedia.org/wiki/Distance_d%27un_point_%C3%A0_un_plan
	double lambda = - ((a_ * point.x() + b_ * point.y() + c_ * point.z() + d_) / normsquare_);
	getCoordinates(0) = lambda * a_ + point.x();
	getCoordinates(1) = lambda * b_ + point.y();
	getCoordinates(2) = lambda * c_ + point.z();
    return lambda > 0 ? (lambda * norm_) : -(lambda * norm_);
}

double Obstacle::Distance(const Vector3d& point) const
{
    // http://fr.wikipedia.org/wiki/Distance_d%27un_point_%C3%A0_un_plan
    double lambda = - ((a_ * point.x() + b_ * point.y() + c_ * point.z() + d_) / normsquare_);
    return lambda > 0 ? (lambda * norm_) : -(lambda * norm_);
}

bool Obstacle::IsAbove(const Vector3d& point) const
{
	Vector3d nNormal = n_; nNormal.normalize();
	Vector3d projection;
	double distance = Distance(point, projection);
	return (point - ( projection + distance * nNormal )).norm() < 0.000000001;
}

const Vector3d Obstacle::ProjectUp(const Vector3d& point) const// projects a points onto obstacle plan and rises it up a little
{
	Vector3d res = Eigen::matrix4TimesVect3(basisInverse_, point);
	Vector3d nNormal = n_; nNormal.normalize();
	res(2) = nNormal(2) * 0.1;
	return Eigen::matrix4TimesVect3(basis_, res);
}

