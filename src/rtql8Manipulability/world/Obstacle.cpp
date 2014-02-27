
#include "rtql8Manipulability/world/Obstacle.h"
#include "utils/MatrixDefs.h"
#include "utils/MatrixDefsInternal.h"

#include <math.h>

using namespace manip_core;
using namespace Eigen;

namespace
{
double clamp(double a, double min, double max)
{
   return a > max ? max : (a < min ? min : a);
}

//triangle / point distance
// source http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
Vector3 ClosesPointOnTriangle(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& sourcePosition)
{
    Vector3 edge0 = p1 - p0;
    Vector3 edge1 = p2 - p0;
    Vector3 v0 = p0 - sourcePosition;

    double a = edge0.dot( edge0 );
    double b = edge0.dot( edge1 );
    double c = edge1.dot( edge1 );
    double d = edge0.dot( v0 );
    double e = edge1.dot( v0 );

    double det = a*c - b*b;
    double s = b*e - c*d;
    double t = b*d - a*e;

    if ( s + t < det )
    {
        if ( s < 0.f )
        {
            if ( t < 0.f )
            {
                if ( d < 0.f )
                {
                    s = clamp( -d/a, 0.f, 1.f );
                    t = 0.f;
                }
                else
                {
                    s = 0.f;
                    t = clamp( -e/c, 0.f, 1.f );
                }
            }
            else
            {
                s = 0.f;
                t = clamp( -e/c, 0.f, 1.f );
            }
        }
        else if ( t < 0.f )
        {
            s = clamp( -d/a, 0.f, 1.f );
            t = 0.f;
        }
        else
        {
            float invDet = 1.f / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        if ( s < 0.f )
        {
            double tmp0 = b+d;
            double tmp1 = c+e;
            if ( tmp1 > tmp0 )
            {
                double numer = tmp1 - tmp0;
                double denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                t = clamp( -e/c, 0.f, 1.f );
                s = 0.f;
            }
        }
        else if ( t < 0.f )
        {
            if ( a+d > b+e )
            {
                double numer = c+e-b-d;
                double denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                s = clamp( -e/c, 0.f, 1.f );
                t = 0.f;
            }
        }
        else
        {
            double numer = c+e-b-d;
            double denom = a-2*b+c;
            s = clamp( numer/denom, 0.f, 1.f );
            t = 1.f - s;
        }
    }

    return p0 + s * edge0 + t * edge1;
}
}


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

double Obstacle::DistanceToPlan(const Vector3d& point, Vector3d& getCoordinates) const
{
	// http://fr.wikipedia.org/wiki/Distance_d%27un_point_%C3%A0_un_plan
	double lambda = - ((a_ * point.x() + b_ * point.y() + c_ * point.z() + d_) / normsquare_);
	getCoordinates(0) = lambda * a_ + point.x();
	getCoordinates(1) = lambda * b_ + point.y();
	getCoordinates(2) = lambda * c_ + point.z();
    return lambda > 0 ? (lambda * norm_) : -(lambda * norm_);
}

double Obstacle::DistanceToPlan(const Vector3d& point) const
{
    // http://fr.wikipedia.org/wiki/Distance_d%27un_point_%C3%A0_un_plan
    double lambda = - ((a_ * point.x() + b_ * point.y() + c_ * point.z() + d_) / normsquare_);
    return lambda > 0 ? (lambda * norm_) : -(lambda * norm_);
}

double Obstacle::Distance(const Vector3d& point) const
{
    return (ClosesPointOnTriangle(p1_, p2_, p3_, point)-point).norm();
}

bool Obstacle::IsAbove(const Vector3d& point) const
{
    Vector3d nNormal = n_; nNormal.normalize();
    Vector3d projection;
    double distance = DistanceToPlan(point, projection);
	return (point - ( projection + distance * nNormal )).norm() < 0.000000001;
}

const Vector3d Obstacle::ProjectUp(const Vector3d& point) const// projects a points onto obstacle plan and rises it up a little
{
    Vector3d res = Eigen::matrix4TimesVect3(basisInverse_, point);
    Vector3d nNormal = n_; nNormal.normalize();
	res(2) = nNormal(2) * 0.1;
	return Eigen::matrix4TimesVect3(basis_, res);
}

