
#include <Eigen/Dense>
#include <Eigen/SVD>

#ifndef _MATRIXDEFS
#define _MATRIXDEFS

namespace Eigen
{
	typedef Eigen::Vector4d Vector4;
	typedef Eigen::Vector3d Vector3;
	typedef Eigen::Vector2d Vector2;
	typedef Eigen::VectorXd VectorX;
	typedef Eigen::Matrix4d Matrix4;
	typedef double Numeric;
	typedef Eigen::Matrix3d Matrix3;

	void Matrix3ToMatrix4(const Matrix3& from, Matrix4& to);

	void Matrix4ToMatrix3(const Matrix4& from, Matrix3& to);

	/* Rotates rotated around axis by angle theta, and returns it */
	Matrix4 Translate(Numeric x, Numeric y, Numeric z);
	Matrix4 Translate(Vector3 vector);
	Matrix4 Rotx4(Numeric theta);
	Matrix4 Roty4(Numeric theta);
	Matrix4 Rotz4(Numeric theta);

	Matrix3 Rotx3(Numeric theta);
	Matrix3 Roty3(Numeric theta);
	Matrix3 Rotz3(Numeric theta);

	// project vector onto another and returns cosinnus angle
	double Project(const Vector3& from, const Vector3& to);

	Vector3 ProjectOnPlan(const Vector3& normalAxis, const Vector3& vector);

	bool NotZero(const Vector3& vect);

	/*Tu consid�res u,v deux vecteurs de norme L2 = 1 dans R^3
	Tu cherches la rotation R, telle que Ru=v.
	R = cos theta * I + (I x [a,a,a])^T * sin theta + (1 - cos theta) * a*a^T
	avec :
	cos theta = u . v
	sin theta = ||u x v||
	a=u x v / sin theta
	I �tant l'identit�, * le produit matriciel, x le cross product et ^T la transpos�e.
	http://fr.wikipedia.org/wiki/Rotation_vectorielle
	D�riv�e de la formule de Rodriguez*/
	void GetRotationMatrix(const Vector3& from, const Vector3& to, Matrix3& result);

	// TODO forward dec
	Vector3& Rotate(const Vector3& axis, Vector3& rotated, Numeric theta);

	void vect4ToVect3(const VectorX& from, Vector3& to);

	extern void vect3ToVect4(const Vector3& from, VectorX& to);

	Vector3 matrix4TimesVect3(const Matrix4& mat4, const Vector3& vect3);

	Vector3 matrix4TimesVect4(const Matrix4& mat4, const VectorX& vect4);

} //namespace Eigen

#endif //_MATRIXDEFS
