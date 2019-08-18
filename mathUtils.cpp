#include "mathUtils.h"

using namespace tc;

//-----------------------
// Implementation of Vector
//-----------------------


 double &
Vector::operator [] (int i)
{
	return (&x)[i];
}


 const double &
Vector::operator [] (int i) const
{
	return (&x)[i];
}



Vector::Vector()
{
	// empty
}



Vector::Vector(double a)
{
	x = y = z = a;
}



Vector::Vector(double a, double b, double c)
{
	x = a;
	y = b;
	z = c;
}



Vector::Vector(const Vector &v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}


 const Vector &
Vector::operator = (const Vector &v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}




bool Vector::operator == (const Vector &v) const
{
	return x == v.x && y == v.y && z == v.z;
}



bool Vector::operator != (const Vector &v) const
{
	return x != v.x || y != v.y || z != v.z;
}


bool
Vector::equalWithAbsError(const Vector &v, double e) const
{
	for (int i = 0; i < 3; i++)
		if (!(abs((*this)[i] - v[i]) < e))
			return false;

	return true;
}

 double
Vector::dot(const Vector &v) const
{
	return x * v.x + y * v.y + z * v.z;
}


 double
Vector::operator ^ (const Vector &v) const
{
	return dot(v);
}


 Vector
Vector::cross(const Vector &v) const
{
	return Vector(y * v.z - z * v.y,
		z * v.x - x * v.z,
		x * v.y - y * v.x);
}


 const Vector &
Vector::operator %= (const Vector &v)
{
	double a = y * v.z - z * v.y;
	double b = z * v.x - x * v.z;
	double c = x * v.y - y * v.x;
	x = a;
	y = b;
	z = c;
	return *this;
}


 Vector
Vector::operator % (const Vector &v) const
{
	return Vector(y * v.z - z * v.y,
		z * v.x - x * v.z,
		x * v.y - y * v.x);
}


 const Vector &
Vector::operator += (const Vector &v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}


 Vector
Vector::operator + (const Vector &v) const
{
	return Vector(x + v.x, y + v.y, z + v.z);
}


 const Vector &
Vector::operator -= (const Vector &v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}


 Vector
Vector::operator - (const Vector &v) const
{
	return Vector(x - v.x, y - v.y, z - v.z);
}


 Vector
Vector::operator - () const
{
	return Vector(-x, -y, -z);
}


 const Vector &
Vector::negate()
{
	x = -x;
	y = -y;
	z = -z;
	return *this;
}


 const Vector &
Vector::operator *= (const Vector &v)
{
	x *= v.x;
	y *= v.y;
	z *= v.z;
	return *this;
}


 const Vector &
Vector::operator *= (double a)
{
	x *= a;
	y *= a;
	z *= a;
	return *this;
}


 Vector
Vector::operator * (const Vector &v) const
{
	return Vector(x * v.x, y * v.y, z * v.z);
}


 Vector
Vector::operator * (double a) const
{
	return Vector(x * a, y * a, z * a);
}


 const Vector &
Vector::operator /= (const Vector &v)
{
	x /= v.x;
	y /= v.y;
	z /= v.z;
	return *this;
}


 const Vector &
Vector::operator /= (double a)
{
	x /= a;
	y /= a;
	z /= a;
	return *this;
}


 Vector
Vector::operator / (const Vector &v) const
{
	return Vector(x / v.x, y / v.y, z / v.z);
}


 Vector
Vector::operator / (double a) const
{
	return Vector(x / a, y / a, z / a);
}


double
Vector::lengthTiny() const
{
	double absX = (x >= double(0)) ? x : -x;
	double absY = (y >= double(0)) ? y : -y;
	double absZ = (z >= double(0)) ? z : -z;

	double max = absX;

	if (max < absY)
		max = absY;

	if (max < absZ)
		max = absZ;

	if (max == double(0))
		return double(0);

	//
	// Do not replace the divisions by max with multiplications by 1/max.
	// Computing 1/max can overflow but the divisions below will always
	// produce results less than or equal to 1.
	//

	absX /= max;
	absY /= max;
	absZ /= max;

	return max * sqrt(absX * absX + absY * absY + absZ * absZ);
}


 double
Vector::length() const
{
	double length2 = dot(*this);

	if (length2 < double(2) * std::numeric_limits<double>::min())
		return lengthTiny();

	return sqrt(length2);
}


 double
Vector::length2() const
{
	return dot(*this);
}


const Vector &
Vector::normalize()
{
	double l = length();

	if (l != double(0))
	{
		//
		// Do not replace the divisions by l with multiplications by 1/l.
		// Computing 1/l can overflow but the divisions below will always
		// produce results less than or equal to 1.
		//

		x /= l;
		y /= l;
		z /= l;
	}

	return *this;
}


const Vector &
Vector::normalizeNonNull()
{
	double l = length();
	x /= l;
	y /= l;
	z /= l;
	return *this;
}


Vector
Vector::normalized() const
{
	double l = length();

	if (l == double(0))
		return Vector(double(0));

	return Vector(x / l, y / l, z / l);
}


Vector
Vector::normalizedNonNull() const
{
	double l = length();
	return Vector(x / l, y / l, z / l);
}