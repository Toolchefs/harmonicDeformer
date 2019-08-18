#include "intersect.h"

#include <vector>

using namespace tc;

double getmin(const std::vector<Vector> &points, Vector axis)
{
	double min = std::numeric_limits<double>::max();

	for (int ctr = 0; ctr < points.size(); ctr++)
	{
		double dotprod = points[ctr] ^ axis;
		if (dotprod < min) min = dotprod;
	}
	return min;
}

double getmax(const std::vector<Vector> &points, Vector axis)
{
	double max = -std::numeric_limits<double>::max();

	for (int ctr = 0; ctr < points.size(); ctr++)
	{
		double dotprod = points[ctr] ^ axis;
		if (dotprod > max) max = dotprod;
	}
	return max;
}

bool isect(const std::vector<Vector> &points1, const std::vector<Vector> &points2, Vector axis)
{
	if (getmin(points1, axis) > getmax(points2, axis)) return false;
	if (getmax(points1, axis) < getmin(points2, axis)) return false;
	return true;
}

//SAT algorithm
bool tc::voxelTriangleIntersection(const Vector& v1, const Vector& v2, const Vector& v3, const Vector& min, const Vector& max)
{
	std::vector<Vector> boxpoints;
	boxpoints.push_back(Vector(max[0], max[1], max[2]));
	boxpoints.push_back(Vector(max[0], max[1], min[2]));
	boxpoints.push_back(Vector(max[0], min[1], max[2]));
	boxpoints.push_back(Vector(max[0], min[1], min[2]));
	boxpoints.push_back(Vector(min[0], max[1], max[2]));
	boxpoints.push_back(Vector(min[0], max[1], min[2]));
	boxpoints.push_back(Vector(min[0], min[1], max[2]));
	boxpoints.push_back(Vector(min[0], min[1], min[2]));

	std::vector<Vector> tripoints;
	tripoints.push_back(v1);
	tripoints.push_back(v2);
	tripoints.push_back(v3);

	// test the x, y, and z axes
	if (!isect(boxpoints, tripoints, Vector(1, 0, 0))) return false;
	if (!isect(boxpoints, tripoints, Vector(0, 1, 0))) return false;
	if (!isect(boxpoints, tripoints, Vector(0, 0, 1))) return false;

	// test the triangle normal
	Vector triedge1 = v2 - v1;
	Vector triedge2 = v3 - v2;
	Vector trinormal = triedge1 % triedge2;
	if (!isect(boxpoints, tripoints, trinormal)) return false;

	// test the 9 edge cross products
	Vector triedge3 = tripoints[0] - tripoints[2];

	Vector boxedge1 = Vector(1, 0, 0);
	Vector boxedge2 = Vector(0, 1, 0);
	Vector boxedge3 = Vector(0, 0, 1);

	if (!isect(boxpoints, tripoints, (boxedge1 % triedge1))) return false;
	if (!isect(boxpoints, tripoints, (boxedge1 % triedge2))) return false;
	if (!isect(boxpoints, tripoints, (boxedge1 % triedge3))) return false;
											   
	if (!isect(boxpoints, tripoints, (boxedge2 % triedge1))) return false;
	if (!isect(boxpoints, tripoints, (boxedge2 % triedge2))) return false;
	if (!isect(boxpoints, tripoints, (boxedge2 % triedge3))) return false;
											   
	if (!isect(boxpoints, tripoints, (boxedge3 % triedge1))) return false;
	if (!isect(boxpoints, tripoints, (boxedge3 % triedge2))) return false;
	if (!isect(boxpoints, tripoints, (boxedge3 % triedge3))) return false;

	return true;
}

void Barycentric(const Vector& p, const Vector& a, const Vector& b, const Vector& c, double &u, double &v, double &w)
{
	Vector v0 = b - a, v1 = c - a, v2 = p - a;
	double d00 = v0 ^ v0;
	double d01 = v0 ^ v1;
	double d11 = v1 ^ v1;
	double d20 = v2 ^ v0;
	double d21 = v2 ^ v1;
	double denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0 - v - w;
}


// from realt time colision detection
Vector tc::ClosestPoint(const Vector& a, const Vector& b, const Vector& c, const Vector& p, Vector& baryCoordinates)
{
	/*
	// Check if P in vertex region outside A
	Vector ab = b - a;
	Vector ac = c - a;
	Vector ap = p - a;
	double d1 = ab.dot(ap);
	double d2 = ac.dot(ap);
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		baryCoordinates.x = 1.0;
		baryCoordinates.y = 0.0;
		baryCoordinates.z = 0.0;
		return a; // barycentric coordinates (1,0,0)
	}
	// Check if P in vertex region outside B
	Vector bp = p - b;
	double d3 = ab.dot(bp);
	double d4 = ac.dot(bp);
	if (d3 >= 0.0f && d4 <= d3)
	{
		baryCoordinates.x = 0.0;
		baryCoordinates.y = 1.0;
		baryCoordinates.z = 0.0;
		return b; // barycentric coordinates (0,1,0)
	}
	// Check if P in edge region of AB, if so return projection of P onto AB
	double vc = d1*d4 - d3*d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		double v = d1 / (d1 - d3);
		baryCoordinates.x = 1.0 - v;
		baryCoordinates.y = v;
		baryCoordinates.z = 0.0;
		return a + ab * v; // barycentric coordinates (1-v,v,0)
	}
	// Check if P in vertex region outside C
	Vector cp = p - c;
	double d5 = ab.dot(cp);
	double d6 = ac.dot(cp);
	if (d6 >= 0.0f && d5 <= d6)
	{
		baryCoordinates.x = 0.0;
		baryCoordinates.y = 0.0;
		baryCoordinates.z = 1.0;
		return c; // barycentric coordinates (0,0,1)
	}

	// Check if P in edge region of AC, if so return projection of P onto AC
	double vb = d5*d2 - d1*d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
		double w = d2 / (d2 - d6);
		baryCoordinates.x = 1.0 - w;
		baryCoordinates.y = 0;
		baryCoordinates.z = w;
		return a + ac * w; // barycentric coordinates (1-w,0,w)
	}
	// Check if P in edge region of BC, if so return projection of P onto BC
	double va = d3*d6 - d5*d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
		double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		baryCoordinates.x = 0;
		baryCoordinates.y = 1.0-w;
		baryCoordinates.z = w;
		return b + (c - b) * w; // barycentric coordinates (0,1-w,w)
	}
	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	double denom = 1.0f / (va + vb + vc);
	double v = vb * denom;
	double w = vc * denom;
	baryCoordinates.x = 1.0f - v - w;
	baryCoordinates.y = v;
	baryCoordinates.z = w;
	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
	*/
	Vector diff = p - a;
	Vector edge0 = b - a;
	Vector edge1 = c - a;
	double a00 = edge0.dot(edge0);
	double a01 = edge0.dot( edge1);
	double a11 = edge1.dot(edge1);
	double b0 = -1.0 * (diff.dot(edge0));
	double b1 = -1.0 * (diff.dot(edge1));
	double const zero = (double)0;
	double const one = (double)1;
	double det = a00 * a11 - a01 * a01;
	double t0 = a01 * b1 - a11 * b0;
	double t1 = a01 * b0 - a00 * b1;

	if (t0 + t1 <= det)
	{
		if (t0 < zero)
		{
			if (t1 < zero)  // region 4
			{
				if (b0 < zero)
				{
					t1 = zero;
					if (-b0 >= a00)  // V0
					{
						t0 = one;
					}
					else  // E01
					{
						t0 = -b0 / a00;
					}
				}
				else
				{
					t0 = zero;
					if (b1 >= zero)  // V0
					{
						t1 = zero;
					}
					else if (-b1 >= a11)  // V2
					{
						t1 = one;
					}
					else  // E20
					{
						t1 = -b1 / a11;
					}
				}
			}
			else  // region 3
			{
				t0 = zero;
				if (b1 >= zero)  // V0
				{
					t1 = zero;
				}
				else if (-b1 >= a11)  // V2
				{
					t1 = one;
				}
				else  // E20
				{
					t1 = -b1 / a11;
				}
			}
		}
		else if (t1 < zero)  // region 5
		{
			t1 = zero;
			if (b0 >= zero)  // V0
			{
				t0 = zero;
			}
			else if (-b0 >= a00)  // V1
			{
				t0 = one;
			}
			else  // E01
			{
				t0 = -b0 / a00;
			}
		}
		else  // region 0, interior
		{
			double invDet = one / det;
			t0 *= invDet;
			t1 *= invDet;
		}
	}
	else
	{
		double tmp0, tmp1, numer, denom;

		if (t0 < zero)  // region 2
		{
			tmp0 = a01 + b0;
			tmp1 = a11 + b1;
			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - ((double)2)*a01 + a11;
				if (numer >= denom)  // V1
				{
					t0 = one;
					t1 = zero;
				}
				else  // E12
				{
					t0 = numer / denom;
					t1 = one - t0;
				}
			}
			else
			{
				t0 = zero;
				if (tmp1 <= zero)  // V2
				{
					t1 = one;
				}
				else if (b1 >= zero)  // V0
				{
					t1 = zero;
				}
				else  // E20
				{
					t1 = -b1 / a11;
				}
			}
		}
		else if (t1 < zero)  // region 6
		{
			tmp0 = a01 + b1;
			tmp1 = a00 + b0;
			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - ((double)2)*a01 + a11;
				if (numer >= denom)  // V2
				{
					t1 = one;
					t0 = zero;
				}
				else  // E12
				{
					t1 = numer / denom;
					t0 = one - t1;
				}
			}
			else
			{
				t1 = zero;
				if (tmp1 <= zero)  // V1
				{
					t0 = one;
				}
				else if (b0 >= zero)  // V0
				{
					t0 = zero;
				}
				else  // E01
				{
					t0 = -b0 / a00;
				}
			}
		}
		else  // region 1
		{
			numer = a11 + b1 - a01 - b0;
			if (numer <= zero)  // V2
			{
				t0 = zero;
				t1 = one;
			}
			else
			{
				denom = a00 - ((double)2)*a01 + a11;
				if (numer >= denom)  // V1
				{
					t0 = one;
					t1 = zero;
				}
				else  // 12
				{
					t0 = numer / denom;
					t1 = one - t0;
				}
			}
		}
	}


	baryCoordinates.x = one - t0 - t1;
	baryCoordinates.y = t0;
	baryCoordinates.z = t1;
	return a + edge0 * t0 + edge1 * t1;
}


