#pragma once

#include "mathUtils.h"

namespace tc
{
	bool voxelTriangleIntersection(const Vector& v1, const Vector& v2, const Vector& v3, const Vector& min, const Vector& max);

	Vector ClosestPoint(const Vector& a, const Vector& b, const Vector& c, const Vector& p, Vector& baryCoordinates);
}