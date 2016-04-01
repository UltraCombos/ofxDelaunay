// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2013-2015 NVIDIA Corporation. All rights reserved.
#pragma once

#include "ofMain.h"

// Moller and Trumbore's method
inline bool IntersectRayTriTwoSided(const ofVec3f& p, const ofVec3f& dir, const ofVec3f& a, const ofVec3f& b, const ofVec3f& c, float& t, float& u, float& v, float& w, float& sign)//Vec3* normal)
{
	ofVec3f ab = b - a;
	ofVec3f ac = c - a;
	ofVec3f n = ab.getCrossed(ac);
	
	float d = (-dir).dot(n);
	float ood = 1.0f / d; // No need to check for division by zero here as infinity aritmetic will save us...
	ofVec3f ap = p - a;

	t = ap.dot(n) * ood;
	if (t < 0.0f)
		return false;

	ofVec3f e = (-dir).getCrossed(ap);
	v = ac.dot(e) * ood;
	if (v < 0.0f || v > 1.0f) // ...here...
		return false;
	w = -ab.dot(e) * ood;
	if (w < 0.0f || v + w > 1.0f) // ...and here
		return false;

	u = 1.0f - v - w;
	//if (normal)
	//*normal = n;
	sign = d;

	return true;
}

inline bool IntersectRayAABB(const ofVec3f& start, const ofVec3f& dir, const ofVec3f& min, const ofVec3f& max, float& t, ofVec3f* normal)
{
	//! calculate candidate plane on each axis
	float tx = -1.0f, ty = -1.0f, tz = -1.0f;
	bool inside = true;

	//! use unrolled loops

	//! x
	if (start.x < min.x)
	{
		if (dir.x != 0.0f)
			tx = (min.x - start.x) / dir.x;
		inside = false;
	}
	else if (start.x > max.x)
	{
		if (dir.x != 0.0f)
			tx = (max.x - start.x) / dir.x;
		inside = false;
	}

	//! y
	if (start.y < min.y)
	{
		if (dir.y != 0.0f)
			ty = (min.y - start.y) / dir.y;
		inside = false;
	}
	else if (start.y > max.y)
	{
		if (dir.y != 0.0f)
			ty = (max.y - start.y) / dir.y;
		inside = false;
	}

	//! z
	if (start.z < min.z)
	{
		if (dir.z != 0.0f)
			tz = (min.z - start.z) / dir.z;
		inside = false;
	}
	else if (start.z > max.z)
	{
		if (dir.z != 0.0f)
			tz = (max.z - start.z) / dir.z;
		inside = false;
	}

	//! if point inside all planes
	if (inside)
	{
		t = 0.0f;
		return true;
	}

	//! we now have t values for each of possible intersection planes
	//! find the maximum to get the intersection point
	float tmax = tx;
	int taxis = 0;

	if (ty > tmax)
	{
		tmax = ty;
		taxis = 1;
	}
	if (tz > tmax)
	{
		tmax = tz;
		taxis = 2;
	}

	if (tmax < 0.0f)
		return false;

	//! check that the intersection point lies on the plane we picked
	//! we don't test the axis of closest intersection for precision reasons

	//! no eps for now
	float eps = 0.0f;

	ofVec3f hit = start + dir*tmax;

	if ((hit.x < min.x - eps || hit.x > max.x + eps) && taxis != 0)
		return false;
	if ((hit.y < min.y - eps || hit.y > max.y + eps) && taxis != 1)
		return false;
	if ((hit.z < min.z - eps || hit.z > max.z + eps) && taxis != 2)
		return false;

	//! output results
	t = tmax;

	return true;
}

inline ofVec3f vec3_min(const ofVec3f& a, const ofVec3f& b)
{
	ofVec3f res;
	res.x = MIN(a.x, b.x);
	res.y = MIN(a.y, b.y);
	res.z = MIN(a.z, b.z);
	return res;
}

inline ofVec3f vec3_max(const ofVec3f& a, const ofVec3f& b)
{
	ofVec3f res;
	res.x = MAX(a.x, b.x);
	res.y = MAX(a.y, b.y);
	res.z = MAX(a.z, b.z);
	return res;
}