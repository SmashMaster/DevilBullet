/*
 * DevilBullet modifications (c) 2019 Sam Johnson https://github.com/SmashMaster
 * 
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.linearmath;


import javax.vecmath.Mat3;
import javax.vecmath.Vec3;

/**
 * Utility functions for axis aligned bounding boxes (AABB).
 * 
 * @author jezek2
 */
public class AabbUtil2 {

	public static void aabbExpand(Vec3 aabbMin, Vec3 aabbMax, Vec3 expansionMin, Vec3 expansionMax) {
		aabbMin.add(expansionMin);
		aabbMax.add(expansionMax);
	}

	public static int outcode(Vec3 p, Vec3 halfExtent) {
		return (p.x < -halfExtent.x ? 0x01 : 0x0) |
				(p.x > halfExtent.x ? 0x08 : 0x0) |
				(p.y < -halfExtent.y ? 0x02 : 0x0) |
				(p.y > halfExtent.y ? 0x10 : 0x0) |
				(p.z < -halfExtent.z ? 0x4 : 0x0) |
				(p.z > halfExtent.z ? 0x20 : 0x0);
	}
	
	public static boolean rayAabb(Vec3 rayFrom, Vec3 rayTo, Vec3 aabbMin, Vec3 aabbMax, float[] param, Vec3 normal) {
		Vec3 aabbHalfExtent = new Vec3();
		Vec3 aabbCenter = new Vec3();
		Vec3 source = new Vec3();
		Vec3 target = new Vec3();
		Vec3 r = new Vec3();
		Vec3 hitNormal = new Vec3();

		aabbHalfExtent.subHere(aabbMax, aabbMin);
		aabbHalfExtent.mult(0.5f);

		aabbCenter.addHere(aabbMax, aabbMin);
		aabbCenter.mult(0.5f);

		source.subHere(rayFrom, aabbCenter);
		target.subHere(rayTo, aabbCenter);

		int sourceOutcode = outcode(source, aabbHalfExtent);
		int targetOutcode = outcode(target, aabbHalfExtent);
		if ((sourceOutcode & targetOutcode) == 0x0) {
			float lambda_enter = 0f;
			float lambda_exit = param[0];
			r.subHere(target, source);

			float normSign = 1f;
			hitNormal.set(0f, 0f, 0f);
			int bit = 1;

			for (int j = 0; j < 2; j++) {
				for (int i = 0; i != 3; ++i) {
					if ((sourceOutcode & bit) != 0) {
						float lambda = (-VectorUtil.getCoord(source, i) - VectorUtil.getCoord(aabbHalfExtent, i) * normSign) / VectorUtil.getCoord(r, i);
						if (lambda_enter <= lambda) {
							lambda_enter = lambda;
							hitNormal.set(0f, 0f, 0f);
							VectorUtil.setCoord(hitNormal, i, normSign);
						}
					}
					else if ((targetOutcode & bit) != 0) {
						float lambda = (-VectorUtil.getCoord(source, i) - VectorUtil.getCoord(aabbHalfExtent, i) * normSign) / VectorUtil.getCoord(r, i);
						//btSetMin(lambda_exit, lambda);
						lambda_exit = Math.min(lambda_exit, lambda);
					}
					bit <<= 1;
				}
				normSign = -1f;
			}
			if (lambda_enter <= lambda_exit) {
				param[0] = lambda_enter;
				normal.set(hitNormal);
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Conservative test for overlap between two AABBs.
	 */
	public static boolean testAabbAgainstAabb2(Vec3 aabbMin1, Vec3 aabbMax1, Vec3 aabbMin2, Vec3 aabbMax2) {
		boolean overlap = true;
		overlap = (aabbMin1.x > aabbMax2.x || aabbMax1.x < aabbMin2.x) ? false : overlap;
		overlap = (aabbMin1.z > aabbMax2.z || aabbMax1.z < aabbMin2.z) ? false : overlap;
		overlap = (aabbMin1.y > aabbMax2.y || aabbMax1.y < aabbMin2.y) ? false : overlap;
		return overlap;
	}
	
	/**
	 * Conservative test for overlap between triangle and AABB.
	 */
	public static boolean testTriangleAgainstAabb2(Vec3[] vertices, Vec3 aabbMin, Vec3 aabbMax) {
		Vec3 p1 = vertices[0];
		Vec3 p2 = vertices[1];
		Vec3 p3 = vertices[2];

		if (Math.min(Math.min(p1.x, p2.x), p3.x) > aabbMax.x) return false;
		if (Math.max(Math.max(p1.x, p2.x), p3.x) < aabbMin.x) return false;

		if (Math.min(Math.min(p1.z, p2.z), p3.z) > aabbMax.z) return false;
		if (Math.max(Math.max(p1.z, p2.z), p3.z) < aabbMin.z) return false;

		if (Math.min(Math.min(p1.y, p2.y), p3.y) > aabbMax.y) return false;
		if (Math.max(Math.max(p1.y, p2.y), p3.y) < aabbMin.y) return false;
		
		return true;
	}

	public static void transformAabb(Vec3 halfExtents, float margin, Transform t, Vec3 aabbMinOut, Vec3 aabbMaxOut) {
		Vec3 halfExtentsWithMargin = new Vec3();
		halfExtentsWithMargin.x = halfExtents.x + margin;
		halfExtentsWithMargin.y = halfExtents.y + margin;
		halfExtentsWithMargin.z = halfExtents.z + margin;

		Mat3 abs_b = new Mat3(t.basis);
		MatrixUtil.absolute(abs_b);

		Vec3 tmp = new Vec3();

		Vec3 center = new Vec3(t.origin);
		Vec3 extent = new Vec3();
		abs_b.getRow(0, tmp);
		extent.x = tmp.dot(halfExtentsWithMargin);
		abs_b.getRow(1, tmp);
		extent.y = tmp.dot(halfExtentsWithMargin);
		abs_b.getRow(2, tmp);
		extent.z = tmp.dot(halfExtentsWithMargin);

		aabbMinOut.subHere(center, extent);
		aabbMaxOut.addHere(center, extent);
	}

	public static void transformAabb(Vec3 localAabbMin, Vec3 localAabbMax, float margin, Transform trans, Vec3 aabbMinOut, Vec3 aabbMaxOut) {
		assert (localAabbMin.x <= localAabbMax.x);
		assert (localAabbMin.y <= localAabbMax.y);
		assert (localAabbMin.z <= localAabbMax.z);

		Vec3 localHalfExtents = new Vec3();
		localHalfExtents.subHere(localAabbMax, localAabbMin);
		localHalfExtents.mult(0.5f);

		localHalfExtents.x += margin;
		localHalfExtents.y += margin;
		localHalfExtents.z += margin;

		Vec3 localCenter = new Vec3();
		localCenter.addHere(localAabbMax, localAabbMin);
		localCenter.mult(0.5f);

		Mat3 abs_b = new Mat3(trans.basis);
		MatrixUtil.absolute(abs_b);

		Vec3 center = new Vec3(localCenter);
		trans.transform(center);

		Vec3 extent = new Vec3();
		Vec3 tmp = new Vec3();

		abs_b.getRow(0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		abs_b.getRow(1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		abs_b.getRow(2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		aabbMinOut.subHere(center, extent);
		aabbMaxOut.addHere(center, extent);
	}

}
