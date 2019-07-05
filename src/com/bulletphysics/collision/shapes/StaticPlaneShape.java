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

package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;

/**
 * StaticPlaneShape simulates an infinite non-moving (static) collision plane.
 * 
 * @author jezek2
 */
public class StaticPlaneShape extends ConcaveShape {

	protected final Vec3 localAabbMin = new Vec3();
	protected final Vec3 localAabbMax = new Vec3();
	
	protected final Vec3 planeNormal = new Vec3();
	protected float planeConstant;
	protected final Vec3 localScaling = new Vec3(0f, 0f, 0f);

	public StaticPlaneShape(Vec3 planeNormal, float planeConstant) {
		VectorUtil.normalize(this.planeNormal, planeNormal);
		this.planeConstant = planeConstant;
	}

	public Vec3 getPlaneNormal(Vec3 out) {
		out.set(planeNormal);
		return out;
	}

	public float getPlaneConstant() {
		return planeConstant;
	}
	
	@Override
	public void processAllTriangles(TriangleCallback callback, Vec3 aabbMin, Vec3 aabbMax) {
		Vec3 tmp = new Vec3();
		Vec3 tmp1 = new Vec3();
		Vec3 tmp2 = new Vec3();

		Vec3 halfExtents = new Vec3();
		VectorUtil.sub(halfExtents, aabbMax, aabbMin);
		halfExtents.mult(0.5f);

		float radius = halfExtents.length();
		Vec3 center = new Vec3();
		VectorUtil.add(center, aabbMax, aabbMin);
		center.mult(0.5f);

		// this is where the triangles are generated, given AABB and plane equation (normal/constant)

		Vec3 tangentDir0 = new Vec3(), tangentDir1 = new Vec3();

		// tangentDir0/tangentDir1 can be precalculated
		TransformUtil.planeSpace1(planeNormal, tangentDir0, tangentDir1);

		Vec3 supVertex0 = new Vec3(), supVertex1 = new Vec3();

		Vec3 projectedCenter = new Vec3();
		VectorUtil.scale(tmp, planeNormal.dot(center) - planeConstant, planeNormal);
		VectorUtil.sub(projectedCenter, center, tmp);

		Vec3[] triangle = new Vec3[] { new Vec3(), new Vec3(), new Vec3() };

		VectorUtil.scale(tmp1, radius, tangentDir0);
		VectorUtil.scale(tmp2, radius, tangentDir1);
		VectorUtil.add(triangle[0], projectedCenter, tmp1, tmp2);

		VectorUtil.scale(tmp1, radius, tangentDir0);
		VectorUtil.scale(tmp2, radius, tangentDir1);
		VectorUtil.sub(tmp, tmp1, tmp2);
		VectorUtil.add(triangle[1], projectedCenter, tmp);

		VectorUtil.scale(tmp1, radius, tangentDir0);
		VectorUtil.scale(tmp2, radius, tangentDir1);
		VectorUtil.sub(tmp, tmp1, tmp2);
		VectorUtil.sub(triangle[2], projectedCenter, tmp);

		callback.processTriangle(triangle, 0, 0);

		VectorUtil.scale(tmp1, radius, tangentDir0);
		VectorUtil.scale(tmp2, radius, tangentDir1);
		VectorUtil.sub(tmp, tmp1, tmp2);
		VectorUtil.sub(triangle[0], projectedCenter, tmp);

		VectorUtil.scale(tmp1, radius, tangentDir0);
		VectorUtil.scale(tmp2, radius, tangentDir1);
		VectorUtil.sub(tmp, tmp1, tmp2);
		VectorUtil.sub(triangle[1], projectedCenter, tmp);

		VectorUtil.scale(tmp1, radius, tangentDir0);
		VectorUtil.scale(tmp2, radius, tangentDir1);
		VectorUtil.add(triangle[2], projectedCenter, tmp1, tmp2);

		callback.processTriangle(triangle, 0, 1);
	}

	@Override
	public void getAabb(Transform t, Vec3 aabbMin, Vec3 aabbMax) {
		aabbMin.set(-1e30f, -1e30f, -1e30f);
		aabbMax.set(1e30f, 1e30f, 1e30f);
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.STATIC_PLANE_PROXYTYPE;
	}

	@Override
	public void setLocalScaling(Vec3 scaling) {
		localScaling.set(scaling);
	}

	@Override
	public Vec3 getLocalScaling(Vec3 out) {
		out.set(localScaling);
		return out;
	}

	@Override
	public void calculateLocalInertia(float mass, Vec3 inertia) {
		//moving concave objects not supported
		inertia.set(0f, 0f, 0f);
	}

	@Override
	public String getName() {
		return "STATICPLANE";
	}

}
