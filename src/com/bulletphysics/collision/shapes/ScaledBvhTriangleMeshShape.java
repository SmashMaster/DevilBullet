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
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import javax.vecmath.Mat3;
import javax.vecmath.Vec3;

// JAVA NOTE: ScaledBvhTriangleMeshShape from 2.73 SP1

/**
 * The ScaledBvhTriangleMeshShape allows to instance a scaled version of an existing
 * {@link BvhTriangleMeshShape}. Note that each {@link BvhTriangleMeshShape} still can
 * have its own local scaling, independent from this ScaledBvhTriangleMeshShape 'localScaling'.
 *
 * @author jezek2
 */
public class ScaledBvhTriangleMeshShape extends ConcaveShape {

	protected final Vec3 localScaling = new Vec3();
	protected BvhTriangleMeshShape bvhTriMeshShape;

	public ScaledBvhTriangleMeshShape(BvhTriangleMeshShape childShape, Vec3 localScaling) {
		this.localScaling.set(localScaling);
		this.bvhTriMeshShape = childShape;
	}

	public BvhTriangleMeshShape getChildShape() {
		return bvhTriMeshShape;
	}

	@Override
	public void processAllTriangles(TriangleCallback callback, Vec3 aabbMin, Vec3 aabbMax) {
		ScaledTriangleCallback scaledCallback = new ScaledTriangleCallback(callback, localScaling);

		Vec3 invLocalScaling = new Vec3();
		invLocalScaling.set(1.f / localScaling.x, 1.f / localScaling.y, 1.f / localScaling.z);

		Vec3 scaledAabbMin = new Vec3();
		Vec3 scaledAabbMax = new Vec3();

		// support negative scaling
		scaledAabbMin.x = localScaling.x >= 0f ? aabbMin.x * invLocalScaling.x : aabbMax.x * invLocalScaling.x;
		scaledAabbMin.y = localScaling.y >= 0f ? aabbMin.y * invLocalScaling.y : aabbMax.y * invLocalScaling.y;
		scaledAabbMin.z = localScaling.z >= 0f ? aabbMin.z * invLocalScaling.z : aabbMax.z * invLocalScaling.z;

		scaledAabbMax.x = localScaling.x <= 0f ? aabbMin.x * invLocalScaling.x : aabbMax.x * invLocalScaling.x;
		scaledAabbMax.y = localScaling.y <= 0f ? aabbMin.y * invLocalScaling.y : aabbMax.y * invLocalScaling.y;
		scaledAabbMax.z = localScaling.z <= 0f ? aabbMin.z * invLocalScaling.z : aabbMax.z * invLocalScaling.z;

		bvhTriMeshShape.processAllTriangles(scaledCallback, scaledAabbMin, scaledAabbMax);
	}

	@Override
	public void getAabb(Transform trans, Vec3 aabbMin, Vec3 aabbMax) {
		Vec3 localAabbMin = bvhTriMeshShape.getLocalAabbMin(new Vec3());
		Vec3 localAabbMax = bvhTriMeshShape.getLocalAabbMax(new Vec3());

		Vec3 tmpLocalAabbMin = new Vec3();
		Vec3 tmpLocalAabbMax = new Vec3();
		VectorUtil.mul(tmpLocalAabbMin, localAabbMin, localScaling);
		VectorUtil.mul(tmpLocalAabbMax, localAabbMax, localScaling);

		localAabbMin.x = (localScaling.x >= 0f) ? tmpLocalAabbMin.x : tmpLocalAabbMax.x;
		localAabbMin.y = (localScaling.y >= 0f) ? tmpLocalAabbMin.y : tmpLocalAabbMax.y;
		localAabbMin.z = (localScaling.z >= 0f) ? tmpLocalAabbMin.z : tmpLocalAabbMax.z;
		localAabbMax.x = (localScaling.x <= 0f) ? tmpLocalAabbMin.x : tmpLocalAabbMax.x;
		localAabbMax.y = (localScaling.y <= 0f) ? tmpLocalAabbMin.y : tmpLocalAabbMax.y;
		localAabbMax.z = (localScaling.z <= 0f) ? tmpLocalAabbMin.z : tmpLocalAabbMax.z;

		Vec3 localHalfExtents = new Vec3();
		localHalfExtents.subHere(localAabbMax, localAabbMin);
		localHalfExtents.mult(0.5f);

		float margin = bvhTriMeshShape.getMargin();
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

		aabbMin.subHere(center, extent);
		aabbMax.addHere(center, extent);
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
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
	}

	@Override
	public String getName() {
		return "SCALEDBVHTRIANGLEMESH";
	}

	////////////////////////////////////////////////////////////////////////////

	private static class ScaledTriangleCallback extends TriangleCallback {
		private TriangleCallback originalCallback;
		private Vec3 localScaling;
		private Vec3[] newTriangle = new Vec3[3];

		public ScaledTriangleCallback(TriangleCallback originalCallback, Vec3 localScaling) {
			this.originalCallback = originalCallback;
			this.localScaling = localScaling;
			
			for (int i=0; i<newTriangle.length; i++) {
				newTriangle[i] = new Vec3();
			}
		}

		public void processTriangle(Vec3[] triangle, int partId, int triangleIndex) {
			VectorUtil.mul(newTriangle[0], triangle[0], localScaling);
			VectorUtil.mul(newTriangle[1], triangle[1], localScaling);
			VectorUtil.mul(newTriangle[2], triangle[2], localScaling);
			originalCallback.processTriangle(newTriangle, partId, triangleIndex);
		}
	}

}
