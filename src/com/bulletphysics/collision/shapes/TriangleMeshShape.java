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

import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Mat3;
import javax.vecmath.Vec3;

/**
 * Concave triangle mesh abstract class. Use {@link BvhTriangleMeshShape} as concrete
 * implementation.
 * 
 * @author jezek2
 */
public abstract class TriangleMeshShape extends ConcaveShape {

	protected final Vec3 localAabbMin = new Vec3();
	protected final Vec3 localAabbMax = new Vec3();
	protected StridingMeshInterface meshInterface;

	/**
	 * TriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
	 * Don't use btTriangleMeshShape but use btBvhTriangleMeshShape instead!
	 */
	protected TriangleMeshShape(StridingMeshInterface meshInterface) {
		this.meshInterface = meshInterface;
		
		// JAVA NOTE: moved to BvhTriangleMeshShape
		//recalcLocalAabb();
	}
	
	public Vec3 localGetSupportingVertex(Vec3 vec, Vec3 out) {
		Vec3 tmp = new Vec3();

		Vec3 supportVertex = out;

		Transform ident = new Transform();
		ident.setIdentity();

		SupportVertexCallback supportCallback = new SupportVertexCallback(vec, ident);

		Vec3 aabbMax = new Vec3();
		aabbMax.set(1e30f, 1e30f, 1e30f);
		tmp.negateHere(aabbMax);

		processAllTriangles(supportCallback, tmp, aabbMax);

		supportCallback.getSupportVertexLocal(supportVertex);

		return out;
	}

	public Vec3 localGetSupportingVertexWithoutMargin(Vec3 vec, Vec3 out) {
		assert (false);
		return localGetSupportingVertex(vec, out);
	}

	public void recalcLocalAabb() {
		for (int i = 0; i < 3; i++) {
			Vec3 vec = new Vec3();
			vec.set(0f, 0f, 0f);
			VectorUtil.setCoord(vec, i, 1f);
			Vec3 tmp = localGetSupportingVertex(vec, new Vec3());
			VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(tmp, i) + collisionMargin);
			VectorUtil.setCoord(vec, i, -1f);
			localGetSupportingVertex(vec, tmp);
			VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(tmp, i) - collisionMargin);
		}
	}

	@Override
	public void getAabb(Transform trans, Vec3 aabbMin, Vec3 aabbMax) {
		Vec3 tmp = new Vec3();

		Vec3 localHalfExtents = new Vec3();
		localHalfExtents.subHere(localAabbMax, localAabbMin);
		localHalfExtents.mult(0.5f);

		Vec3 localCenter = new Vec3();
		localCenter.addHere(localAabbMax, localAabbMin);
		localCenter.mult(0.5f);

		Mat3 abs_b = new Mat3(trans.basis);
		MatrixUtil.absolute(abs_b);

		Vec3 center = new Vec3(localCenter);
		trans.transform(center);

		Vec3 extent = new Vec3();
		MatrixUtil.getRow(abs_b, 0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		MatrixUtil.getRow(abs_b, 1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		MatrixUtil.getRow(abs_b, 2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		Vec3 margin = new Vec3();
		margin.set(getMargin(), getMargin(), getMargin());
		extent.add(margin);

		aabbMin.subHere(center, extent);
		aabbMax.addHere(center, extent);
	}

	@Override
	public void processAllTriangles(TriangleCallback callback, Vec3 aabbMin, Vec3 aabbMax) {
		FilteredCallback filterCallback = new FilteredCallback(callback, aabbMin, aabbMax);

		meshInterface.internalProcessAllTriangles(filterCallback, aabbMin, aabbMax);
	}

	@Override
	public void calculateLocalInertia(float mass, Vec3 inertia) {
		// moving concave objects not supported
		assert (false);
		inertia.set(0f, 0f, 0f);
	}


	@Override
	public void setLocalScaling(Vec3 scaling) {
		meshInterface.setScaling(scaling);
		recalcLocalAabb();
	}

	@Override
	public Vec3 getLocalScaling(Vec3 out) {
		return meshInterface.getScaling(out);
	}
	
	public StridingMeshInterface getMeshInterface() {
		return meshInterface;
	}

	public Vec3 getLocalAabbMin(Vec3 out) {
		out.set(localAabbMin);
		return out;
	}

	public Vec3 getLocalAabbMax(Vec3 out) {
		out.set(localAabbMax);
		return out;
	}

	@Override
	public String getName() {
		return "TRIANGLEMESH";
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	private class SupportVertexCallback extends TriangleCallback {
		private final Vec3 supportVertexLocal = new Vec3(0f, 0f, 0f);
		public final Transform worldTrans = new Transform();
		public float maxDot = -1e30f;
		public final Vec3 supportVecLocal = new Vec3();

		public SupportVertexCallback(Vec3 supportVecWorld,Transform trans) {
			this.worldTrans.set(trans);
			MatrixUtil.transposeTransform(supportVecLocal, supportVecWorld, worldTrans.basis);
		}
		
		public void processTriangle(Vec3[] triangle, int partId, int triangleIndex) {
			for (int i = 0; i < 3; i++) {
				float dot = supportVecLocal.dot(triangle[i]);
				if (dot > maxDot) {
					maxDot = dot;
					supportVertexLocal.set(triangle[i]);
				}
			}
		}

		public Vec3 getSupportVertexWorldSpace(Vec3 out) {
			out.set(supportVertexLocal);
			worldTrans.transform(out);
			return out;
		}

		public Vec3 getSupportVertexLocal(Vec3 out) {
			out.set(supportVertexLocal);
			return out;
		}
	}
	
	private static class FilteredCallback extends InternalTriangleIndexCallback {
		public TriangleCallback callback;
		public final Vec3 aabbMin = new Vec3();
		public final Vec3 aabbMax = new Vec3();

		public FilteredCallback(TriangleCallback callback, Vec3 aabbMin, Vec3 aabbMax) {
			this.callback = callback;
			this.aabbMin.set(aabbMin);
			this.aabbMax.set(aabbMax);
		}

		public void internalProcessTriangleIndex(Vec3[] triangle, int partId, int triangleIndex) {
			if (AabbUtil2.testTriangleAgainstAabb2(triangle, aabbMin, aabbMax)) {
				// check aabb in triangle-space, before doing this
				callback.processTriangle(triangle, partId, triangleIndex);
			}
		}
	}

}
