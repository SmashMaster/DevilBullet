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

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;

/**
 * ConvexInternalShape is an internal base class, shared by most convex shape implementations.
 * 
 * @author jezek2
 */
public abstract class ConvexInternalShape extends ConvexShape {

	// local scaling. collisionMargin is not scaled !
	protected final Vec3 localScaling = new Vec3(1f, 1f, 1f);
	protected final Vec3 implicitShapeDimensions = new Vec3();
	protected float collisionMargin = BulletGlobals.CONVEX_DISTANCE_MARGIN;

	/**
	 * getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version.
	 */
	@Override
	public void getAabb(Transform t, Vec3 aabbMin, Vec3 aabbMax) {
		getAabbSlow(t, aabbMin, aabbMax);
	}
	
	@Override
	public void getAabbSlow(Transform trans, Vec3 minAabb, Vec3 maxAabb) {
		float margin = getMargin();
		Vec3 vec = new Vec3();
		Vec3 tmp1 = new Vec3();
		Vec3 tmp2 = new Vec3();
		
		for (int i=0;i<3;i++)
		{
			vec.set(0f, 0f, 0f);
			VectorUtil.setCoord(vec, i, 1f);

			MatrixUtil.transposeTransform(tmp1, vec, trans.basis);
			localGetSupportingVertex(tmp1, tmp2);
			
			trans.transform(tmp2);

			VectorUtil.setCoord(maxAabb, i, VectorUtil.getCoord(tmp2, i) + margin);

			VectorUtil.setCoord(vec, i, -1f);

			MatrixUtil.transposeTransform(tmp1, vec, trans.basis);
			localGetSupportingVertex(tmp1, tmp2);
			trans.transform(tmp2);

			VectorUtil.setCoord(minAabb, i, VectorUtil.getCoord(tmp2, i) - margin);
		}
	}

	@Override
	public Vec3 localGetSupportingVertex(Vec3 vec, Vec3 out) {
		Vec3 supVertex = localGetSupportingVertexWithoutMargin(vec, out);

		if (getMargin() != 0f) {
			Vec3 vecnorm = new Vec3(vec);
			if (vecnorm.squareLength() < (BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON)) {
				vecnorm.set(-1f, -1f, -1f);
			}
			vecnorm.normalize();
			VectorUtil.scaleAdd(supVertex, getMargin(), vecnorm, supVertex);
		}
		return out;
	}
	
	public void setLocalScaling(Vec3 scaling) {
		VectorUtil.absolute(localScaling, scaling);
	}
	
	public Vec3 getLocalScaling(Vec3 out) {
		out.set(localScaling);
		return out;
	}

	public float getMargin() {
		return collisionMargin;
	}

	public void setMargin(float margin) {
		this.collisionMargin = margin;
	}

	@Override
	public int getNumPreferredPenetrationDirections() {
		return 0;
	}

	@Override
	public void getPreferredPenetrationDirection(int index, Vec3 penetrationVector) {
		throw new InternalError();
	}
	
}
