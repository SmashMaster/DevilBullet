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
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;

/**
 * CollisionShape class provides an interface for collision shapes that can be
 * shared among {@link CollisionObject}s.
 * 
 * @author jezek2
 */
public abstract class CollisionShape {

	//protected final BulletStack stack = BulletStack.get();

	protected Object userPointer;
	
	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	public abstract void getAabb(Transform t, Vec3 aabbMin, Vec3 aabbMax);

	public void getBoundingSphere(Vec3 center, float[] radius) {
		Vec3 tmp = new Vec3();

		Transform tr = new Transform();
		tr.setIdentity();
		Vec3 aabbMin = new Vec3(), aabbMax = new Vec3();

		getAabb(tr, aabbMin, aabbMax);

		VectorUtil.sub(tmp, aabbMax, aabbMin);
		radius[0] = tmp.length() * 0.5f;

		VectorUtil.sub(tmp, aabbMin, aabbMax);
		VectorUtil.scale(center, 0.5f, tmp);
	}

	///getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.
	public float getAngularMotionDisc() {
		Vec3 center = new Vec3();
		float[] disc = new float[1]; // TODO: stack
		getBoundingSphere(center, disc);
		disc[0] += center.length();
		return disc[0];
	}

	///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	///result is conservative
	public void calculateTemporalAabb(Transform curTrans, Vec3 linvel, Vec3 angvel, float timeStep, Vec3 temporalAabbMin, Vec3 temporalAabbMax) {
		//start with static aabb
		getAabb(curTrans, temporalAabbMin, temporalAabbMax);

		float temporalAabbMaxx = temporalAabbMax.x;
		float temporalAabbMaxy = temporalAabbMax.y;
		float temporalAabbMaxz = temporalAabbMax.z;
		float temporalAabbMinx = temporalAabbMin.x;
		float temporalAabbMiny = temporalAabbMin.y;
		float temporalAabbMinz = temporalAabbMin.z;

		// add linear motion
		Vec3 linMotion = new Vec3(linvel);
		linMotion.mult(timeStep);

		//todo: simd would have a vector max/min operation, instead of per-element access
		if (linMotion.x > 0f) {
			temporalAabbMaxx += linMotion.x;
		}
		else {
			temporalAabbMinx += linMotion.x;
		}
		if (linMotion.y > 0f) {
			temporalAabbMaxy += linMotion.y;
		}
		else {
			temporalAabbMiny += linMotion.y;
		}
		if (linMotion.z > 0f) {
			temporalAabbMaxz += linMotion.z;
		}
		else {
			temporalAabbMinz += linMotion.z;
		}

		//add conservative angular motion
		float angularMotion = angvel.length() * getAngularMotionDisc() * timeStep;
		Vec3 angularMotion3d = new Vec3();
		angularMotion3d.set(angularMotion, angularMotion, angularMotion);
		temporalAabbMin.set(temporalAabbMinx, temporalAabbMiny, temporalAabbMinz);
		temporalAabbMax.set(temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz);

		temporalAabbMin.sub(angularMotion3d);
		temporalAabbMax.add(angularMotion3d);
	}

//#ifndef __SPU__
	public boolean isPolyhedral() {
		return getShapeType().isPolyhedral();
	}

	public boolean isConvex() {
		return getShapeType().isConvex();
	}

	public boolean isConcave() {
		return getShapeType().isConcave();
	}

	public boolean isCompound() {
		return getShapeType().isCompound();
	}

	///isInfinite is used to catch simulation error (aabb check)
	public boolean isInfinite() {
		return getShapeType().isInfinite();
	}

	public abstract BroadphaseNativeType getShapeType();

	public abstract void setLocalScaling(Vec3 scaling);
	
	// TODO: returns const
	public abstract Vec3 getLocalScaling(Vec3 out);

	public abstract void calculateLocalInertia(float mass, Vec3 inertia);


//debugging support
	public abstract String getName();
//#endif //__SPU__
	public abstract void setMargin(float margin);

	public abstract float getMargin();
	
	// optional user data pointer
	public void setUserPointer(Object userPtr) {
		userPointer = userPtr;
	}

	public Object getUserPointer() {
		return userPointer;
	}
	
}
