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

package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;

/**
 * SolverBody is an internal data structure for the constraint solver. Only necessary
 * data is packed to increase cache coherence/performance.
 * 
 * @author jezek2
 */
public class SolverBody {
	
	//protected final BulletStack stack = BulletStack.get();

	public final Vec3 angularVelocity = new Vec3();
	public float angularFactor;
	public float invMass;
	public float friction;
	public RigidBody originalBody;
	public final Vec3 linearVelocity = new Vec3();
	public final Vec3 centerOfMassPosition = new Vec3();

	public final Vec3 pushVelocity = new Vec3();
	public final Vec3 turnVelocity = new Vec3();
	
	public void getVelocityInLocalPoint(Vec3 rel_pos, Vec3 velocity) {
		Vec3 tmp = new Vec3();
		VectorUtil.cross(tmp, angularVelocity, rel_pos);
		VectorUtil.add(velocity, linearVelocity, tmp);
	}

	/**
	 * Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position.
	 */
	public void internalApplyImpulse(Vec3 linearComponent, Vec3 angularComponent, float impulseMagnitude) {
		if (invMass != 0f) {
			VectorUtil.scaleAdd(linearVelocity, impulseMagnitude, linearComponent, linearVelocity);
			VectorUtil.scaleAdd(angularVelocity, impulseMagnitude * angularFactor, angularComponent, angularVelocity);
		}
	}

	public void internalApplyPushImpulse(Vec3 linearComponent, Vec3 angularComponent, float impulseMagnitude) {
		if (invMass != 0f) {
			VectorUtil.scaleAdd(pushVelocity, impulseMagnitude, linearComponent, pushVelocity);
			VectorUtil.scaleAdd(turnVelocity, impulseMagnitude * angularFactor, angularComponent, turnVelocity);
		}
	}
	
	public void writebackVelocity() {
		if (invMass != 0f) {
			originalBody.setLinearVelocity(linearVelocity);
			originalBody.setAngularVelocity(angularVelocity);
			//m_originalBody->setCompanionId(-1);
		}
	}

	public void writebackVelocity(float timeStep) {
		if (invMass != 0f) {
			originalBody.setLinearVelocity(linearVelocity);
			originalBody.setAngularVelocity(angularVelocity);

			// correct the position/orientation based on push/turn recovery
			Transform newTransform = new Transform();
			Transform curTrans = originalBody.getWorldTransform(new Transform());
			TransformUtil.integrateTransform(curTrans, pushVelocity, turnVelocity, timeStep, newTransform);
			originalBody.setWorldTransform(newTransform);

			//m_originalBody->setCompanionId(-1);
		}
	}
	
	public void readVelocity() {
		if (invMass != 0f) {
			originalBody.getLinearVelocity(linearVelocity);
			originalBody.getAngularVelocity(angularVelocity);
		}
	}
	
}
