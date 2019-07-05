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

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le�n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/

package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.VectorUtil;
import javax.vecmath.Vec3;

/**
 *
 * @author jezek2
 */
public class TranslationalLimitMotor {
	
	//protected final BulletStack stack = BulletStack.get();
	
	public final Vec3 lowerLimit = new Vec3(); //!< the constraint lower limits
	public final Vec3 upperLimit = new Vec3(); //!< the constraint upper limits
	public final Vec3 accumulatedImpulse = new Vec3();
	
	public float limitSoftness; //!< Softness for linear limit
	public float damping; //!< Damping for linear limit
	public float restitution; //! Bounce parameter for linear limit

	public TranslationalLimitMotor() {
		lowerLimit.set(0f, 0f, 0f);
		upperLimit.set(0f, 0f, 0f);
		accumulatedImpulse.set(0f, 0f, 0f);

		limitSoftness = 0.7f;
		damping = 1.0f;
		restitution = 0.5f;
	}

	public TranslationalLimitMotor(TranslationalLimitMotor other) {
		lowerLimit.set(other.lowerLimit);
		upperLimit.set(other.upperLimit);
		accumulatedImpulse.set(other.accumulatedImpulse);

		limitSoftness = other.limitSoftness;
		damping = other.damping;
		restitution = other.restitution;
	}

	/**
	 * Test limit.<p>
	 * - free means upper &lt; lower,<br>
	 * - locked means upper == lower<br>
	 * - limited means upper &gt; lower<br>
	 * - limitIndex: first 3 are linear, next 3 are angular
	 */
	public boolean isLimited(int limitIndex) {
		return (VectorUtil.getCoord(upperLimit, limitIndex) >= VectorUtil.getCoord(lowerLimit, limitIndex));
	}

	public float solveLinearAxis(float timeStep, float jacDiagABInv, RigidBody body1, Vec3 pointInA, RigidBody body2, Vec3 pointInB, int limit_index, Vec3 axis_normal_on_a, Vec3 anchorPos) {
		Vec3 tmp = new Vec3();
		Vec3 tmpVec = new Vec3();
		
		// find relative velocity
		Vec3 rel_pos1 = new Vec3();
		//rel_pos1.sub(pointInA, body1.getCenterOfMassPosition(tmpVec));
		rel_pos1.subHere(anchorPos, body1.getCenterOfMassPosition(tmpVec));

		Vec3 rel_pos2 = new Vec3();
		//rel_pos2.sub(pointInB, body2.getCenterOfMassPosition(tmpVec));
		rel_pos2.subHere(anchorPos, body2.getCenterOfMassPosition(tmpVec));

		Vec3 vel1 = body1.getVelocityInLocalPoint(rel_pos1, new Vec3());
		Vec3 vel2 = body2.getVelocityInLocalPoint(rel_pos2, new Vec3());
		Vec3 vel = new Vec3();
		vel.subHere(vel1, vel2);

		float rel_vel = axis_normal_on_a.dot(vel);

		// apply displacement correction

		// positional error (zeroth order error)
		tmp.subHere(pointInA, pointInB);
		float depth = -(tmp).dot(axis_normal_on_a);
		float lo = -1e30f;
		float hi = 1e30f;

		float minLimit = VectorUtil.getCoord(lowerLimit, limit_index);
		float maxLimit = VectorUtil.getCoord(upperLimit, limit_index);

		// handle the limits
		if (minLimit < maxLimit) {
			{
				if (depth > maxLimit) {
					depth -= maxLimit;
					lo = 0f;

				}
				else {
					if (depth < minLimit) {
						depth -= minLimit;
						hi = 0f;
					}
					else {
						return 0.0f;
					}
				}
			}
		}

		float normalImpulse = limitSoftness * (restitution * depth / timeStep - damping * rel_vel) * jacDiagABInv;

		float oldNormalImpulse = VectorUtil.getCoord(accumulatedImpulse, limit_index);
		float sum = oldNormalImpulse + normalImpulse;
		VectorUtil.setCoord(accumulatedImpulse, limit_index, sum > hi ? 0f : sum < lo ? 0f : sum);
		normalImpulse = VectorUtil.getCoord(accumulatedImpulse, limit_index) - oldNormalImpulse;

		Vec3 impulse_vector = new Vec3();
		impulse_vector.scale(normalImpulse, axis_normal_on_a);
		body1.applyImpulse(impulse_vector, rel_pos1);

		tmp.negateHere(impulse_vector);
		body2.applyImpulse(tmp, rel_pos2);
		return normalImpulse;
	}
	
}
