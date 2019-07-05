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
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Mat3;
import javax.vecmath.Vec3;

/**
 * Point to point constraint between two rigid bodies each with a pivot point that
 * descibes the "ballsocket" location in local space.
 * 
 * @author jezek2
 */
public class Point2PointConstraint extends TypedConstraint {

	private final JacobianEntry[] jac = new JacobianEntry[]/*[3]*/ { new JacobianEntry(), new JacobianEntry(), new JacobianEntry() }; // 3 orthogonal linear constraints
	
	private final Vec3 pivotInA = new Vec3();
	private final Vec3 pivotInB = new Vec3();

	public ConstraintSetting setting = new ConstraintSetting();
	
	public Point2PointConstraint() {
		super(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE);
	}

	public Point2PointConstraint(RigidBody rbA, RigidBody rbB, Vec3 pivotInA, Vec3 pivotInB) {
		super(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE, rbA, rbB);
		this.pivotInA.set(pivotInA);
		this.pivotInB.set(pivotInB);
	}

	public Point2PointConstraint(RigidBody rbA, Vec3 pivotInA) {
		super(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE, rbA);
		this.pivotInA.set(pivotInA);
		this.pivotInB.set(pivotInA);
		rbA.getCenterOfMassTransform(new Transform()).transform(this.pivotInB);
	}

	@Override
	public void buildJacobian() {
		appliedImpulse = 0f;

		Vec3 normal = new Vec3();
		normal.set(0f, 0f, 0f);

		Mat3 tmpMat1 = new Mat3();
		Mat3 tmpMat2 = new Mat3();
		Vec3 tmp1 = new Vec3();
		Vec3 tmp2 = new Vec3();
		Vec3 tmpVec = new Vec3();
		
		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(new Transform());

		for (int i = 0; i < 3; i++) {
			VectorUtil.setCoord(normal, i, 1f);

                        Mat3.transpose(centerOfMassA.basis, tmpMat1);
                        Mat3.transpose(centerOfMassB.basis, tmpMat2);

			tmp1.set(pivotInA);
			centerOfMassA.transform(tmp1);
			tmp1.sub(rbA.getCenterOfMassPosition(tmpVec));

			tmp2.set(pivotInB);
			centerOfMassB.transform(tmp2);
			tmp2.sub(rbB.getCenterOfMassPosition(tmpVec));

			jac[i].init(
					tmpMat1,
					tmpMat2,
					tmp1,
					tmp2,
					normal,
					rbA.getInvInertiaDiagLocal(new Vec3()),
					rbA.getInvMass(),
					rbB.getInvInertiaDiagLocal(new Vec3()),
					rbB.getInvMass());
			VectorUtil.setCoord(normal, i, 0f);
		}
	}

	@Override
	public void solveConstraint(float timeStep) {
		Vec3 tmp = new Vec3();
		Vec3 tmp2 = new Vec3();
		Vec3 tmpVec = new Vec3();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(new Transform());
		
		Vec3 pivotAInW = new Vec3(pivotInA);
		centerOfMassA.transform(pivotAInW);

		Vec3 pivotBInW = new Vec3(pivotInB);
		centerOfMassB.transform(pivotBInW);

		Vec3 normal = new Vec3();
		normal.set(0f, 0f, 0f);

		//btVector3 angvelA = m_rbA.getCenterOfMassTransform().getBasis().transpose() * m_rbA.getAngularVelocity();
		//btVector3 angvelB = m_rbB.getCenterOfMassTransform().getBasis().transpose() * m_rbB.getAngularVelocity();

		for (int i = 0; i < 3; i++) {
			VectorUtil.setCoord(normal, i, 1f);
			float jacDiagABInv = 1f / jac[i].getDiagonal();

			Vec3 rel_pos1 = new Vec3();
			rel_pos1.subHere(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
			Vec3 rel_pos2 = new Vec3();
			rel_pos2.subHere(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
			// this jacobian entry could be re-used for all iterations

			Vec3 vel1 = rbA.getVelocityInLocalPoint(rel_pos1, new Vec3());
			Vec3 vel2 = rbB.getVelocityInLocalPoint(rel_pos2, new Vec3());
			Vec3 vel = new Vec3();
			vel.subHere(vel1, vel2);

			float rel_vel;
			rel_vel = normal.dot(vel);

			/*
			//velocity error (first order error)
			btScalar rel_vel = m_jac[i].getRelativeVelocity(m_rbA.getLinearVelocity(),angvelA,
			m_rbB.getLinearVelocity(),angvelB);
			 */

			// positional error (zeroth order error)
			tmp.subHere(pivotAInW, pivotBInW);
			float depth = -tmp.dot(normal); //this is the error projected on the normal

			float impulse = depth * setting.tau / timeStep * jacDiagABInv - setting.damping * rel_vel * jacDiagABInv;

			float impulseClamp = setting.impulseClamp;
			if (impulseClamp > 0f) {
				if (impulse < -impulseClamp) {
					impulse = -impulseClamp;
				}
				if (impulse > impulseClamp) {
					impulse = impulseClamp;
				}
			}

			appliedImpulse += impulse;
			Vec3 impulse_vector = new Vec3();
			impulse_vector.scale(impulse, normal);
			tmp.subHere(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
			rbA.applyImpulse(impulse_vector, tmp);
			tmp.negateHere(impulse_vector);
			tmp2.subHere(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
			rbB.applyImpulse(tmp, tmp2);

			VectorUtil.setCoord(normal, i, 0f);
		}
	}
	
	public void updateRHS(float timeStep) {
	}

	public void setPivotA(Vec3 pivotA) {
		pivotInA.set(pivotA);
	}

	public void setPivotB(Vec3 pivotB) {
		pivotInB.set(pivotB);
	}

	public Vec3 getPivotInA(Vec3 out) {
		out.set(pivotInA);
		return out;
	}

	public Vec3 getPivotInB(Vec3 out) {
		out.set(pivotInB);
		return out;
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	public static class ConstraintSetting {
		public float tau = 0.3f;
		public float damping = 1f;
		public float impulseClamp = 0f;
	}
	
}
