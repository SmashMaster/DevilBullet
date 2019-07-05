/*
 * DevilBullet modifications (c) 2019 Sam Johnson https://github.com/SmashMaster
 * 
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * btConeTwistConstraint is Copyright (c) 2007 Starbreeze Studios
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
 * 
 * Written by: Marcus Hennix
 */

package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.ScalarUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Mat3;
import com.samrj.devil.math.Quat;
import com.samrj.devil.math.Vec3;

/**
 * ConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc).
 * 
 * @author jezek2
 */
public class ConeTwistConstraint extends TypedConstraint {

	private JacobianEntry[] jac/*[3]*/ = new JacobianEntry[] { new JacobianEntry(), new JacobianEntry(), new JacobianEntry() }; //3 orthogonal linear constraints

	private final Transform rbAFrame = new Transform();
	private final Transform rbBFrame = new Transform();

	private float limitSoftness;
	private float biasFactor;
	private float relaxationFactor;

	private float swingSpan1;
	private float swingSpan2;
	private float twistSpan;

	private final Vec3 swingAxis = new Vec3();
	private final Vec3 twistAxis = new Vec3();

	private float kSwing;
	private float kTwist;

	private float twistLimitSign;
	private float swingCorrection;
	private float twistCorrection;

	private float accSwingLimitImpulse;
	private float accTwistLimitImpulse;

	private boolean angularOnly = false;
	private boolean solveTwistLimit;
	private boolean solveSwingLimit;
	
	public ConeTwistConstraint() {
		super(TypedConstraintType.CONETWIST_CONSTRAINT_TYPE);
	}

	public ConeTwistConstraint(RigidBody rbA, RigidBody rbB, Transform rbAFrame, Transform rbBFrame) {
		super(TypedConstraintType.CONETWIST_CONSTRAINT_TYPE, rbA, rbB);
		this.rbAFrame.set(rbAFrame);
		this.rbBFrame.set(rbBFrame);

		swingSpan1 = 1e30f;
		swingSpan2 = 1e30f;
		twistSpan = 1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;

		solveTwistLimit = false;
		solveSwingLimit = false;
	}

	public ConeTwistConstraint(RigidBody rbA, Transform rbAFrame) {
		super(TypedConstraintType.CONETWIST_CONSTRAINT_TYPE, rbA);
		this.rbAFrame.set(rbAFrame);
		this.rbBFrame.set(this.rbAFrame);

		swingSpan1 = 1e30f;
		swingSpan2 = 1e30f;
		twistSpan = 1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;

		solveTwistLimit = false;
		solveSwingLimit = false;
	}
	
	@Override
	public void buildJacobian() {
		Vec3 tmp = new Vec3();
		Vec3 tmp1 = new Vec3();
		Vec3 tmp2 = new Vec3();

		Transform tmpTrans = new Transform();

		appliedImpulse = 0f;

		// set bias, sign, clear accumulator
		swingCorrection = 0f;
		twistLimitSign = 0f;
		solveTwistLimit = false;
		solveSwingLimit = false;
		accTwistLimitImpulse = 0f;
		accSwingLimitImpulse = 0f;

		if (!angularOnly) {
			Vec3 pivotAInW = new Vec3(rbAFrame.origin);
			rbA.getCenterOfMassTransform(tmpTrans).transform(pivotAInW);

			Vec3 pivotBInW = new Vec3(rbBFrame.origin);
			rbB.getCenterOfMassTransform(tmpTrans).transform(pivotBInW);

			Vec3 relPos = new Vec3();
			VectorUtil.sub(relPos, pivotBInW, pivotAInW);

			// TODO: stack
			Vec3[] normal/*[3]*/ = new Vec3[]{new Vec3(), new Vec3(), new Vec3()};
			if (relPos.squareLength() > BulletGlobals.FLT_EPSILON) {
				VectorUtil.normalize(normal[0], relPos);
			}
			else {
				normal[0].set(1f, 0f, 0f);
			}

			TransformUtil.planeSpace1(normal[0], normal[1], normal[2]);

			for (int i = 0; i < 3; i++) {
				Mat3 mat1 = rbA.getCenterOfMassTransform(new Transform()).basis;
				mat1.transpose();

				Mat3 mat2 = rbB.getCenterOfMassTransform(new Transform()).basis;
				mat2.transpose();

				VectorUtil.sub(tmp1, pivotAInW, rbA.getCenterOfMassPosition(tmp));
				VectorUtil.sub(tmp2, pivotBInW, rbB.getCenterOfMassPosition(tmp));

				jac[i].init(
						mat1,
						mat2,
						tmp1,
						tmp2,
						normal[i],
						rbA.getInvInertiaDiagLocal(new Vec3()),
						rbA.getInvMass(),
						rbB.getInvInertiaDiagLocal(new Vec3()),
						rbB.getInvMass());
			}
		}

		Vec3 b1Axis1 = new Vec3(), b1Axis2 = new Vec3(), b1Axis3 = new Vec3();
		Vec3 b2Axis1 = new Vec3(), b2Axis2 = new Vec3();

                MatrixUtil.getColumn(rbAFrame.basis, 0, b1Axis1);
		MatrixUtil.transform(getRigidBodyA().getCenterOfMassTransform(tmpTrans).basis, b1Axis1);

                MatrixUtil.getColumn(rbBFrame.basis, 0, b2Axis1);
		MatrixUtil.transform(getRigidBodyB().getCenterOfMassTransform(tmpTrans).basis, b2Axis1);

		float swing1 = 0f, swing2 = 0f;

		float swx = 0f, swy = 0f;
		float thresh = 10f;
		float fact;

		// Get Frame into world space
		if (swingSpan1 >= 0.05f) {
                        MatrixUtil.getColumn(rbAFrame.basis, 1, b1Axis2);
			MatrixUtil.transform(getRigidBodyA().getCenterOfMassTransform(tmpTrans).basis, b1Axis2);
//			swing1 = ScalarUtil.atan2Fast(b2Axis1.dot(b1Axis2), b2Axis1.dot(b1Axis1));
			swx = b2Axis1.dot(b1Axis1);
			swy = b2Axis1.dot(b1Axis2);
			swing1 = ScalarUtil.atan2Fast(swy, swx);
			fact = (swy*swy + swx*swx) * thresh * thresh;
			fact = fact / (fact + 1f);
			swing1 *= fact;
		}

		if (swingSpan2 >= 0.05f) {
                        MatrixUtil.getColumn(rbAFrame.basis, 2, b1Axis3);
			MatrixUtil.transform(getRigidBodyA().getCenterOfMassTransform(tmpTrans).basis, b1Axis3);
//			swing2 = ScalarUtil.atan2Fast(b2Axis1.dot(b1Axis3), b2Axis1.dot(b1Axis1));
			swx = b2Axis1.dot(b1Axis1);
			swy = b2Axis1.dot(b1Axis3);
			swing2 = ScalarUtil.atan2Fast(swy, swx);
			fact = (swy*swy + swx*swx) * thresh * thresh;
			fact = fact / (fact + 1f);
			swing2 *= fact;
		}

		float RMaxAngle1Sq = 1.0f / (swingSpan1 * swingSpan1);
		float RMaxAngle2Sq = 1.0f / (swingSpan2 * swingSpan2);
		float EllipseAngle = Math.abs(swing1*swing1) * RMaxAngle1Sq + Math.abs(swing2*swing2) * RMaxAngle2Sq;

		if (EllipseAngle > 1.0f) {
			swingCorrection = EllipseAngle - 1.0f;
			solveSwingLimit = true;

			// Calculate necessary axis & factors
			VectorUtil.scale(tmp1, b2Axis1.dot(b1Axis2), b1Axis2);
			VectorUtil.scale(tmp2, b2Axis1.dot(b1Axis3), b1Axis3);
			VectorUtil.add(tmp, tmp1, tmp2);
			VectorUtil.cross(swingAxis, b2Axis1, tmp);
			swingAxis.normalize();

			float swingAxisSign = (b2Axis1.dot(b1Axis1) >= 0.0f) ? 1.0f : -1.0f;
			swingAxis.mult(swingAxisSign);

			kSwing = 1f / (getRigidBodyA().computeAngularImpulseDenominator(swingAxis) +
					getRigidBodyB().computeAngularImpulseDenominator(swingAxis));

		}

		// Twist limits
		if (twistSpan >= 0f) {
			//Vec3 b2Axis2 = new Vec3();
                        MatrixUtil.getColumn(rbBFrame.basis, 1, b2Axis2);
			MatrixUtil.transform(getRigidBodyB().getCenterOfMassTransform(tmpTrans).basis, b2Axis2);

			Quat rotationArc = QuaternionUtil.shortestArcQuat(b2Axis1, b1Axis1, new Quat());
			Vec3 TwistRef = QuaternionUtil.quatRotate(rotationArc, b2Axis2, new Vec3());
			float twist = ScalarUtil.atan2Fast(TwistRef.dot(b1Axis3), TwistRef.dot(b1Axis2));

			float lockedFreeFactor = (twistSpan > 0.05f) ? limitSoftness : 0f;
			if (twist <= -twistSpan * lockedFreeFactor) {
				twistCorrection = -(twist + twistSpan);
				solveTwistLimit = true;

				VectorUtil.add(twistAxis, b2Axis1, b1Axis1);
				twistAxis.mult(0.5f);
				twistAxis.normalize();
				twistAxis.mult(-1.0f);

				kTwist = 1f / (getRigidBodyA().computeAngularImpulseDenominator(twistAxis) +
						getRigidBodyB().computeAngularImpulseDenominator(twistAxis));

			}
			else if (twist > twistSpan * lockedFreeFactor) {
				twistCorrection = (twist - twistSpan);
				solveTwistLimit = true;

				VectorUtil.add(twistAxis, b2Axis1, b1Axis1);
				twistAxis.mult(0.5f);
				twistAxis.normalize();

				kTwist = 1f / (getRigidBodyA().computeAngularImpulseDenominator(twistAxis) +
						getRigidBodyB().computeAngularImpulseDenominator(twistAxis));
			}
		}
	}

	@Override
	public void solveConstraint(float timeStep) {
		Vec3 tmp = new Vec3();
		Vec3 tmp2 = new Vec3();

		Vec3 tmpVec = new Vec3();
		Transform tmpTrans = new Transform();

		Vec3 pivotAInW = new Vec3(rbAFrame.origin);
		rbA.getCenterOfMassTransform(tmpTrans).transform(pivotAInW);

		Vec3 pivotBInW = new Vec3(rbBFrame.origin);
		rbB.getCenterOfMassTransform(tmpTrans).transform(pivotBInW);

		float tau = 0.3f;

		// linear part
		if (!angularOnly) {
			Vec3 rel_pos1 = new Vec3();
			VectorUtil.sub(rel_pos1, pivotAInW, rbA.getCenterOfMassPosition(tmpVec));

			Vec3 rel_pos2 = new Vec3();
			VectorUtil.sub(rel_pos2, pivotBInW, rbB.getCenterOfMassPosition(tmpVec));

			Vec3 vel1 = rbA.getVelocityInLocalPoint(rel_pos1, new Vec3());
			Vec3 vel2 = rbB.getVelocityInLocalPoint(rel_pos2, new Vec3());
			Vec3 vel = new Vec3();
			VectorUtil.sub(vel, vel1, vel2);

			for (int i = 0; i < 3; i++) {
				Vec3 normal = jac[i].linearJointAxis;
				float jacDiagABInv = 1f / jac[i].getDiagonal();

				float rel_vel;
				rel_vel = normal.dot(vel);
				// positional error (zeroth order error)
				VectorUtil.sub(tmp, pivotAInW, pivotBInW);
				float depth = -(tmp).dot(normal); // this is the error projected on the normal
				float impulse = depth * tau / timeStep * jacDiagABInv - rel_vel * jacDiagABInv;
				appliedImpulse += impulse;
				Vec3 impulse_vector = new Vec3();
				VectorUtil.scale(impulse_vector, impulse, normal);

				VectorUtil.sub(tmp, pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
				rbA.applyImpulse(impulse_vector, tmp);

				VectorUtil.negate(tmp, impulse_vector);
				VectorUtil.sub(tmp2, pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
				rbB.applyImpulse(tmp, tmp2);
			}
		}

		{
			// solve angular part
			Vec3 angVelA = getRigidBodyA().getAngularVelocity(new Vec3());
			Vec3 angVelB = getRigidBodyB().getAngularVelocity(new Vec3());

			// solve swing limit
			if (solveSwingLimit) {
				VectorUtil.sub(tmp, angVelB, angVelA);
				float amplitude = ((tmp).dot(swingAxis) * relaxationFactor * relaxationFactor + swingCorrection * (1f / timeStep) * biasFactor);
				float impulseMag = amplitude * kSwing;

				// Clamp the accumulated impulse
				float temp = accSwingLimitImpulse;
				accSwingLimitImpulse = Math.max(accSwingLimitImpulse + impulseMag, 0.0f);
				impulseMag = accSwingLimitImpulse - temp;

				Vec3 impulse = new Vec3();
				VectorUtil.scale(impulse, impulseMag, swingAxis);

				rbA.applyTorqueImpulse(impulse);

				VectorUtil.negate(tmp, impulse);
				rbB.applyTorqueImpulse(tmp);
			}

			// solve twist limit
			if (solveTwistLimit) {
				VectorUtil.sub(tmp, angVelB, angVelA);
				float amplitude = ((tmp).dot(twistAxis) * relaxationFactor * relaxationFactor + twistCorrection * (1f / timeStep) * biasFactor);
				float impulseMag = amplitude * kTwist;

				// Clamp the accumulated impulse
				float temp = accTwistLimitImpulse;
				accTwistLimitImpulse = Math.max(accTwistLimitImpulse + impulseMag, 0.0f);
				impulseMag = accTwistLimitImpulse - temp;

				Vec3 impulse = new Vec3();
				VectorUtil.scale(impulse, impulseMag, twistAxis);

				rbA.applyTorqueImpulse(impulse);

				VectorUtil.negate(tmp, impulse);
				rbB.applyTorqueImpulse(tmp);
			}
		}
	}

	public void updateRHS(float timeStep) {
	}

	public void setAngularOnly(boolean angularOnly) {
		this.angularOnly = angularOnly;
	}

	public void setLimit(float _swingSpan1, float _swingSpan2, float _twistSpan) {
		setLimit(_swingSpan1, _swingSpan2, _twistSpan, 0.8f, 0.3f, 1.0f);
	}

	public void setLimit(float _swingSpan1, float _swingSpan2, float _twistSpan, float _softness, float _biasFactor, float _relaxationFactor) {
		swingSpan1 = _swingSpan1;
		swingSpan2 = _swingSpan2;
		twistSpan = _twistSpan;

		limitSoftness = _softness;
		biasFactor = _biasFactor;
		relaxationFactor = _relaxationFactor;
	}

	public Transform getAFrame(Transform out) {
		out.set(rbAFrame);
		return out;
	}

	public Transform getBFrame(Transform out) {
		out.set(rbBFrame);
		return out;
	}

	public boolean getSolveTwistLimit() {
		return solveTwistLimit;
	}

	public boolean getSolveSwingLimit() {
		return solveTwistLimit;
	}

	public float getTwistLimitSign() {
		return twistLimitSign;
	}
	
}
