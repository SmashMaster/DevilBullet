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

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */

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
 * Hinge constraint between two rigid bodies each with a pivot point that descibes
 * the axis location in local space. Axis defines the orientation of the hinge axis.
 * 
 * @author jezek2
 */
public class HingeConstraint extends TypedConstraint {

	private JacobianEntry[] jac/*[3]*/ = new JacobianEntry[] { new JacobianEntry(), new JacobianEntry(), new JacobianEntry() }; // 3 orthogonal linear constraints
	private JacobianEntry[] jacAng/*[3]*/ = new JacobianEntry[] { new JacobianEntry(), new JacobianEntry(), new JacobianEntry() }; // 2 orthogonal angular constraints+ 1 for limit/motor

	private final Transform rbAFrame = new Transform(); // constraint axii. Assumes z is hinge axis.
	private final Transform rbBFrame = new Transform();

	private float motorTargetVelocity;
	private float maxMotorImpulse;

	private float limitSoftness; 
	private float biasFactor; 
	private float relaxationFactor; 

	private float lowerLimit;	
	private float upperLimit;	
	
	private float kHinge;

	private float limitSign;
	private float correction;

	private float accLimitImpulse;

	private boolean angularOnly;
	private boolean enableAngularMotor;
	private boolean solveLimit;

	public HingeConstraint() {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE);
		enableAngularMotor = false;
	}

	public HingeConstraint(RigidBody rbA, RigidBody rbB, Vec3 pivotInA, Vec3 pivotInB, Vec3 axisInA, Vec3 axisInB) {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA, rbB);
		angularOnly = false;
		enableAngularMotor = false;

		rbAFrame.origin.set(pivotInA);

		// since no frame is given, assume this to be zero angle and just pick rb transform axis
		Vec3 rbAxisA1 = new Vec3();
		Vec3 rbAxisA2 = new Vec3();
		
		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		MatrixUtil.getColumn(centerOfMassA.basis, 0, rbAxisA1);
		float projection = axisInA.dot(rbAxisA1);

		if (projection >= 1.0f - BulletGlobals.SIMD_EPSILON) {
			MatrixUtil.getColumn(centerOfMassA.basis, 2, rbAxisA1);
			rbAxisA1.negate();
			MatrixUtil.getColumn(centerOfMassA.basis, 1, rbAxisA2);
		} else if (projection <= -1.0f + BulletGlobals.SIMD_EPSILON) {           
			MatrixUtil.getColumn(centerOfMassA.basis, 2, rbAxisA1);                            
			MatrixUtil.getColumn(centerOfMassA.basis, 1, rbAxisA2);
		} else {
			VectorUtil.cross(rbAxisA2, axisInA, rbAxisA1);                                                                
			VectorUtil.cross(rbAxisA1, rbAxisA2, axisInA);                                                                                            
		}

		MatrixUtil.setRow(rbAFrame.basis, 0, new Vec3(rbAxisA1.x, rbAxisA2.x, axisInA.x));
		MatrixUtil.setRow(rbAFrame.basis, 1, new Vec3(rbAxisA1.y, rbAxisA2.y, axisInA.y));
		MatrixUtil.setRow(rbAFrame.basis, 2, new Vec3(rbAxisA1.z, rbAxisA2.z, axisInA.z));

		Quat rotationArc = QuaternionUtil.shortestArcQuat(axisInA, axisInB, new Quat());
		Vec3 rbAxisB1 = QuaternionUtil.quatRotate(rotationArc, rbAxisA1, new Vec3());
		Vec3 rbAxisB2 = new Vec3();
		VectorUtil.cross(rbAxisB2, axisInB, rbAxisB1);

		rbBFrame.origin.set(pivotInB);
		MatrixUtil.setRow(rbBFrame.basis, 0, new Vec3(rbAxisB1.x, rbAxisB2.x, -axisInB.x));
		MatrixUtil.setRow(rbBFrame.basis, 1, new Vec3(rbAxisB1.y, rbAxisB2.y, -axisInB.y));
		MatrixUtil.setRow(rbBFrame.basis, 2, new Vec3(rbAxisB1.z, rbAxisB2.z, -axisInB.z));			

		// start with free
		lowerLimit = 1e30f;
		upperLimit = -1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;
		limitSoftness = 0.9f;
		solveLimit = false;
	}

	public HingeConstraint(RigidBody rbA, Vec3 pivotInA, Vec3 axisInA) {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA);
		angularOnly = false;
		enableAngularMotor = false;

		// since no frame is given, assume this to be zero angle and just pick rb transform axis
		// fixed axis in worldspace
		Vec3 rbAxisA1 = new Vec3();
		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		MatrixUtil.getColumn(centerOfMassA.basis, 0, rbAxisA1);

		float projection = rbAxisA1.dot(axisInA);
		if (projection > BulletGlobals.FLT_EPSILON) {
			rbAxisA1.mult(projection);
			rbAxisA1.sub(axisInA);
		}
		else {
			MatrixUtil.getColumn(centerOfMassA.basis, 1, rbAxisA1);
		}

		Vec3 rbAxisA2 = new Vec3();
		VectorUtil.cross(rbAxisA2, axisInA, rbAxisA1);

		rbAFrame.origin.set(pivotInA);
		MatrixUtil.setRow(rbAFrame.basis, 0, new Vec3(rbAxisA1.x, rbAxisA2.x, axisInA.x));
		MatrixUtil.setRow(rbAFrame.basis, 1, new Vec3(rbAxisA1.y, rbAxisA2.y, axisInA.y));
		MatrixUtil.setRow(rbAFrame.basis, 2, new Vec3(rbAxisA1.z, rbAxisA2.z, axisInA.z));

		Vec3 axisInB = new Vec3();
		VectorUtil.negate(axisInB, axisInA);
		MatrixUtil.transform(centerOfMassA.basis, axisInB);

		Quat rotationArc = QuaternionUtil.shortestArcQuat(axisInA, axisInB, new Quat());
		Vec3 rbAxisB1 = QuaternionUtil.quatRotate(rotationArc, rbAxisA1, new Vec3());
		Vec3 rbAxisB2 = new Vec3();
		VectorUtil.cross(rbAxisB2, axisInB, rbAxisB1);

		rbBFrame.origin.set(pivotInA);
		centerOfMassA.transform(rbBFrame.origin);
		MatrixUtil.setRow(rbBFrame.basis, 0, new Vec3(rbAxisB1.x, rbAxisB2.x, axisInB.x));
		MatrixUtil.setRow(rbBFrame.basis, 1, new Vec3(rbAxisB1.y, rbAxisB2.y, axisInB.y));
		MatrixUtil.setRow(rbBFrame.basis, 2, new Vec3(rbAxisB1.z, rbAxisB2.z, axisInB.z));

		// start with free
		lowerLimit = 1e30f;
		upperLimit = -1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;
		limitSoftness = 0.9f;
		solveLimit = false;
	}

	public HingeConstraint(RigidBody rbA, RigidBody rbB, Transform rbAFrame, Transform rbBFrame) {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA, rbB);
		this.rbAFrame.set(rbAFrame);
		this.rbBFrame.set(rbBFrame);
		angularOnly = false;
		enableAngularMotor = false;

		// flip axis
		this.rbBFrame.basis.c *= -1f;
		this.rbBFrame.basis.f *= -1f;
		this.rbBFrame.basis.i *= -1f;

		// start with free
		lowerLimit = 1e30f;
		upperLimit = -1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;
		limitSoftness = 0.9f;
		solveLimit = false;
	}

	public HingeConstraint(RigidBody rbA, Transform rbAFrame) {
		super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA);
		this.rbAFrame.set(rbAFrame);
		this.rbBFrame.set(rbAFrame);
		angularOnly = false;
		enableAngularMotor = false;

		// not providing rigidbody B means implicitly using worldspace for body B

		// flip axis
		this.rbBFrame.basis.c *= -1f;
		this.rbBFrame.basis.f *= -1f;
		this.rbBFrame.basis.i *= -1f;

		this.rbBFrame.origin.set(this.rbAFrame.origin);
		rbA.getCenterOfMassTransform(new Transform()).transform(this.rbBFrame.origin);

		// start with free
		lowerLimit = 1e30f;
		upperLimit = -1e30f;
		biasFactor = 0.3f;
		relaxationFactor = 1.0f;
		limitSoftness = 0.9f;
		solveLimit = false;
	}
	
	@Override
	public void buildJacobian() {
		Vec3 tmp = new Vec3();
		Vec3 tmp1 = new Vec3();
		Vec3 tmp2 = new Vec3();
		Vec3 tmpVec = new Vec3();
		Mat3 mat1 = new Mat3();
		Mat3 mat2 = new Mat3();
		
		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(new Transform());

		appliedImpulse = 0f;

		if (!angularOnly) {
			Vec3 pivotAInW = new Vec3(rbAFrame.origin);
			centerOfMassA.transform(pivotAInW);

			Vec3 pivotBInW = new Vec3(rbBFrame.origin);
			centerOfMassB.transform(pivotBInW);

			Vec3 relPos = new Vec3();
			VectorUtil.sub(relPos, pivotBInW, pivotAInW);

			Vec3[] normal/*[3]*/ = new Vec3[]{new Vec3(), new Vec3(), new Vec3()};
			if (relPos.squareLength() > BulletGlobals.FLT_EPSILON) {
				normal[0].set(relPos);
				normal[0].normalize();
			}
			else {
				normal[0].set(1f, 0f, 0f);
			}

			TransformUtil.planeSpace1(normal[0], normal[1], normal[2]);

			for (int i = 0; i < 3; i++) {
                                Mat3.transpose(centerOfMassA.basis, mat1);
                                Mat3.transpose(centerOfMassB.basis, mat2);

				VectorUtil.sub(tmp1, pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
				VectorUtil.sub(tmp2, pivotBInW, rbB.getCenterOfMassPosition(tmpVec));

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

		// calculate two perpendicular jointAxis, orthogonal to hingeAxis
		// these two jointAxis require equal angular velocities for both bodies

		// this is unused for now, it's a todo
		Vec3 jointAxis0local = new Vec3();
		Vec3 jointAxis1local = new Vec3();

		MatrixUtil.getColumn(rbAFrame.basis, 2, tmp);
		TransformUtil.planeSpace1(tmp, jointAxis0local, jointAxis1local);

		// TODO: check this
		//getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn(2);

		Vec3 jointAxis0 = new Vec3(jointAxis0local);
		MatrixUtil.transform(centerOfMassA.basis, jointAxis0);

		Vec3 jointAxis1 = new Vec3(jointAxis1local);
		MatrixUtil.transform(centerOfMassA.basis, jointAxis1);

		Vec3 hingeAxisWorld = new Vec3();
		MatrixUtil.getColumn(rbAFrame.basis, 2, hingeAxisWorld);
		MatrixUtil.transform(centerOfMassA.basis, hingeAxisWorld);

		Mat3.transpose(centerOfMassA.basis, mat1);
                Mat3.transpose(centerOfMassB.basis, mat2);
		jacAng[0].init(jointAxis0,
				mat1,
				mat2,
				rbA.getInvInertiaDiagLocal(new Vec3()),
				rbB.getInvInertiaDiagLocal(new Vec3()));

		// JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
		jacAng[1].init(jointAxis1,
				mat1,
				mat2,
				rbA.getInvInertiaDiagLocal(new Vec3()),
				rbB.getInvInertiaDiagLocal(new Vec3()));

		// JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
		jacAng[2].init(hingeAxisWorld,
				mat1,
				mat2,
				rbA.getInvInertiaDiagLocal(new Vec3()),
				rbB.getInvInertiaDiagLocal(new Vec3()));

		// Compute limit information
		float hingeAngle = getHingeAngle();

		//set bias, sign, clear accumulator
		correction = 0f;
		limitSign = 0f;
		solveLimit = false;
		accLimitImpulse = 0f;

		if (lowerLimit < upperLimit) {
			if (hingeAngle <= lowerLimit * limitSoftness) {
				correction = (lowerLimit - hingeAngle);
				limitSign = 1.0f;
				solveLimit = true;
			}
			else if (hingeAngle >= upperLimit * limitSoftness) {
				correction = upperLimit - hingeAngle;
				limitSign = -1.0f;
				solveLimit = true;
			}
		}

		// Compute K = J*W*J' for hinge axis
		Vec3 axisA = new Vec3();
		MatrixUtil.getColumn(rbAFrame.basis, 2, axisA);
		MatrixUtil.transform(centerOfMassA.basis, axisA);

		kHinge = 1.0f / (getRigidBodyA().computeAngularImpulseDenominator(axisA) +
				getRigidBodyB().computeAngularImpulseDenominator(axisA));
	}

	@Override
	public void solveConstraint(float timeStep) {
		Vec3 tmp = new Vec3();
		Vec3 tmp2 = new Vec3();
		Vec3 tmpVec = new Vec3();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(new Transform());
		
		Vec3 pivotAInW = new Vec3(rbAFrame.origin);
		centerOfMassA.transform(pivotAInW);

		Vec3 pivotBInW = new Vec3(rbBFrame.origin);
		centerOfMassB.transform(pivotBInW);

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

			// get axes in world space
			Vec3 axisA = new Vec3();
			MatrixUtil.getColumn(rbAFrame.basis, 2, axisA);
			MatrixUtil.transform(centerOfMassA.basis, axisA);

			Vec3 axisB = new Vec3();
			MatrixUtil.getColumn(rbBFrame.basis, 2, axisB);
			MatrixUtil.transform(centerOfMassB.basis, axisB);

			Vec3 angVelA = getRigidBodyA().getAngularVelocity(new Vec3());
			Vec3 angVelB = getRigidBodyB().getAngularVelocity(new Vec3());

			Vec3 angVelAroundHingeAxisA = new Vec3();
			VectorUtil.scale(angVelAroundHingeAxisA, axisA.dot(angVelA), axisA);

			Vec3 angVelAroundHingeAxisB = new Vec3();
			VectorUtil.scale(angVelAroundHingeAxisB, axisB.dot(angVelB), axisB);

			Vec3 angAorthog = new Vec3();
			VectorUtil.sub(angAorthog, angVelA, angVelAroundHingeAxisA);

			Vec3 angBorthog = new Vec3();
			VectorUtil.sub(angBorthog, angVelB, angVelAroundHingeAxisB);

			Vec3 velrelOrthog = new Vec3();
			VectorUtil.sub(velrelOrthog, angAorthog, angBorthog);

			{
				// solve orthogonal angular velocity correction
				float relaxation = 1f;
				float len = velrelOrthog.length();
				if (len > 0.00001f) {
					Vec3 normal = new Vec3();
					VectorUtil.normalize(normal, velrelOrthog);

					float denom = getRigidBodyA().computeAngularImpulseDenominator(normal) +
							getRigidBodyB().computeAngularImpulseDenominator(normal);
					// scale for mass and relaxation
					// todo:  expose this 0.9 factor to developer
					velrelOrthog.mult((1f / denom) * relaxationFactor);
				}

				// solve angular positional correction
				// TODO: check
				//Vec3 angularError = -axisA.cross(axisB) *(btScalar(1.)/timeStep);
				Vec3 angularError = new Vec3();
				VectorUtil.cross(angularError, axisA, axisB);
				angularError.negate();
				angularError.mult(1f / timeStep);
				float len2 = angularError.length();
				if (len2 > 0.00001f) {
					Vec3 normal2 = new Vec3();
					VectorUtil.normalize(normal2, angularError);

					float denom2 = getRigidBodyA().computeAngularImpulseDenominator(normal2) +
							getRigidBodyB().computeAngularImpulseDenominator(normal2);
					angularError.mult((1f / denom2) * relaxation);
				}

				VectorUtil.negate(tmp, velrelOrthog);
				tmp.add(angularError);
				rbA.applyTorqueImpulse(tmp);

				VectorUtil.sub(tmp, velrelOrthog, angularError);
				rbB.applyTorqueImpulse(tmp);

				// solve limit
				if (solveLimit) {
					VectorUtil.sub(tmp, angVelB, angVelA);
					float amplitude = ((tmp).dot(axisA) * relaxationFactor + correction * (1f / timeStep) * biasFactor) * limitSign;

					float impulseMag = amplitude * kHinge;

					// Clamp the accumulated impulse
					float temp = accLimitImpulse;
					accLimitImpulse = Math.max(accLimitImpulse + impulseMag, 0f);
					impulseMag = accLimitImpulse - temp;

					Vec3 impulse = new Vec3();
					VectorUtil.scale(impulse, impulseMag * limitSign, axisA);

					rbA.applyTorqueImpulse(impulse);

					VectorUtil.negate(tmp, impulse);
					rbB.applyTorqueImpulse(tmp);
				}
			}

			// apply motor
			if (enableAngularMotor) {
				// todo: add limits too
				Vec3 angularLimit = new Vec3();
				angularLimit.set(0f, 0f, 0f);

				Vec3 velrel = new Vec3();
				VectorUtil.sub(velrel, angVelAroundHingeAxisA, angVelAroundHingeAxisB);
				float projRelVel = velrel.dot(axisA);

				float desiredMotorVel = motorTargetVelocity;
				float motor_relvel = desiredMotorVel - projRelVel;

				float unclippedMotorImpulse = kHinge * motor_relvel;
				// todo: should clip against accumulated impulse
				float clippedMotorImpulse = unclippedMotorImpulse > maxMotorImpulse ? maxMotorImpulse : unclippedMotorImpulse;
				clippedMotorImpulse = clippedMotorImpulse < -maxMotorImpulse ? -maxMotorImpulse : clippedMotorImpulse;
				Vec3 motorImp = new Vec3();
				VectorUtil.scale(motorImp, clippedMotorImpulse, axisA);

				VectorUtil.add(tmp, motorImp, angularLimit);
				rbA.applyTorqueImpulse(tmp);

				VectorUtil.negate(tmp, motorImp);
				tmp.sub(angularLimit);
				rbB.applyTorqueImpulse(tmp);
			}
		}
	}

	public void updateRHS(float timeStep) {
	}

	public float getHingeAngle() {
		Transform centerOfMassA = rbA.getCenterOfMassTransform(new Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(new Transform());
		
		Vec3 refAxis0 = new Vec3();
		MatrixUtil.getColumn(rbAFrame.basis, 0, refAxis0);
		MatrixUtil.transform(centerOfMassA.basis, refAxis0);

		Vec3 refAxis1 = new Vec3();
		MatrixUtil.getColumn(rbAFrame.basis, 1, refAxis1);
		MatrixUtil.transform(centerOfMassA.basis, refAxis1);

		Vec3 swingAxis = new Vec3();
		MatrixUtil.getColumn(rbBFrame.basis, 1, swingAxis);
		MatrixUtil.transform(centerOfMassB.basis, swingAxis);

		return ScalarUtil.atan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
	}
	
	public void setAngularOnly(boolean angularOnly) {
		this.angularOnly = angularOnly;
	}

	public void enableAngularMotor(boolean enableMotor, float targetVelocity, float maxMotorImpulse) {
		this.enableAngularMotor = enableMotor;
		this.motorTargetVelocity = targetVelocity;
		this.maxMotorImpulse = maxMotorImpulse;
	}

	public void setLimit(float low, float high) {
		setLimit(low, high, 0.9f, 0.3f, 1.0f);
	}

	public void setLimit(float low, float high, float _softness, float _biasFactor, float _relaxationFactor) {
		lowerLimit = low;
		upperLimit = high;

		limitSoftness = _softness;
		biasFactor = _biasFactor;
		relaxationFactor = _relaxationFactor;
	}

	public float getLowerLimit() {
		return lowerLimit;
	}

	public float getUpperLimit() {
		return upperLimit;
	}

	public Transform getAFrame(Transform out) {
		out.set(rbAFrame);
		return out;
	}

	public Transform getBFrame(Transform out) {
		out.set(rbBFrame);
		return out;
	}

	public boolean getSolveLimit() {
		return solveLimit;
	}

	public float getLimitSign() {
		return limitSign;
	}

	public boolean getAngularOnly() {
		return angularOnly;
	}

	public boolean getEnableAngularMotor() {
		return enableAngularMotor;
	}

	public float getMotorTargetVelosity() {
		return motorTargetVelocity;
	}

	public float getMaxMotorImpulse() {
		return maxMotorImpulse;
	}
	
}
