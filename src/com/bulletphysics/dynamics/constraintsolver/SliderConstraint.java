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
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

TODO:
 - add clamping od accumulated impulse to improve stability
 - add conversion for ODE constraint solver
*/

package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import javax.vecmath.Mat3;
import javax.vecmath.Vec3;

// JAVA NOTE: SliderConstraint from 2.71

/**
 *
 * @author jezek2
 */
public class SliderConstraint extends TypedConstraint {
	
	public static final float SLIDER_CONSTRAINT_DEF_SOFTNESS    = 1.0f;
	public static final float SLIDER_CONSTRAINT_DEF_DAMPING     = 1.0f;
	public static final float SLIDER_CONSTRAINT_DEF_RESTITUTION = 0.7f;
	
	protected final Transform frameInA = new Transform();
	protected final Transform frameInB = new Transform();
	// use frameA fo define limits, if true
	protected boolean useLinearReferenceFrameA;
	// linear limits
	protected float lowerLinLimit;
	protected float upperLinLimit;
	// angular limits
	protected float lowerAngLimit;
	protected float upperAngLimit;
	// softness, restitution and damping for different cases
	// DirLin - moving inside linear limits
	// LimLin - hitting linear limit
	// DirAng - moving inside angular limits
	// LimAng - hitting angular limit
	// OrthoLin, OrthoAng - against constraint axis
	protected float softnessDirLin;
	protected float restitutionDirLin;
	protected float dampingDirLin;
	protected float softnessDirAng;
	protected float restitutionDirAng;
	protected float dampingDirAng;
	protected float softnessLimLin;
	protected float restitutionLimLin;
	protected float dampingLimLin;
	protected float softnessLimAng;
	protected float restitutionLimAng;
	protected float dampingLimAng;
	protected float softnessOrthoLin;
	protected float restitutionOrthoLin;
	protected float dampingOrthoLin;
	protected float softnessOrthoAng;
	protected float restitutionOrthoAng;
	protected float dampingOrthoAng;
	
	// for interlal use
	protected boolean solveLinLim;
	protected boolean solveAngLim;

	protected JacobianEntry[] jacLin = new JacobianEntry[/*3*/] { new JacobianEntry(), new JacobianEntry(), new JacobianEntry() };
	protected float[] jacLinDiagABInv = new float[3];

	protected JacobianEntry[] jacAng = new JacobianEntry[/*3*/] { new JacobianEntry(), new JacobianEntry(), new JacobianEntry() };

	protected float timeStep;
	protected final Transform calculatedTransformA = new Transform();
	protected final Transform calculatedTransformB = new Transform();

	protected final Vec3 sliderAxis = new Vec3();
	protected final Vec3 realPivotAInW = new Vec3();
	protected final Vec3 realPivotBInW = new Vec3();
	protected final Vec3 projPivotInW = new Vec3();
	protected final Vec3 delta = new Vec3();
	protected final Vec3 depth = new Vec3();
	protected final Vec3 relPosA = new Vec3();
	protected final Vec3 relPosB = new Vec3();

	protected float linPos;

	protected float angDepth;
	protected float kAngle;

	protected boolean poweredLinMotor;
	protected float targetLinMotorVelocity;
	protected float maxLinMotorForce;
	protected float accumulatedLinMotorImpulse;
	
	protected boolean poweredAngMotor;
	protected float targetAngMotorVelocity;
	protected float maxAngMotorForce;
	protected float accumulatedAngMotorImpulse;

    public SliderConstraint() {
		super(TypedConstraintType.SLIDER_CONSTRAINT_TYPE);
		useLinearReferenceFrameA = true;
		initParams();
	}

    public SliderConstraint(RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB ,boolean useLinearReferenceFrameA) {
		super(TypedConstraintType.SLIDER_CONSTRAINT_TYPE, rbA, rbB);
        this.frameInA.set(frameInA);
        this.frameInB.set(frameInB);
		this.useLinearReferenceFrameA = useLinearReferenceFrameA;
		initParams();
	}

	protected void initParams() {
		lowerLinLimit = 1f;
		upperLinLimit = -1f;
		lowerAngLimit = 0f;
		upperAngLimit = 0f;
		softnessDirLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionDirLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingDirLin = 0f;
		softnessDirAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionDirAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingDirAng = 0f;
		softnessOrthoLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionOrthoLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingOrthoLin = SLIDER_CONSTRAINT_DEF_DAMPING;
		softnessOrthoAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionOrthoAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingOrthoAng = SLIDER_CONSTRAINT_DEF_DAMPING;
		softnessLimLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionLimLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingLimLin = SLIDER_CONSTRAINT_DEF_DAMPING;
		softnessLimAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionLimAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingLimAng = SLIDER_CONSTRAINT_DEF_DAMPING;

		poweredLinMotor = false;
		targetLinMotorVelocity = 0f;
		maxLinMotorForce = 0f;
		accumulatedLinMotorImpulse = 0f;

		poweredAngMotor = false;
		targetAngMotorVelocity = 0f;
		maxAngMotorForce = 0f;
		accumulatedAngMotorImpulse = 0f;
	}

	@Override
	public void buildJacobian() {
		if (useLinearReferenceFrameA) {
			buildJacobianInt(rbA, rbB, frameInA, frameInB);
		}
		else {
			buildJacobianInt(rbB, rbA, frameInB, frameInA);
		}
	}

	@Override
	public void solveConstraint(float timeStep) {
		this.timeStep = timeStep;
		if (useLinearReferenceFrameA) {
			solveConstraintInt(rbA, rbB);
		}
		else {
			solveConstraintInt(rbB, rbA);
		}
	}
	
	public Transform getCalculatedTransformA(Transform out) {
		out.set(calculatedTransformA);
		return out;
	}
	
	public Transform getCalculatedTransformB(Transform out) {
		out.set(calculatedTransformB);
		return out;
	}
	
	public Transform getFrameOffsetA(Transform out) {
		out.set(frameInA);
		return out;
	}

	public Transform getFrameOffsetB(Transform out) {
		out.set(frameInB);
		return out;
	}
	
	public float getLowerLinLimit() {
		return lowerLinLimit;
	}

	public void setLowerLinLimit(float lowerLimit) {
		this.lowerLinLimit = lowerLimit;
	}

	public float getUpperLinLimit() {
		return upperLinLimit;
	}

	public void setUpperLinLimit(float upperLimit) {
		this.upperLinLimit = upperLimit;
	}

	public float getLowerAngLimit() {
		return lowerAngLimit;
	}

	public void setLowerAngLimit(float lowerLimit) {
		this.lowerAngLimit = lowerLimit;
	}

	public float getUpperAngLimit() {
		return upperAngLimit;
	}

	public void setUpperAngLimit(float upperLimit) {
		this.upperAngLimit = upperLimit;
	}

	public boolean getUseLinearReferenceFrameA() {
		return useLinearReferenceFrameA;
	}
	
	public float getSoftnessDirLin() {
		return softnessDirLin;
	}

	public float getRestitutionDirLin() {
		return restitutionDirLin;
	}

	public float getDampingDirLin() {
		return dampingDirLin;
	}

	public float getSoftnessDirAng() {
		return softnessDirAng;
	}

	public float getRestitutionDirAng() {
		return restitutionDirAng;
	}

	public float getDampingDirAng() {
		return dampingDirAng;
	}

	public float getSoftnessLimLin() {
		return softnessLimLin;
	}

	public float getRestitutionLimLin() {
		return restitutionLimLin;
	}

	public float getDampingLimLin() {
		return dampingLimLin;
	}

	public float getSoftnessLimAng() {
		return softnessLimAng;
	}

	public float getRestitutionLimAng() {
		return restitutionLimAng;
	}

	public float getDampingLimAng() {
		return dampingLimAng;
	}

	public float getSoftnessOrthoLin() {
		return softnessOrthoLin;
	}

	public float getRestitutionOrthoLin() {
		return restitutionOrthoLin;
	}

	public float getDampingOrthoLin() {
		return dampingOrthoLin;
	}

	public float getSoftnessOrthoAng() {
		return softnessOrthoAng;
	}

	public float getRestitutionOrthoAng() {
		return restitutionOrthoAng;
	}

	public float getDampingOrthoAng() {
		return dampingOrthoAng;
	}
	
	public void setSoftnessDirLin(float softnessDirLin) {
		this.softnessDirLin = softnessDirLin;
	}

	public void setRestitutionDirLin(float restitutionDirLin) {
		this.restitutionDirLin = restitutionDirLin;
	}

	public void setDampingDirLin(float dampingDirLin) {
		this.dampingDirLin = dampingDirLin;
	}

	public void setSoftnessDirAng(float softnessDirAng) {
		this.softnessDirAng = softnessDirAng;
	}

	public void setRestitutionDirAng(float restitutionDirAng) {
		this.restitutionDirAng = restitutionDirAng;
	}

	public void setDampingDirAng(float dampingDirAng) {
		this.dampingDirAng = dampingDirAng;
	}

	public void setSoftnessLimLin(float softnessLimLin) {
		this.softnessLimLin = softnessLimLin;
	}

	public void setRestitutionLimLin(float restitutionLimLin) {
		this.restitutionLimLin = restitutionLimLin;
	}

	public void setDampingLimLin(float dampingLimLin) {
		this.dampingLimLin = dampingLimLin;
	}

	public void setSoftnessLimAng(float softnessLimAng) {
		this.softnessLimAng = softnessLimAng;
	}

	public void setRestitutionLimAng(float restitutionLimAng) {
		this.restitutionLimAng = restitutionLimAng;
	}

	public void setDampingLimAng(float dampingLimAng) {
		this.dampingLimAng = dampingLimAng;
	}

	public void setSoftnessOrthoLin(float softnessOrthoLin) {
		this.softnessOrthoLin = softnessOrthoLin;
	}

	public void setRestitutionOrthoLin(float restitutionOrthoLin) {
		this.restitutionOrthoLin = restitutionOrthoLin;
	}

	public void setDampingOrthoLin(float dampingOrthoLin) {
		this.dampingOrthoLin = dampingOrthoLin;
	}

	public void setSoftnessOrthoAng(float softnessOrthoAng) {
		this.softnessOrthoAng = softnessOrthoAng;
	}

	public void setRestitutionOrthoAng(float restitutionOrthoAng) {
		this.restitutionOrthoAng = restitutionOrthoAng;
	}

	public void setDampingOrthoAng(float dampingOrthoAng) {
		this.dampingOrthoAng = dampingOrthoAng;
	}

	public void setPoweredLinMotor(boolean onOff) {
		this.poweredLinMotor = onOff;
	}

	public boolean getPoweredLinMotor() {
		return poweredLinMotor;
	}

	public void setTargetLinMotorVelocity(float targetLinMotorVelocity) {
		this.targetLinMotorVelocity = targetLinMotorVelocity;
	}

	public float getTargetLinMotorVelocity() {
		return targetLinMotorVelocity;
	}

	public void setMaxLinMotorForce(float maxLinMotorForce) {
		this.maxLinMotorForce = maxLinMotorForce;
	}

	public float getMaxLinMotorForce() {
		return maxLinMotorForce;
	}

	public void setPoweredAngMotor(boolean onOff) {
		this.poweredAngMotor = onOff;
	}

	public boolean getPoweredAngMotor() {
		return poweredAngMotor;
	}

	public void setTargetAngMotorVelocity(float targetAngMotorVelocity) {
		this.targetAngMotorVelocity = targetAngMotorVelocity;
	}

	public float getTargetAngMotorVelocity() {
		return targetAngMotorVelocity;
	}

	public void setMaxAngMotorForce(float maxAngMotorForce) {
		this.maxAngMotorForce = maxAngMotorForce;
	}

	public float getMaxAngMotorForce() {
		return this.maxAngMotorForce;
	}

	public float getLinearPos() {
		return this.linPos;
	}

	// access for ODE solver

	public boolean getSolveLinLimit() {
		return solveLinLim;
	}

	public float getLinDepth() {
		return depth.x;
	}

	public boolean getSolveAngLimit() {
		return solveAngLim;
	}

	public float getAngDepth() {
		return angDepth;
	}
	
	// internal
	
	public void buildJacobianInt(RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB) {
		Transform tmpTrans = new Transform();
		Transform tmpTrans1 = new Transform();
		Transform tmpTrans2 = new Transform();
		Vec3 tmp = new Vec3();
		Vec3 tmp2 = new Vec3();

		// calculate transforms
		calculatedTransformA.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
		calculatedTransformB.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
		realPivotAInW.set(calculatedTransformA.origin);
		realPivotBInW.set(calculatedTransformB.origin);
		calculatedTransformA.basis.getColumn(0, tmp);
		sliderAxis.set(tmp); // along X
		delta.subHere(realPivotBInW, realPivotAInW);
		projPivotInW.scaleAddHere(sliderAxis.dot(delta), sliderAxis, realPivotAInW);
		relPosA.subHere(projPivotInW, rbA.getCenterOfMassPosition(tmp));
		relPosB.subHere(realPivotBInW, rbB.getCenterOfMassPosition(tmp));
		Vec3 normalWorld = new Vec3();

		// linear part
		for (int i=0; i<3; i++) {
			calculatedTransformA.basis.getColumn(i, normalWorld);

			Mat3 mat1 = rbA.getCenterOfMassTransform(tmpTrans1).basis;
			mat1.transpose();

			Mat3 mat2 = rbB.getCenterOfMassTransform(tmpTrans2).basis;
			mat2.transpose();

			jacLin[i].init(
					mat1,
					mat2,
					relPosA,
					relPosB,
					normalWorld,
					rbA.getInvInertiaDiagLocal(tmp),
					rbA.getInvMass(),
					rbB.getInvInertiaDiagLocal(tmp2),
					rbB.getInvMass());
			jacLinDiagABInv[i] = 1f / jacLin[i].getDiagonal();
			VectorUtil.setCoord(depth, i, delta.dot(normalWorld));
		}
		testLinLimits();

		// angular part
		for (int i=0; i<3; i++) {
			calculatedTransformA.basis.getColumn(i, normalWorld);

			Mat3 mat1 = rbA.getCenterOfMassTransform(tmpTrans1).basis;
			mat1.transpose();

			Mat3 mat2 = rbB.getCenterOfMassTransform(tmpTrans2).basis;
			mat2.transpose();

			jacAng[i].init(
					normalWorld,
					mat1,
					mat2,
					rbA.getInvInertiaDiagLocal(tmp),
					rbB.getInvInertiaDiagLocal(tmp2));
		}
		testAngLimits();

		Vec3 axisA = new Vec3();
		calculatedTransformA.basis.getColumn(0, axisA);
		kAngle = 1f / (rbA.computeAngularImpulseDenominator(axisA) + rbB.computeAngularImpulseDenominator(axisA));
		// clear accumulator for motors
		accumulatedLinMotorImpulse = 0f;
		accumulatedAngMotorImpulse = 0f;
	}
	
	public void solveConstraintInt(RigidBody rbA, RigidBody rbB) {
		Vec3 tmp = new Vec3();

		// linear
		Vec3 velA = rbA.getVelocityInLocalPoint(relPosA, new Vec3());
		Vec3 velB = rbB.getVelocityInLocalPoint(relPosB, new Vec3());
		Vec3 vel = new Vec3();
		vel.subHere(velA, velB);

		Vec3 impulse_vector = new Vec3();

		for (int i=0; i<3; i++) {
			Vec3 normal = jacLin[i].linearJointAxis;
			float rel_vel = normal.dot(vel);
			// calculate positional error
			float depth = VectorUtil.getCoord(this.depth, i);
			// get parameters
			float softness = (i != 0)? softnessOrthoLin : (solveLinLim? softnessLimLin : softnessDirLin);
			float restitution = (i != 0)? restitutionOrthoLin : (solveLinLim? restitutionLimLin : restitutionDirLin);
			float damping = (i != 0)? dampingOrthoLin : (solveLinLim? dampingLimLin : dampingDirLin);
			// calcutate and apply impulse
			float normalImpulse = softness * (restitution * depth / timeStep - damping * rel_vel) * jacLinDiagABInv[i];
			impulse_vector.scale(normalImpulse, normal);
			rbA.applyImpulse(impulse_vector, relPosA);
			tmp.negateHere(impulse_vector);
			rbB.applyImpulse(tmp, relPosB);

			if (poweredLinMotor && (i == 0)) {
				// apply linear motor
				if (accumulatedLinMotorImpulse < maxLinMotorForce) {
					float desiredMotorVel = targetLinMotorVelocity;
					float motor_relvel = desiredMotorVel + rel_vel;
					normalImpulse = -motor_relvel * jacLinDiagABInv[i];
					// clamp accumulated impulse
					float new_acc = accumulatedLinMotorImpulse + Math.abs(normalImpulse);
					if (new_acc > maxLinMotorForce) {
						new_acc = maxLinMotorForce;
					}
					float del = new_acc - accumulatedLinMotorImpulse;
					if (normalImpulse < 0f) {
						normalImpulse = -del;
					}
					else {
						normalImpulse = del;
					}
					accumulatedLinMotorImpulse = new_acc;
					// apply clamped impulse
					impulse_vector.scale(normalImpulse, normal);
					rbA.applyImpulse(impulse_vector, relPosA);
					tmp.negateHere(impulse_vector);
					rbB.applyImpulse(tmp, relPosB);
				}
			}
		}

		// angular
		// get axes in world space
		Vec3 axisA = new Vec3();
		calculatedTransformA.basis.getColumn(0, axisA);
		Vec3 axisB = new Vec3();
		calculatedTransformB.basis.getColumn(0, axisB);

		Vec3 angVelA = rbA.getAngularVelocity(new Vec3());
		Vec3 angVelB = rbB.getAngularVelocity(new Vec3());

		Vec3 angVelAroundAxisA = new Vec3();
		angVelAroundAxisA.scale(axisA.dot(angVelA), axisA);
		Vec3 angVelAroundAxisB = new Vec3();
		angVelAroundAxisB.scale(axisB.dot(angVelB), axisB);

		Vec3 angAorthog = new Vec3();
		angAorthog.subHere(angVelA, angVelAroundAxisA);
		Vec3 angBorthog = new Vec3();
		angBorthog.subHere(angVelB, angVelAroundAxisB);
		Vec3 velrelOrthog = new Vec3();
		velrelOrthog.subHere(angAorthog, angBorthog);

		// solve orthogonal angular velocity correction
		float len = velrelOrthog.length();
		if (len > 0.00001f) {
			Vec3 normal = new Vec3();
			normal.normalizeHere(velrelOrthog);
			float denom = rbA.computeAngularImpulseDenominator(normal) + rbB.computeAngularImpulseDenominator(normal);
			velrelOrthog.mult((1f / denom) * dampingOrthoAng * softnessOrthoAng);
		}

		// solve angular positional correction
		Vec3 angularError = new Vec3();
		angularError.crossHere(axisA, axisB);
		angularError.mult(1f / timeStep);
		float len2 = angularError.length();
		if (len2 > 0.00001f) {
			Vec3 normal2 = new Vec3();
			normal2.normalizeHere(angularError);
			float denom2 = rbA.computeAngularImpulseDenominator(normal2) + rbB.computeAngularImpulseDenominator(normal2);
			angularError.mult((1f / denom2) * restitutionOrthoAng * softnessOrthoAng);
		}

		// apply impulse
		tmp.negateHere(velrelOrthog);
		tmp.add(angularError);
		rbA.applyTorqueImpulse(tmp);
		tmp.subHere(velrelOrthog, angularError);
		rbB.applyTorqueImpulse(tmp);
		float impulseMag;

		// solve angular limits
		if (solveAngLim) {
			tmp.subHere(angVelB, angVelA);
			impulseMag = tmp.dot(axisA) * dampingLimAng + angDepth * restitutionLimAng / timeStep;
			impulseMag *= kAngle * softnessLimAng;
		}
		else {
			tmp.subHere(angVelB, angVelA);
			impulseMag = tmp.dot(axisA) * dampingDirAng + angDepth * restitutionDirAng / timeStep;
			impulseMag *= kAngle * softnessDirAng;
		}
		Vec3 impulse = new Vec3();
		impulse.scale(impulseMag, axisA);
		rbA.applyTorqueImpulse(impulse);
		tmp.negateHere(impulse);
		rbB.applyTorqueImpulse(tmp);

		// apply angular motor
		if (poweredAngMotor) {
			if (accumulatedAngMotorImpulse < maxAngMotorForce) {
				Vec3 velrel = new Vec3();
				velrel.subHere(angVelAroundAxisA, angVelAroundAxisB);
				float projRelVel = velrel.dot(axisA);

				float desiredMotorVel = targetAngMotorVelocity;
				float motor_relvel = desiredMotorVel - projRelVel;

				float angImpulse = kAngle * motor_relvel;
				// clamp accumulated impulse
				float new_acc = accumulatedAngMotorImpulse + Math.abs(angImpulse);
				if (new_acc > maxAngMotorForce) {
					new_acc = maxAngMotorForce;
				}
				float del = new_acc - accumulatedAngMotorImpulse;
				if (angImpulse < 0f) {
					angImpulse = -del;
				} else {
					angImpulse = del;
				}
				accumulatedAngMotorImpulse = new_acc;

				// apply clamped impulse
				Vec3 motorImp = new Vec3();
				motorImp.scale(angImpulse, axisA);
				rbA.applyTorqueImpulse(motorImp);
				tmp.negateHere(motorImp);
				rbB.applyTorqueImpulse(tmp);
			}
		}
	}
	
	// shared code used by ODE solver
	
	public void calculateTransforms() {
		Transform tmpTrans = new Transform();

		if (useLinearReferenceFrameA) {
			calculatedTransformA.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
			calculatedTransformB.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
		}
		else {
			calculatedTransformA.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
			calculatedTransformB.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
		}
		realPivotAInW.set(calculatedTransformA.origin);
		realPivotBInW.set(calculatedTransformB.origin);
		calculatedTransformA.basis.getColumn(0, sliderAxis); // along X
		delta.subHere(realPivotBInW, realPivotAInW);
		projPivotInW.scaleAddHere(sliderAxis.dot(delta), sliderAxis, realPivotAInW);
		Vec3 normalWorld = new Vec3();
		// linear part
		for (int i=0; i<3; i++) {
			calculatedTransformA.basis.getColumn(i, normalWorld);
			VectorUtil.setCoord(depth, i, delta.dot(normalWorld));
		}
	}

	public void testLinLimits() {
		solveLinLim = false;
		linPos = depth.x;
		if (lowerLinLimit <= upperLinLimit) {
			if (depth.x > upperLinLimit) {
				depth.x -= upperLinLimit;
				solveLinLim = true;
			}
			else if (depth.x < lowerLinLimit) {
				depth.x -= lowerLinLimit;
				solveLinLim = true;
			}
			else {
				depth.x = 0f;
			}
		}
		else {
			depth.x = 0f;
		}
	}
	
	public void testAngLimits() {
		angDepth = 0f;
		solveAngLim = false;
		if (lowerAngLimit <= upperAngLimit) {
			Vec3 axisA0 = new Vec3();
			calculatedTransformA.basis.getColumn(1, axisA0);
			Vec3 axisA1 = new Vec3();
			calculatedTransformA.basis.getColumn(2, axisA1);
			Vec3 axisB0 = new Vec3();
			calculatedTransformB.basis.getColumn(1, axisB0);

			float rot = (float) Math.atan2(axisB0.dot(axisA1), axisB0.dot(axisA0));
			if (rot < lowerAngLimit) {
				angDepth = rot - lowerAngLimit;
				solveAngLim = true;
			}
			else if (rot > upperAngLimit) {
				angDepth = rot - upperAngLimit;
				solveAngLim = true;
			}
		}
	}
	
	// access for PE Solver
	
	public Vec3 getAncorInA(Vec3 out) {
		Transform tmpTrans = new Transform();

		Vec3 ancorInA = out;
		ancorInA.scaleAddHere((lowerLinLimit + upperLinLimit) * 0.5f, sliderAxis, realPivotAInW);
		rbA.getCenterOfMassTransform(tmpTrans);
		tmpTrans.inverse();
		tmpTrans.transform(ancorInA);
		return ancorInA;
	}

	public Vec3 getAncorInB(Vec3 out) {
		Vec3 ancorInB = out;
		ancorInB.set(frameInB.origin);
		return ancorInB;
	}

}
