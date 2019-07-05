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

package com.bulletphysics.dynamics;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseProxy;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionObjectType;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.util.ObjectArrayList;
import com.samrj.devil.math.Mat3;
import com.samrj.devil.math.Quat;
import javax.vecmath.Vec3;

/**
 * RigidBody is the main class for rigid body objects. It is derived from
 * {@link CollisionObject}, so it keeps reference to {@link CollisionShape}.<p>
 * 
 * It is recommended for performance and memory use to share {@link CollisionShape}
 * objects whenever possible.<p>
 * 
 * There are 3 types of rigid bodies:<br>
 * <ol>
 * <li>Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.</li>
 * <li>Fixed objects with zero mass. They are not moving (basically collision objects).</li>
 * <li>Kinematic objects, which are objects without mass, but the user can move them. There
 *     is on-way interaction, and Bullet calculates a velocity based on the timestep and
 *     previous and current world transform.</li>
 * </ol>
 * 
 * Bullet automatically deactivates dynamic rigid bodies, when the velocity is below
 * a threshold for a given time.<p>
 * 
 * Deactivated (sleeping) rigid bodies don't take any processing time, except a minor
 * broadphase collision detection impact (to allow active objects to activate/wake up
 * sleeping objects).
 * 
 * @author jezek2
 */
public class RigidBody extends CollisionObject {

	private static final float MAX_ANGVEL = BulletGlobals.SIMD_HALF_PI;
	
	private final Mat3 invInertiaTensorWorld = new Mat3();
	private final Vec3 linearVelocity = new Vec3();
	private final Vec3 angularVelocity = new Vec3();
	private float inverseMass;
	private float angularFactor;

	private final Vec3 gravity = new Vec3();
	private final Vec3 invInertiaLocal = new Vec3();
	private final Vec3 totalForce = new Vec3();
	private final Vec3 totalTorque = new Vec3();
	
	private float linearDamping;
	private float angularDamping;

	private boolean additionalDamping;
	private float additionalDampingFactor;
	private float additionalLinearDampingThresholdSqr;
	private float additionalAngularDampingThresholdSqr;
	private float additionalAngularDampingFactor;

	private float linearSleepingThreshold;
	private float angularSleepingThreshold;

	// optionalMotionState allows to automatic synchronize the world transform for active objects
	private MotionState optionalMotionState;

	// keep track of typed constraints referencing this rigid body
	private final ObjectArrayList<TypedConstraint> constraintRefs = new ObjectArrayList<TypedConstraint>();

	// for experimental overriding of friction/contact solver func
	public int contactSolverType;
	public int frictionSolverType;
	
	private static int uniqueId = 0;
	public int debugBodyId;
	
	public RigidBody(RigidBodyConstructionInfo constructionInfo) {
		setupRigidBody(constructionInfo);
	}

	public RigidBody(float mass, MotionState motionState, CollisionShape collisionShape) {
		this(mass, motionState, collisionShape, new Vec3(0f, 0f, 0f));
	}
	
	public RigidBody(float mass, MotionState motionState, CollisionShape collisionShape, Vec3 localInertia) {
		RigidBodyConstructionInfo cinfo = new RigidBodyConstructionInfo(mass, motionState, collisionShape, localInertia);
		setupRigidBody(cinfo);
	}
	
	private void setupRigidBody(RigidBodyConstructionInfo constructionInfo) {
		internalType = CollisionObjectType.RIGID_BODY;
		
		linearVelocity.set(0f, 0f, 0f);
		angularVelocity.set(0f, 0f, 0f);
		angularFactor = 1f;
		gravity.set(0f, 0f, 0f);
		totalForce.set(0f, 0f, 0f);
		totalTorque.set(0f, 0f, 0f);
		linearDamping = 0f;
		angularDamping = 0.5f;
		linearSleepingThreshold = constructionInfo.linearSleepingThreshold;
		angularSleepingThreshold = constructionInfo.angularSleepingThreshold;
		optionalMotionState = constructionInfo.motionState;
		contactSolverType = 0;
		frictionSolverType = 0;
		additionalDamping = constructionInfo.additionalDamping;
		additionalDampingFactor = constructionInfo.additionalDampingFactor;
		additionalLinearDampingThresholdSqr = constructionInfo.additionalLinearDampingThresholdSqr;
		additionalAngularDampingThresholdSqr = constructionInfo.additionalAngularDampingThresholdSqr;
		additionalAngularDampingFactor = constructionInfo.additionalAngularDampingFactor;

		if (optionalMotionState != null)
		{
			optionalMotionState.getWorldTransform(worldTransform);
		} else
		{
			worldTransform.set(constructionInfo.startWorldTransform);
		}

		interpolationWorldTransform.set(worldTransform);
		interpolationLinearVelocity.set(0f, 0f, 0f);
		interpolationAngularVelocity.set(0f, 0f, 0f);

		// moved to CollisionObject
		friction = constructionInfo.friction;
		restitution = constructionInfo.restitution;

		setCollisionShape(constructionInfo.collisionShape);
		debugBodyId = uniqueId++;

		setMassProps(constructionInfo.mass, constructionInfo.localInertia);
		setDamping(constructionInfo.linearDamping, constructionInfo.angularDamping);
		updateInertiaTensor();
	}
	
	public void destroy() {
		// No constraints should point to this rigidbody
		// Remove constraints from the dynamics world before you delete the related rigidbodies. 
		assert (constraintRefs.size() == 0);
	}

	public void proceedToTransform(Transform newTrans) {
		setCenterOfMassTransform(newTrans);
	}
	
	/**
	 * To keep collision detection and dynamics separate we don't store a rigidbody pointer,
	 * but a rigidbody is derived from CollisionObject, so we can safely perform an upcast.
	 */
	public static RigidBody upcast(CollisionObject colObj) {
		if (colObj.getInternalType() == CollisionObjectType.RIGID_BODY) {
			return (RigidBody)colObj;
		}
		return null;
	}

	/**
	 * Continuous collision detection needs prediction.
	 */
	public void predictIntegratedTransform(float timeStep, Transform predictedTransform) {
		TransformUtil.integrateTransform(worldTransform, linearVelocity, angularVelocity, timeStep, predictedTransform);
	}
	
	public void saveKinematicState(float timeStep) {
		//todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
		if (timeStep != 0f) {
			//if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
			if (getMotionState() != null) {
				getMotionState().getWorldTransform(worldTransform);
			}
			//Vec3 linVel = new Vec3(), angVel = new Vec3();

			TransformUtil.calculateVelocity(interpolationWorldTransform, worldTransform, timeStep, linearVelocity, angularVelocity);
			interpolationLinearVelocity.set(linearVelocity);
			interpolationAngularVelocity.set(angularVelocity);
			interpolationWorldTransform.set(worldTransform);
		//printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
		}
	}
	
	public void applyGravity() {
		if (isStaticOrKinematicObject())
			return;

		applyCentralForce(gravity);
	}
	
	public void setGravity(Vec3 acceleration) {
		if (inverseMass != 0f) {
			gravity.scale(1f / inverseMass, acceleration);
		}
	}

	public Vec3 getGravity(Vec3 out) {
		out.set(gravity);
		return out;
	}

	public void setDamping(float lin_damping, float ang_damping) {
		linearDamping = MiscUtil.GEN_clamped(lin_damping, 0f, 1f);
		angularDamping = MiscUtil.GEN_clamped(ang_damping, 0f, 1f);
	}

	public float getLinearDamping() {
		return linearDamping;
	}

	public float getAngularDamping() {
		return angularDamping;
	}

	public float getLinearSleepingThreshold() {
		return linearSleepingThreshold;
	}

	public float getAngularSleepingThreshold() {
		return angularSleepingThreshold;
	}

	/**
	 * Damps the velocity, using the given linearDamping and angularDamping.
	 */
	public void applyDamping(float timeStep) {
		// On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
		// todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway

		//#define USE_OLD_DAMPING_METHOD 1
		//#ifdef USE_OLD_DAMPING_METHOD
		//linearVelocity.scale(MiscUtil.GEN_clamped((1f - timeStep * linearDamping), 0f, 1f));
		//angularVelocity.scale(MiscUtil.GEN_clamped((1f - timeStep * angularDamping), 0f, 1f));
		//#else
		linearVelocity.mult((float)Math.pow(1f - linearDamping, timeStep));
		angularVelocity.mult((float)Math.pow(1f - angularDamping, timeStep));
		//#endif

		if (additionalDamping) {
			// Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
			// Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
			if ((angularVelocity.squareLength() < additionalAngularDampingThresholdSqr) &&
					(linearVelocity.squareLength() < additionalLinearDampingThresholdSqr)) {
				angularVelocity.mult(additionalDampingFactor);
				linearVelocity.mult(additionalDampingFactor);
			}

			float speed = linearVelocity.length();
			if (speed < linearDamping) {
				float dampVel = 0.005f;
				if (speed > dampVel) {
					Vec3 dir = new Vec3(linearVelocity);
					dir.normalize();
					dir.mult(dampVel);
					linearVelocity.sub(dir);
				}
				else {
					linearVelocity.set(0f, 0f, 0f);
				}
			}

			float angSpeed = angularVelocity.length();
			if (angSpeed < angularDamping) {
				float angDampVel = 0.005f;
				if (angSpeed > angDampVel) {
					Vec3 dir = new Vec3(angularVelocity);
					dir.normalize();
					dir.mult(angDampVel);
					angularVelocity.sub(dir);
				}
				else {
					angularVelocity.set(0f, 0f, 0f);
				}
			}
		}
	}

	public void setMassProps(float mass, Vec3 inertia) {
		if (mass == 0f) {
			collisionFlags |= CollisionFlags.STATIC_OBJECT;
			inverseMass = 0f;
		}
		else {
			collisionFlags &= (~CollisionFlags.STATIC_OBJECT);
			inverseMass = 1f / mass;
		}

		invInertiaLocal.set(inertia.x != 0f ? 1f / inertia.x : 0f,
				inertia.y != 0f ? 1f / inertia.y : 0f,
				inertia.z != 0f ? 1f / inertia.z : 0f);
	}

	public float getInvMass() {
		return inverseMass;
	}

	public Mat3 getInvInertiaTensorWorld(Mat3 out) {
		out.set(invInertiaTensorWorld);
		return out;
	}
	
	public void integrateVelocities(float step) {
		if (isStaticOrKinematicObject()) {
			return;
		}

		linearVelocity.scaleAddHere(inverseMass * step, totalForce, linearVelocity);
		Vec3 tmp = new Vec3(totalTorque);
		MatrixUtil.transform(invInertiaTensorWorld, tmp);
		angularVelocity.scaleAddHere(step, tmp, angularVelocity);

		// clamp angular velocity. collision calculations will fail on higher angular velocities	
		float angvel = angularVelocity.length();
		if (angvel * step > MAX_ANGVEL) {
			angularVelocity.mult((MAX_ANGVEL / step) / angvel);
		}
	}

	public void setCenterOfMassTransform(Transform xform) {
		if (isStaticOrKinematicObject()) {
			interpolationWorldTransform.set(worldTransform);
		}
		else {
			interpolationWorldTransform.set(xform);
		}
		getLinearVelocity(interpolationLinearVelocity);
		getAngularVelocity(interpolationAngularVelocity);
		worldTransform.set(xform);
		updateInertiaTensor();
	}

	public void applyCentralForce(Vec3 force) {
		totalForce.add(force);
	}
	
	public Vec3 getInvInertiaDiagLocal(Vec3 out) {
		out.set(invInertiaLocal);
		return out;
	}

	public void setInvInertiaDiagLocal(Vec3 diagInvInertia) {
		invInertiaLocal.set(diagInvInertia);
	}

	public void setSleepingThresholds(float linear, float angular) {
		linearSleepingThreshold = linear;
		angularSleepingThreshold = angular;
	}

	public void applyTorque(Vec3 torque) {
		totalTorque.add(torque);
	}

	public void applyForce(Vec3 force, Vec3 rel_pos) {
		applyCentralForce(force);
		
		Vec3 tmp = new Vec3();
		tmp.crossHere(rel_pos, force);
		tmp.mult(angularFactor);
		applyTorque(tmp);
	}

	public void applyCentralImpulse(Vec3 impulse) {
		linearVelocity.scaleAddHere(inverseMass, impulse, linearVelocity);
	}
	
	public void applyTorqueImpulse(Vec3 torque) {
		Vec3 tmp = new Vec3(torque);
		MatrixUtil.transform(invInertiaTensorWorld, tmp);
		angularVelocity.add(tmp);
	}

	public void applyImpulse(Vec3 impulse, Vec3 rel_pos) {
		if (inverseMass != 0f) {
			applyCentralImpulse(impulse);
			if (angularFactor != 0f) {
				Vec3 tmp = new Vec3();
				tmp.crossHere(rel_pos, impulse);
				tmp.mult(angularFactor);
				applyTorqueImpulse(tmp);
			}
		}
	}

	/**
	 * Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position.
	 */
	public void internalApplyImpulse(Vec3 linearComponent, Vec3 angularComponent, float impulseMagnitude) {
		if (inverseMass != 0f) {
			linearVelocity.scaleAddHere(impulseMagnitude, linearComponent, linearVelocity);
			if (angularFactor != 0f) {
				angularVelocity.scaleAddHere(impulseMagnitude * angularFactor, angularComponent, angularVelocity);
			}
		}
	}

	public void clearForces() {
		totalForce.set(0f, 0f, 0f);
		totalTorque.set(0f, 0f, 0f);
	}
	
	public void updateInertiaTensor() {
		Mat3 mat1 = new Mat3();
		MatrixUtil.scale(mat1, worldTransform.basis, invInertiaLocal);

		Mat3 mat2 = new Mat3(worldTransform.basis);
		mat2.transpose();

                Mat3.mult(mat1, mat2, invInertiaTensorWorld);
	}
	
	public Vec3 getCenterOfMassPosition(Vec3 out) {
		out.set(worldTransform.origin);
		return out;
	}

	public Quat getOrientation(Quat out) {
		MatrixUtil.getRotation(worldTransform.basis, out);
		return out;
	}
	
	public Transform getCenterOfMassTransform(Transform out) {
		out.set(worldTransform);
		return out;
	}

	public Vec3 getLinearVelocity(Vec3 out) {
		out.set(linearVelocity);
		return out;
	}

	public Vec3 getAngularVelocity(Vec3 out) {
		out.set(angularVelocity);
		return out;
	}

	public void setLinearVelocity(Vec3 lin_vel) {
		assert (collisionFlags != CollisionFlags.STATIC_OBJECT);
		linearVelocity.set(lin_vel);
	}

	public void setAngularVelocity(Vec3 ang_vel) {
		assert (collisionFlags != CollisionFlags.STATIC_OBJECT);
		angularVelocity.set(ang_vel);
	}

	public Vec3 getVelocityInLocalPoint(Vec3 rel_pos, Vec3 out) {
		// we also calculate lin/ang velocity for kinematic objects
		Vec3 vec = out;
		vec.crossHere(angularVelocity, rel_pos);
		vec.add(linearVelocity);
		return out;

		//for kinematic objects, we could also use use:
		//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
	}

	public void translate(Vec3 v) {
		worldTransform.origin.add(v);
	}
	
	public void getAabb(Vec3 aabbMin, Vec3 aabbMax) {
		getCollisionShape().getAabb(worldTransform, aabbMin, aabbMax);
	}

	public float computeImpulseDenominator(Vec3 pos, Vec3 normal) {
		Vec3 r0 = new Vec3();
		r0.subHere(pos, getCenterOfMassPosition(new Vec3()));

		Vec3 c0 = new Vec3();
		c0.crossHere(r0, normal);

		Vec3 tmp = new Vec3();
		MatrixUtil.transposeTransform(tmp, c0, getInvInertiaTensorWorld(new Mat3()));

		Vec3 vec = new Vec3();
		vec.crossHere(tmp, r0);

		return inverseMass + normal.dot(vec);
	}

	public float computeAngularImpulseDenominator(Vec3 axis) {
		Vec3 vec = new Vec3();
		MatrixUtil.transposeTransform(vec, axis, getInvInertiaTensorWorld(new Mat3()));
		return axis.dot(vec);
	}

	public void updateDeactivation(float timeStep) {
		if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == DISABLE_DEACTIVATION)) {
			return;
		}

		if ((getLinearVelocity(new Vec3()).squareLength() < linearSleepingThreshold * linearSleepingThreshold) &&
				(getAngularVelocity(new Vec3()).squareLength() < angularSleepingThreshold * angularSleepingThreshold)) {
			deactivationTime += timeStep;
		}
		else {
			deactivationTime = 0f;
			setActivationState(0);
		}
	}

	public boolean wantsSleeping() {
		if (getActivationState() == DISABLE_DEACTIVATION) {
			return false;
		}

		// disable deactivation
		if (BulletGlobals.isDeactivationDisabled() || (BulletGlobals.getDeactivationTime() == 0f)) {
			return false;
		}

		if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == WANTS_DEACTIVATION)) {
			return true;
		}

		if (deactivationTime > BulletGlobals.getDeactivationTime()) {
			return true;
		}
		return false;
	}
	
	public BroadphaseProxy getBroadphaseProxy() {
		return broadphaseHandle;
	}

	public void setNewBroadphaseProxy(BroadphaseProxy broadphaseProxy) {
		this.broadphaseHandle = broadphaseProxy;
	}

	public MotionState getMotionState() {
		return optionalMotionState;
	}

	public void setMotionState(MotionState motionState) {
		this.optionalMotionState = motionState;
		if (optionalMotionState != null) {
			motionState.getWorldTransform(worldTransform);
		}
	}

	public void setAngularFactor(float angFac) {
		angularFactor = angFac;
	}

	public float getAngularFactor() {
		return angularFactor;
	}

	/**
	 * Is this rigidbody added to a CollisionWorld/DynamicsWorld/Broadphase?
	 */
	public boolean isInWorld() {
		return (getBroadphaseProxy() != null);
	}

	@Override
	public boolean checkCollideWithOverride(CollisionObject co) {
		// TODO: change to cast
		RigidBody otherRb = RigidBody.upcast(co);
		if (otherRb == null) {
			return true;
		}

		for (int i = 0; i < constraintRefs.size(); ++i) {
			TypedConstraint c = constraintRefs.getQuick(i);
			if (c.getRigidBodyA() == otherRb || c.getRigidBodyB() == otherRb) {
				return false;
			}
		}

		return true;
	}

	public void addConstraintRef(TypedConstraint c) {
		int index = constraintRefs.indexOf(c);
		if (index == -1) {
			constraintRefs.add(c);
		}

		checkCollideWith = true;
	}
	
	public void removeConstraintRef(TypedConstraint c) {
		constraintRefs.remove(c);
		checkCollideWith = (constraintRefs.size() > 0);
	}

	public TypedConstraint getConstraintRef(int index) {
		return constraintRefs.getQuick(index);
	}

	public int getNumConstraintRefs() {
		return constraintRefs.size();
	}
	
}
