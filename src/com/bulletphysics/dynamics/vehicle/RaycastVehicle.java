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

package com.bulletphysics.dynamics.vehicle;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.ContactConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraintType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.FloatArrayList;
import com.bulletphysics.util.ObjectArrayList;
import com.samrj.devil.math.Mat3;
import com.samrj.devil.math.Quat;
import javax.vecmath.Vec3;

/**
 * Raycast vehicle, very special constraint that turn a rigidbody into a vehicle.
 * 
 * @author jezek2
 */
public class RaycastVehicle extends TypedConstraint {
	
	private final ArrayPool<float[]> floatArrays = ArrayPool.get(float.class);

	private static RigidBody s_fixedObject = new RigidBody(0, null, null);
	private static final float sideFrictionStiffness2 = 1.0f;
	
	protected ObjectArrayList<Vec3> forwardWS = new ObjectArrayList<Vec3>();
	protected ObjectArrayList<Vec3> axle = new ObjectArrayList<Vec3>();
	protected FloatArrayList forwardImpulse = new FloatArrayList();
	protected FloatArrayList sideImpulse = new FloatArrayList();

	private float tau;
	private float damping;
	private VehicleRaycaster vehicleRaycaster;
	private float pitchControl = 0f;
	private float steeringValue; 
	private float currentVehicleSpeedKmHour;

	private RigidBody chassisBody;

	private int indexRightAxis = 0;
	private int indexUpAxis = 2;
	private int indexForwardAxis = 1;
	
	public ObjectArrayList<WheelInfo> wheelInfo = new ObjectArrayList<WheelInfo>();

	// constructor to create a car from an existing rigidbody
	public RaycastVehicle(VehicleTuning tuning, RigidBody chassis, VehicleRaycaster raycaster) {
		super(TypedConstraintType.VEHICLE_CONSTRAINT_TYPE);
		this.vehicleRaycaster = raycaster;
		this.chassisBody = chassis;
		defaultInit(tuning);
	}
	
	private void defaultInit(VehicleTuning tuning) {
		currentVehicleSpeedKmHour = 0f;
		steeringValue = 0f;
	}

	/**
	 * Basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed.
	 */
	public WheelInfo addWheel(Vec3 connectionPointCS, Vec3 wheelDirectionCS0, Vec3 wheelAxleCS, float suspensionRestLength, float wheelRadius, VehicleTuning tuning, boolean isFrontWheel) {
		WheelInfoConstructionInfo ci = new WheelInfoConstructionInfo();

		ci.chassisConnectionCS.set(connectionPointCS);
		ci.wheelDirectionCS.set(wheelDirectionCS0);
		ci.wheelAxleCS.set(wheelAxleCS);
		ci.suspensionRestLength = suspensionRestLength;
		ci.wheelRadius = wheelRadius;
		ci.suspensionStiffness = tuning.suspensionStiffness;
		ci.wheelsDampingCompression = tuning.suspensionCompression;
		ci.wheelsDampingRelaxation = tuning.suspensionDamping;
		ci.frictionSlip = tuning.frictionSlip;
		ci.bIsFrontWheel = isFrontWheel;
		ci.maxSuspensionTravelCm = tuning.maxSuspensionTravelCm;

		wheelInfo.add(new WheelInfo(ci));

		WheelInfo wheel = wheelInfo.getQuick(getNumWheels() - 1);

		updateWheelTransformsWS(wheel, false);
		updateWheelTransform(getNumWheels() - 1, false);
		return wheel;
	}

	public Transform getWheelTransformWS(int wheelIndex, Transform out) {
		assert (wheelIndex < getNumWheels());
		WheelInfo wheel = wheelInfo.getQuick(wheelIndex);
		out.set(wheel.worldTransform);
		return out;
	}

	public void updateWheelTransform(int wheelIndex) {
		updateWheelTransform(wheelIndex, true);
	}
	
	public void updateWheelTransform(int wheelIndex, boolean interpolatedTransform) {
		WheelInfo wheel = wheelInfo.getQuick(wheelIndex);
		updateWheelTransformsWS(wheel, interpolatedTransform);
		Vec3 up = new Vec3();
		up.negateHere(wheel.raycastInfo.wheelDirectionWS);
		Vec3 right = wheel.raycastInfo.wheelAxleWS;
		Vec3 fwd = new Vec3();
		fwd.crossHere(up, right);
		fwd.normalize();
		// up = right.cross(fwd);
		// up.normalize();

		// rotate around steering over de wheelAxleWS
		float steering = wheel.steering;

		Quat steeringOrn = new Quat();
		QuaternionUtil.setRotation(steeringOrn, up, steering); //wheel.m_steering);
		Mat3 steeringMat = new Mat3();
		MatrixUtil.setRotation(steeringMat, steeringOrn);

		Quat rotatingOrn = new Quat();
		QuaternionUtil.setRotation(rotatingOrn, right, -wheel.rotation);
		Mat3 rotatingMat = new Mat3();
		MatrixUtil.setRotation(rotatingMat, rotatingOrn);

		Mat3 basis2 = new Mat3();
		MatrixUtil.setRow(basis2, 0, new Vec3(right.x, fwd.x, up.x));
		MatrixUtil.setRow(basis2, 1, new Vec3(right.y, fwd.y, up.y));
		MatrixUtil.setRow(basis2, 2, new Vec3(right.z, fwd.z, up.z));

		Mat3 wheelBasis = wheel.worldTransform.basis;
                Mat3.mult(steeringMat, rotatingMat, wheelBasis);
		wheelBasis.mult(basis2);

		wheel.worldTransform.origin.scaleAddHere(wheel.raycastInfo.suspensionLength, wheel.raycastInfo.wheelDirectionWS, wheel.raycastInfo.hardPointWS);
	}
	
	public void resetSuspension() {
		int i;
		for (i = 0; i < wheelInfo.size(); i++) {
			WheelInfo wheel = wheelInfo.getQuick(i);
			wheel.raycastInfo.suspensionLength = wheel.getSuspensionRestLength();
			wheel.suspensionRelativeVelocity = 0f;

			wheel.raycastInfo.contactNormalWS.negateHere(wheel.raycastInfo.wheelDirectionWS);
			//wheel_info.setContactFriction(btScalar(0.0));
			wheel.clippedInvContactDotSuspension = 1f;
		}
	}

	public void updateWheelTransformsWS(WheelInfo wheel) {
		updateWheelTransformsWS(wheel, true);
	}
	
	public void updateWheelTransformsWS(WheelInfo wheel, boolean interpolatedTransform) {
		wheel.raycastInfo.isInContact = false;

		Transform chassisTrans = getChassisWorldTransform(new Transform());
		if (interpolatedTransform && (getRigidBody().getMotionState() != null)) {
			getRigidBody().getMotionState().getWorldTransform(chassisTrans);
		}

		wheel.raycastInfo.hardPointWS.set(wheel.chassisConnectionPointCS);
		chassisTrans.transform(wheel.raycastInfo.hardPointWS);

		wheel.raycastInfo.wheelDirectionWS.set(wheel.wheelDirectionCS);
		MatrixUtil.transform(chassisTrans.basis, wheel.raycastInfo.wheelDirectionWS);

		wheel.raycastInfo.wheelAxleWS.set(wheel.wheelAxleCS);
		MatrixUtil.transform(chassisTrans.basis, wheel.raycastInfo.wheelAxleWS);
	}

	public float rayCast(WheelInfo wheel) {
		updateWheelTransformsWS(wheel, false);

		float depth = -1f;

		float raylen = wheel.getSuspensionRestLength() + wheel.wheelsRadius;

		Vec3 rayvector = new Vec3();
		rayvector.scale(raylen, wheel.raycastInfo.wheelDirectionWS);
		Vec3 source = wheel.raycastInfo.hardPointWS;
		wheel.raycastInfo.contactPointWS.addHere(source, rayvector);
		Vec3 target = wheel.raycastInfo.contactPointWS;

		float param = 0f;

		VehicleRaycasterResult rayResults = new VehicleRaycasterResult();

		assert (vehicleRaycaster != null);

		Object object = vehicleRaycaster.castRay(source, target, rayResults);

		wheel.raycastInfo.groundObject = null;

		if (object != null) {
			param = rayResults.distFraction;
			depth = raylen * rayResults.distFraction;
			wheel.raycastInfo.contactNormalWS.set(rayResults.hitNormalInWorld);
			wheel.raycastInfo.isInContact = true;

			wheel.raycastInfo.groundObject = s_fixedObject; // todo for driving on dynamic/movable objects!;
			//wheel.m_raycastInfo.m_groundObject = object;

			float hitDistance = param * raylen;
			wheel.raycastInfo.suspensionLength = hitDistance - wheel.wheelsRadius;
			// clamp on max suspension travel

			float minSuspensionLength = wheel.getSuspensionRestLength() - wheel.maxSuspensionTravelCm * 0.01f;
			float maxSuspensionLength = wheel.getSuspensionRestLength() + wheel.maxSuspensionTravelCm * 0.01f;
			if (wheel.raycastInfo.suspensionLength < minSuspensionLength) {
				wheel.raycastInfo.suspensionLength = minSuspensionLength;
			}
			if (wheel.raycastInfo.suspensionLength > maxSuspensionLength) {
				wheel.raycastInfo.suspensionLength = maxSuspensionLength;
			}

			wheel.raycastInfo.contactPointWS.set(rayResults.hitPointInWorld);

			float denominator = wheel.raycastInfo.contactNormalWS.dot(wheel.raycastInfo.wheelDirectionWS);

			Vec3 chassis_velocity_at_contactPoint = new Vec3();
			Vec3 relpos = new Vec3();
			relpos.subHere(wheel.raycastInfo.contactPointWS, getRigidBody().getCenterOfMassPosition(new Vec3()));

			getRigidBody().getVelocityInLocalPoint(relpos, chassis_velocity_at_contactPoint);

			float projVel = wheel.raycastInfo.contactNormalWS.dot(chassis_velocity_at_contactPoint);

			if (denominator >= -0.1f) {
				wheel.suspensionRelativeVelocity = 0f;
				wheel.clippedInvContactDotSuspension = 1f / 0.1f;
			}
			else {
				float inv = -1f / denominator;
				wheel.suspensionRelativeVelocity = projVel * inv;
				wheel.clippedInvContactDotSuspension = inv;
			}

		}
		else {
			// put wheel info as in rest position
			wheel.raycastInfo.suspensionLength = wheel.getSuspensionRestLength();
			wheel.suspensionRelativeVelocity = 0f;
			wheel.raycastInfo.contactNormalWS.negateHere(wheel.raycastInfo.wheelDirectionWS);
			wheel.clippedInvContactDotSuspension = 1f;
		}

		return depth;
	}
	
	public Transform getChassisWorldTransform(Transform out) {
		/*
		if (getRigidBody()->getMotionState())
		{
			btTransform chassisWorldTrans;
			getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
			return chassisWorldTrans;
		}
		*/

		return getRigidBody().getCenterOfMassTransform(out);
	}
	
	public void updateVehicle(float step) {
		for (int i = 0; i < getNumWheels(); i++) {
			updateWheelTransform(i, false);
		}
		
		Vec3 tmp = new Vec3();

		currentVehicleSpeedKmHour = 3.6f * getRigidBody().getLinearVelocity(tmp).length();

		Transform chassisTrans = getChassisWorldTransform(new Transform());

		Vec3 forwardW = new Vec3();
		forwardW.set(
				chassisTrans.basis.getEntry(0, indexForwardAxis),
				chassisTrans.basis.getEntry(1, indexForwardAxis),
				chassisTrans.basis.getEntry(2, indexForwardAxis));

		if (forwardW.dot(getRigidBody().getLinearVelocity(tmp)) < 0f) {
			currentVehicleSpeedKmHour *= -1f;
		}

		//
		// simulate suspension
		//

		int i = 0;
		for (i = 0; i < wheelInfo.size(); i++) {
			float depth;
			depth = rayCast(wheelInfo.getQuick(i));
		}

		updateSuspension(step);

		for (i = 0; i < wheelInfo.size(); i++) {
			// apply suspension force
			WheelInfo wheel = wheelInfo.getQuick(i);

			float suspensionForce = wheel.wheelsSuspensionForce;

			float gMaxSuspensionForce = 6000f;
			if (suspensionForce > gMaxSuspensionForce) {
				suspensionForce = gMaxSuspensionForce;
			}
			Vec3 impulse = new Vec3();
			impulse.scale(suspensionForce * step, wheel.raycastInfo.contactNormalWS);
			Vec3 relpos = new Vec3();
			relpos.subHere(wheel.raycastInfo.contactPointWS, getRigidBody().getCenterOfMassPosition(tmp));

			getRigidBody().applyImpulse(impulse, relpos);
		}

		updateFriction(step);

		for (i = 0; i < wheelInfo.size(); i++) {
			WheelInfo wheel = wheelInfo.getQuick(i);
			Vec3 relpos = new Vec3();
			relpos.subHere(wheel.raycastInfo.hardPointWS, getRigidBody().getCenterOfMassPosition(tmp));
			Vec3 vel = getRigidBody().getVelocityInLocalPoint(relpos, new Vec3());

			if (wheel.raycastInfo.isInContact) {
				Transform chassisWorldTransform = getChassisWorldTransform(new Transform());

				Vec3 fwd = new Vec3();
				fwd.set(
						chassisWorldTransform.basis.getEntry(0, indexForwardAxis),
						chassisWorldTransform.basis.getEntry(1, indexForwardAxis),
						chassisWorldTransform.basis.getEntry(2, indexForwardAxis));

				float proj = fwd.dot(wheel.raycastInfo.contactNormalWS);
				tmp.scale(proj, wheel.raycastInfo.contactNormalWS);
				fwd.sub(tmp);

				float proj2 = fwd.dot(vel);

				wheel.deltaRotation = (proj2 * step) / (wheel.wheelsRadius);
				wheel.rotation += wheel.deltaRotation;

			}
			else {
				wheel.rotation += wheel.deltaRotation;
			}

			wheel.deltaRotation *= 0.99f; // damping of rotation when not in contact
		}
	}

	public void setSteeringValue(float steering, int wheel) {
		assert (wheel >= 0 && wheel < getNumWheels());

		WheelInfo wheel_info = getWheelInfo(wheel);
		wheel_info.steering = steering;
	}
	
	public float getSteeringValue(int wheel) {
		return getWheelInfo(wheel).steering;
	}

	public void applyEngineForce(float force, int wheel) {
		assert (wheel >= 0 && wheel < getNumWheels());
		WheelInfo wheel_info = getWheelInfo(wheel);
		wheel_info.engineForce = force;
	}

	public WheelInfo getWheelInfo(int index) {
		assert ((index >= 0) && (index < getNumWheels()));

		return wheelInfo.getQuick(index);
	}

	public void setBrake(float brake, int wheelIndex) {
		assert ((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
		getWheelInfo(wheelIndex).brake = brake;
	}

	public void updateSuspension(float deltaTime) {
		float chassisMass = 1f / chassisBody.getInvMass();

		for (int w_it = 0; w_it < getNumWheels(); w_it++) {
			WheelInfo wheel_info = wheelInfo.getQuick(w_it);

			if (wheel_info.raycastInfo.isInContact) {
				float force;
				//	Spring
				{
					float susp_length = wheel_info.getSuspensionRestLength();
					float current_length = wheel_info.raycastInfo.suspensionLength;

					float length_diff = (susp_length - current_length);

					force = wheel_info.suspensionStiffness * length_diff * wheel_info.clippedInvContactDotSuspension;
				}

				// Damper
				{
					float projected_rel_vel = wheel_info.suspensionRelativeVelocity;
					{
						float susp_damping;
						if (projected_rel_vel < 0f) {
							susp_damping = wheel_info.wheelsDampingCompression;
						}
						else {
							susp_damping = wheel_info.wheelsDampingRelaxation;
						}
						force -= susp_damping * projected_rel_vel;
					}
				}

				// RESULT
				wheel_info.wheelsSuspensionForce = force * chassisMass;
				if (wheel_info.wheelsSuspensionForce < 0f) {
					wheel_info.wheelsSuspensionForce = 0f;
				}
			}
			else {
				wheel_info.wheelsSuspensionForce = 0f;
			}
		}
	}
	
	private float calcRollingFriction(WheelContactPoint contactPoint) {
		Vec3 tmp = new Vec3();
		
		float j1 = 0f;
		
		Vec3 contactPosWorld = contactPoint.frictionPositionWorld;

		Vec3 rel_pos1 = new Vec3();
		rel_pos1.subHere(contactPosWorld, contactPoint.body0.getCenterOfMassPosition(tmp));
		Vec3 rel_pos2 = new Vec3();
		rel_pos2.subHere(contactPosWorld, contactPoint.body1.getCenterOfMassPosition(tmp));

		float maxImpulse = contactPoint.maxImpulse;

		Vec3 vel1 = contactPoint.body0.getVelocityInLocalPoint(rel_pos1, new Vec3());
		Vec3 vel2 = contactPoint.body1.getVelocityInLocalPoint(rel_pos2, new Vec3());
		Vec3 vel = new Vec3();
		vel.subHere(vel1, vel2);

		float vrel = contactPoint.frictionDirectionWorld.dot(vel);

		// calculate j that moves us to zero relative velocity
		j1 = -vrel * contactPoint.jacDiagABInv;
		j1 = Math.min(j1, maxImpulse);
		j1 = Math.max(j1, -maxImpulse);

		return j1;
	}
	
	public void updateFriction(float timeStep) {
		// calculate the impulse, so that the wheels don't move sidewards
		int numWheel = getNumWheels();
		if (numWheel == 0) {
			return;
		}

		MiscUtil.resize(forwardWS, numWheel, Vec3.class);
		MiscUtil.resize(axle, numWheel, Vec3.class);
		MiscUtil.resize(forwardImpulse, numWheel, 0f);
		MiscUtil.resize(sideImpulse, numWheel, 0f);

		Vec3 tmp = new Vec3();

		int numWheelsOnGround = 0;

		// collapse all those loops into one!
		for (int i = 0; i < getNumWheels(); i++) {
			WheelInfo wheel_info = wheelInfo.getQuick(i);
			RigidBody groundObject = (RigidBody) wheel_info.raycastInfo.groundObject;
			if (groundObject != null) {
				numWheelsOnGround++;
			}
			sideImpulse.set(i, 0f);
			forwardImpulse.set(i, 0f);
		}

		{
			Transform wheelTrans = new Transform();
			for (int i = 0; i < getNumWheels(); i++) {

				WheelInfo wheel_info = wheelInfo.getQuick(i);

				RigidBody groundObject = (RigidBody) wheel_info.raycastInfo.groundObject;

				if (groundObject != null) {
					getWheelTransformWS(i, wheelTrans);

					Mat3 wheelBasis0 = new Mat3(wheelTrans.basis);
					axle.getQuick(i).set(
							wheelBasis0.getEntry(0, indexRightAxis),
							wheelBasis0.getEntry(1, indexRightAxis),
							wheelBasis0.getEntry(2, indexRightAxis));

					Vec3 surfNormalWS = wheel_info.raycastInfo.contactNormalWS;
					float proj = axle.getQuick(i).dot(surfNormalWS);
					tmp.scale(proj, surfNormalWS);
					axle.getQuick(i).sub(tmp);
					axle.getQuick(i).normalize();

					forwardWS.getQuick(i).crossHere(surfNormalWS, axle.getQuick(i));
					forwardWS.getQuick(i).normalize();

					float[] floatPtr = floatArrays.getFixed(1);
					ContactConstraint.resolveSingleBilateral(chassisBody, wheel_info.raycastInfo.contactPointWS,
							groundObject, wheel_info.raycastInfo.contactPointWS,
							0f, axle.getQuick(i), floatPtr, timeStep);
					sideImpulse.set(i, floatPtr[0]);
					floatArrays.release(floatPtr);

					sideImpulse.set(i, sideImpulse.get(i) * sideFrictionStiffness2);
				}
			}
		}

		float sideFactor = 1f;
		float fwdFactor = 0.5f;

		boolean sliding = false;
		{
			for (int wheel = 0; wheel < getNumWheels(); wheel++) {
				WheelInfo wheel_info = wheelInfo.getQuick(wheel);
				RigidBody groundObject = (RigidBody) wheel_info.raycastInfo.groundObject;

				float rollingFriction = 0f;

				if (groundObject != null) {
					if (wheel_info.engineForce != 0f) {
						rollingFriction = wheel_info.engineForce * timeStep;
					}
					else {
						float defaultRollingFrictionImpulse = 0f;
						float maxImpulse = wheel_info.brake != 0f ? wheel_info.brake : defaultRollingFrictionImpulse;
						WheelContactPoint contactPt = new WheelContactPoint(chassisBody, groundObject, wheel_info.raycastInfo.contactPointWS, forwardWS.getQuick(wheel), maxImpulse);
						rollingFriction = calcRollingFriction(contactPt);
					}
				}

				// switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

				forwardImpulse.set(wheel, 0f);
				wheelInfo.getQuick(wheel).skidInfo = 1f;

				if (groundObject != null) {
					wheelInfo.getQuick(wheel).skidInfo = 1f;

					float maximp = wheel_info.wheelsSuspensionForce * timeStep * wheel_info.frictionSlip;
					float maximpSide = maximp;

					float maximpSquared = maximp * maximpSide;

					forwardImpulse.set(wheel, rollingFriction); //wheelInfo.m_engineForce* timeStep;

					float x = (forwardImpulse.get(wheel)) * fwdFactor;
					float y = (sideImpulse.get(wheel)) * sideFactor;

					float impulseSquared = (x * x + y * y);

					if (impulseSquared > maximpSquared) {
						sliding = true;

						float factor = maximp / (float) Math.sqrt(impulseSquared);

						wheelInfo.getQuick(wheel).skidInfo *= factor;
					}
				}

			}
		}

		if (sliding) {
			for (int wheel = 0; wheel < getNumWheels(); wheel++) {
				if (sideImpulse.get(wheel) != 0f) {
					if (wheelInfo.getQuick(wheel).skidInfo < 1f) {
						forwardImpulse.set(wheel, forwardImpulse.get(wheel) * wheelInfo.getQuick(wheel).skidInfo);
						sideImpulse.set(wheel, sideImpulse.get(wheel) * wheelInfo.getQuick(wheel).skidInfo);
					}
				}
			}
		}

		// apply the impulses
		{
			for (int wheel = 0; wheel < getNumWheels(); wheel++) {
				WheelInfo wheel_info = wheelInfo.getQuick(wheel);

				Vec3 rel_pos = new Vec3();
				rel_pos.subHere(wheel_info.raycastInfo.contactPointWS, chassisBody.getCenterOfMassPosition(tmp));

				if (forwardImpulse.get(wheel) != 0f) {
					tmp.scale(forwardImpulse.get(wheel), forwardWS.getQuick(wheel));
					chassisBody.applyImpulse(tmp, rel_pos);
				}
				if (sideImpulse.get(wheel) != 0f) {
					RigidBody groundObject = (RigidBody) wheelInfo.getQuick(wheel).raycastInfo.groundObject;

					Vec3 rel_pos2 = new Vec3();
					rel_pos2.subHere(wheel_info.raycastInfo.contactPointWS, groundObject.getCenterOfMassPosition(tmp));

					Vec3 sideImp = new Vec3();
					sideImp.scale(sideImpulse.get(wheel), axle.getQuick(wheel));

					rel_pos.z *= wheel_info.rollInfluence;
					chassisBody.applyImpulse(sideImp, rel_pos);

					// apply friction impulse on the ground
					tmp.negateHere(sideImp);
					groundObject.applyImpulse(tmp, rel_pos2);
				}
			}
		}
	}
	
	@Override
	public void buildJacobian() {
		// not yet
	}

	@Override
	public void solveConstraint(float timeStep) {
		// not yet
	}
	
	public int getNumWheels() {
		return wheelInfo.size();
	}

	public void setPitchControl(float pitch) {
		this.pitchControl = pitch;
	}

	public RigidBody getRigidBody() {
		return chassisBody;
	}

	public int getRightAxis() {
		return indexRightAxis;
	}

	public int getUpAxis() {
		return indexUpAxis;
	}

	public int getForwardAxis() {
		return indexForwardAxis;
	}

	/**
	 * Worldspace forward vector.
	 */
	public Vec3 getForwardVector(Vec3 out) {
		Transform chassisTrans = getChassisWorldTransform(new Transform());

		out.set(
				chassisTrans.basis.getEntry(0, indexForwardAxis),
				chassisTrans.basis.getEntry(1, indexForwardAxis),
				chassisTrans.basis.getEntry(2, indexForwardAxis));

		return out;
	}

	/**
	 * Velocity of vehicle (positive if velocity vector has same direction as foward vector).
	 */
	public float getCurrentSpeedKmHour() {
		return currentVehicleSpeedKmHour;
	}

	public void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex) {
		this.indexRightAxis = rightIndex;
		this.indexUpAxis = upIndex;
		this.indexForwardAxis = forwardIndex;
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	private static class WheelContactPoint {
		public RigidBody body0;
		public RigidBody body1;
		public final Vec3 frictionPositionWorld = new Vec3();
		public final Vec3 frictionDirectionWorld = new Vec3();
		public float jacDiagABInv;
		public float maxImpulse;

		public WheelContactPoint(RigidBody body0, RigidBody body1, Vec3 frictionPosWorld, Vec3 frictionDirectionWorld, float maxImpulse) {
			this.body0 = body0;
			this.body1 = body1;
			this.frictionPositionWorld.set(frictionPosWorld);
			this.frictionDirectionWorld.set(frictionDirectionWorld);
			this.maxImpulse = maxImpulse;

			float denom0 = body0.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
			float denom1 = body1.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
			float relaxation = 1f;
			jacDiagABInv = relaxation / (denom0 + denom1);
		}
	}

}
