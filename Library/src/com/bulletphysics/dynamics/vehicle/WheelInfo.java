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
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;

/**
 * WheelInfo contains information per wheel about friction and suspension.
 * 
 * @author jezek2
 */
public class WheelInfo {

	//protected final BulletStack stack = BulletStack.get();
	
	public final RaycastInfo raycastInfo = new RaycastInfo();

	public final Transform worldTransform = new Transform();
	
	public final Vec3 chassisConnectionPointCS = new Vec3(); // const
	public final Vec3 wheelDirectionCS = new Vec3(); // const
	public final Vec3 wheelAxleCS = new Vec3(); // const or modified by steering
	public float suspensionRestLength1; // const
	public float maxSuspensionTravelCm;
	public float wheelsRadius; // const
	public float suspensionStiffness; // const
	public float wheelsDampingCompression; // const
	public float wheelsDampingRelaxation; // const
	public float frictionSlip;
	public float steering;
	public float rotation;
	public float deltaRotation;
	public float rollInfluence;

	public float engineForce;

	public float brake;
	
	public boolean bIsFrontWheel;
	
	public Object clientInfo; // can be used to store pointer to sync transforms...

	public float clippedInvContactDotSuspension;
	public float suspensionRelativeVelocity;
	// calculated by suspension
	public float wheelsSuspensionForce;
	public float skidInfo;
	
	public WheelInfo(WheelInfoConstructionInfo ci) {
		suspensionRestLength1 = ci.suspensionRestLength;
		maxSuspensionTravelCm = ci.maxSuspensionTravelCm;

		wheelsRadius = ci.wheelRadius;
		suspensionStiffness = ci.suspensionStiffness;
		wheelsDampingCompression = ci.wheelsDampingCompression;
		wheelsDampingRelaxation = ci.wheelsDampingRelaxation;
		chassisConnectionPointCS.set(ci.chassisConnectionCS);
		wheelDirectionCS.set(ci.wheelDirectionCS);
		wheelAxleCS.set(ci.wheelAxleCS);
		frictionSlip = ci.frictionSlip;
		steering = 0f;
		engineForce = 0f;
		rotation = 0f;
		deltaRotation = 0f;
		brake = 0f;
		rollInfluence = 0.1f;
		bIsFrontWheel = ci.bIsFrontWheel;
	}
	
	public float getSuspensionRestLength() {
		return suspensionRestLength1;
	}

	public void updateWheel(RigidBody chassis, RaycastInfo raycastInfo) {
		if (raycastInfo.isInContact) {
			float project = raycastInfo.contactNormalWS.dot(raycastInfo.wheelDirectionWS);
			Vec3 chassis_velocity_at_contactPoint = new Vec3();
			Vec3 relpos = new Vec3();
			VectorUtil.sub(relpos, raycastInfo.contactPointWS, chassis.getCenterOfMassPosition(new Vec3()));
			chassis.getVelocityInLocalPoint(relpos, chassis_velocity_at_contactPoint);
			float projVel = raycastInfo.contactNormalWS.dot(chassis_velocity_at_contactPoint);
			if (project >= -0.1f) {
				suspensionRelativeVelocity = 0f;
				clippedInvContactDotSuspension = 1f / 0.1f;
			}
			else {
				float inv = -1f / project;
				suspensionRelativeVelocity = projVel * inv;
				clippedInvContactDotSuspension = inv;
			}
		}
		else {
			// Not in contact : position wheel in a nice (rest length) position
			raycastInfo.suspensionLength = getSuspensionRestLength();
			suspensionRelativeVelocity = 0f;
			VectorUtil.negate(raycastInfo.contactNormalWS, raycastInfo.wheelDirectionWS);
			clippedInvContactDotSuspension = 1f;
		}
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	public static class RaycastInfo {
		// set by raycaster
		public final Vec3 contactNormalWS = new Vec3(); // contactnormal
		public final Vec3 contactPointWS = new Vec3(); // raycast hitpoint
		public float suspensionLength;
		public final Vec3 hardPointWS = new Vec3(); // raycast starting point
		public final Vec3 wheelDirectionWS = new Vec3(); // direction in worldspace
		public final Vec3 wheelAxleWS = new Vec3(); // axle in worldspace
		public boolean isInContact;
		public Object groundObject; // could be general void* ptr
	}
	
}
