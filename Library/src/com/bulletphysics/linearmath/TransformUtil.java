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

package com.bulletphysics.linearmath;

import com.bulletphysics.BulletGlobals;
import com.samrj.devil.math.Mat3;
import com.samrj.devil.math.Quat;
import com.samrj.devil.math.Vec3;

/**
 * Utility functions for transforms.
 * 
 * @author jezek2
 */
public class TransformUtil {
	
	public static final float SIMDSQRT12 = 0.7071067811865475244008443621048490f;
	public static final float ANGULAR_MOTION_THRESHOLD = 0.5f*BulletGlobals.SIMD_HALF_PI;
        
	public static float recipSqrt(float x) {
		return 1f / (float)Math.sqrt(x);  /* reciprocal square root */
	}

	public static void planeSpace1(Vec3 n, Vec3 p, Vec3 q) {
		if (Math.abs(n.z) > SIMDSQRT12) {
			// choose p in y-z plane
			float a = n.y * n.y + n.z * n.z;
			float k = recipSqrt(a);
			p.set(0, -n.z * k, n.y * k);
			// set q = n x p
			q.set(a * k, -n.x * p.z, n.x * p.y);
		}
		else {
			// choose p in x-y plane
			float a = n.x * n.x + n.y * n.y;
			float k = recipSqrt(a);
			p.set(-n.y * k, n.x * k, 0);
			// set q = n x p
			q.set(-n.z * p.y, n.z * p.x, a * k);
		}
	}
	
	public static void integrateTransform(Transform curTrans, Vec3 linvel, Vec3 angvel, float timeStep, Transform predictedTransform) {
		VectorUtil.scaleAdd(predictedTransform.origin, timeStep, linvel, curTrans.origin);
//	//#define QUATERNION_DERIVATIVE
//	#ifdef QUATERNION_DERIVATIVE
//		btQuaternion predictedOrn = curTrans.getRotation();
//		predictedOrn += (angvel * predictedOrn) * (timeStep * btScalar(0.5));
//		predictedOrn.normalize();
//	#else
		// Exponential map
		// google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia
		
		Vec3 axis = new Vec3();
		float fAngle = angvel.length();

		// limit the angular motion
		if (fAngle * timeStep > ANGULAR_MOTION_THRESHOLD) {
			fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
		}

		if (fAngle < 0.001f) {
			// use Taylor's expansions of sync function
			VectorUtil.scale(axis, 0.5f * timeStep - (timeStep * timeStep * timeStep) * (0.020833333333f) * fAngle * fAngle, angvel);
		}
		else {
			// sync(fAngle) = sin(c*fAngle)/t
			VectorUtil.scale(axis, (float) Math.sin(0.5f * fAngle * timeStep) / fAngle, angvel);
		}
		Quat dorn = new Quat();
		dorn.set((float) Math.cos(fAngle * timeStep * 0.5f), axis.x, axis.y, axis.z);
		Quat orn0 = curTrans.getRotation(new Quat());

		Quat predictedOrn = Quat.mult(dorn, orn0);
		predictedOrn.normalize();
//  #endif
		predictedTransform.setRotation(predictedOrn);
	}

	public static void calculateVelocity(Transform transform0, Transform transform1, float timeStep, Vec3 linVel, Vec3 angVel) {
		VectorUtil.sub(linVel, transform1.origin, transform0.origin);
		linVel.mult(1f / timeStep);

		Vec3 axis = new Vec3();
		float[] angle = new float[1];
		calculateDiffAxisAngle(transform0, transform1, axis, angle);
		VectorUtil.scale(angVel, angle[0] / timeStep, axis);
	}
	
	public static void calculateDiffAxisAngle(Transform transform0, Transform transform1, Vec3 axis, float[] angle) {
// #ifdef USE_QUATERNION_DIFF
//		btQuaternion orn0 = transform0.getRotation();
//		btQuaternion orn1a = transform1.getRotation();
//		btQuaternion orn1 = orn0.farthest(orn1a);
//		btQuaternion dorn = orn1 * orn0.inverse();
// #else
		Mat3 tmp = new Mat3();
		tmp.set(transform0.basis);
		MatrixUtil.invert(tmp);

		Mat3 dmat = Mat3.mult(transform1.basis, tmp);

		Quat dorn = new Quat();
		MatrixUtil.getRotation(dmat, dorn);
// #endif

		// floating point inaccuracy can lead to w component > 1..., which breaks 

		dorn.normalize();

		angle[0] = QuaternionUtil.getAngle(dorn);
		axis.set(dorn.x, dorn.y, dorn.z);
		// TODO: probably not needed
		//axis[3] = btScalar(0.);

		// check for axis length
		float len = axis.squareLength();
		if (len < BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON) {
			axis.set(1f, 0f, 0f);
		}
		else {
			axis.mult(1f / (float) Math.sqrt(len));
		}
	}
	
}
