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

package com.bulletphysics.demos.concaveconvexcast;

import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.dispatch.CollisionWorld.ClosestConvexResultCallback;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.demos.opengl.IGL;
import static com.bulletphysics.demos.opengl.IGL.*;
import com.bulletphysics.linearmath.Clock;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Quat;
import com.samrj.devil.math.Vec3;

/**
 * Scrolls back and forth over terrain.
 *
 * @author jezek2
 */
public class ConvexcastBatch {

	public static final int NUMRAYS_IN_BAR = 100;

	public Vec3[] source = new Vec3[NUMRAYS_IN_BAR];
	public Vec3[] dest = new Vec3[NUMRAYS_IN_BAR];
	public Vec3[] direction = new Vec3[NUMRAYS_IN_BAR];
	public Vec3[] hit_com = new Vec3[NUMRAYS_IN_BAR];
	public Vec3[] hit_surface = new Vec3[NUMRAYS_IN_BAR];
	public float[] hit_fraction = new float[NUMRAYS_IN_BAR];
	public Vec3[] normal = new Vec3[NUMRAYS_IN_BAR];

	{
		for (int i=0; i<NUMRAYS_IN_BAR; i++) {
			source[i] = new Vec3();
			dest[i] = new Vec3();
			direction[i] = new Vec3();
			hit_com[i] = new Vec3();
			hit_surface[i] = new Vec3();
			normal[i] = new Vec3();
		}
	}

	public int frame_counter;
	public int ms;
	public int sum_ms;
	public int sum_ms_samples;
	public int min_ms;
	public int max_ms;

	//#ifdef USE_BT_CLOCK
	public Clock frame_timer = new Clock();
	//#endif //USE_BT_CLOCK

	public float dx;
	public float min_x;
	public float max_x;
	public float min_y;
	public float max_y;
	public float sign;

	public final Vec3 boxShapeHalfExtents = new Vec3();
	public final BoxShape boxShape;

	public ConvexcastBatch() {
		boxShape = new BoxShape(new Vec3(0f, 0f, 0f));
		ms = 0;
		max_ms = 0;
		min_ms = 9999;
		sum_ms_samples = 0;
		sum_ms = 0;
	}

	public ConvexcastBatch(boolean unused, float ray_length, float min_z, float max_z) {
		this(unused, ray_length, min_z, max_z, -10, 10);
	}

	public ConvexcastBatch(boolean unused, float ray_length, float min_z, float max_z, float min_y, float max_y) {
		boxShapeHalfExtents.set(1f, 1f, 1f);
		boxShape = new BoxShape(boxShapeHalfExtents);
		frame_counter = 0;
		ms = 0;
		max_ms = 0;
		min_ms = 9999;
		sum_ms_samples = 0;
		sum_ms = 0;
		dx = 10f;
		min_x = -40;
		max_x = 20;
		this.min_y = min_y;
		this.max_y = max_y;
		sign = 1f;
		float dalpha = 2f * (float)Math.PI / NUMRAYS_IN_BAR;
		for (int i=0; i<NUMRAYS_IN_BAR; i++) {
			float z = (max_z - min_z) / NUMRAYS_IN_BAR * i + min_z;
			source[i].set(min_x, max_y, z);
			dest[i].set(min_x + ray_length, min_y, z);
			normal[i].set(1f, 0f, 0f);
		}
	}

	public ConvexcastBatch (float ray_length, float z) {
		this(ray_length, z, -1000, 10);
	}

	public ConvexcastBatch(float ray_length, float z, float min_y, float max_y) {
		boxShapeHalfExtents.set(1f, 1f, 1f);
		boxShape = new BoxShape(boxShapeHalfExtents);
		frame_counter = 0;
		ms = 0;
		max_ms = 0;
		min_ms = 9999;
		sum_ms_samples = 0;
		sum_ms = 0;
		dx = 10f;
		min_x = -40;
		max_x = 20;
		this.min_y = min_y;
		this.max_y = max_y;
		sign = 1f;
		float dalpha = 2f * (float)Math.PI / NUMRAYS_IN_BAR;
		for (int i=0; i<NUMRAYS_IN_BAR; i++) {
			float alpha = dalpha * i;
			// rotate around by alpha degrees y
			Quat q = new Quat(alpha, 0f, 1f, 0f);
			direction[i].set(1f, 0f, 0f);
			Quat tmpQuat = new Quat(q);
			QuaternionUtil.mul(tmpQuat, direction[i]);
			direction[i].set(tmpQuat.x, tmpQuat.y, tmpQuat.z);
			//direction[i].set(direction[i]);
			source[i].set(min_x, max_y, z);
			VectorUtil.scaleAdd(dest[i], ray_length, direction[i], source[i]);
			dest[i].y = min_y;
			normal[i].set(1f, 0f, 0f);
		}
	}

	public void move(float dt) {
		if (dt > (1f / 60f)) {
			dt = 1f / 60f;
		}
		
		for (int i=0; i<NUMRAYS_IN_BAR; i++) {
			source[i].x += dx * dt * sign;
			dest[i].x += dx * dt * sign;
		}

		if (source[0].x < min_x) {
			sign = 1f;
		}
		else if (source[0].x > max_x) {
			sign = -1f;
		}
	}

	public void cast(CollisionWorld cw) {
		//#ifdef USE_BT_CLOCK
		frame_timer.reset();
		//#endif //USE_BT_CLOCK

		for (int i=0; i<NUMRAYS_IN_BAR; i++) {
			ClosestConvexResultCallback cb = new ClosestConvexResultCallback(source[i], dest[i]);

			Quat qFrom = new Quat();
			Quat qTo = new Quat();
			QuaternionUtil.setRotation(qFrom, new Vec3(1f, 0f, 0f), 0f);
			QuaternionUtil.setRotation(qTo, new Vec3(1f, 0f, 0f), 0.7f);

			Transform from = new Transform();
			Transform to = new Transform();
			from.basis.setRotation(qFrom);
			from.origin.set(source[i]);
			to.basis.setRotation(qTo);
			to.origin.set(dest[i]);
			
			cw.convexSweepTest(boxShape, from, to, cb);

			if (cb.hasHit()) {
				hit_surface[i].set(cb.hitPointWorld);
				VectorUtil.setInterpolate3(hit_com[i], source[i], dest[i], cb.closestHitFraction);
				hit_fraction[i] = cb.closestHitFraction;
				normal[i].set(cb.hitNormalWorld);
				normal[i].normalize();
			}
			else {
				hit_com[i].set(dest[i]);
				hit_surface[i].set(dest[i]);
				hit_fraction[i] = 1f;
				normal[i].set(1f, 0f, 0f);
			}

		}

		//#ifdef USE_BT_CLOCK
		ms += frame_timer.getTimeMilliseconds();
		//#endif //USE_BT_CLOCK
		
		frame_counter++;
		if (frame_counter > 50) {
			min_ms = ms < min_ms ? ms : min_ms;
			max_ms = ms > max_ms ? ms : max_ms;
			sum_ms += ms;
			sum_ms_samples++;
			float mean_ms = (float) sum_ms / (float) sum_ms_samples;
			System.out.printf("%d rays in %d ms %d %d %f\n", NUMRAYS_IN_BAR * frame_counter, ms, min_ms, max_ms, mean_ms);
			ms = 0;
			frame_counter = 0;
		}
	}

	public void drawCube(IGL gl, Transform T) {
		float[] m = new float[16];
		T.getOpenGLMatrix(m);
		gl.glPushMatrix();
		gl.glMultMatrix(m);
		gl.glScalef(2f * boxShapeHalfExtents.x, 2f * boxShapeHalfExtents.y, 2f * boxShapeHalfExtents.z);
		gl.drawCube(1f);
		gl.glPopMatrix();
	}

	public void draw(IGL gl) {
		gl.glDisable(GL_LIGHTING);
		gl.glColor3f(0f, 1f, 0f);
		gl.glBegin(GL_LINES);
		for (int i=0; i<NUMRAYS_IN_BAR; i++) {
			gl.glVertex3f(source[i].x, source[i].y, source[i].z);
			gl.glVertex3f(hit_com[i].x, hit_com[i].y, hit_com[i].z);
		}
		gl.glColor3f(1f, 1f, 1f);
		//gl.glBegin(GL_LINES);
		float normal_scale = 10f; // easier to see if this is big
		for (int i=0; i<NUMRAYS_IN_BAR; i++) {
			gl.glVertex3f(hit_surface[i].x, hit_surface[i].y, hit_surface[i].z);
			gl.glVertex3f(hit_surface[i].x + normal_scale * normal[i].x, hit_surface[i].y + normal_scale * normal[i].y, hit_surface[i].z + normal_scale * normal[i].z);
		}
		gl.glEnd();
		gl.glColor3f(0f, 1f, 1f);
		Quat qFrom = new Quat();
		Quat qTo = new Quat();
		QuaternionUtil.setRotation(qFrom, new Vec3(1f, 0f, 0f), 0f);
		QuaternionUtil.setRotation(qTo, new Vec3(1f, 0f, 0f), 0.7f);
		for (int i=0; i<NUMRAYS_IN_BAR; i++) {
			Transform from = new Transform();
			from.basis.setRotation(qFrom);
			from.origin.set(source[i]);

			Transform to = new Transform();
			to.basis.setRotation(qTo);
			to.origin.set(dest[i]);

			Vec3 linVel = new Vec3();
			Vec3 angVel = new Vec3();

			TransformUtil.calculateVelocity(from, to, 1f, linVel, angVel);
			Transform T = new Transform();
			TransformUtil.integrateTransform(from, linVel, angVel, hit_fraction[i], T);
			drawCube(gl, T);
		}
		gl.glEnable(GL_LIGHTING);
	}

}
