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

import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.samrj.devil.math.Vec3;

/**
 * IDebugDraw interface class allows hooking up a debug renderer to visually debug
 * simulations.<p>
 * 
 * Typical use case: create a debug drawer object, and assign it to a {@link CollisionWorld}
 * or {@link DynamicsWorld} using setDebugDrawer and call debugDrawWorld.<p>
 * 
 * A class that implements the IDebugDraw interface has to implement the drawLine
 * method at a minimum.
 * 
 * @author jezek2
 */
public abstract class IDebugDraw {
	
	//protected final BulletStack stack = BulletStack.get();

	public abstract void drawLine(Vec3 from, Vec3 to, Vec3 color);
	
	public void drawTriangle(Vec3 v0, Vec3 v1, Vec3 v2, Vec3 n0, Vec3 n1, Vec3 n2, Vec3 color, float alpha) {
		drawTriangle(v0, v1, v2, color, alpha);
	}
	
	public void drawTriangle(Vec3 v0, Vec3 v1, Vec3 v2, Vec3 color, float alpha) {
		drawLine(v0, v1, color);
		drawLine(v1, v2, color);
		drawLine(v2, v0, color);
	}

	public abstract void drawContactPoint(Vec3 PointOnB, Vec3 normalOnB, float distance, int lifeTime, Vec3 color);

	public abstract void reportErrorWarning(String warningString);

	public abstract void draw3dText(Vec3 location, String textString);

	public abstract void setDebugMode(int debugMode);

	public abstract int getDebugMode();

	public void drawAabb(Vec3 from, Vec3 to, Vec3 color) {
		Vec3 halfExtents = new Vec3(to);
		halfExtents.sub(from);
		halfExtents.mult(0.5f);

		Vec3 center = new Vec3(to);
		center.add(from);
		center.mult(0.5f);

		int i, j;

		Vec3 edgecoord = new Vec3();
		edgecoord.set(1f, 1f, 1f);
		Vec3 pa = new Vec3(), pb = new Vec3();
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 3; j++) {
				pa.set(edgecoord.x * halfExtents.x, edgecoord.y * halfExtents.y, edgecoord.z * halfExtents.z);
				pa.add(center);

				int othercoord = j % 3;

				VectorUtil.mulCoord(edgecoord, othercoord, -1f);
				pb.set(edgecoord.x * halfExtents.x, edgecoord.y * halfExtents.y, edgecoord.z * halfExtents.z);
				pb.add(center);

				drawLine(pa, pb, color);
			}
			edgecoord.set(-1f, -1f, -1f);
			if (i < 3) {
				VectorUtil.mulCoord(edgecoord, i, -1f);
			}
		}
	}
}
