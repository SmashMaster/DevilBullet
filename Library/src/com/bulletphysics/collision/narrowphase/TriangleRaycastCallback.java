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

package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;

/**
 *
 * @author jezek2
 */
public abstract class TriangleRaycastCallback extends TriangleCallback {
	
	//protected final BulletStack stack = BulletStack.get();

	public final Vec3 from = new Vec3();
	public final Vec3 to = new Vec3();

	public float hitFraction;

	public TriangleRaycastCallback(Vec3 from, Vec3 to) {
		this.from.set(from);
		this.to.set(to);
		this.hitFraction = 1f;
	}
	
	public void processTriangle(Vec3[] triangle, int partId, int triangleIndex) {
		Vec3 vert0 = triangle[0];
		Vec3 vert1 = triangle[1];
		Vec3 vert2 = triangle[2];

		Vec3 v10 = new Vec3();
		VectorUtil.sub(v10, vert1, vert0);

		Vec3 v20 = new Vec3();
		VectorUtil.sub(v20, vert2, vert0);

		Vec3 triangleNormal = new Vec3();
		VectorUtil.cross(triangleNormal, v10, v20);

		float dist = vert0.dot(triangleNormal);
		float dist_a = triangleNormal.dot(from);
		dist_a -= dist;
		float dist_b = triangleNormal.dot(to);
		dist_b -= dist;

		if (dist_a * dist_b >= 0f) {
			return; // same sign
		}

		float proj_length = dist_a - dist_b;
		float distance = (dist_a) / (proj_length);
		// Now we have the intersection point on the plane, we'll see if it's inside the triangle
		// Add an epsilon as a tolerance for the raycast,
		// in case the ray hits exacly on the edge of the triangle.
		// It must be scaled for the triangle size.

		if (distance < hitFraction) {
			float edge_tolerance = triangleNormal.squareLength();
			edge_tolerance *= -0.0001f;
			Vec3 point = new Vec3();
			VectorUtil.setInterpolate3(point, from, to, distance);
			{
				Vec3 v0p = new Vec3();
				VectorUtil.sub(v0p, vert0, point);
				Vec3 v1p = new Vec3();
				VectorUtil.sub(v1p, vert1, point);
				Vec3 cp0 = new Vec3();
				VectorUtil.cross(cp0, v0p, v1p);

				if (cp0.dot(triangleNormal) >= edge_tolerance) {
					Vec3 v2p = new Vec3();
					VectorUtil.sub(v2p, vert2, point);
					Vec3 cp1 = new Vec3();
					VectorUtil.cross(cp1, v1p, v2p);
					if (cp1.dot(triangleNormal) >= edge_tolerance) {
						Vec3 cp2 = new Vec3();
						VectorUtil.cross(cp2, v2p, v0p);

						if (cp2.dot(triangleNormal) >= edge_tolerance) {

							if (dist_a > 0f) {
								hitFraction = reportHit(triangleNormal, distance, partId, triangleIndex);
							}
							else {
								Vec3 tmp = new Vec3();
								VectorUtil.negate(tmp, triangleNormal);
								hitFraction = reportHit(tmp, distance, partId, triangleIndex);
							}
						}
					}
				}
			}
		}
	}

	public abstract float reportHit(Vec3 hitNormalLocal, float hitFraction, int partId, int triangleIndex );

}
