/*
 * DevilBullet modifications (c) 2019 Sam Johnson https://github.com/SmashMaster
 * 
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
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

package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.shapes.TriangleShape;
import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;
import com.samrj.devil.math.Vec4;

/**
 *
 * @author jezek2
 */
public class TriangleShapeEx extends TriangleShape {

	public TriangleShapeEx() {
		super();
	}

	public TriangleShapeEx(Vec3 p0, Vec3 p1, Vec3 p2) {
		super(p0, p1, p2);
	}

	@Override
	public void getAabb(Transform t, Vec3 aabbMin, Vec3 aabbMax) {
		Vec3 tv0 = new Vec3(vertices1[0]);
		t.transform(tv0);
		Vec3 tv1 = new Vec3(vertices1[1]);
		t.transform(tv1);
		Vec3 tv2 = new Vec3(vertices1[2]);
		t.transform(tv2);

		AABB trianglebox = new AABB();
		trianglebox.init(tv0,tv1,tv2,collisionMargin);
		
		aabbMin.set(trianglebox.min);
		aabbMax.set(trianglebox.max);
	}

	public void applyTransform(Transform t) {
		t.transform(vertices1[0]);
		t.transform(vertices1[1]);
		t.transform(vertices1[2]);
	}

	public void buildTriPlane(Vec4 plane) {
		Vec3 tmp1 = new Vec3();
		Vec3 tmp2 = new Vec3();

		Vec3 normal = new Vec3();
		VectorUtil.sub(tmp1, vertices1[1], vertices1[0]);
		VectorUtil.sub(tmp2, vertices1[2], vertices1[0]);
		VectorUtil.cross(normal, tmp1, tmp2);
		normal.normalize();

		plane.set(normal.x, normal.y, normal.z, vertices1[0].dot(normal));
	}

	public boolean overlap_test_conservative(TriangleShapeEx other) {
		float total_margin = getMargin() + other.getMargin();

		Vec4 plane0 = new Vec4();
		buildTriPlane(plane0);
		Vec4 plane1 = new Vec4();
		other.buildTriPlane(plane1);

		// classify points on other triangle
		float dis0 = ClipPolygon.distance_point_plane(plane0, other.vertices1[0]) - total_margin;

		float dis1 = ClipPolygon.distance_point_plane(plane0, other.vertices1[1]) - total_margin;

		float dis2 = ClipPolygon.distance_point_plane(plane0, other.vertices1[2]) - total_margin;

		if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) {
			return false; // classify points on this triangle
		}
		dis0 = ClipPolygon.distance_point_plane(plane1, vertices1[0]) - total_margin;

		dis1 = ClipPolygon.distance_point_plane(plane1, vertices1[1]) - total_margin;

		dis2 = ClipPolygon.distance_point_plane(plane1, vertices1[2]) - total_margin;

		if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) {
			return false;
		}
		return true;
	}
	
}
