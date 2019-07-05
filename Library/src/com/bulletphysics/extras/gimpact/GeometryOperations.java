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

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec3;
import com.samrj.devil.math.Vec4;

/**
 *
 * @author jezek2
 */
class GeometryOperations {

	public static final float PLANEDIREPSILON = 0.0000001f;
	public static final float PARALELENORMALS = 0.000001f;
	
	public static final float CLAMP(float number, float minval, float maxval) {
		return (number < minval? minval : (number > maxval? maxval : number));
	}

	/**
	 * Calc a plane from a triangle edge an a normal.
	 */
	public static void edge_plane(Vec3 e1, Vec3 e2, Vec3 normal, Vec4 plane) {
		Vec3 planenormal = new Vec3();
		VectorUtil.sub(planenormal, e2, e1);
		VectorUtil.cross(planenormal, planenormal, normal);
		planenormal.normalize();

		plane.set(planenormal.x, planenormal.y, planenormal.z, e2.dot(planenormal));
	}
	
	/**
	 * Finds the closest point(cp) to (v) on a segment (e1,e2).
	 */
	public static void closest_point_on_segment(Vec3 cp, Vec3 v, Vec3 e1, Vec3 e2) {
		Vec3 n = new Vec3();
		VectorUtil.sub(n, e2, e1);
		VectorUtil.sub(cp, v, e1);
		float _scalar = cp.dot(n) / n.dot(n);
		if (_scalar < 0.0f) {
			cp = e1;
		}
		else if (_scalar > 1.0f) {
			cp = e2;
		}
		else {
			VectorUtil.scaleAdd(cp, _scalar, n, e1);
		}
	}
	
	/**
	 * Line plane collision.
	 * 
	 * @return -0 if the ray never intersects, -1 if the ray collides in front, -2 if the ray collides in back
	 */
	public static int line_plane_collision(Vec4 plane, Vec3 vDir, Vec3 vPoint, Vec3 pout, float[] tparam, float tmin, float tmax) {
		float _dotdir = VectorUtil.dot3(vDir, plane);

		if (Math.abs(_dotdir) < PLANEDIREPSILON) {
			tparam[0] = tmax;
			return 0;
		}

		float _dis = ClipPolygon.distance_point_plane(plane, vPoint);
		int returnvalue = _dis < 0.0f ? 2 : 1;
		tparam[0] = -_dis / _dotdir;

		if (tparam[0] < tmin) {
			returnvalue = 0;
			tparam[0] = tmin;
		}
		else if (tparam[0] > tmax) {
			returnvalue = 0;
			tparam[0] = tmax;
		}
		VectorUtil.scaleAdd(pout, tparam[0], vDir, vPoint);
		return returnvalue;
	}
	
	/**
	 * Find closest points on segments.
	 */
	public static void segment_collision(Vec3 vA1, Vec3 vA2, Vec3 vB1, Vec3 vB2, Vec3 vPointA, Vec3 vPointB) {
		Vec3 AD = new Vec3();
		VectorUtil.sub(AD, vA2, vA1);

		Vec3 BD = new Vec3();
		VectorUtil.sub(BD, vB2, vB1);

		Vec3 N = new Vec3();
		VectorUtil.cross(N, AD, BD);
		float[] tp = new float[] { N.squareLength() };

		Vec4 _M = new Vec4();//plane

		if (tp[0] < BulletGlobals.SIMD_EPSILON)//ARE PARALELE
		{
			// project B over A
			boolean invert_b_order = false;
			_M.x = vB1.dot(AD);
			_M.y = vB2.dot(AD);

			if (_M.x > _M.y) {
				invert_b_order = true;
				//BT_SWAP_NUMBERS(_M[0],_M[1]);
				_M.x = _M.x + _M.y;
				_M.y = _M.x - _M.y;
				_M.x = _M.x - _M.y;
			}
			_M.z = vA1.dot(AD);
			_M.w = vA2.dot(AD);
			// mid points
			N.x = (_M.x + _M.y) * 0.5f;
			N.y = (_M.z + _M.w) * 0.5f;

			if (N.x < N.y) {
				if (_M.y < _M.z) {
					vPointB = invert_b_order ? vB1 : vB2;
					vPointA = vA1;
				}
				else if (_M.y < _M.w) {
					vPointB = invert_b_order ? vB1 : vB2;
					closest_point_on_segment(vPointA, vPointB, vA1, vA2);
				}
				else {
					vPointA = vA2;
					closest_point_on_segment(vPointB, vPointA, vB1, vB2);
				}
			}
			else {
				if (_M.w < _M.x) {
					vPointB = invert_b_order ? vB2 : vB1;
					vPointA = vA2;
				}
				else if (_M.w < _M.y) {
					vPointA = vA2;
					closest_point_on_segment(vPointB, vPointA, vB1, vB2);
				}
				else {
					vPointB = invert_b_order ? vB1 : vB2;
					closest_point_on_segment(vPointA, vPointB, vA1, vA2);
				}
			}
			return;
		}

		VectorUtil.cross(N, N, BD);
		_M.set(N.x, N.y, N.z, vB1.dot(N));

		// get point A as the plane collision point
		line_plane_collision(_M, AD, vA1, vPointA, tp, 0f, 1f);

		/*Closest point on segment*/
		VectorUtil.sub(vPointB, vPointA, vB1);
		tp[0] = vPointB.dot(BD);
		tp[0] /= BD.dot(BD);
		tp[0] = CLAMP(tp[0], 0.0f, 1.0f);

		VectorUtil.scaleAdd(vPointB, tp[0], BD, vB1);
	}
	
}