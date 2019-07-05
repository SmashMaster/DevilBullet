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
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Vec4;
import javax.vecmath.Mat3;
import javax.vecmath.Vec3;

/**
 *
 * @author jezek2
 */
class BoxCollision {
	
	public static final float BOX_PLANE_EPSILON = 0.000001f;

	public static boolean BT_GREATER(float x, float y) {
		return Math.abs(x) > y;
	}

	public static float BT_MAX3(float a, float b, float c) {
		return Math.max(a, Math.max(b, c));
	}

	public static float BT_MIN3(float a, float b, float c) {
		return Math.min(a, Math.min(b, c));
	}
	
	public static boolean TEST_CROSS_EDGE_BOX_MCR(Vec3 edge, Vec3 absolute_edge, Vec3 pointa, Vec3 pointb, Vec3 _extend, int i_dir_0, int i_dir_1, int i_comp_0, int i_comp_1) {
		float dir0 = -VectorUtil.getCoord(edge, i_dir_0);
		float dir1 = VectorUtil.getCoord(edge, i_dir_1);
		float pmin = VectorUtil.getCoord(pointa, i_comp_0) * dir0 + VectorUtil.getCoord(pointa, i_comp_1) * dir1;
		float pmax = VectorUtil.getCoord(pointb, i_comp_0) * dir0 + VectorUtil.getCoord(pointb, i_comp_1) * dir1;
		if (pmin > pmax) {
			//BT_SWAP_NUMBERS(pmin,pmax);
			pmin = pmin + pmax;
			pmax = pmin - pmax;
			pmin = pmin - pmax;
		}
		float abs_dir0 = VectorUtil.getCoord(absolute_edge, i_dir_0);
		float abs_dir1 = VectorUtil.getCoord(absolute_edge, i_dir_1);
		float rad = VectorUtil.getCoord(_extend, i_comp_0) * abs_dir0 + VectorUtil.getCoord(_extend, i_comp_1) * abs_dir1;
		if (pmin > rad || -rad > pmax) {
			return false;
		}
		return true;
	}

	public static boolean TEST_CROSS_EDGE_BOX_X_AXIS_MCR(Vec3 edge, Vec3 absolute_edge, Vec3 pointa, Vec3 pointb, Vec3 _extend) {
		return TEST_CROSS_EDGE_BOX_MCR(edge, absolute_edge, pointa, pointb, _extend, 2, 1, 1, 2);
	}

	public static boolean TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(Vec3 edge, Vec3 absolute_edge, Vec3 pointa, Vec3 pointb, Vec3 _extend) {
		return TEST_CROSS_EDGE_BOX_MCR(edge, absolute_edge, pointa, pointb, _extend, 0, 2, 2, 0);
	}

	public static boolean TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(Vec3 edge, Vec3 absolute_edge, Vec3 pointa, Vec3 pointb, Vec3 _extend) {
		return TEST_CROSS_EDGE_BOX_MCR(edge, absolute_edge, pointa, pointb, _extend, 1, 0, 0, 1);
	}
	
	/**
	 * Returns the dot product between a vec3f and the col of a matrix.
	 */
	public static float bt_mat3_dot_col(Mat3 mat, Vec3 vec3, int colindex) {
		return vec3.x*mat.getEntry(0, colindex) + vec3.y*mat.getEntry(1, colindex) + vec3.z*mat.getEntry(2, colindex);
	}

	/**
	 * Compairison of transformation objects.
	 */
	public static boolean compareTransformsEqual(Transform t1, Transform t2) {
		return t1.equals(t2);
	}
	
	////////////////////////////////////////////////////////////////////////////

	public static class BoxBoxTransformCache {
		public final Vec3 T1to0 = new Vec3(); // Transforms translation of model1 to model 0
		public final Mat3 R1to0 = new Mat3(); // Transforms Rotation of model1 to model 0, equal  to R0' * R1
		public final Mat3 AR = new Mat3();    // Absolute value of m_R1to0
		
		public void set(BoxBoxTransformCache cache) {
			throw new UnsupportedOperationException();
		}
		
		public void calc_absolute_matrix() {
			//static const btVector3 vepsi(1e-6f,1e-6f,1e-6f);
			//m_AR[0] = vepsi + m_R1to0[0].absolute();
			//m_AR[1] = vepsi + m_R1to0[1].absolute();
			//m_AR[2] = vepsi + m_R1to0[2].absolute();

			for (int i=0; i<3; i++) {
				for (int j=0; j<3; j++) {
					AR.setEntry(i, j, 1e-6f + Math.abs(R1to0.getEntry(i, j)));
				}
			}
		}

		/**
		 * Calc the transformation relative  1 to 0. Inverts matrics by transposing.
		 */
		public void calc_from_homogenic(Transform trans0, Transform trans1) {
			Transform temp_trans = new Transform();
			temp_trans.inverse(trans0);
			temp_trans.mul(trans1);

			T1to0.set(temp_trans.origin);
			R1to0.set(temp_trans.basis);

			calc_absolute_matrix();
		}
		
		/**
		 * Calcs the full invertion of the matrices. Useful for scaling matrices.
		 */
		public void calc_from_full_invert(Transform trans0, Transform trans1) {
			R1to0.invertHere(trans0.basis);
			T1to0.negateHere(trans0.origin);
			R1to0.transform(T1to0);

			Vec3 tmp = new Vec3();
			tmp.set(trans1.origin);
			R1to0.transform(tmp);
			T1to0.add(tmp);

			R1to0.mult(trans1.basis);

			calc_absolute_matrix();
		}
		
		public Vec3 transform(Vec3 point, Vec3 out) {
			if (point == out) {
				point = new Vec3(point);
			}
			
			Vec3 tmp = new Vec3();
			R1to0.getRow(0, tmp);
			out.x = tmp.dot(point) + T1to0.x;
			R1to0.getRow(1, tmp);
			out.y = tmp.dot(point) + T1to0.y;
			R1to0.getRow(2, tmp);
			out.z = tmp.dot(point) + T1to0.z;
			return out;
		}
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	public static class AABB {
		public final Vec3 min = new Vec3();
		public final Vec3 max = new Vec3();
		
		public AABB() {
		}

		public AABB(Vec3 V1, Vec3 V2, Vec3 V3) {
			calc_from_triangle(V1, V2, V3);
		}

		public AABB(Vec3 V1, Vec3 V2, Vec3 V3, float margin) {
			calc_from_triangle_margin(V1, V2, V3, margin);
		}

		public AABB(AABB other) {
			set(other);
		}

		public AABB(AABB other, float margin) {
			this(other);
			min.x -= margin;
			min.y -= margin;
			min.z -= margin;
			max.x += margin;
			max.y += margin;
			max.z += margin;
		}

		public void init(Vec3 V1, Vec3 V2, Vec3 V3, float margin) {
			calc_from_triangle_margin(V1, V2, V3, margin);
		}

		public void set(AABB other) {
			min.set(other.min);
			max.set(other.max);
		}

		public void invalidate() {
			min.set(BulletGlobals.SIMD_INFINITY, BulletGlobals.SIMD_INFINITY, BulletGlobals.SIMD_INFINITY);
			max.set(-BulletGlobals.SIMD_INFINITY, -BulletGlobals.SIMD_INFINITY, -BulletGlobals.SIMD_INFINITY);
		}

		public void increment_margin(float margin) {
			min.x -= margin;
			min.y -= margin;
			min.z -= margin;
			max.x += margin;
			max.y += margin;
			max.z += margin;
		}

		public void copy_with_margin(AABB other, float margin) {
			min.x = other.min.x - margin;
			min.y = other.min.y - margin;
			min.z = other.min.z - margin;

			max.x = other.max.x + margin;
			max.y = other.max.y + margin;
			max.z = other.max.z + margin;
		}
		
		public void calc_from_triangle(Vec3 V1, Vec3 V2, Vec3 V3) {
			min.x = BT_MIN3(V1.x, V2.x, V3.x);
			min.y = BT_MIN3(V1.y, V2.y, V3.y);
			min.z = BT_MIN3(V1.z, V2.z, V3.z);

			max.x = BT_MAX3(V1.x, V2.x, V3.x);
			max.y = BT_MAX3(V1.y, V2.y, V3.y);
			max.z = BT_MAX3(V1.z, V2.z, V3.z);
		}

		public void calc_from_triangle_margin(Vec3 V1, Vec3 V2, Vec3 V3, float margin) {
			calc_from_triangle(V1, V2, V3);
			min.x -= margin;
			min.y -= margin;
			min.z -= margin;
			max.x += margin;
			max.y += margin;
			max.z += margin;
		}
		
		/**
		 * Apply a transform to an AABB.
		 */
		public void appy_transform(Transform trans) {
			Vec3 tmp = new Vec3();

			Vec3 center = new Vec3();
			center.addHere(max, min);
			center.mult(0.5f);

			Vec3 extends_ = new Vec3();
			extends_.subHere(max, center);

			// Compute new center
			trans.transform(center);

			Vec3 textends = new Vec3();

			trans.basis.getRow(0, tmp);
			tmp.absolute();
			textends.x = extends_.dot(tmp);

			trans.basis.getRow(1, tmp);
			tmp.absolute();
			textends.y = extends_.dot(tmp);

			trans.basis.getRow(2, tmp);
			tmp.absolute();
			textends.z = extends_.dot(tmp);

			min.subHere(center, textends);
			max.addHere(center, textends);
		}

		/**
		 * Apply a transform to an AABB.
		 */
		public void appy_transform_trans_cache(BoxBoxTransformCache trans) {
			Vec3 tmp = new Vec3();

			Vec3 center = new Vec3();
			center.addHere(max, min);
			center.mult(0.5f);

			Vec3 extends_ = new Vec3();
			extends_.subHere(max, center);

			// Compute new center
			trans.transform(center, center);

			Vec3 textends = new Vec3();

			trans.R1to0.getRow(0, tmp);
			tmp.absolute();
			textends.x = extends_.dot(tmp);

			trans.R1to0.getRow(1, tmp);
			tmp.absolute();
			textends.y = extends_.dot(tmp);

			trans.R1to0.getRow(2, tmp);
			tmp.absolute();
			textends.z = extends_.dot(tmp);

			min.subHere(center, textends);
			max.addHere(center, textends);
		}
		
		/**
		 * Merges a Box.
		 */
		public void merge(AABB box) {
			min.x = Math.min(min.x, box.min.x);
			min.y = Math.min(min.y, box.min.y);
			min.z = Math.min(min.z, box.min.z);

			max.x = Math.max(max.x, box.max.x);
			max.y = Math.max(max.y, box.max.y);
			max.z = Math.max(max.z, box.max.z);
		}

		/**
		 * Merges a point.
		 */
		public void merge_point(Vec3 point) {
			min.x = Math.min(min.x, point.x);
			min.y = Math.min(min.y, point.y);
			min.z = Math.min(min.z, point.z);

			max.x = Math.max(max.x, point.x);
			max.y = Math.max(max.y, point.y);
			max.z = Math.max(max.z, point.z);
		}
		
		/**
		 * Gets the extend and center.
		 */
		public void get_center_extend(Vec3 center, Vec3 extend) {
			center.addHere(max, min);
			center.mult(0.5f);

			extend.subHere(max, center);
		}
		
		/**
		 * Finds the intersecting box between this box and the other.
		 */
		public void find_intersection(AABB other, AABB intersection) {
			intersection.min.x = Math.max(other.min.x, min.x);
			intersection.min.y = Math.max(other.min.y, min.y);
			intersection.min.z = Math.max(other.min.z, min.z);

			intersection.max.x = Math.min(other.max.x, max.x);
			intersection.max.y = Math.min(other.max.y, max.y);
			intersection.max.z = Math.min(other.max.z, max.z);
		}

		public boolean has_collision(AABB other) {
			if (min.x > other.max.x ||
			    max.x < other.min.x ||
			    min.y > other.max.y ||
			    max.y < other.min.y ||
			    min.z > other.max.z ||
			    max.z < other.min.z) {
				return false;
			}
			return true;
		}
		
		/**
		 * Finds the Ray intersection parameter.
		 * 
		 * @param aabb     aligned box
		 * @param vorigin  a vec3f with the origin of the ray
		 * @param vdir     a vec3f with the direction of the ray
		 */
		public boolean collide_ray(Vec3 vorigin, Vec3 vdir) {
			Vec3 extents = new Vec3(), center = new Vec3();
			get_center_extend(center, extents);

			float Dx = vorigin.x - center.x;
			if (BT_GREATER(Dx, extents.x) && Dx * vdir.x >= 0.0f) return false;
			
			float Dy = vorigin.y - center.y;
			if (BT_GREATER(Dy, extents.y) && Dy * vdir.y >= 0.0f) return false;
			
			float Dz = vorigin.z - center.z;
			if (BT_GREATER(Dz, extents.z) && Dz * vdir.z >= 0.0f) return false;
			
			float f = vdir.y * Dz - vdir.z * Dy;
			if (Math.abs(f) > extents.y * Math.abs(vdir.z) + extents.z * Math.abs(vdir.y)) return false;
			
			f = vdir.z * Dx - vdir.x * Dz;
			if (Math.abs(f) > extents.x * Math.abs(vdir.z) + extents.z * Math.abs(vdir.x)) return false;
			
			f = vdir.x * Dy - vdir.y * Dx;
			if (Math.abs(f) > extents.x * Math.abs(vdir.y) + extents.y * Math.abs(vdir.x)) return false;
			
			return true;
		}
	
		public void projection_interval(Vec3 direction, float[] vmin, float[] vmax) {
			Vec3 tmp = new Vec3();

			Vec3 center = new Vec3();
			Vec3 extend = new Vec3();
			get_center_extend(center, extend);

			float _fOrigin = direction.dot(center);
			tmp.absoluteHere(direction);
			float _fMaximumExtent = extend.dot(tmp);
			vmin[0] = _fOrigin - _fMaximumExtent;
			vmax[0] = _fOrigin + _fMaximumExtent;
		}

		public PlaneIntersectionType plane_classify(Vec4 plane) {
			Vec3 tmp = new Vec3();

			float[] _fmin = new float[1], _fmax = new float[1];
			tmp.set(plane.x, plane.y, plane.z);
			projection_interval(tmp, _fmin, _fmax);

			if (plane.w > _fmax[0] + BOX_PLANE_EPSILON) {
				return PlaneIntersectionType.BACK_PLANE; // 0
			}

			if (plane.w + BOX_PLANE_EPSILON >= _fmin[0]) {
				return PlaneIntersectionType.COLLIDE_PLANE; //1
			}
			
			return PlaneIntersectionType.FRONT_PLANE; //2
		}
		
		public boolean overlapping_trans_conservative(AABB box, Transform trans1_to_0) {
			AABB tbox = new AABB(box);
			tbox.appy_transform(trans1_to_0);
			return has_collision(tbox);
		}

		public boolean overlapping_trans_conservative2(AABB box, BoxBoxTransformCache trans1_to_0) {
			AABB tbox = new AABB(box);
			tbox.appy_transform_trans_cache(trans1_to_0);
			return has_collision(tbox);
		}

		/**
		 * transcache is the transformation cache from box to this AABB.
		 */
		public boolean overlapping_trans_cache(AABB box, BoxBoxTransformCache transcache, boolean fulltest) {
			Vec3 tmp = new Vec3();

			// Taken from OPCODE
			Vec3 ea = new Vec3(), eb = new Vec3(); //extends
			Vec3 ca = new Vec3(), cb = new Vec3(); //extends
			get_center_extend(ca, ea);
			box.get_center_extend(cb, eb);

			Vec3 T = new Vec3();
			float t, t2;

			// Class I : A's basis vectors
			for (int i=0; i<3; i++) {
				transcache.R1to0.getRow(i, tmp);
				VectorUtil.setCoord(T, i, tmp.dot(cb) + VectorUtil.getCoord(transcache.T1to0, i) - VectorUtil.getCoord(ca, i));

				transcache.AR.getRow(i, tmp);
				t = tmp.dot(eb) + VectorUtil.getCoord(ea, i);
				if (BT_GREATER(VectorUtil.getCoord(T, i), t)) {
					return false;
				}
			}
			// Class II : B's basis vectors
			for (int i=0; i<3; i++) {
				t = bt_mat3_dot_col(transcache.R1to0, T, i);
				t2 = bt_mat3_dot_col(transcache.AR, ea, i) + VectorUtil.getCoord(eb, i);
				if (BT_GREATER(t, t2)) {
					return false;
				}
			}
			// Class III : 9 cross products
			if (fulltest) {
				int m, n, o, p, q, r;
				for (int i = 0; i < 3; i++) {
					m = (i+1) % 3;
					n = (i+2) % 3;
					o = (i == 0)? 1:0;
					p = (i == 2)? 1:2;
					for (int j=0; j<3; j++) {
						q = j == 2 ? 1 : 2;
						r = j == 0 ? 1 : 0;
						t = VectorUtil.getCoord(T, n) * transcache.R1to0.getEntry(m, j) - VectorUtil.getCoord(T, m) * transcache.R1to0.getEntry(n, j);
						t2 = VectorUtil.getCoord(ea, o) * transcache.AR.getEntry(p, j) + VectorUtil.getCoord(ea, p) * transcache.AR.getEntry(o, j) +
								VectorUtil.getCoord(eb, r) * transcache.AR.getEntry(i, q) + VectorUtil.getCoord(eb, q) * transcache.AR.getEntry(i, r);
						if (BT_GREATER(t, t2)) {
							return false;
						}
					}
				}
			}
			return true;
		}
		
		/**
		 * Simple test for planes.
		 */
		public boolean collide_plane(Vec4 plane) {
			PlaneIntersectionType classify = plane_classify(plane);
			return (classify == PlaneIntersectionType.COLLIDE_PLANE);
		}
		
		/**
		 * Test for a triangle, with edges.
		 */
		public boolean collide_triangle_exact(Vec3 p1, Vec3 p2, Vec3 p3, Vec4 triangle_plane) {
			if (!collide_plane(triangle_plane)) {
				return false;
			}
			Vec3 center = new Vec3(), extends_ = new Vec3();
			get_center_extend(center, extends_);

			Vec3 v1 = new Vec3();
			v1.subHere(p1, center);
			Vec3 v2 = new Vec3();
			v2.subHere(p2, center);
			Vec3 v3 = new Vec3();
			v3.subHere(p3, center);

			// First axis
			Vec3 diff = new Vec3();
			diff.subHere(v2, v1);
			Vec3 abs_diff = new Vec3();
			abs_diff.absoluteHere(diff);

			// Test With X axis
			TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff, abs_diff, v1, v3, extends_);
			// Test With Y axis
			TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff, abs_diff, v1, v3, extends_);
			// Test With Z axis
			TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff, abs_diff, v1, v3, extends_);

			diff.subHere(v3, v2);
			abs_diff.absoluteHere(diff);

			// Test With X axis
			TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff, abs_diff, v2, v1, extends_);
			// Test With Y axis
			TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff, abs_diff, v2, v1, extends_);
			// Test With Z axis
			TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff, abs_diff, v2, v1, extends_);

			diff.subHere(v1, v3);
			abs_diff.absoluteHere(diff);

			// Test With X axis
			TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff, abs_diff, v3, v2, extends_);
			// Test With Y axis
			TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff, abs_diff, v3, v2, extends_);
			// Test With Z axis
			TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff, abs_diff, v3, v2, extends_);

			return true;
		}
	}
	
}
