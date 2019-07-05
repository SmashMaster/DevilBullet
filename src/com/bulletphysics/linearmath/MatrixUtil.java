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
import com.bulletphysics.util.ArrayPool;
import com.samrj.devil.math.Mat3;
import com.samrj.devil.math.Quat;
import javax.vecmath.Vec3;

/**
 * Utility functions for matrices.
 * 
 * @author jezek2
 */
public class MatrixUtil {
        public static void transform(Mat3 m, Vec3 v, Vec3 r) {
            float x = m.a*v.x + m.b*v.y + m.c*v.z; 
            float y = m.d*v.x + m.e*v.y + m.f*v.z; 
            float z = m.g*v.x + m.h*v.y + m.i*v.z; 
            r.set(x, y, z);
        }
    
        public static void transform(Mat3 matrix, Vec3 vector) {
                transform(matrix, vector, vector);
        }
        
        public static void getRow(Mat3 m, int i, Vec3 r)
        {
            switch (i)
            {
                case 0: r.x = m.a;
                        r.y = m.b;
                        r.z = m.c; return;
                case 1: r.x = m.d;
                        r.y = m.e;
                        r.z = m.f; return;
                case 2: r.x = m.g;
                        r.y = m.h;
                        r.z = m.i; return;
                default: throw new ArrayIndexOutOfBoundsException();
            }
        }
        
        public static void getColumn(Mat3 m, int i, Vec3 r)
        {
            switch (i)
            {
                case 0: r.x = m.a;
                        r.y = m.d;
                        r.z = m.g; return;
                case 1: r.x = m.b;
                        r.y = m.e;
                        r.z = m.h; return;
                case 2: r.x = m.c;
                        r.y = m.f;
                        r.z = m.i; return;
                default: throw new ArrayIndexOutOfBoundsException();
            }
        }
        
        public static void setRow(Mat3 m, int i, Vec3 v)
        {
            switch (i)
            {
                case 0: m.a = v.x;
                        m.b = v.y;
                        m.c = v.z; return;
                case 1: m.d = v.x;
                        m.e = v.y;
                        m.f = v.z; return;
                case 2: m.g = v.x;
                        m.h = v.y;
                        m.i = v.z; return;
                default: throw new ArrayIndexOutOfBoundsException();
            }
        }
        
        public static void setColumn(Mat3 m, int i, Vec3 v)
        {
            switch (i)
            {
                case 0: m.a = v.x;
                        m.d = v.y;
                        m.g = v.z; return;
                case 1: m.b = v.x;
                        m.e = v.y;
                        m.h = v.z; return;
                case 2: m.c = v.x;
                        m.f = v.y;
                        m.i = v.z; return;
                default: throw new ArrayIndexOutOfBoundsException();
            }
        }
	
	public static void scale(Mat3 dest, Mat3 mat, Vec3 s) {
		dest.a = mat.a * s.x;   dest.b = mat.b * s.y;   dest.c = mat.c * s.z;
		dest.d = mat.d * s.x;   dest.e = mat.e * s.y;   dest.f = mat.f * s.z;
		dest.g = mat.g * s.x;   dest.h = mat.h * s.y;   dest.i = mat.i * s.z;
	}
	
	public static void absolute(Mat3 mat) {
		mat.a = Math.abs(mat.a);
		mat.b = Math.abs(mat.b);
		mat.c = Math.abs(mat.c);
		mat.d = Math.abs(mat.d);
		mat.e = Math.abs(mat.e);
		mat.f = Math.abs(mat.f);
		mat.g = Math.abs(mat.g);
		mat.h = Math.abs(mat.h);
		mat.i = Math.abs(mat.i);
	}
	
	public static void setFromOpenGLSubMatrix(Mat3 mat, float[] m) {
		mat.a = m[0]; mat.b = m[4]; mat.c = m[8];
		mat.d = m[1]; mat.e = m[5]; mat.f = m[9];
		mat.g = m[2]; mat.h = m[6]; mat.i = m[10];
	}

	public static void getOpenGLSubMatrix(Mat3 mat, float[] m) {
		m[0] = mat.a;
		m[1] = mat.d;
		m[2] = mat.g;
		m[3] = 0f;
		m[4] = mat.b;
		m[5] = mat.e;
		m[6] = mat.h;
		m[7] = 0f;
		m[8] = mat.c;
		m[9] = mat.f;
		m[10] = mat.i;
		m[11] = 0f;
	}
	
	/**
	 * Sets rotation matrix from euler angles. The euler angles are applied in ZYX
	 * order. This means a vector is first rotated about X then Y and then Z axis.
	 */
	public static void setEulerZYX(Mat3 mat, float eulerX, float eulerY, float eulerZ) {
		float ci = (float) Math.cos(eulerX);
		float cj = (float) Math.cos(eulerY);
		float ch = (float) Math.cos(eulerZ);
		float si = (float) Math.sin(eulerX);
		float sj = (float) Math.sin(eulerY);
		float sh = (float) Math.sin(eulerZ);
		float cc = ci * ch;
		float cs = ci * sh;
		float sc = si * ch;
		float ss = si * sh;

		mat.set(cj * ch, sj * sc - cs, sj * cc + ss,
                        cj * sh, sj * ss + cc, sj * cs - sc,
                        -sj, cj * si, cj * ci);
	}
	
	private static float tdotx(Mat3 mat, Vec3 vec) {
		return mat.a * vec.x + mat.d * vec.y + mat.g * vec.z;
	}

	private static float tdoty(Mat3 mat, Vec3 vec) {
		return mat.b * vec.x + mat.e * vec.y + mat.h * vec.z;
	}

	private static float tdotz(Mat3 mat, Vec3 vec) {
		return mat.c * vec.x + mat.f * vec.y + mat.i * vec.z;
	}
	
	public static void transposeTransform(Vec3 dest, Vec3 vec, Mat3 mat) {
		float x = tdotx(mat, vec);
		float y = tdoty(mat, vec);
		float z = tdotz(mat, vec);
		dest.x = x;
		dest.y = y;
		dest.z = z;
	}
	
	public static void setRotation(Mat3 dest, Quat q) {
		float d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
		assert (d != 0f);
		float s = 2f / d;
		float xs = q.x * s, ys = q.y * s, zs = q.z * s;
		float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
		float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
		float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
		dest.a = 1f - (yy + zz);
		dest.b = xy - wz;
		dest.c = xz + wy;
		dest.d = xy + wz;
		dest.e = 1f - (xx + zz);
		dest.f = yz - wx;
		dest.g = xz - wy;
		dest.h = yz + wx;
		dest.i = 1f - (xx + yy);
	}
	
	public static void getRotation(Mat3 mat, Quat dest) {
		ArrayPool<float[]> floatArrays = ArrayPool.get(float.class);
		
		float trace = mat.a + mat.e + mat.i;
		float[] temp = floatArrays.getFixed(4);

		if (trace > 0f) {
			float s = (float) Math.sqrt(trace + 1f);
			temp[3] = (s * 0.5f);
			s = 0.5f / s;

			temp[0] = ((mat.h - mat.f) * s);
			temp[1] = ((mat.c - mat.g) * s);
			temp[2] = ((mat.d - mat.b) * s);
		}
		else {
			int i = mat.a < mat.e ? (mat.e < mat.i ? 2 : 1) : (mat.a < mat.i ? 2 : 0);
			int j = (i + 1) % 3;
			int k = (i + 2) % 3;

			float s = (float) Math.sqrt(mat.getEntry(i, i) - mat.getEntry(j, j) - mat.getEntry(k, k) + 1f);
			temp[i] = s * 0.5f;
			s = 0.5f / s;

			temp[3] = (mat.getEntry(k, j) - mat.getEntry(j, k)) * s;
			temp[j] = (mat.getEntry(j, i) + mat.getEntry(i, j)) * s;
			temp[k] = (mat.getEntry(k, i) + mat.getEntry(i, k)) * s;
		}
		dest.set(temp[3], temp[0], temp[1], temp[2]);
		
		floatArrays.release(temp);
	}

	private static float cofac(Mat3 mat, int r1, int c1, int r2, int c2) {
		return mat.getEntry(r1, c1) * mat.getEntry(r2, c2) - mat.getEntry(r1, c2) * mat.getEntry(r2, c1);
	}
	
	public static void invert(Mat3 mat) {
		float co_x = cofac(mat, 1, 1, 2, 2);
		float co_y = cofac(mat, 1, 2, 2, 0);
		float co_z = cofac(mat, 1, 0, 2, 1);
		
		float det = mat.a*co_x + mat.b*co_y + mat.c*co_z;
		assert (det != 0f);
		
		float s = 1f / det;
		float m00 = co_x * s;
		float m01 = cofac(mat, 0, 2, 2, 1) * s;
		float m02 = cofac(mat, 0, 1, 1, 2) * s;
		float m10 = co_y * s;
		float m11 = cofac(mat, 0, 0, 2, 2) * s;
		float m12 = cofac(mat, 0, 2, 1, 0) * s;
		float m20 = co_z * s;
		float m21 = cofac(mat, 0, 1, 2, 0) * s;
		float m22 = cofac(mat, 0, 0, 1, 1) * s;
		
		mat.a = m00;
		mat.b = m01;
		mat.c = m02;
		mat.d = m10;
		mat.e = m11;
		mat.f = m12;
		mat.g = m20;
		mat.h = m21;
		mat.i = m22;
	}

	/**
	 * Diagonalizes this matrix by the Jacobi method. rot stores the rotation
	 * from the coordinate system in which the matrix is diagonal to the original
	 * coordinate system, i.e., old_this = rot * new_this * rot^T. The iteration
	 * stops when all off-diagonal elements are less than the threshold multiplied
	 * by the sum of the absolute values of the diagonal, or when maxSteps have
	 * been executed. Note that this matrix is assumed to be symmetric.
	 */
	// JAVA NOTE: diagonalize method from 2.71
	public static void diagonalize(Mat3 mat, Mat3 rot, float threshold, int maxSteps) {
		Vec3 row = new Vec3();

		rot.setIdentity();
		for (int step = maxSteps; step > 0; step--) {
			// find off-diagonal element [p][q] with largest magnitude
			int p = 0;
			int q = 1;
			int r = 2;
			float max = Math.abs(mat.b);
			float v = Math.abs(mat.c);
			if (v > max) {
				q = 2;
				r = 1;
				max = v;
			}
			v = Math.abs(mat.f);
			if (v > max) {
				p = 1;
				q = 2;
				r = 0;
				max = v;
			}

			float t = threshold * (Math.abs(mat.a) + Math.abs(mat.e) + Math.abs(mat.i));
			if (max <= t) {
				if (max <= BulletGlobals.SIMD_EPSILON * t) {
					return;
				}
				step = 1;
			}

			// compute Jacobi rotation J which leads to a zero for element [p][q]
			float mpq = mat.getEntry(p, q);
			float theta = (mat.getEntry(q, q) - mat.getEntry(p, p)) / (2 * mpq);
			float theta2 = theta * theta;
			float cos;
			float sin;
			if ((theta2 * theta2) < (10f / BulletGlobals.SIMD_EPSILON)) {
				t = (theta >= 0f) ? 1f / (theta + (float) Math.sqrt(1f + theta2))
						: 1f / (theta - (float) Math.sqrt(1f + theta2));
				cos = 1f / (float) Math.sqrt(1f + t * t);
				sin = cos * t;
			}
			else {
				// approximation for large theta-value, i.e., a nearly diagonal matrix
				t = 1 / (theta * (2 + 0.5f / theta2));
				cos = 1 - 0.5f * t * t;
				sin = cos * t;
			}

			// apply rotation to matrix (this = J^T * this * J)
			mat.setEntry(p, q, 0f);
			mat.setEntry(q, p, 0f);
			mat.setEntry(p, p, mat.getEntry(p, p) - t * mpq);
			mat.setEntry(q, q, mat.getEntry(q, q) + t * mpq);
			float mrp = mat.getEntry(r, p);
			float mrq = mat.getEntry(r, q);
			mat.setEntry(r, p, cos * mrp - sin * mrq);
			mat.setEntry(p, r, cos * mrp - sin * mrq);
			mat.setEntry(r, q, cos * mrq + sin * mrp);
			mat.setEntry(q, r, cos * mrq + sin * mrp);

			// apply rotation to rot (rot = rot * J)
			for (int i=0; i<3; i++) {
                                MatrixUtil.getRow(rot, i, row);

				mrp = VectorUtil.getCoord(row, p);
				mrq = VectorUtil.getCoord(row, q);
				VectorUtil.setCoord(row, p, cos * mrp - sin * mrq);
				VectorUtil.setCoord(row, q, cos * mrq + sin * mrp);
                                MatrixUtil.setRow(rot, i, row);
			}
		}
	}
	
}
