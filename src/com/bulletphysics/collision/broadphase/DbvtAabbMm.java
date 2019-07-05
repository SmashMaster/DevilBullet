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

// Dbvt implementation by Nathanael Presson

package com.bulletphysics.collision.broadphase;

import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import javax.vecmath.Vec3;

/**
 *
 * @author jezek2
 */
public class DbvtAabbMm {

	private final Vec3 mi = new Vec3();
	private final Vec3 mx = new Vec3();

	public DbvtAabbMm() {
	}

	public DbvtAabbMm(DbvtAabbMm o) {
		set(o);
	}
	
	public void set(DbvtAabbMm o) {
		mi.set(o.mi);
		mx.set(o.mx);
	}
	
	public static void swap(DbvtAabbMm p1, DbvtAabbMm p2) {
		Vec3 tmp = new Vec3();
		
		tmp.set(p1.mi);
		p1.mi.set(p2.mi);
		p2.mi.set(tmp);

		tmp.set(p1.mx);
		p1.mx.set(p2.mx);
		p2.mx.set(tmp);
	}

	public Vec3 Center(Vec3 out) {
		out.addHere(mi, mx);
		out.mult(0.5f);
		return out;
	}
	
	public Vec3 Lengths(Vec3 out) {
		out.subHere(mx, mi);
		return out;
	}
	
	public Vec3 Extents(Vec3 out) {
		out.subHere(mx, mi);
		out.mult(0.5f);
		return out;
	}
	
	public Vec3 Mins() {
		return mi;
	}

	public Vec3 Maxs() {
		return mx;
	}
	
	public static DbvtAabbMm FromCE(Vec3 c, Vec3 e, DbvtAabbMm out) {
		DbvtAabbMm box = out;
		box.mi.subHere(c, e);
		box.mx.addHere(c, e);
		return box;
	}

	public static DbvtAabbMm FromCR(Vec3 c, float r, DbvtAabbMm out) {
		Vec3 tmp = new Vec3();
		tmp.set(r, r, r);
		return FromCE(c, tmp, out);
	}

	public static DbvtAabbMm FromMM(Vec3 mi, Vec3 mx, DbvtAabbMm out) {
		DbvtAabbMm box = out;
		box.mi.set(mi);
		box.mx.set(mx);
		return box;
	}
	
	//public static  DbvtAabbMm	FromPoints( btVector3* pts,int n);
	//public static  DbvtAabbMm	FromPoints( btVector3** ppts,int n);
	
	public void Expand(Vec3 e) {
		mi.sub(e);
		mx.add(e);
	}

	public void SignedExpand(Vec3 e) {
		if (e.x > 0) {
			mx.x += e.x;
		}
		else {
			mi.x += e.x;
		}
		
		if (e.y > 0) {
			mx.y += e.y;
		}
		else {
			mi.y += e.y;
		}
		
		if (e.z > 0) {
			mx.z += e.z;
		}
		else {
			mi.z += e.z;
		}
	}

	public boolean Contain(DbvtAabbMm a) {
		return ((mi.x <= a.mi.x) &&
		        (mi.y <= a.mi.y) &&
		        (mi.z <= a.mi.z) &&
		        (mx.x >= a.mx.x) &&
		        (mx.y >= a.mx.y) &&
		        (mx.z >= a.mx.z));
	}

	public int Classify(Vec3 n, float o, int s) {
		Vec3 pi = new Vec3();
		Vec3 px = new Vec3();

		switch (s) {
			case (0 + 0 + 0):
				px.set(mi.x, mi.y, mi.z);
				pi.set(mx.x, mx.y, mx.z);
				break;
			case (1 + 0 + 0):
				px.set(mx.x, mi.y, mi.z);
				pi.set(mi.x, mx.y, mx.z);
				break;
			case (0 + 2 + 0):
				px.set(mi.x, mx.y, mi.z);
				pi.set(mx.x, mi.y, mx.z);
				break;
			case (1 + 2 + 0):
				px.set(mx.x, mx.y, mi.z);
				pi.set(mi.x, mi.y, mx.z);
				break;
			case (0 + 0 + 4):
				px.set(mi.x, mi.y, mx.z);
				pi.set(mx.x, mx.y, mi.z);
				break;
			case (1 + 0 + 4):
				px.set(mx.x, mi.y, mx.z);
				pi.set(mi.x, mx.y, mi.z);
				break;
			case (0 + 2 + 4):
				px.set(mi.x, mx.y, mx.z);
				pi.set(mx.x, mi.y, mi.z);
				break;
			case (1 + 2 + 4):
				px.set(mx.x, mx.y, mx.z);
				pi.set(mi.x, mi.y, mi.z);
				break;
		}
		
		if ((n.dot(px) + o) < 0) {
			return -1;
		}
		if ((n.dot(pi) + o) >= 0) {
			return +1;
		}
		return 0;
	}

	public float ProjectMinimum(Vec3 v, int signs) {
		Vec3[] b = new Vec3[] { mx, mi };
		Vec3 p = new Vec3();
		p.set(b[(signs >> 0) & 1].x,
		      b[(signs >> 1) & 1].y,
		      b[(signs >> 2) & 1].z);
		return p.dot(v);
	}
	 
	public static boolean Intersect(DbvtAabbMm a, DbvtAabbMm b) {
		return ((a.mi.x <= b.mx.x) &&
		        (a.mx.x >= b.mi.x) &&
		        (a.mi.y <= b.mx.y) &&
		        (a.mx.y >= b.mi.y) &&
		        (a.mi.z <= b.mx.z) &&
		        (a.mx.z >= b.mi.z));
	}

	public static boolean Intersect(DbvtAabbMm a, DbvtAabbMm b, Transform xform) {
		Vec3 d0 = new Vec3();
		Vec3 d1 = new Vec3();
		Vec3 tmp = new Vec3();

		// JAVA NOTE: check
		b.Center(d0);
		xform.transform(d0);
		d0.sub(a.Center(tmp));

		MatrixUtil.transposeTransform(d1, d0, xform.basis);

		float[] s0 = new float[] { 0, 0 };
		float[] s1 = new float[2];
		s1[0] = xform.origin.dot(d0);
		s1[1] = s1[0];

		a.AddSpan(d0, s0, 0, s0, 1);
		b.AddSpan(d1, s1, 0, s1, 1);
		if (s0[0] > (s1[1])) {
			return false;
		}
		if (s0[1] < (s1[0])) {
			return false;
		}
		return true;
	}

	public static boolean Intersect(DbvtAabbMm a, Vec3 b) {
		return ((b.x >= a.mi.x) &&
		        (b.y >= a.mi.y) &&
		        (b.z >= a.mi.z) &&
		        (b.x <= a.mx.x) &&
		        (b.y <= a.mx.y) &&
		        (b.z <= a.mx.z));
	}

	public static boolean Intersect(DbvtAabbMm a, Vec3 org, Vec3 invdir, int[] signs) {
		Vec3[] bounds = new Vec3[]{a.mi, a.mx};
		float txmin = (bounds[signs[0]].x - org.x) * invdir.x;
		float txmax = (bounds[1 - signs[0]].x - org.x) * invdir.x;
		float tymin = (bounds[signs[1]].y - org.y) * invdir.y;
		float tymax = (bounds[1 - signs[1]].y - org.y) * invdir.y;
		if ((txmin > tymax) || (tymin > txmax)) {
			return false;
		}
		
		if (tymin > txmin) {
			txmin = tymin;
		}
		if (tymax < txmax) {
			txmax = tymax;
		}
		float tzmin = (bounds[signs[2]].z - org.z) * invdir.z;
		float tzmax = (bounds[1 - signs[2]].z - org.z) * invdir.z;
		if ((txmin > tzmax) || (tzmin > txmax)) {
			return false;
		}
		
		if (tzmin > txmin) {
			txmin = tzmin;
		}
		if (tzmax < txmax) {
			txmax = tzmax;
		}
		return (txmax > 0);
	}

	public static float Proximity(DbvtAabbMm a, DbvtAabbMm b) {
		Vec3 d = new Vec3();
		Vec3 tmp = new Vec3();

		d.addHere(a.mi, a.mx);
		tmp.addHere(b.mi, b.mx);
		d.sub(tmp);
		return Math.abs(d.x) + Math.abs(d.y) + Math.abs(d.z);
	}

	public static void Merge(DbvtAabbMm a, DbvtAabbMm b, DbvtAabbMm r) {
		for (int i=0; i<3; i++) {
			if (VectorUtil.getCoord(a.mi, i) < VectorUtil.getCoord(b.mi, i)) {
				VectorUtil.setCoord(r.mi, i, VectorUtil.getCoord(a.mi, i));
			}
			else {
				VectorUtil.setCoord(r.mi, i, VectorUtil.getCoord(b.mi, i));
			}
			
			if (VectorUtil.getCoord(a.mx, i) > VectorUtil.getCoord(b.mx, i)) {
				VectorUtil.setCoord(r.mx, i, VectorUtil.getCoord(a.mx, i));
			}
			else {
				VectorUtil.setCoord(r.mx, i, VectorUtil.getCoord(b.mx, i));
			}
		}
	}

	public static boolean NotEqual(DbvtAabbMm a, DbvtAabbMm b) {
		return ((a.mi.x != b.mi.x) ||
		        (a.mi.y != b.mi.y) ||
		        (a.mi.z != b.mi.z) ||
		        (a.mx.x != b.mx.x) ||
		        (a.mx.y != b.mx.y) ||
		        (a.mx.z != b.mx.z));
	}
	
	private void AddSpan(Vec3 d, float[] smi, int smi_idx, float[] smx, int smx_idx) {
		for (int i=0; i<3; i++) {
			if (VectorUtil.getCoord(d, i) < 0) {
				smi[smi_idx] += VectorUtil.getCoord(mx, i) * VectorUtil.getCoord(d, i);
				smx[smx_idx] += VectorUtil.getCoord(mi, i) * VectorUtil.getCoord(d, i);
			}
			else {
				smi[smi_idx] += VectorUtil.getCoord(mi, i) * VectorUtil.getCoord(d, i);
				smx[smx_idx] += VectorUtil.getCoord(mx, i) * VectorUtil.getCoord(d, i);
			}
		}
	}
	
}
