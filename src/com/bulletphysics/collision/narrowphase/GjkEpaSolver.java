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

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.ObjectStackList;
import com.samrj.devil.math.Quat;
import java.util.Arrays;
import javax.vecmath.Mat3;
import javax.vecmath.Vec3;

/*
GJK-EPA collision solver by Nathanael Presson
Nov.2006
*/

/**
 * GjkEpaSolver contributed under zlib by Nathanael Presson.
 * 
 * @author jezek2
 */
public class GjkEpaSolver {

	protected final ArrayPool<float[]> floatArrays = ArrayPool.get(float.class);

	protected final ObjectStackList<Mkv> stackMkv = new ObjectStackList<Mkv>(Mkv.class);
	protected final ObjectStackList<He> stackHe = new ObjectStackList<He>(He.class);
	protected final ObjectStackList<Face> stackFace = new ObjectStackList<Face>(Face.class);

	protected void pushStack() {
		stackMkv.push();
		stackHe.push();
		stackFace.push();
	}

	protected void popStack() {
		stackMkv.pop();
		stackHe.pop();
		stackFace.pop();
	}

	public enum ResultsStatus {
		Separated,		/* Shapes doesnt penetrate												*/ 
		Penetrating,	/* Shapes are penetrating												*/ 
		GJK_Failed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/ 
		EPA_Failed,		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/ 
	}
	
	public static class Results {
		public ResultsStatus status;
		public final Vec3[] witnesses/*[2]*/ = new Vec3[] { new Vec3(), new Vec3() };
		public final Vec3 normal = new Vec3();
		public float depth;
		public int epa_iterations;
		public int gjk_iterations;
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	private static final float cstInf = BulletGlobals.SIMD_INFINITY;
	private static final float cstPi = BulletGlobals.SIMD_PI;
	private static final float cst2Pi = BulletGlobals.SIMD_2_PI;
	private static final int GJK_maxiterations = 128;
	private static final int GJK_hashsize = 1 << 6;
	private static final int GJK_hashmask = GJK_hashsize - 1;
	private static final float GJK_insimplex_eps = 0.0001f;
	private static final float GJK_sqinsimplex_eps = GJK_insimplex_eps * GJK_insimplex_eps;
	private static final int EPA_maxiterations = 256;
	private static final float EPA_inface_eps = 0.01f;
	private static final float EPA_accuracy = 0.001f;
	
	////////////////////////////////////////////////////////////////////////////

	public static class Mkv {
		public final Vec3 w = new Vec3(); // Minkowski vertice
		public final Vec3 r = new Vec3(); // Ray

		public void set(Mkv m) {
			w.set(m.w);
			r.set(m.r);
		}
	}

	public static class He {
		public final Vec3 v = new Vec3();
		public He n;
	}
	
	protected class GJK {
		//protected final BulletStack stack = BulletStack.get();
		
		//public btStackAlloc sa;
		//public Block sablock;
		public final He[] table = new He[GJK_hashsize];
		public final Mat3[] wrotations/*[2]*/ = new Mat3[] { new Mat3(), new Mat3() };
		public final Vec3[] positions/*[2]*/ = new Vec3[] { new Vec3(), new Vec3() };
		public final ConvexShape[] shapes = new ConvexShape[2];
		public final Mkv[] simplex = new Mkv[5];
		public final Vec3 ray = new Vec3();
		public /*unsigned*/ int order;
		public /*unsigned*/ int iterations;
		public float margin;
		public boolean failed;
		
		{
			for (int i=0; i<simplex.length; i++) simplex[i] = new Mkv();
		}

		public GJK() {
		}

		public GJK(/*StackAlloc psa,*/
				Mat3 wrot0, Vec3 pos0, ConvexShape shape0,
				Mat3 wrot1, Vec3 pos1, ConvexShape shape1) {
			this(wrot0, pos0, shape0, wrot1, pos1, shape1, 0f);
		}

		public GJK(/*StackAlloc psa,*/
				Mat3 wrot0, Vec3 pos0, ConvexShape shape0,
				Mat3 wrot1, Vec3 pos1, ConvexShape shape1,
				float pmargin) {
			init(wrot0, pos0, shape0, wrot1, pos1, shape1, pmargin);
		}
		
		public void init(/*StackAlloc psa,*/
				Mat3 wrot0, Vec3 pos0, ConvexShape shape0,
				Mat3 wrot1, Vec3 pos1, ConvexShape shape1,
				float pmargin) {
			pushStack();
			wrotations[0].set(wrot0);
			positions[0].set(pos0);
			shapes[0] = shape0;
			wrotations[1].set(wrot1);
			positions[1].set(pos1);
			shapes[1] = shape1;
			//sa		=psa;
			//sablock	=sa->beginBlock();
			margin = pmargin;
			failed = false;
		}
		
		public void destroy() {
			popStack();
		}
		
		// vdh: very dummy hash
		public /*unsigned*/ int Hash(Vec3 v) {
			int h = (int)(v.x * 15461) ^ (int)(v.y * 83003) ^ (int)(v.z * 15473);
			return (h * 169639) & GJK_hashmask;
		}

		public Vec3 LocalSupport(Vec3 d, /*unsigned*/ int i, Vec3 out) {
			Vec3 tmp = new Vec3();
			MatrixUtil.transposeTransform(tmp, d, wrotations[i]);

			shapes[i].localGetSupportingVertex(tmp, out);
			wrotations[i].transform(out);
			out.add(positions[i]);

			return out;
		}
		
		public void Support(Vec3 d, Mkv v) {
			v.r.set(d);

			Vec3 tmp1 = LocalSupport(d, 0, new Vec3());

			Vec3 tmp = new Vec3();
			tmp.set(d);
			tmp.negate();
			Vec3 tmp2 = LocalSupport(tmp, 1, new Vec3());

			v.w.subHere(tmp1, tmp2);
			v.w.scaleAddHere(margin, d, v.w);
		}

		public boolean FetchSupport() {
			int h = Hash(ray);
			He e = table[h];
			while (e != null) {
				if (e.v.equals(ray)) {
					--order;
					return false;
				}
				else {
					e = e.n;
				}
			}
			//e = (He*)sa->allocate(sizeof(He));
			//e = new He();
			e = stackHe.get();
			e.v.set(ray);
			e.n = table[h];
			table[h] = e;
			Support(ray, simplex[++order]);
			return (ray.dot(simplex[order].w) > 0);
		}

		public boolean SolveSimplex2(Vec3 ao, Vec3 ab) {
			if (ab.dot(ao) >= 0) {
				Vec3 cabo = new Vec3();
				cabo.crossHere(ab, ao);
				if (cabo.squareLength() > GJK_sqinsimplex_eps) {
					ray.crossHere(cabo, ab);
				}
				else {
					return true;
				}
			}
			else {
				order = 0;
				simplex[0].set(simplex[1]);
				ray.set(ao);
			}
			return (false);
		}

		public boolean SolveSimplex3(Vec3 ao, Vec3 ab, Vec3 ac)
		{
			Vec3 tmp = new Vec3();
			tmp.crossHere(ab, ac);
			return (SolveSimplex3a(ao,ab,ac,tmp));
		}
		
		public boolean SolveSimplex3a(Vec3 ao, Vec3 ab, Vec3 ac, Vec3 cabc) {
			// TODO: optimize

			Vec3 tmp = new Vec3();
			tmp.crossHere(cabc, ab);

			Vec3 tmp2 = new Vec3();
			tmp2.crossHere(cabc, ac);

			if (tmp.dot(ao) < -GJK_insimplex_eps) {
				order = 1;
				simplex[0].set(simplex[1]);
				simplex[1].set(simplex[2]);
				return SolveSimplex2(ao, ab);
			}
			else if (tmp2.dot(ao) > +GJK_insimplex_eps) {
				order = 1;
				simplex[1].set(simplex[2]);
				return SolveSimplex2(ao, ac);
			}
			else {
				float d = cabc.dot(ao);
				if (Math.abs(d) > GJK_insimplex_eps) {
					if (d > 0) {
						ray.set(cabc);
					}
					else {
						ray.negateHere(cabc);

						Mkv swapTmp = new Mkv();
						swapTmp.set(simplex[0]);
						simplex[0].set(simplex[1]);
						simplex[1].set(swapTmp);
					}
					return false;
				}
				else {
					return true;
				}
			}
		}
		
		public boolean SolveSimplex4(Vec3 ao, Vec3 ab, Vec3 ac, Vec3 ad) {
			// TODO: optimize

			Vec3 crs = new Vec3();

			Vec3 tmp = new Vec3();
			tmp.crossHere(ab, ac);

			Vec3 tmp2 = new Vec3();
			tmp2.crossHere(ac, ad);

			Vec3 tmp3 = new Vec3();
			tmp3.crossHere(ad, ab);

			if (tmp.dot(ao) > GJK_insimplex_eps) {
				crs.set(tmp);
				order = 2;
				simplex[0].set(simplex[1]);
				simplex[1].set(simplex[2]);
				simplex[2].set(simplex[3]);
				return SolveSimplex3a(ao, ab, ac, crs);
			}
			else if (tmp2.dot(ao) > GJK_insimplex_eps) {
				crs.set(tmp2);
				order = 2;
				simplex[2].set(simplex[3]);
				return SolveSimplex3a(ao, ac, ad, crs);
			}
			else if (tmp3.dot(ao) > GJK_insimplex_eps) {
				crs.set(tmp3);
				order = 2;
				simplex[1].set(simplex[0]);
				simplex[0].set(simplex[2]);
				simplex[2].set(simplex[3]);
				return SolveSimplex3a(ao, ad, ab, crs);
			}
			else {
				return (true);
			}
		}
		
		public boolean SearchOrigin() {
			Vec3 tmp = new Vec3();
			tmp.set(1f, 0f, 0f);
			return SearchOrigin(tmp);
		}
		
		public boolean SearchOrigin(Vec3 initray) {
			Vec3 tmp1 = new Vec3();
			Vec3 tmp2 = new Vec3();
			Vec3 tmp3 = new Vec3();
			Vec3 tmp4 = new Vec3();

			iterations = 0;
			order = -1;
			failed = false;
			ray.set(initray);
			ray.normalize();

			Arrays.fill(table, null);

			FetchSupport();
			ray.negateHere(simplex[0].w);
			for (; iterations < GJK_maxiterations; ++iterations) {
				float rl = ray.length();
				ray.mult(1f / (rl > 0f ? rl : 1f));
				if (FetchSupport()) {
					boolean found = false;
					switch (order) {
						case 1: {
							tmp1.negateHere(simplex[1].w);
							tmp2.subHere(simplex[0].w, simplex[1].w);
							found = SolveSimplex2(tmp1, tmp2);
							break;
						}
						case 2: {
							tmp1.negateHere(simplex[2].w);
							tmp2.subHere(simplex[1].w, simplex[2].w);
							tmp3.subHere(simplex[0].w, simplex[2].w);
							found = SolveSimplex3(tmp1, tmp2, tmp3);
							break;
						}
						case 3: {
							tmp1.negateHere(simplex[3].w);
							tmp2.subHere(simplex[2].w, simplex[3].w);
							tmp3.subHere(simplex[1].w, simplex[3].w);
							tmp4.subHere(simplex[0].w, simplex[3].w);
							found = SolveSimplex4(tmp1, tmp2, tmp3, tmp4);
							break;
						}
					}
					if (found) {
						return true;
					}
				}
				else {
					return false;
				}
			}
			failed = true;
			return false;
		}
		
		public boolean EncloseOrigin() {
			Vec3 tmp = new Vec3();
			Vec3 tmp1 = new Vec3();
			Vec3 tmp2 = new Vec3();

			switch (order) {
				// Point
				case 0:
					break;
				// Line
				case 1: {
					Vec3 ab = new Vec3();
					ab.subHere(simplex[1].w, simplex[0].w);

					Vec3[] b = new Vec3[] { new Vec3(), new Vec3(), new Vec3() };
					b[0].set(1f, 0f, 0f);
					b[1].set(0f, 1f, 0f);
					b[2].set(0f, 0f, 1f);
					
					b[0].crossHere(ab, b[0]);
					b[1].crossHere(ab, b[1]);
					b[2].crossHere(ab, b[2]);

					float m[] = new float[] { b[0].squareLength(), b[1].squareLength(), b[2].squareLength() };

					Quat tmpQuat = new Quat();
					tmp.normalizeHere(ab);
					QuaternionUtil.setRotation(tmpQuat, tmp, cst2Pi / 3f);

					Mat3 r = new Mat3();
					MatrixUtil.setRotation(r, tmpQuat);

					Vec3 w = new Vec3();
					w.set(b[m[0] > m[1] ? m[0] > m[2] ? 0 : 2 : m[1] > m[2] ? 1 : 2]);

					tmp.normalizeHere(w);
					Support(tmp, simplex[4]); r.transform(w);
					tmp.normalizeHere(w);
					Support(tmp, simplex[2]); r.transform(w);
					tmp.normalizeHere(w);
					Support(tmp, simplex[3]); r.transform(w);
					order = 4;
					return (true);
				}
				// Triangle
				case 2: {
					tmp1.subHere(simplex[1].w, simplex[0].w);
					tmp2.subHere(simplex[2].w, simplex[0].w);
					Vec3 n = new Vec3();
					n.crossHere(tmp1, tmp2);
					n.normalize();

					Support(n, simplex[3]);

					tmp.negateHere(n);
					Support(tmp, simplex[4]);
					order = 4;
					return (true);
				}
				// Tetrahedron
				case 3:
					return (true);
				// Hexahedron
				case 4:
					return (true);
			}
			return (false);
		}
		
	}

	////////////////////////////////////////////////////////////////////////////

	private static int[] mod3 = new int[] { 0, 1, 2, 0, 1 };

	private static final int[][] tetrahedron_fidx/*[4][3]*/ = new int[][] {{2,1,0},{3,0,1},{3,1,2},{3,2,0}};
	private static final int[][] tetrahedron_eidx/*[6][4]*/ = new int[][] {{0,0,2,1},{0,1,1,1},{0,2,3,1},{1,0,3,2},{2,0,1,2},{3,0,2,2}};

	private static final int[][] hexahedron_fidx/*[6][3]*/ = new int[][] {{2,0,4},{4,1,2},{1,4,0},{0,3,1},{0,2,3},{1,3,2}};
	private static final int[][] hexahedron_eidx/*[9][4]*/ = new int[][] {{0,0,4,0},{0,1,2,1},{0,2,1,2},{1,1,5,2},{1,0,2,0},{2,2,3,2},{3,1,5,0},{3,0,4,2},{5,1,4,1}};

	public static class Face {
		public final Mkv[] v = new Mkv[3];
		public final Face[] f = new Face[3];
		public final int[] e = new int[3];
		public final Vec3 n = new Vec3();
		public float d;
		public int mark;
		public Face prev;
		public Face next;
	}
	
	protected class EPA {
		//protected final BulletStack stack = BulletStack.get();
		
		public GJK gjk;
		//public btStackAlloc* sa;
		public Face root;
		public int nfaces;
		public int iterations;
		public final Vec3[][] features = new Vec3[2][3];
		public final Vec3[] nearest/*[2]*/ = new Vec3[] { new Vec3(), new Vec3() };
		public final Vec3 normal = new Vec3();
		public float depth;
		public boolean failed;
		
		{
			for (int i=0; i<features.length; i++) {
				for (int j=0; j<features[i].length; j++) {
					features[i][j] = new Vec3();
				}
			}
		}

		public EPA(GJK pgjk) {
			gjk = pgjk;
			//sa = pgjk->sa;
		}
		
		public Vec3 GetCoordinates(Face face, Vec3 out) {
			Vec3 tmp = new Vec3();
			Vec3 tmp1 = new Vec3();
			Vec3 tmp2 = new Vec3();

			Vec3 o = new Vec3();
			o.scale(-face.d, face.n);

			float[] a = floatArrays.getFixed(3);

			tmp1.subHere(face.v[0].w, o);
			tmp2.subHere(face.v[1].w, o);
			tmp.crossHere(tmp1, tmp2);
			a[0] = tmp.length();

			tmp1.subHere(face.v[1].w, o);
			tmp2.subHere(face.v[2].w, o);
			tmp.crossHere(tmp1, tmp2);
			a[1] = tmp.length();

			tmp1.subHere(face.v[2].w, o);
			tmp2.subHere(face.v[0].w, o);
			tmp.crossHere(tmp1, tmp2);
			a[2] = tmp.length();

			float sm = a[0] + a[1] + a[2];

			out.set(a[1], a[2], a[0]);
			out.mult(1f / (sm > 0f ? sm : 1f));

			floatArrays.release(a);

			return out;
		}
		
		public Face FindBest() {
			Face bf = null;
			if (root != null) {
				Face cf = root;
				float bd = cstInf;
				do {
					if (cf.d < bd) {
						bd = cf.d;
						bf = cf;
					}
				}
				while (null != (cf = cf.next));
			}
			return bf;
		}

		public boolean Set(Face f, Mkv a, Mkv b, Mkv c) {
			Vec3 tmp1 = new Vec3();
			Vec3 tmp2 = new Vec3();
			Vec3 tmp3 = new Vec3();

			Vec3 nrm = new Vec3();
			tmp1.subHere(b.w, a.w);
			tmp2.subHere(c.w, a.w);
			nrm.crossHere(tmp1, tmp2);

			float len = nrm.length();

			tmp1.crossHere(a.w, b.w);
			tmp2.crossHere(b.w, c.w);
			tmp3.crossHere(c.w, a.w);

			boolean valid = (tmp1.dot(nrm) >= -EPA_inface_eps) &&
					(tmp2.dot(nrm) >= -EPA_inface_eps) &&
					(tmp3.dot(nrm) >= -EPA_inface_eps);

			f.v[0] = a;
			f.v[1] = b;
			f.v[2] = c;
			f.mark = 0;
			f.n.scale(1f / (len > 0f ? len : cstInf), nrm);
			f.d = Math.max(0, -f.n.dot(a.w));
			return valid;
		}
		
		public Face NewFace(Mkv a, Mkv b, Mkv c) {
			//Face pf = new Face();
			Face pf = stackFace.get();
			if (Set(pf, a, b, c)) {
				if (root != null) {
					root.prev = pf;
				}
				pf.prev = null;
				pf.next = root;
				root = pf;
				++nfaces;
			}
			else {
				pf.prev = pf.next = null;
			}
			return (pf);
		}
	
		public void Detach(Face face) {
			if (face.prev != null || face.next != null) {
				--nfaces;
				if (face == root) {
					root = face.next;
					root.prev = null;
				}
				else {
					if (face.next == null) {
						face.prev.next = null;
					}
					else {
						face.prev.next = face.next;
						face.next.prev = face.prev;
					}
				}
				face.prev = face.next = null;
			}
		}

		public void Link(Face f0, int e0, Face f1, int e1) {
			f0.f[e0] = f1; f1.e[e1] = e0;
			f1.f[e1] = f0; f0.e[e0] = e1;
		}

		public Mkv Support(Vec3 w) {
			//Mkv v = new Mkv();
			Mkv v = stackMkv.get();
			gjk.Support(w, v);
			return v;
		}
		
		public int BuildHorizon(int markid, Mkv w, Face f, int e, Face[] cf, Face[] ff) {
			int ne = 0;
			if (f.mark != markid) {
				int e1 = mod3[e + 1];
				if ((f.n.dot(w.w) + f.d) > 0) {
					Face nf = NewFace(f.v[e1], f.v[e], w);
					Link(nf, 0, f, e);
					if (cf[0] != null) {
						Link(cf[0], 1, nf, 2);
					}
					else {
						ff[0] = nf;
					}
					cf[0] = nf;
					ne = 1;
				}
				else {
					int e2 = mod3[e + 2];
					Detach(f);
					f.mark = markid;
					ne += BuildHorizon(markid, w, f.f[e1], f.e[e1], cf, ff);
					ne += BuildHorizon(markid, w, f.f[e2], f.e[e2], cf, ff);
				}
			}
			return (ne);
		}

		public float EvaluatePD() {
			return EvaluatePD(EPA_accuracy);
		}
		
		public float EvaluatePD(float accuracy) {
			pushStack();
			try {
				Vec3 tmp = new Vec3();

				//btBlock* sablock = sa->beginBlock();
				Face bestface = null;
				int markid = 1;
				depth = -cstInf;
				normal.set(0f, 0f, 0f);
				root = null;
				nfaces = 0;
				iterations = 0;
				failed = false;
				/* Prepare hull		*/
				if (gjk.EncloseOrigin()) {
					//const U* pfidx = 0;
					int[][] pfidx_ptr = null;
					int pfidx_index = 0;

					int nfidx = 0;
					//const U* peidx = 0;
					int[][] peidx_ptr = null;
					int peidx_index = 0;

					int neidx = 0;
					Mkv[] basemkv = new Mkv[5];
					Face[] basefaces = new Face[6];
					switch (gjk.order) {
						// Tetrahedron
						case 3:
							 {
								//pfidx=(const U*)fidx;
								pfidx_ptr = tetrahedron_fidx;
								pfidx_index = 0;

								nfidx = 4;

								//peidx=(const U*)eidx;
								peidx_ptr = tetrahedron_eidx;
								peidx_index = 0;

								neidx = 6;
							}
							break;
						// Hexahedron
						case 4:
							 {
								//pfidx=(const U*)fidx;
								pfidx_ptr = hexahedron_fidx;
								pfidx_index = 0;

								nfidx = 6;

								//peidx=(const U*)eidx;
								peidx_ptr = hexahedron_eidx;
								peidx_index = 0;

								neidx = 9;
							}
							break;
					}
					int i;

					for (i = 0; i <= gjk.order; ++i) {
						basemkv[i] = new Mkv();
						basemkv[i].set(gjk.simplex[i]);
					}
					for (i = 0; i < nfidx; ++i, pfidx_index++) {
						basefaces[i] = NewFace(basemkv[pfidx_ptr[pfidx_index][0]], basemkv[pfidx_ptr[pfidx_index][1]], basemkv[pfidx_ptr[pfidx_index][2]]);
					}
					for (i = 0; i < neidx; ++i, peidx_index++) {
						Link(basefaces[peidx_ptr[peidx_index][0]], peidx_ptr[peidx_index][1], basefaces[peidx_ptr[peidx_index][2]], peidx_ptr[peidx_index][3]);
					}
				}
				if (0 == nfaces) {
					//sa->endBlock(sablock);
					return (depth);
				}
				/* Expand hull		*/
				for (; iterations < EPA_maxiterations; ++iterations) {
					Face bf = FindBest();
					if (bf != null) {
						tmp.negateHere(bf.n);
						Mkv w = Support(tmp);
						float d = bf.n.dot(w.w) + bf.d;
						bestface = bf;
						if (d < -accuracy) {
							Face[] cf = new Face[]{null};
							Face[] ff = new Face[]{null};
							int nf = 0;
							Detach(bf);
							bf.mark = ++markid;
							for (int i = 0; i < 3; ++i) {
								nf += BuildHorizon(markid, w, bf.f[i], bf.e[i], cf, ff);
							}
							if (nf <= 2) {
								break;
							}
							Link(cf[0], 1, ff[0], 2);
						}
						else {
							break;
						}
					}
					else {
						break;
					}
				}
				/* Extract contact	*/
				if (bestface != null) {
					Vec3 b = GetCoordinates(bestface, new Vec3());
					normal.set(bestface.n);
					depth = Math.max(0, bestface.d);
					for (int i = 0; i < 2; ++i) {
						float s = i != 0 ? -1f : 1f;
						for (int j = 0; j < 3; ++j) {
							tmp.scale(s, bestface.v[j].r);
							gjk.LocalSupport(tmp, i, features[i][j]);
						}
					}

					Vec3 tmp1 = new Vec3();
					Vec3 tmp2 = new Vec3();
					Vec3 tmp3 = new Vec3();

					tmp1.scale(b.x, features[0][0]);
					tmp2.scale(b.y, features[0][1]);
					tmp3.scale(b.z, features[0][2]);
					VectorUtil.add(nearest[0], tmp1, tmp2, tmp3);

					tmp1.scale(b.x, features[1][0]);
					tmp2.scale(b.y, features[1][1]);
					tmp3.scale(b.z, features[1][2]);
					VectorUtil.add(nearest[1], tmp1, tmp2, tmp3);
				}
				else {
					failed = true;
				}
				//sa->endBlock(sablock);
				return (depth);
			}
			finally {
				popStack();
			}
		}
		
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	private GJK gjk = new GJK();
	
	public boolean collide(ConvexShape shape0, Transform wtrs0,
			ConvexShape shape1, Transform wtrs1,
			float radialmargin/*,
			btStackAlloc* stackAlloc*/,
			Results results) {
		
		// Initialize
		results.witnesses[0].set(0f, 0f, 0f);
		results.witnesses[1].set(0f, 0f, 0f);
		results.normal.set(0f, 0f, 0f);
		results.depth = 0;
		results.status = ResultsStatus.Separated;
		results.epa_iterations = 0;
		results.gjk_iterations = 0;
		/* Use GJK to locate origin		*/
		gjk.init(/*stackAlloc,*/
				wtrs0.basis, wtrs0.origin, shape0,
				wtrs1.basis, wtrs1.origin, shape1,
				radialmargin + EPA_accuracy);
		try {
			boolean collide = gjk.SearchOrigin();
			results.gjk_iterations = gjk.iterations + 1;
			if (collide) {
				/* Then EPA for penetration depth	*/
				EPA epa = new EPA(gjk);
				float pd = epa.EvaluatePD();
				results.epa_iterations = epa.iterations + 1;
				if (pd > 0) {
					results.status = ResultsStatus.Penetrating;
					results.normal.set(epa.normal);
					results.depth = pd;
					results.witnesses[0].set(epa.nearest[0]);
					results.witnesses[1].set(epa.nearest[1]);
					return (true);
				}
				else {
					if (epa.failed) {
						results.status = ResultsStatus.EPA_Failed;
					}
				}
			}
			else {
				if (gjk.failed) {
					results.status = ResultsStatus.GJK_Failed;
				}
			}
			return (false);
		}
		finally {
			gjk.destroy();
		}
	}
	
}
