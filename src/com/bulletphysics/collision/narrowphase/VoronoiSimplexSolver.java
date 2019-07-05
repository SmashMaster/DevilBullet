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

import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectPool;
import com.samrj.devil.math.Vec3;

/**
 * VoronoiSimplexSolver is an implementation of the closest point distance algorithm
 * from a 1-4 points simplex to the origin. Can be used with GJK, as an alternative
 * to Johnson distance algorithm.
 * 
 * @author jezek2
 */
public class VoronoiSimplexSolver extends SimplexSolverInterface {

	//protected final BulletStack stack = BulletStack.get();
	protected final ObjectPool<SubSimplexClosestResult> subsimplexResultsPool = ObjectPool.get(SubSimplexClosestResult.class);
	
	private static final int VORONOI_SIMPLEX_MAX_VERTS = 5;
	
	private static final int VERTA = 0;
	private static final int VERTB = 1;
	private static final int VERTC = 2;
	private static final int VERTD = 3;

	public int numVertices;

	public final Vec3[] simplexVectorW = new Vec3[VORONOI_SIMPLEX_MAX_VERTS];
	public final Vec3[] simplexPointsP = new Vec3[VORONOI_SIMPLEX_MAX_VERTS];
	public final Vec3[] simplexPointsQ = new Vec3[VORONOI_SIMPLEX_MAX_VERTS];

	public final Vec3 cachedP1 = new Vec3();
	public final Vec3 cachedP2 = new Vec3();
	public final Vec3 cachedV = new Vec3();
	public final Vec3 lastW = new Vec3();
	public boolean cachedValidClosest;

	public final SubSimplexClosestResult cachedBC = new SubSimplexClosestResult();

	public boolean needsUpdate;
	
	{
		for (int i=0; i<VORONOI_SIMPLEX_MAX_VERTS; i++) {
			simplexVectorW[i] = new Vec3();
			simplexPointsP[i] = new Vec3();
			simplexPointsQ[i] = new Vec3();
		}
	}

	public void removeVertex(int index) {
		assert(numVertices>0);
		numVertices--;
		simplexVectorW[index].set(simplexVectorW[numVertices]);
		simplexPointsP[index].set(simplexPointsP[numVertices]);
		simplexPointsQ[index].set(simplexPointsQ[numVertices]);
	}
	
	public void	reduceVertices(UsageBitfield usedVerts) {
		if ((numVertices() >= 4) && (!usedVerts.usedVertexD))
			removeVertex(3);

		if ((numVertices() >= 3) && (!usedVerts.usedVertexC))
			removeVertex(2);

		if ((numVertices() >= 2) && (!usedVerts.usedVertexB))
			removeVertex(1);

		if ((numVertices() >= 1) && (!usedVerts.usedVertexA))
			removeVertex(0);
	}
	
	public boolean updateClosestVectorAndPoints() {
		if (needsUpdate)
		{
			cachedBC.reset();

			needsUpdate = false;

			switch (numVertices())
			{
			case 0:
					cachedValidClosest = false;
					break;
			case 1:
				{
					cachedP1.set(simplexPointsP[0]);
					cachedP2.set(simplexPointsQ[0]);
					VectorUtil.sub(cachedV, cachedP1, cachedP2); //== m_simplexVectorW[0]
					cachedBC.reset();
					cachedBC.setBarycentricCoordinates(1f, 0f, 0f, 0f);
					cachedValidClosest = cachedBC.isValid();
					break;
				}
			case 2:
				{
					Vec3 tmp = new Vec3();
					
					//closest point origin from line segment
					Vec3 from = simplexVectorW[0];
					Vec3 to = simplexVectorW[1];
					Vec3 nearest = new Vec3();

					Vec3 p = new Vec3();
					p.set(0f, 0f, 0f);
					Vec3 diff = new Vec3();
					VectorUtil.sub(diff, p, from);

					Vec3 v = new Vec3();
					VectorUtil.sub(v, to, from);

					float t = v.dot(diff);

					if (t > 0) {
						float dotVV = v.dot(v);
						if (t < dotVV) {
							t /= dotVV;
							VectorUtil.scale(tmp, t, v);
							diff.sub(tmp);
							cachedBC.usedVertices.usedVertexA = true;
							cachedBC.usedVertices.usedVertexB = true;
						} else {
							t = 1;
							diff.sub(v);
							// reduce to 1 point
							cachedBC.usedVertices.usedVertexB = true;
						}
					} else
					{
						t = 0;
						//reduce to 1 point
						cachedBC.usedVertices.usedVertexA = true;
					}
					cachedBC.setBarycentricCoordinates(1f-t, t, 0f, 0f);
					
					VectorUtil.scale(tmp, t, v);
					VectorUtil.add(nearest, from, tmp);

					VectorUtil.sub(tmp, simplexPointsP[1], simplexPointsP[0]);
					tmp.mult(t);
					VectorUtil.add(cachedP1, simplexPointsP[0], tmp);

					VectorUtil.sub(tmp, simplexPointsQ[1], simplexPointsQ[0]);
					tmp.mult(t);
					VectorUtil.add(cachedP2, simplexPointsQ[0], tmp);

					VectorUtil.sub(cachedV, cachedP1, cachedP2);

					reduceVertices(cachedBC.usedVertices);

					cachedValidClosest = cachedBC.isValid();
					break;
				}
			case 3: 
				{ 
					Vec3 tmp1 = new Vec3();
					Vec3 tmp2 = new Vec3();
					Vec3 tmp3 = new Vec3();
					
					// closest point origin from triangle 
					Vec3 p = new Vec3();
					p.set(0f, 0f, 0f);

					Vec3 a = simplexVectorW[0]; 
					Vec3 b = simplexVectorW[1]; 
					Vec3 c = simplexVectorW[2]; 

					closestPtPointTriangle(p,a,b,c,cachedBC);

					VectorUtil.scale(tmp1, cachedBC.barycentricCoords[0], simplexPointsP[0]);
					VectorUtil.scale(tmp2, cachedBC.barycentricCoords[1], simplexPointsP[1]);
					VectorUtil.scale(tmp3, cachedBC.barycentricCoords[2], simplexPointsP[2]);
					VectorUtil.add(cachedP1, tmp1, tmp2, tmp3);

					VectorUtil.scale(tmp1, cachedBC.barycentricCoords[0], simplexPointsQ[0]);
					VectorUtil.scale(tmp2, cachedBC.barycentricCoords[1], simplexPointsQ[1]);
					VectorUtil.scale(tmp3, cachedBC.barycentricCoords[2], simplexPointsQ[2]);
					VectorUtil.add(cachedP2, tmp1, tmp2, tmp3);

					VectorUtil.sub(cachedV, cachedP1, cachedP2);

					reduceVertices(cachedBC.usedVertices);
					cachedValidClosest = cachedBC.isValid(); 

					break; 
				}
			case 4:
				{
					Vec3 tmp1 = new Vec3();
					Vec3 tmp2 = new Vec3();
					Vec3 tmp3 = new Vec3();
					Vec3 tmp4 = new Vec3();
					
					Vec3 p = new Vec3();
					p.set(0f, 0f, 0f);

					Vec3 a = simplexVectorW[0];
					Vec3 b = simplexVectorW[1];
					Vec3 c = simplexVectorW[2];
					Vec3 d = simplexVectorW[3];

					boolean hasSeperation = closestPtPointTetrahedron(p,a,b,c,d,cachedBC);

					if (hasSeperation)
					{
						VectorUtil.scale(tmp1, cachedBC.barycentricCoords[0], simplexPointsP[0]);
						VectorUtil.scale(tmp2, cachedBC.barycentricCoords[1], simplexPointsP[1]);
						VectorUtil.scale(tmp3, cachedBC.barycentricCoords[2], simplexPointsP[2]);
						VectorUtil.scale(tmp4, cachedBC.barycentricCoords[3], simplexPointsP[3]);
						VectorUtil.add(cachedP1, tmp1, tmp2, tmp3, tmp4);

						VectorUtil.scale(tmp1, cachedBC.barycentricCoords[0], simplexPointsQ[0]);
						VectorUtil.scale(tmp2, cachedBC.barycentricCoords[1], simplexPointsQ[1]);
						VectorUtil.scale(tmp3, cachedBC.barycentricCoords[2], simplexPointsQ[2]);
						VectorUtil.scale(tmp4, cachedBC.barycentricCoords[3], simplexPointsQ[3]);
						VectorUtil.add(cachedP2, tmp1, tmp2, tmp3, tmp4);

						VectorUtil.sub(cachedV, cachedP1, cachedP2);
						reduceVertices (cachedBC.usedVertices);
					} else
					{
	//					printf("sub distance got penetration\n");

						if (cachedBC.degenerate)
						{
							cachedValidClosest = false;
						} else
						{
							cachedValidClosest = true;
							//degenerate case == false, penetration = true + zero
							cachedV.set(0f, 0f, 0f);
						}
						break;
					}

					cachedValidClosest = cachedBC.isValid();

					//closest point origin from tetrahedron
					break;
				}
			default:
				{
					cachedValidClosest = false;
				}
			}
		}

		return cachedValidClosest;
	}

	public boolean closestPtPointTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c, SubSimplexClosestResult result) {
		result.usedVertices.reset();

		// Check if P in vertex region outside A
		Vec3 ab = new Vec3();
		VectorUtil.sub(ab, b, a);

		Vec3 ac = new Vec3();
		VectorUtil.sub(ac, c, a);

		Vec3 ap = new Vec3();
		VectorUtil.sub(ap, p, a);

		float d1 = ab.dot(ap);
		float d2 = ac.dot(ap);

		if (d1 <= 0f && d2 <= 0f) 
		{
			result.closestPointOnSimplex.set(a);
			result.usedVertices.usedVertexA = true;
			result.setBarycentricCoordinates(1f, 0f, 0f, 0f);
			return true; // a; // barycentric coordinates (1,0,0)
		}

		// Check if P in vertex region outside B
		Vec3 bp = new Vec3();
		VectorUtil.sub(bp, p, b);

		float d3 = ab.dot(bp);
		float d4 = ac.dot(bp);

		if (d3 >= 0f && d4 <= d3) 
		{
			result.closestPointOnSimplex.set(b);
			result.usedVertices.usedVertexB = true;
			result.setBarycentricCoordinates(0, 1f, 0f, 0f);

			return true; // b; // barycentric coordinates (0,1,0)
		}

		// Check if P in edge region of AB, if so return projection of P onto AB
		float vc = d1*d4 - d3*d2;
		if (vc <= 0f && d1 >= 0f && d3 <= 0f) {
			float v = d1 / (d1 - d3);
			VectorUtil.scaleAdd(result.closestPointOnSimplex, v, ab, a);
			result.usedVertices.usedVertexA = true;
			result.usedVertices.usedVertexB = true;
			result.setBarycentricCoordinates(1f-v, v, 0f, 0f);
			return true;
			//return a + v * ab; // barycentric coordinates (1-v,v,0)
		}

		// Check if P in vertex region outside C
		Vec3 cp = new Vec3();
		VectorUtil.sub(cp, p, c);

		float d5 = ab.dot(cp);
		float d6 = ac.dot(cp);

		if (d6 >= 0f && d5 <= d6) 
		{
			result.closestPointOnSimplex.set(c);
			result.usedVertices.usedVertexC = true;
			result.setBarycentricCoordinates(0f, 0f, 1f, 0f);
			return true;//c; // barycentric coordinates (0,0,1)
		}

		// Check if P in edge region of AC, if so return projection of P onto AC
		float vb = d5*d2 - d1*d6;
		if (vb <= 0f && d2 >= 0f && d6 <= 0f) {
			float w = d2 / (d2 - d6);
			VectorUtil.scaleAdd(result.closestPointOnSimplex, w, ac, a);
			result.usedVertices.usedVertexA = true;
			result.usedVertices.usedVertexC = true;
			result.setBarycentricCoordinates(1f-w, 0f, w, 0f);
			return true;
			//return a + w * ac; // barycentric coordinates (1-w,0,w)
		}

		// Check if P in edge region of BC, if so return projection of P onto BC
		float va = d3*d6 - d5*d4;
		if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f) {
			float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

			Vec3 tmp = new Vec3();
			VectorUtil.sub(tmp, c, b);
			VectorUtil.scaleAdd(result.closestPointOnSimplex, w, tmp, b);

			result.usedVertices.usedVertexB = true;
			result.usedVertices.usedVertexC = true;
			result.setBarycentricCoordinates(0, 1f-w, w, 0f);
			return true;		
		   // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
		}

		// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
		float denom = 1f / (va + vb + vc);
		float v = vb * denom;
		float w = vc * denom;

		Vec3 tmp1 = new Vec3();
		Vec3 tmp2 = new Vec3();

		VectorUtil.scale(tmp1, v, ab);
		VectorUtil.scale(tmp2, w, ac);
		VectorUtil.add(result.closestPointOnSimplex, a, tmp1, tmp2);
		result.usedVertices.usedVertexA = true;
		result.usedVertices.usedVertexB = true;
		result.usedVertices.usedVertexC = true;
		result.setBarycentricCoordinates(1f-v-w, v, w, 0f);

		return true;
		//	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = btScalar(1.0) - v - w
	}
	
	/// Test if point p and d lie on opposite sides of plane through abc
	public static int pointOutsideOfPlane(Vec3 p, Vec3 a, Vec3 b, Vec3 c, Vec3 d)
	{
		Vec3 tmp = new Vec3();

		Vec3 normal = new Vec3();
		VectorUtil.sub(normal, b, a);
		VectorUtil.sub(tmp, c, a);
		VectorUtil.cross(normal, normal, tmp);

		VectorUtil.sub(tmp, p, a);
		float signp = tmp.dot(normal); // [AP AB AC]

		VectorUtil.sub(tmp, d, a);
		float signd = tmp.dot(normal); // [AD AB AC]

	//#ifdef CATCH_DEGENERATE_TETRAHEDRON
//	#ifdef BT_USE_DOUBLE_PRECISION
//	if (signd * signd < (btScalar(1e-8) * btScalar(1e-8)))
//		{
//			return -1;
//		}
//	#else
		if (signd * signd < ((1e-4f) * (1e-4f)))
		{
	//		printf("affine dependent/degenerate\n");//
			return -1;
		}
	//#endif

	//#endif
		// Points on opposite sides if expression signs are opposite
		return (signp * signd < 0f)? 1 : 0;
	}
	
	public boolean closestPtPointTetrahedron(Vec3 p, Vec3 a, Vec3 b, Vec3 c, Vec3 d, SubSimplexClosestResult finalResult) {
		SubSimplexClosestResult tempResult = subsimplexResultsPool.get();
		tempResult.reset();
		try {
			Vec3 tmp = new Vec3();
			Vec3 q = new Vec3();

			// Start out assuming point inside all halfspaces, so closest to itself
			finalResult.closestPointOnSimplex.set(p);
			finalResult.usedVertices.reset();
			finalResult.usedVertices.usedVertexA = true;
			finalResult.usedVertices.usedVertexB = true;
			finalResult.usedVertices.usedVertexC = true;
			finalResult.usedVertices.usedVertexD = true;

			int pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d);
			int pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b);
			int	pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c);
			int	pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a);

		   if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
		   {
			   finalResult.degenerate = true;
			   return false;
		   }

		   if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0)
			 {
				 return false;
			 }


			float bestSqDist = Float.MAX_VALUE;
			// If point outside face abc then compute closest point on abc
			if (pointOutsideABC != 0) 
			{
				closestPtPointTriangle(p, a, b, c,tempResult);
				q.set(tempResult.closestPointOnSimplex);

				VectorUtil.sub(tmp, q, p);
				float sqDist = tmp.dot(tmp);
				// Update best closest point if (squared) distance is less than current best
				if (sqDist < bestSqDist) {
					bestSqDist = sqDist;
					finalResult.closestPointOnSimplex.set(q);
					//convert result bitmask!
					finalResult.usedVertices.reset();
					finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
					finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexB;
					finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
					finalResult.setBarycentricCoordinates(
							tempResult.barycentricCoords[VERTA],
							tempResult.barycentricCoords[VERTB],
							tempResult.barycentricCoords[VERTC],
							0
					);

				}
			}


			// Repeat test for face acd
			if (pointOutsideACD != 0) 
			{
				closestPtPointTriangle(p, a, c, d,tempResult);
				q.set(tempResult.closestPointOnSimplex);
				//convert result bitmask!

				VectorUtil.sub(tmp, q, p);
				float sqDist = tmp.dot(tmp);
				if (sqDist < bestSqDist) 
				{
					bestSqDist = sqDist;
					finalResult.closestPointOnSimplex.set(q);
					finalResult.usedVertices.reset();
					finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;

					finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexB;
					finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexC;
					finalResult.setBarycentricCoordinates(
							tempResult.barycentricCoords[VERTA],
							0,
							tempResult.barycentricCoords[VERTB],
							tempResult.barycentricCoords[VERTC]
					);

				}
			}
			// Repeat test for face adb


			if (pointOutsideADB != 0)
			{
				closestPtPointTriangle(p, a, d, b,tempResult);
				q.set(tempResult.closestPointOnSimplex);
				//convert result bitmask!

				VectorUtil.sub(tmp, q, p);
				float sqDist = tmp.dot(tmp);
				if (sqDist < bestSqDist) 
				{
					bestSqDist = sqDist;
					finalResult.closestPointOnSimplex.set(q);
					finalResult.usedVertices.reset();
					finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
					finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexC;

					finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;
					finalResult.setBarycentricCoordinates(
							tempResult.barycentricCoords[VERTA],
							tempResult.barycentricCoords[VERTC],
							0,
							tempResult.barycentricCoords[VERTB]
					);

				}
			}
			// Repeat test for face bdc


			if (pointOutsideBDC != 0)
			{
				closestPtPointTriangle(p, b, d, c,tempResult);
				q.set(tempResult.closestPointOnSimplex);
				//convert result bitmask!
				VectorUtil.sub(tmp, q, p);
				float sqDist = tmp.dot(tmp);
				if (sqDist < bestSqDist) 
				{
					bestSqDist = sqDist;
					finalResult.closestPointOnSimplex.set(q);
					finalResult.usedVertices.reset();
					//
					finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexA;
					finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
					finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;

					finalResult.setBarycentricCoordinates(
							0,
							tempResult.barycentricCoords[VERTA],
							tempResult.barycentricCoords[VERTC],
							tempResult.barycentricCoords[VERTB]
					);

				}
			}

			//help! we ended up full !

			if (finalResult.usedVertices.usedVertexA &&
				finalResult.usedVertices.usedVertexB &&
				finalResult.usedVertices.usedVertexC &&
				finalResult.usedVertices.usedVertexD) 
			{
				return true;
			}

			return true;
		}
		finally {
			subsimplexResultsPool.release(tempResult);
		}
	}
	
	/**
	 * Clear the simplex, remove all the vertices.
	 */
	public void reset() {
		cachedValidClosest = false;
		numVertices = 0;
		needsUpdate = true;
		lastW.set(1e30f, 1e30f, 1e30f);
		cachedBC.reset();
	}

	public void addVertex(Vec3 w, Vec3 p, Vec3 q) {
		lastW.set(w);
		needsUpdate = true;

		simplexVectorW[numVertices].set(w);
		simplexPointsP[numVertices].set(p);
		simplexPointsQ[numVertices].set(q);

		numVertices++;
	}

	/**
	 * Return/calculate the closest vertex.
	 */
	public boolean closest(Vec3 v) {
		boolean succes = updateClosestVectorAndPoints();
		v.set(cachedV);
		return succes;
	}

	public float maxVertex() {
		int i, numverts = numVertices();
		float maxV = 0f;
		for (i = 0; i < numverts; i++) {
			float curLen2 = simplexVectorW[i].squareLength();
			if (maxV < curLen2) {
				maxV = curLen2;
			}
		}
		return maxV;
	}

	public boolean fullSimplex() {
		return (numVertices == 4);
	}

	public int getSimplex(Vec3[] pBuf, Vec3[] qBuf, Vec3[] yBuf) {
		for (int i = 0; i < numVertices(); i++) {
			yBuf[i].set(simplexVectorW[i]);
			pBuf[i].set(simplexPointsP[i]);
			qBuf[i].set(simplexPointsQ[i]);
		}
		return numVertices();
	}

	public boolean inSimplex(Vec3 w) {
		boolean found = false;
		int i, numverts = numVertices();
		//btScalar maxV = btScalar(0.);

		//w is in the current (reduced) simplex
		for (i = 0; i < numverts; i++) {
			if (simplexVectorW[i].equals(w)) {
				found = true;
			}
		}

		//check in case lastW is already removed
		if (w.equals(lastW)) {
			return true;
		}

		return found;
	}

	public void backup_closest(Vec3 v) {
		v.set(cachedV);
	}

	public boolean emptySimplex() {
		return (numVertices() == 0);
	}

	public void compute_points(Vec3 p1, Vec3 p2) {
		updateClosestVectorAndPoints();
		p1.set(cachedP1);
		p2.set(cachedP2);
	}

	public int numVertices() {
		return numVertices;
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	public static class UsageBitfield {
		public boolean usedVertexA;
		public boolean usedVertexB;
		public boolean usedVertexC;
		public boolean usedVertexD;
		
		public void reset() {
			usedVertexA = false;
			usedVertexB = false;
			usedVertexC = false;
			usedVertexD = false;
		}
	}
	
	public static class SubSimplexClosestResult {
		public final Vec3 closestPointOnSimplex = new Vec3();
		//MASK for m_usedVertices
		//stores the simplex vertex-usage, using the MASK, 
		// if m_usedVertices & MASK then the related vertex is used
		public final UsageBitfield usedVertices = new UsageBitfield();
		public final float[] barycentricCoords = new float[4];
		public boolean degenerate;
		
		public void reset() {
			degenerate = false;
			setBarycentricCoordinates(0f, 0f, 0f, 0f);
			usedVertices.reset();
		}

		public boolean isValid() {
			boolean valid = (barycentricCoords[0] >= 0f) &&
					(barycentricCoords[1] >= 0f) &&
					(barycentricCoords[2] >= 0f) &&
					(barycentricCoords[3] >= 0f);
			return valid;
		}

		public void setBarycentricCoordinates(float a, float b, float c, float d) {
			barycentricCoords[0] = a;
			barycentricCoords[1] = b;
			barycentricCoords[2] = c;
			barycentricCoords[3] = d;
		}
	}
	
}
