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

package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import javax.vecmath.Vec3;

/**
 * Single triangle shape.
 * 
 * @author jezek2
 */
public class TriangleShape extends PolyhedralConvexShape {
	
	public final Vec3[] vertices1/*[3]*/ = new Vec3[] { new Vec3(), new Vec3(), new Vec3() };

	// JAVA NOTE: added
	public TriangleShape() {
	}
	
	public TriangleShape(Vec3 p0, Vec3 p1, Vec3 p2) {
		vertices1[0].set(p0);
		vertices1[1].set(p1);
		vertices1[2].set(p2);
	}
	
	// JAVA NOTE: added
	public void init(Vec3 p0, Vec3 p1, Vec3 p2) {
		vertices1[0].set(p0);
		vertices1[1].set(p1);
		vertices1[2].set(p2);
	}

	@Override
	public int getNumVertices() {
		return 3;
	}

	public Vec3 getVertexPtr(int index) {
		return vertices1[index];
	}
	
	@Override
	public void getVertex(int index, Vec3 vert) {
		vert.set(vertices1[index]);
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.TRIANGLE_SHAPE_PROXYTYPE;
	}

	@Override
	public int getNumEdges() {
		return 3;
	}

	@Override
	public void getEdge(int i, Vec3 pa, Vec3 pb) {
		getVertex(i, pa);
		getVertex((i + 1) % 3, pb);
	}

	@Override
	public void getAabb(Transform t, Vec3 aabbMin, Vec3 aabbMax) {
//		btAssert(0);
		getAabbSlow(t, aabbMin, aabbMax);
	}

	@Override
	public Vec3 localGetSupportingVertexWithoutMargin(Vec3 dir, Vec3 out) {
		Vec3 dots = new Vec3();
		dots.set(dir.dot(vertices1[0]), dir.dot(vertices1[1]), dir.dot(vertices1[2]));
		out.set(vertices1[VectorUtil.maxAxis(dots)]);
		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vec3[] vectors, Vec3[] supportVerticesOut, int numVectors) {
		Vec3 dots = new Vec3();

		for (int i = 0; i < numVectors; i++) {
			Vec3 dir = vectors[i];
			dots.set(dir.dot(vertices1[0]), dir.dot(vertices1[1]), dir.dot(vertices1[2]));
			supportVerticesOut[i].set(vertices1[VectorUtil.maxAxis(dots)]);
		}
	}

	@Override
	public void getPlane(Vec3 planeNormal, Vec3 planeSupport, int i) {
		getPlaneEquation(i,planeNormal,planeSupport);
	}

	@Override
	public int getNumPlanes() {
		return 1;
	}

	public void calcNormal(Vec3 normal) {
		Vec3 tmp1 = new Vec3();
		Vec3 tmp2 = new Vec3();

		tmp1.subHere(vertices1[1], vertices1[0]);
		tmp2.subHere(vertices1[2], vertices1[0]);

		normal.crossHere(tmp1, tmp2);
		normal.normalize();
	}

	public void getPlaneEquation(int i, Vec3 planeNormal, Vec3 planeSupport) {
		calcNormal(planeNormal);
		planeSupport.set(vertices1[0]);
	}

	@Override
	public void calculateLocalInertia(float mass, Vec3 inertia) {
		assert (false);
		inertia.set(0f, 0f, 0f);
	}
	
	@Override
	public boolean isInside(Vec3 pt, float tolerance) {
		Vec3 normal = new Vec3();
		calcNormal(normal);
		// distance to plane
		float dist = pt.dot(normal);
		float planeconst = vertices1[0].dot(normal);
		dist -= planeconst;
		if (dist >= -tolerance && dist <= tolerance) {
			// inside check on edge-planes
			int i;
			for (i = 0; i < 3; i++) {
				Vec3 pa = new Vec3(), pb = new Vec3();
				getEdge(i, pa, pb);
				Vec3 edge = new Vec3();
				edge.subHere(pb, pa);
				Vec3 edgeNormal = new Vec3();
				edgeNormal.crossHere(edge, normal);
				edgeNormal.normalize();
				/*float*/ dist = pt.dot(edgeNormal);
				float edgeConst = pa.dot(edgeNormal);
				dist -= edgeConst;
				if (dist < -tolerance) {
					return false;
				}
			}

			return true;
		}

		return false;
	}

	@Override
	public String getName() {
		return "Triangle";
	}

	@Override
	public int getNumPreferredPenetrationDirections() {
		return 2;
	}

	@Override
	public void getPreferredPenetrationDirection(int index, Vec3 penetrationVector) {
		calcNormal(penetrationVector);
		if (index != 0) {
			penetrationVector.mult(-1f);
		}
	}

}
