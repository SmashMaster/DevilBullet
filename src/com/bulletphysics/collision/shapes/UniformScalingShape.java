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
import javax.vecmath.Vec3;

/**
 * UniformScalingShape allows to re-use uniform scaled instances of {@link ConvexShape}
 * in a memory efficient way. Istead of using {@link UniformScalingShape}, it is better
 * to use the non-uniform setLocalScaling method on convex shapes that implement it.
 * 
 * @author jezek2
 */
public class UniformScalingShape extends ConvexShape {

	private ConvexShape childConvexShape;
	private float uniformScalingFactor;

	public UniformScalingShape(ConvexShape convexChildShape, float uniformScalingFactor) {
		this.childConvexShape = convexChildShape;
		this.uniformScalingFactor = uniformScalingFactor;
	}

	public float getUniformScalingFactor() {
		return uniformScalingFactor;
	}

	public ConvexShape getChildShape() {
		return childConvexShape;
	}
	
	@Override
	public Vec3 localGetSupportingVertex(Vec3 vec, Vec3 out) {
		childConvexShape.localGetSupportingVertex(vec, out);
		out.mult(uniformScalingFactor);
		return out;
	}

	@Override
	public Vec3 localGetSupportingVertexWithoutMargin(Vec3 vec, Vec3 out) {
		childConvexShape.localGetSupportingVertexWithoutMargin(vec, out);
		out.mult(uniformScalingFactor);
		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vec3[] vectors, Vec3[] supportVerticesOut, int numVectors) {
		childConvexShape.batchedUnitVectorGetSupportingVertexWithoutMargin(vectors, supportVerticesOut, numVectors);
		for (int i=0; i<numVectors; i++) {
			supportVerticesOut[i].mult(uniformScalingFactor);
		}
	}

	@Override
	public void getAabbSlow(Transform t, Vec3 aabbMin, Vec3 aabbMax) {
		childConvexShape.getAabbSlow(t, aabbMin, aabbMax);
		Vec3 aabbCenter = new Vec3();
		aabbCenter.addHere(aabbMax, aabbMin);
		aabbCenter.mult(0.5f);

		Vec3 scaledAabbHalfExtends = new Vec3();
		scaledAabbHalfExtends.subHere(aabbMax, aabbMin);
		scaledAabbHalfExtends.mult(0.5f * uniformScalingFactor);

		aabbMin.subHere(aabbCenter, scaledAabbHalfExtends);
		aabbMax.addHere(aabbCenter, scaledAabbHalfExtends);
	}

	@Override
	public void setLocalScaling(Vec3 scaling) {
		childConvexShape.setLocalScaling(scaling);
	}

	@Override
	public Vec3 getLocalScaling(Vec3 out) {
		childConvexShape.getLocalScaling(out);
		return out;
	}

	@Override
	public void setMargin(float margin) {
		childConvexShape.setMargin(margin);
	}

	@Override
	public float getMargin() {
		return childConvexShape.getMargin() * uniformScalingFactor;
	}

	@Override
	public int getNumPreferredPenetrationDirections() {
		return childConvexShape.getNumPreferredPenetrationDirections();
	}

	@Override
	public void getPreferredPenetrationDirection(int index, Vec3 penetrationVector) {
		childConvexShape.getPreferredPenetrationDirection(index, penetrationVector);
	}

	@Override
	public void getAabb(Transform t, Vec3 aabbMin, Vec3 aabbMax) {
		childConvexShape.getAabb(t, aabbMin, aabbMax);
		Vec3 aabbCenter = new Vec3();
		aabbCenter.addHere(aabbMax, aabbMin);
		aabbCenter.mult(0.5f);

		Vec3 scaledAabbHalfExtends = new Vec3();
		scaledAabbHalfExtends.subHere(aabbMax, aabbMin);
		scaledAabbHalfExtends.mult(0.5f * uniformScalingFactor);

		aabbMin.subHere(aabbCenter, scaledAabbHalfExtends);
		aabbMax.addHere(aabbCenter, scaledAabbHalfExtends);
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.UNIFORM_SCALING_SHAPE_PROXYTYPE;
	}

	@Override
	public void calculateLocalInertia(float mass, Vec3 inertia) {
		// this linear upscaling is not realistic, but we don't deal with large mass ratios...
		childConvexShape.calculateLocalInertia(mass, inertia);
		inertia.mult(uniformScalingFactor);
	}

	@Override
	public String getName() {
		return "UniformScalingShape";
	}

}
