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
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectArrayList;
import com.samrj.devil.math.Mat3;
import javax.vecmath.Vec3;

// JAVA NOTE: CompoundShape from 2.71

/**
 * CompoundShape allows to store multiple other {@link CollisionShape}s. This allows
 * for moving concave collision objects. This is more general than the {@link BvhTriangleMeshShape}.
 * 
 * @author jezek2
 */
public class CompoundShape extends CollisionShape {

	private final ObjectArrayList<CompoundShapeChild> children = new ObjectArrayList<CompoundShapeChild>();
	private final Vec3 localAabbMin = new Vec3(1e30f, 1e30f, 1e30f);
	private final Vec3 localAabbMax = new Vec3(-1e30f, -1e30f, -1e30f);

	private OptimizedBvh aabbTree = null;

	private float collisionMargin = 0f;
	protected final Vec3 localScaling = new Vec3(1f, 1f, 1f);

	public void addChildShape(Transform localTransform, CollisionShape shape) {
		//m_childTransforms.push_back(localTransform);
		//m_childShapes.push_back(shape);
		CompoundShapeChild child = new CompoundShapeChild();
		child.transform.set(localTransform);
		child.childShape = shape;
		child.childShapeType = shape.getShapeType();
		child.childMargin = shape.getMargin();

		children.add(child);

		// extend the local aabbMin/aabbMax
		Vec3 _localAabbMin = new Vec3(), _localAabbMax = new Vec3();
		shape.getAabb(localTransform, _localAabbMin, _localAabbMax);

		// JAVA NOTE: rewritten
//		for (int i=0;i<3;i++)
//		{
//			if (this.localAabbMin[i] > _localAabbMin[i])
//			{
//				this.localAabbMin[i] = _localAabbMin[i];
//			}
//			if (this.localAabbMax[i] < _localAabbMax[i])
//			{
//				this.localAabbMax[i] = _localAabbMax[i];
//			}
//		}
		VectorUtil.setMin(this.localAabbMin, _localAabbMin);
		VectorUtil.setMax(this.localAabbMax, _localAabbMax);
	}

	/**
	 * Remove all children shapes that contain the specified shape.
	 */
	public void removeChildShape(CollisionShape shape) {
		boolean done_removing;

		// Find the children containing the shape specified, and remove those children.
		do {
			done_removing = true;

			for (int i = 0; i < children.size(); i++) {
				if (children.getQuick(i).childShape == shape) {
					children.removeQuick(i);
					done_removing = false;  // Do another iteration pass after removing from the vector
					break;
				}
			}
		}
		while (!done_removing);

		recalculateLocalAabb();
	}
	
	public int getNumChildShapes() {
		return children.size();
	}

	public CollisionShape getChildShape(int index) {
		return children.getQuick(index).childShape;
	}

	public Transform getChildTransform(int index, Transform out) {
		out.set(children.getQuick(index).transform);
		return out;
	}

	public ObjectArrayList<CompoundShapeChild> getChildList() {
		return children;
	}

	/**
	 * getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version.
	 */
	@Override
	public void getAabb(Transform trans, Vec3 aabbMin, Vec3 aabbMax) {
		Vec3 localHalfExtents = new Vec3();
		localHalfExtents.subHere(localAabbMax, localAabbMin);
		localHalfExtents.mult(0.5f);
		localHalfExtents.x += getMargin();
		localHalfExtents.y += getMargin();
		localHalfExtents.z += getMargin();

		Vec3 localCenter = new Vec3();
		localCenter.addHere(localAabbMax, localAabbMin);
		localCenter.mult(0.5f);

		Mat3 abs_b = new Mat3(trans.basis);
		MatrixUtil.absolute(abs_b);

		Vec3 center = new Vec3(localCenter);
		trans.transform(center);

		Vec3 tmp = new Vec3();

		Vec3 extent = new Vec3();
		MatrixUtil.getRow(abs_b, 0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		MatrixUtil.getRow(abs_b, 1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		MatrixUtil.getRow(abs_b, 2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		aabbMin.subHere(center, extent);
		aabbMax.addHere(center, extent);
	}

	/**
	 * Re-calculate the local Aabb. Is called at the end of removeChildShapes.
	 * Use this yourself if you modify the children or their transforms.
	 */
	public void recalculateLocalAabb() {
		// Recalculate the local aabb
		// Brute force, it iterates over all the shapes left.
		localAabbMin.set(1e30f, 1e30f, 1e30f);
		localAabbMax.set(-1e30f, -1e30f, -1e30f);

		Vec3 tmpLocalAabbMin = new Vec3();
		Vec3 tmpLocalAabbMax = new Vec3();

		// extend the local aabbMin/aabbMax
		for (int j = 0; j < children.size(); j++) {
			children.getQuick(j).childShape.getAabb(children.getQuick(j).transform, tmpLocalAabbMin, tmpLocalAabbMax);
			
			for (int i = 0; i < 3; i++) {
				if (VectorUtil.getCoord(localAabbMin, i) > VectorUtil.getCoord(tmpLocalAabbMin, i)) {
					VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(tmpLocalAabbMin, i));
				}
				if (VectorUtil.getCoord(localAabbMax, i) < VectorUtil.getCoord(tmpLocalAabbMax, i)) {
					VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(tmpLocalAabbMax, i));
				}
			}
		}
	}
	
	@Override
	public void setLocalScaling(Vec3 scaling) {
		localScaling.set(scaling);
	}

	@Override
	public Vec3 getLocalScaling(Vec3 out) {
		out.set(localScaling);
		return out;
	}

	@Override
	public void calculateLocalInertia(float mass, Vec3 inertia) {
		// approximation: take the inertia from the aabb for now
		Transform ident = new Transform();
		ident.setIdentity();
		Vec3 aabbMin = new Vec3(), aabbMax = new Vec3();
		getAabb(ident, aabbMin, aabbMax);

		Vec3 halfExtents = new Vec3();
		halfExtents.subHere(aabbMax, aabbMin);
		halfExtents.mult(0.5f);

		float lx = 2f * halfExtents.x;
		float ly = 2f * halfExtents.y;
		float lz = 2f * halfExtents.z;

		inertia.x = (mass / 12f) * (ly * ly + lz * lz);
		inertia.y = (mass / 12f) * (lx * lx + lz * lz);
		inertia.z = (mass / 12f) * (lx * lx + ly * ly);
	}
	
	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.COMPOUND_SHAPE_PROXYTYPE;
	}

	@Override
	public void setMargin(float margin) {
		collisionMargin = margin;
	}

	@Override
	public float getMargin() {
		return collisionMargin;
	}

	@Override
	public String getName() {
		return "Compound";
	}

	// this is optional, but should make collision queries faster, by culling non-overlapping nodes
	// void	createAabbTreeFromChildren();
	
	public OptimizedBvh getAabbTree() {
		return aabbTree;
	}

	/**
	 * Computes the exact moment of inertia and the transform from the coordinate
	 * system defined by the principal axes of the moment of inertia and the center
	 * of mass to the current coordinate system. "masses" points to an array
	 * of masses of the children. The resulting transform "principal" has to be
	 * applied inversely to all children transforms in order for the local coordinate
	 * system of the compound shape to be centered at the center of mass and to coincide
	 * with the principal axes. This also necessitates a correction of the world transform
	 * of the collision object by the principal transform.
	 */
	public void calculatePrincipalAxisTransform(float[] masses, Transform principal, Vec3 inertia) {
		int n = children.size();

		float totalMass = 0;
		Vec3 center = new Vec3();
		center.set(0, 0, 0);
		for (int k = 0; k < n; k++) {
			center.scaleAddHere(masses[k], children.getQuick(k).transform.origin, center);
			totalMass += masses[k];
		}
		center.mult(1f / totalMass);
		principal.origin.set(center);

		Mat3 tensor = new Mat3();
		tensor.setZero();

		for (int k = 0; k < n; k++) {
			Vec3 i = new Vec3();
			children.getQuick(k).childShape.calculateLocalInertia(masses[k], i);

			Transform t = children.getQuick(k).transform;
			Vec3 o = new Vec3();
			o.subHere(t.origin, center);

			// compute inertia tensor in coordinate system of compound shape
			Mat3 j = Mat3.transpose(t.basis);

			j.a *= i.x;
			j.b *= i.x;
			j.c *= i.x;
			j.d *= i.y;
			j.e *= i.y;
			j.f *= i.y;
			j.g *= i.z;
			j.h *= i.z;
			j.i *= i.z;

                        Mat3.mult(t.basis, j, j);

			// add inertia tensor
                        tensor.a += j.a;
                        tensor.b += j.b;
                        tensor.c += j.c;
                        tensor.d += j.d;
                        tensor.e += j.e;
                        tensor.f += j.f;
                        tensor.g += j.g;
                        tensor.h += j.h;
                        tensor.i += j.i;

			// compute inertia tensor of pointmass at o
			float o2 = o.squareLength();
			j.set(o2, 0, 0,
                              0, o2, 0,
                              0, 0, o2);
			j.a += o.x * -o.x;
			j.b += o.y * -o.x;
			j.c += o.z * -o.x;
			j.d += o.x * -o.y;
			j.e += o.y * -o.y;
			j.f += o.z * -o.y;
			j.g += o.x * -o.z;
			j.h += o.y * -o.z;
			j.i += o.z * -o.z;

			// add inertia tensor of pointmass
			tensor.a += masses[k] * j.a;
			tensor.b += masses[k] * j.b;
			tensor.c += masses[k] * j.c;
			tensor.d += masses[k] * j.d;
			tensor.e += masses[k] * j.e;
			tensor.f += masses[k] * j.f;
			tensor.g += masses[k] * j.g;
			tensor.h += masses[k] * j.h;
			tensor.i += masses[k] * j.i;
		}

		MatrixUtil.diagonalize(tensor, principal.basis, 0.00001f, 20);

		inertia.set(tensor.a, tensor.e, tensor.i);
	}

}
