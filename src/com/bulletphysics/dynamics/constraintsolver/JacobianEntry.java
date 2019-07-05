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

package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.samrj.devil.math.Mat3;
import javax.vecmath.Vec3;

//notes:
// Another memory optimization would be to store m_1MinvJt in the remaining 3 w components
// which makes the btJacobianEntry memory layout 16 bytes
// if you only are interested in angular part, just feed massInvA and massInvB zero

/**
 * Jacobian entry is an abstraction that allows to describe constraints.
 * It can be used in combination with a constraint solver.
 * Can be used to relate the effect of an impulse to the constraint error.
 * 
 * @author jezek2
 */
public class JacobianEntry {
	
	//protected final BulletStack stack = BulletStack.get();
	
	public final Vec3 linearJointAxis = new Vec3();
	public final Vec3 aJ = new Vec3();
	public final Vec3 bJ = new Vec3();
	public final Vec3 m_0MinvJt = new Vec3();
	public final Vec3 m_1MinvJt = new Vec3();
	// Optimization: can be stored in the w/last component of one of the vectors
	public float Adiag;

	public JacobianEntry() {
	}

	/**
	 * Constraint between two different rigidbodies.
	 */
	public void init(Mat3 world2A,
			Mat3 world2B,
			Vec3 rel_pos1, Vec3 rel_pos2,
			Vec3 jointAxis,
			Vec3 inertiaInvA,
			float massInvA,
			Vec3 inertiaInvB,
			float massInvB)
	{
		linearJointAxis.set(jointAxis);

		aJ.crossHere(rel_pos1, linearJointAxis);
		MatrixUtil.transform(world2A, aJ);

		bJ.set(linearJointAxis);
		bJ.negate();
		bJ.crossHere(rel_pos2, bJ);
		MatrixUtil.transform(world2B, bJ);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = massInvA + m_0MinvJt.dot(aJ) + massInvB + m_1MinvJt.dot(bJ);

		assert (Adiag > 0f);
	}

	/**
	 * Angular constraint between two different rigidbodies.
	 */
	public void init(Vec3 jointAxis,
		Mat3 world2A,
		Mat3 world2B,
		Vec3 inertiaInvA,
		Vec3 inertiaInvB)
	{
		linearJointAxis.set(0f, 0f, 0f);

		aJ.set(jointAxis);
		MatrixUtil.transform(world2A, aJ);

		bJ.set(jointAxis);
		bJ.negate();
		MatrixUtil.transform(world2B, bJ);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = m_0MinvJt.dot(aJ) + m_1MinvJt.dot(bJ);

		assert (Adiag > 0f);
	}

	/**
	 * Angular constraint between two different rigidbodies.
	 */
	public void init(Vec3 axisInA,
		Vec3 axisInB,
		Vec3 inertiaInvA,
		Vec3 inertiaInvB)
	{
		linearJointAxis.set(0f, 0f, 0f);
		aJ.set(axisInA);

		bJ.set(axisInB);
		bJ.negate();

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = m_0MinvJt.dot(aJ) + m_1MinvJt.dot(bJ);

		assert (Adiag > 0f);
	}

	/**
	 * Constraint on one rigidbody.
	 */
	public void init(
		Mat3 world2A,
		Vec3 rel_pos1, Vec3 rel_pos2,
		Vec3 jointAxis,
		Vec3 inertiaInvA, 
		float massInvA)
	{
		linearJointAxis.set(jointAxis);

		aJ.crossHere(rel_pos1, jointAxis);
		MatrixUtil.transform(world2A, aJ);

		bJ.set(jointAxis);
		bJ.negate();
		bJ.crossHere(rel_pos2, bJ);
		MatrixUtil.transform(world2A, bJ);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		m_1MinvJt.set(0f, 0f, 0f);
		Adiag = massInvA + m_0MinvJt.dot(aJ);

		assert (Adiag > 0f);
	}

	public float getDiagonal() { return Adiag; }

	/**
	 * For two constraints on the same rigidbody (for example vehicle friction).
	 */
	public float getNonDiagonal(JacobianEntry jacB, float massInvA) {
		JacobianEntry jacA = this;
		float lin = massInvA * jacA.linearJointAxis.dot(jacB.linearJointAxis);
		float ang = jacA.m_0MinvJt.dot(jacB.aJ);
		return lin + ang;
	}

	/**
	 * For two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies).
	 */
	public float getNonDiagonal(JacobianEntry jacB, float massInvA, float massInvB) {
		JacobianEntry jacA = this;

		Vec3 lin = new Vec3();
		VectorUtil.mul(lin, jacA.linearJointAxis, jacB.linearJointAxis);

		Vec3 ang0 = new Vec3();
		VectorUtil.mul(ang0, jacA.m_0MinvJt, jacB.aJ);

		Vec3 ang1 = new Vec3();
		VectorUtil.mul(ang1, jacA.m_1MinvJt, jacB.bJ);

		Vec3 lin0 = new Vec3();
		lin0.scale(massInvA, lin);

		Vec3 lin1 = new Vec3();
		lin1.scale(massInvB, lin);

		Vec3 sum = new Vec3();
		VectorUtil.add(sum, ang0, ang1, lin0, lin1);

		return sum.x + sum.y + sum.z;
	}

	public float getRelativeVelocity(Vec3 linvelA, Vec3 angvelA, Vec3 linvelB, Vec3 angvelB) {
		Vec3 linrel = new Vec3();
		linrel.subHere(linvelA, linvelB);

		Vec3 angvela = new Vec3();
		VectorUtil.mul(angvela, angvelA, aJ);

		Vec3 angvelb = new Vec3();
		VectorUtil.mul(angvelb, angvelB, bJ);

		VectorUtil.mul(linrel, linrel, linearJointAxis);

		angvela.add(angvelb);
		angvela.add(linrel);

		float rel_vel2 = angvela.x + angvela.y + angvela.z;
		return rel_vel2 + BulletGlobals.FLT_EPSILON;
	}
	
}
