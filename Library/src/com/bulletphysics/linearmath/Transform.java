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

import com.bulletphysics.collision.shapes.UniformScalingShape;
import com.samrj.devil.math.Mat3;
import com.samrj.devil.math.Quat;
import com.samrj.devil.math.Vec3;

/**
 * Transform represents translation and rotation (rigid transform). Scaling and
 * shearing is not supported.<p>
 * 
 * You can use local shape scaling or {@link UniformScalingShape} for static rescaling
 * of collision objects.
 * 
 * @author jezek2
 */
public class Transform {
    
	//protected BulletStack stack;

	/** Rotation matrix of this Transform. */
	public final Mat3 basis = new Mat3();
	
	/** Translation vector of this Transform. */
	public final Vec3 origin = new Vec3();

	public Transform() {
	}

	public Transform(Mat3 mat) {
		basis.set(mat);
	}

	public Transform(Transform tr) {
		set(tr);
	}
        
        public Transform(com.samrj.devil.math.Transform tr) {
                set(tr);
        }
	
	public void set(Transform tr) {
		basis.set(tr.basis);
		origin.set(tr.origin);
	}
        
	public void set(Mat3 mat) {
		basis.set(mat);
		origin.set(0f, 0f, 0f);
	}
        
        public void set(com.samrj.devil.math.Transform tr) {
            basis.setRotation(tr.rot);
            origin.set(tr.pos);
        }
        
	public void transform(Vec3 v) {
		MatrixUtil.transform(basis, v);
		v.add(origin);
	}

	public void setIdentity() {
		basis.setIdentity();
		origin.set(0f, 0f, 0f);
	}
	
	public void inverse() {
		basis.transpose();
		origin.mult(-1f);
		MatrixUtil.transform(basis, origin);
	}

	public void inverse(Transform tr) {
		set(tr);
		inverse();
	}
	
	public void mul(Transform tr) {
		Vec3 vec = new Vec3(tr.origin);
		transform(vec);

		basis.mult(tr.basis);
		origin.set(vec);
	}
        
	public void mul(Transform tr1, Transform tr2) {
		Vec3 vec = new Vec3(tr2.origin);
		tr1.transform(vec);

                Mat3.mult(tr1.basis, tr2.basis, basis);
		origin.set(vec);
	}
	
	public void invXform(Vec3 inVec, Vec3 out) {
		VectorUtil.sub(out, inVec, origin);

		Mat3 mat = new Mat3(basis);
		mat.transpose();
		MatrixUtil.transform(mat, out);
	}
	
	public Quat getRotation(Quat out) {
		MatrixUtil.getRotation(basis, out);
		return out;
	}
	
	public void setRotation(Quat q) {
		MatrixUtil.setRotation(basis, q);
	}
	
	public void setFromOpenGLMatrix(float[] m) {
		MatrixUtil.setFromOpenGLSubMatrix(basis, m);
		origin.set(m[12], m[13], m[14]);
	}

	public void getOpenGLMatrix(float[] m) {
		MatrixUtil.getOpenGLSubMatrix(basis, m);
		m[12] = origin.x;
		m[13] = origin.y;
		m[14] = origin.z;
		m[15] = 1f;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == null || !(obj instanceof Transform)) return false;
		Transform tr = (Transform)obj;
		return basis.equals(tr.basis) && origin.equals(tr.origin);
	}

	@Override
	public int hashCode() {
		int hash = 3;
		hash = 41 * hash + basis.hashCode();
		hash = 41 * hash + origin.hashCode();
		return hash;
	}
	
}
