/*
 * $RCSfile$
 *
 * Copyright 1997-2008 Sun Microsystems, Inc.  All Rights Reserved.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 only, as
 * published by the Free Software Foundation.  Sun designates this
 * particular file as subject to the "Classpath" exception as provided
 * by Sun in the LICENSE file that accompanied this code.
 *
 * This code is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * version 2 for more details (a copy is included in the LICENSE file that
 * accompanied this code).
 *
 * You should have received a copy of the GNU General Public License version
 * 2 along with this work; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Please contact Sun Microsystems, Inc., 4150 Network Circle, Santa Clara,
 * CA 95054 USA or visit www.sun.com if you need additional information or
 * have any questions.
 *
 * $Revision: 127 $
 * $Date: 2008-02-28 21:18:51 +0100 (Thu, 28 Feb 2008) $
 * $State$
 */

package javax.vecmath;

/**
 * A 4 element unit quaternion represented by single precision floating 
 * point x,y,z,w coordinates.  The quaternion is always normalized.
 *
 */
public class Quat implements java.io.Serializable {
  /**
   * The x coordinate.
   */
  public	float	x;

  /**
   * The y coordinate.
   */
  public	float	y;

  /**
   * The z coordinate.
   */
  public	float	z;

  /**
   * The w coordinate.
   */
  public	float	w;


  /**
   * Constructs and initializes a Quat4f from the specified xyzw coordinates.
   * @param x the x coordinate
   * @param y the y coordinate
   * @param z the z coordinate
   * @param w the w scalar component
   */
  public Quat(float x, float y, float z, float w)
  {
      float mag;
      mag = (float)(1.0/Math.sqrt( x*x + y*y + z*z + w*w ));
      this.x =  x*mag;
      this.y =  y*mag;
      this.z =  z*mag;
      this.w =  w*mag;

  }

  /**
   * Constructs and initializes a Quat4f from the specified Quat4f.
   * @param q1 the Quat4f containing the initialization x y z w data
   */
  public Quat(Quat q1)
  {
       this.x = q1.x;
        this.y = q1.y;
        this.z = q1.z;
        this.w = q1.w;
  }

  /**
   * Constructs and initializes a Quat4f to (0.0,0.0,0.0,0.0).
   */
  public Quat()
  {
  }
  
  
    /**
     * Sets the value of this tuple to the specified xyzw coordinates.
     * @param x the x coordinate
     * @param y the y coordinate
     * @param z the z coordinate
     * @param w the w coordinate
     */
    public final void set(float x, float y, float z, float w)
    {
	this.x = x;
	this.y = y;
	this.z = z;
	this.w = w;
    }

  /**
   * Sets the value of this quaternion to the quaternion product of
   * quaternions q1 and q2 (this = q1 * q2).  
   * Note that this is safe for aliasing (e.g. this can be q1 or q2).
   * @param q1 the first quaternion
   * @param q2 the second quaternion
   */
  public final void mulHere(Quat q1, Quat q2)
  {
    if (this != q1 && this != q2) {
      this.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
      this.x = q1.w*q2.x + q2.w*q1.x + q1.y*q2.z - q1.z*q2.y;
      this.y = q1.w*q2.y + q2.w*q1.y - q1.x*q2.z + q1.z*q2.x;
      this.z = q1.w*q2.z + q2.w*q1.z + q1.x*q2.y - q1.y*q2.x;
    } else {
      float	x, y, w;

      w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
      x = q1.w*q2.x + q2.w*q1.x + q1.y*q2.z - q1.z*q2.y;
      y = q1.w*q2.y + q2.w*q1.y - q1.x*q2.z + q1.z*q2.x;
      this.z = q1.w*q2.z + q2.w*q1.z + q1.x*q2.y - q1.y*q2.x;
      this.w = w;
      this.x = x;
      this.y = y;
    }
  }


 /**
   * Sets the value of this quaternion to the quaternion product of
   * itself and q1 (this = this * q1).  
   * @param q1 the other quaternion
   */
  public final void mul(Quat q1)
  {
      float     x, y, w; 

       w = this.w*q1.w - this.x*q1.x - this.y*q1.y - this.z*q1.z;
       x = this.w*q1.x + q1.w*this.x + this.y*q1.z - this.z*q1.y;
       y = this.w*q1.y + q1.w*this.y - this.x*q1.z + this.z*q1.x;
       this.z = this.w*q1.z + q1.w*this.z + this.x*q1.y - this.y*q1.x;
       this.w = w;
       this.x = x;
       this.y = y;
  }


  /**
   * Normalizes the value of this quaternion in place.
   */
  public final void normalize()
  {
    float norm;

    norm = (this.x*this.x + this.y*this.y + this.z*this.z + this.w*this.w);

    if (norm > 0.0f) {
      norm = 1.0f / (float)Math.sqrt(norm);
      this.x *= norm;
      this.y *= norm;
      this.z *= norm;
      this.w *= norm;
    } else {
      this.x = (float) 0.0;
      this.y = (float) 0.0;
      this.z = (float) 0.0;
      this.w = (float) 0.0;
    }
  }
}
