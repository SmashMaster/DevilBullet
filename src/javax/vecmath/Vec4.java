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
 * A 4-element tuple represented by single-precision floating point x,y,z,w 
 * coordinates.
 *
 */
public class Vec4 implements java.io.Serializable {
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
   * Constructs and initializes a Tuple4f from the specified xyzw coordinates.
   * @param x the x coordinate
   * @param y the y coordinate
   * @param z the z coordinate
   * @param w the w coordinate
   */
  public Vec4(float x, float y, float z, float w)
  {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }

  /**
   * Constructs and initializes a Tuple4f from the specified Tuple4f.
   * @param t1 the Tuple4f containing the initialization x y z w data
   */
  public Vec4(Vec4 t1)
  {
    this.x = t1.x;
    this.y = t1.y;
    this.z = t1.z;
    this.w = t1.w;
  }

  /**
   * Constructs and initializes a Tuple4f to (0,0,0,0).
   */
  public Vec4()
  {
    this.x = 0.0f;
    this.y = 0.0f;
    this.z = 0.0f;
    this.w = 0.0f;
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
     * Sets the value of this tuple to the value of tuple t1.
     * @param t1 the tuple to be copied
     */
    public final void set(Vec4 t1)
    {
	this.x = t1.x;
	this.y = t1.y;
	this.z = t1.z;
	this.w = t1.w;
    }

    /**
     * Sets the x,y,z components of this vector to the corresponding
     * components of tuple t1.  The w component of this vector
     * is set to 0.
     * @param t1 the tuple to be copied
     *
     * @since vecmath 1.2
     */
    public final void set(Vector3f t1) {
	this.x = t1.x;
	this.y = t1.y;
	this.z = t1.z;
	this.w = 0.0f;
    }

   /**
     * Returns a string that contains the values of this Tuple4f.
     * The form is (x,y,z,w).
     * @return the String representation
     */  
    public String toString() {
        return "(" + this.x + ", " + this.y + ", " + this.z + ", " + this.w + ")";
    }
    
  /**
    *  Sets each component of this tuple to its absolute value.
    */
  public final void absolute()
  {
     x = Math.abs(x);
     y = Math.abs(y);
     z = Math.abs(z);
     w = Math.abs(w);
  }
}
