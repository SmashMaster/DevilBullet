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
 * A generic 3-element tuple that is represented by single precision-floating  
 * point x,y,z coordinates.
 *
 */
public class Vector3f implements java.io.Serializable, Cloneable {
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
     * Constructs and initializes a Tuple3f from the specified xyz coordinates.
     * @param x the x coordinate
     * @param y the y coordinate
     * @param z the z coordinate
     */
    public Vector3f(float x, float y, float z)
    {
	this.x = x;
	this.y = y;
	this.z = z;
    }

    /**
     * Constructs and initializes a Tuple3f from the specified Tuple3f.
     * @param t1 the Tuple3f containing the initialization x y z data
     */
    public Vector3f(Vector3f t1)
    {
	this.x = t1.x;
	this.y = t1.y;
	this.z = t1.z;
    }

    /**
     * Constructs and initializes a Tuple3f to (0,0,0).
     */
    public Vector3f()
    {
	this.x = 0.0f;
	this.y = 0.0f;
	this.z = 0.0f;
    }


   /**
     * Returns a string that contains the values of this Tuple3f.
     * The form is (x,y,z).
     * @return the String representation
     */  
    public String toString() {
	return "(" + this.x + ", " + this.y + ", " + this.z + ")";
    }


    /**
     * Sets the value of this tuple to the specified xyz coordinates.
     * @param x the x coordinate
     * @param y the y coordinate
     * @param z the z coordinate
     */
    public final void set(float x, float y, float z)
    {
	this.x = x;
	this.y = y;
	this.z = z;
    }

    /**
     * Sets the value of this tuple to the value of tuple t1.
     * @param t1 the tuple to be copied
     */
    public final void set(Vector3f t1)
    {
	this.x = t1.x;
	this.y = t1.y;
	this.z = t1.z;
    }

    /**
     * Sets the value of this tuple to the vector sum of tuples t1 and t2.
     * @param t1 the first tuple
     * @param t2 the second tuple
     */
    public final void add(Vector3f t1, Vector3f t2)
    {
	this.x = t1.x + t2.x;
	this.y = t1.y + t2.y;
	this.z = t1.z + t2.z;
    }


    /**  
     * Sets the value of this tuple to the vector sum of itself and tuple t1.
     * @param t1 the other tuple
     */  
    public final void add(Vector3f t1)
    {
        this.x += t1.x;
        this.y += t1.y;
        this.z += t1.z;
    }


    /**
     * Sets the value of this tuple to the vector difference
     * of tuples t1 and t2 (this = t1 - t2).
     * @param t1 the first tuple
     * @param t2 the second tuple
     */
    public final void sub(Vector3f t1, Vector3f t2)
    {
	this.x = t1.x - t2.x;
	this.y = t1.y - t2.y;
	this.z = t1.z - t2.z;
    }


   /**  
     * Sets the value of this tuple to the vector difference of
     * itself and tuple t1 (this = this - t1) .
     * @param t1 the other tuple
     */  
    public final void sub(Vector3f t1)
    { 
        this.x -= t1.x;
        this.y -= t1.y;
        this.z -= t1.z;
    }


    /**
     * Sets the value of this tuple to the negation of tuple t1.
     * @param t1 the source tuple
     */
    public final void negate(Vector3f t1)
    {
	this.x = -t1.x;
	this.y = -t1.y;
	this.z = -t1.z;
    }


    /**
     * Negates the value of this tuple in place.
     */
    public final void negate()
    {
	this.x = -this.x;
	this.y = -this.y;
	this.z = -this.z;
    }


    /**
     * Sets the value of this vector to the scalar multiplication
     * of tuple t1.
     * @param s the scalar value
     * @param t1 the source tuple
     */
    public final void scale(float s, Vector3f t1)
    {
	this.x = s*t1.x;
	this.y = s*t1.y;
	this.z = s*t1.z;
    }


    /**
     * Sets the value of this tuple to the scalar multiplication
     * of the scale factor with this.
     * @param s the scalar value
     */
    public final void scale(float s)
    {
	this.x *= s;
	this.y *= s;
	this.z *= s;
    }


    /**
     * Sets the value of this tuple to the scalar multiplication
     * of tuple t1 and then adds tuple t2 (this = s*t1 + t2).
     * @param s the scalar value
     * @param t1 the tuple to be scaled and added
     * @param t2 the tuple to be added without a scale
     */
    public final void scaleAdd(float s, Vector3f t1, Vector3f t2)
    {
	this.x = s*t1.x + t2.x;
	this.y = s*t1.y + t2.y;
	this.z = s*t1.z + t2.z;
    }

   /**
     * Returns true if the Object t1 is of type Tuple3f and all of the
     * data members of t1 are equal to the corresponding data members in
     * this Tuple3f.
     * @param t1  the vector with which the comparison is made
     * @return  true or false
     */ 
    public boolean equals(Vector3f t1)
    {
        try {
           return(this.x == t1.x && this.y == t1.y && this.z == t1.z);
        }
        catch (NullPointerException e2) {return false;}
    }
    
   /**
     * Returns true if the Object t1 is of type Tuple3f and all of the
     * data members of t1 are equal to the corresponding data members in
     * this Tuple3f.
     * @param t1  the Object with which the comparison is made
     * @return  true or false
     */ 
    public boolean equals(Object t1)
    {
        try {
           Vector3f t2 = (Vector3f) t1;
           return(this.x == t2.x && this.y == t2.y && this.z == t2.z);
        }
        catch (NullPointerException e2) {return false;}
        catch (ClassCastException   e1) {return false;}
    }
 
  /** 
    *  Sets each component of the tuple parameter to its absolute
    *  value and places the modified values into this tuple.
    *  @param t   the source tuple, which will not be modified
    */   
  public final void absolute(Vector3f t)
  {
       x = Math.abs(t.x);
       y = Math.abs(t.y);
       z = Math.abs(t.z);
  }
 
  /**
    *  Sets each component of this tuple to its absolute value.
    */
  public final void absolute()
  {
     x = Math.abs(x);
     y = Math.abs(y);
     z = Math.abs(z);
  }


  /**  
    *  Linearly interpolates between tuples t1 and t2 and places the 
    *  result into this tuple:  this = (1-alpha)*t1 + alpha*t2.
    *  @param t1  the first tuple
    *  @param t2  the second tuple  
    *  @param alpha  the alpha interpolation parameter  
    */   
  public final void interpolate(Vector3f t1, Vector3f t2, float alpha) 
  { 
           this.x = (1-alpha)*t1.x + alpha*t2.x;
           this.y = (1-alpha)*t1.y + alpha*t2.y;
           this.z = (1-alpha)*t1.z + alpha*t2.z;
  }
    
   /**
     * Returns the squared length of this vector.
     * @return the squared length of this vector
     */
    public final float lengthSquared()
    {
        return (this.x*this.x + this.y*this.y + this.z*this.z);
    }

    /**
     * Returns the length of this vector.
     * @return the length of this vector
     */
    public final float length()
    {
        return (float)
             Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
    }


  /**
     * Sets this vector to be the vector cross product of vectors v1 and v2.
     * @param v1 the first vector
     * @param v2 the second vector
     */
    public final void cross(Vector3f v1, Vector3f v2)
    {
        float x,y;

        x = v1.y*v2.z - v1.z*v2.y;
        y = v2.x*v1.z - v2.z*v1.x;
        this.z = v1.x*v2.y - v1.y*v2.x;
        this.x = x;
        this.y = y;
    }

 /**
   * Computes the dot product of this vector and vector v1.
   * @param v1 the other vector
   * @return the dot product of this vector and v1
   */
  public final float dot(Vector3f v1)
    {
      return (this.x*v1.x + this.y*v1.y + this.z*v1.z);
    }

   /**
     * Sets the value of this vector to the normalization of vector v1.
     * @param v1 the un-normalized vector
     */
    public final void normalize(Vector3f v1)
    {
        float norm;

        norm = (float) (1.0/Math.sqrt(v1.x*v1.x + v1.y*v1.y + v1.z*v1.z));
        this.x = v1.x*norm;
        this.y = v1.y*norm;
        this.z = v1.z*norm;
    }

    /**
     * Normalizes this vector in place.
     */
    public final void normalize()
    {
        float norm;

        norm = (float)
               (1.0/Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z));
        this.x *= norm;
        this.y *= norm;
        this.z *= norm;
    }
}
