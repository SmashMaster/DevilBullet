/*
 * $RCSfile$
 *
 * Copyright 1996-2008 Sun Microsystems, Inc.  All Rights Reserved.
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

import com.samrj.devil.math.Quat;

/**
 * A single precision floating point 3 by 3 matrix.
 * Primarily to support 3D rotations.
 *
 */
public class Matrix3f implements java.io.Serializable {
  /** 
    * The first matrix element in the first row.
    */
    public	float	a;

  /** 
    * The second matrix element in the first row.
    */
    public	float	b;

  /** 
    * The third matrix element in the first row.
    */
    public	float	c;

  /** 
    * The first matrix element in the second row.
    */
    public	float	d;

  /** 
    * The second matrix element in the second row.
    */
    public	float	e;

  /** 
    * The third matrix element in the second row.
    */
    public	float	f;

  /** 
    * The first matrix element in the third row.
    */
    public	float	g;

  /** 
    * The second matrix element in the third row.
    */
    public	float	h;

  /** 
    * The third matrix element in the third row.
    */
    public	float	i;
  
   /**
     *  Constructs a new matrix with the same values as the
     *  Matrix3f parameter.
     *  @param m1  the source matrix
     */  
   public Matrix3f(Matrix3f m1)
   {
        this.a = m1.a;
        this.b = m1.b;
        this.c = m1.c;
 
        this.d = m1.d;
        this.e = m1.e;
        this.f = m1.f;
 
        this.g = m1.g;
        this.h = m1.h;
        this.i = m1.i;

   }

    /**
     * Constructs and initializes a Matrix3f to all zeros.
     */
    public Matrix3f()
    {
    }

   /**
     * Returns a string that contains the values of this Matrix3f.
     * @return the String representation
     */
    public String toString() {
      return
	this.a + ", " + this.b + ", " + this.c + "\n" +
	this.d + ", " + this.e + ", " + this.f + "\n" +
	this.g + ", " + this.h + ", " + this.i + "\n";
    }

    /**
     * Sets this Matrix3f to identity.
     */
    public final void setIdentity()
    {
	this.a = (float) 1.0;
	this.b = (float) 0.0;
	this.c = (float) 0.0;

	this.d = (float) 0.0;
	this.e = (float) 1.0;
	this.f = (float) 0.0;

	this.g = (float) 0.0;
	this.h = (float) 0.0;
	this.i = (float) 1.0;
    }

    /**
     * Sets the specified element of this matrix3f to the value provided.
     * @param row the row number to be modified (zero indexed)
     * @param column the column number to be modified (zero indexed)
     * @param value the new value
     */
    public final void setEntry(int row, int column, float value)
    {
	switch (row) 
	  {
	  case 0:
	    switch(column)
	      {
	      case 0:
		this.a = value;
		break;
	      case 1:
		this.b = value;
		break;
	      case 2:
		this.c = value;
		break;
	      default:
		throw new ArrayIndexOutOfBoundsException();
	      }
	    break;

	  case 1:
	    switch(column) 
	      {
	      case 0:
		this.d = value;
		break;
	      case 1:
		this.e = value;
		break;
	      case 2:
		this.f = value;
		break;
	      default:
		throw new ArrayIndexOutOfBoundsException();
	      }
	    break;
	  
	  case 2:
	    switch(column) 
	      {
	      case 0:
		this.g = value;
		break;
	      case 1:
		this.h = value;
		break;
	      case 2:
		this.i = value;
		break;
	      default:

		throw new ArrayIndexOutOfBoundsException();
	      }
	    break;

	  default:
		throw new ArrayIndexOutOfBoundsException();
	  }
    }

    /**
     * Copies the matrix values in the specified row into the vector parameter. 
     * @param row  the matrix row
     * @param v    the vector into which the matrix row values will be copied
     */
    public final void getRow(int row, Vector3f v) {
         if( row == 0 ) {
           v.x = a;
           v.y = b;
           v.z = c;
        } else if(row == 1) {
           v.x = d;
           v.y = e;
           v.z = f;
        } else if(row == 2) {
           v.x = g;
           v.y = h;
           v.z = i;
        } else {
          throw new ArrayIndexOutOfBoundsException();
        }

    }

    /**
     * Copies the matrix values in the specified column into the vector 
     * parameter.
     * @param column  the matrix column
     * @param v    the vector into which the matrix row values will be copied
     */  
    public final void getColumn(int column, Vector3f v) {
        if( column == 0 ) {
           v.x = a;
           v.y = d;
           v.z = g;
        } else if(column == 1) {
           v.x = b;
           v.y = e;
           v.z = h;
        }else if(column == 2){
           v.x = c;
           v.y = f;
           v.z = i;
        } else {
           throw new ArrayIndexOutOfBoundsException();
        }

    }

    /**
     * Retrieves the value at the specified row and column of this
     * matrix.
     * @param row the row number to be retrieved (zero indexed)
     * @param column the column number to be retrieved (zero indexed)
     * @return the value at the indexed element.
     */
    public final float getEntry(int row, int column)
    {
	switch (row) 
	  {
	  case 0:
	    switch(column)
	      {
	      case 0:
		return(this.a);
	      case 1:
		return(this.b);
	      case 2:
		return(this.c);
	      default:
                break;
	      }
	    break;
	  case 1:
	    switch(column) 
	      {
	      case 0:
		return(this.d);
	      case 1:
		return(this.e);
	      case 2:
		return(this.f);
	      default:
                break;
	      }
	    break;
	  
	  case 2:
	    switch(column) 
	      {
	      case 0:
		return(this.g);
	      case 1:
		return(this.h);
	      case 2:
		return(this.i);
	      default:
                break;
	      }
	    break;
	    
	  default:
            break;
	  }
       throw new ArrayIndexOutOfBoundsException();
    }

    /**
     * Sets the specified row of this matrix3f to the three values provided.
     * @param row the row number to be modified (zero indexed)
     * @param x the first column element
     * @param y the second column element
     * @param z the third column element
     */
    public final void setRow(int row, float x, float y, float z)
    {
	switch (row) {
	case 0:
	    this.a = x;
	    this.b = y;
	    this.c = z;
	    break;

	case 1:
	    this.d = x;
	    this.e = y;
	    this.f = z;
	    break;

	case 2:
	    this.g = x;
	    this.h = y;
	    this.i = z;
	    break;

	default:
	  throw new ArrayIndexOutOfBoundsException();
	}
    }

    /**
     * Sets the specified row of this matrix3f to the Vector provided.
     * @param row the row number to be modified (zero indexed)
     * @param v the replacement row
     */
    public final void setRow(int row, Vector3f v)
    {
	switch (row) {
	case 0:
	    this.a = v.x;
	    this.b = v.y;
	    this.c = v.z;
	    break;

	case 1:
	    this.d = v.x;
	    this.e = v.y;
	    this.f = v.z;
	    break;

	case 2:
	    this.g = v.x;
	    this.h = v.y;
	    this.i = v.z;
	    break;

	default:
	  throw new ArrayIndexOutOfBoundsException();
	}
    }

    /**
     * Sets the value of this matrix to the matrix sum of itself and 
     * matrix m1.
     * @param m1 the other matrix
     */
    public final void add(Matrix3f m1)
    {  
        this.a += m1.a;
        this.b += m1.b;
        this.c += m1.c;
 
        this.d += m1.d;
        this.e += m1.e;
        this.f += m1.f;
 
        this.g += m1.g;
        this.h += m1.h;
        this.i += m1.i;
    }  
    
    /**
     * Sets the value of this matrix to its transpose.
     */
    public final void transpose()
    {
	float temp;

	temp = this.d;
	this.d = this.b;
	this.b = temp;

	temp = this.g;
	this.g = this.c;
	this.c = temp;

	temp = this.h;
	this.h = this.f;
	this.f = temp;
    }

    /**
     * Sets the value of this matrix to the transpose of the argument matrix.
     * @param m1 the matrix to be transposed
     */
    public final void transposeHere(Matrix3f m1)
    {
	if (this != m1) {
	    this.a = m1.a;
	    this.b = m1.d;
	    this.c = m1.g;

	    this.d = m1.b;
	    this.e = m1.e;
	    this.f = m1.h;

	    this.g = m1.c;
	    this.h = m1.f;
	    this.i = m1.i;
	} else
	    this.transpose();
    }

    /**
     * Sets the value of this matrix to the matrix conversion of the
     * (single precision) quaternion argument.
     * @param q1 the quaternion to be converted
     */
    public final void setRotation(Quat q1)
    {
	this.a = 1.0f - 2.0f*q1.y*q1.y - 2.0f*q1.z*q1.z;
	this.d = 2.0f*(q1.x*q1.y + q1.w*q1.z);
	this.g = 2.0f*(q1.x*q1.z - q1.w*q1.y);

	this.b = 2.0f*(q1.x*q1.y - q1.w*q1.z);
	this.e = 1.0f - 2.0f*q1.x*q1.x - 2.0f*q1.z*q1.z;
	this.h = 2.0f*(q1.y*q1.z + q1.w*q1.x);

	this.c = 2.0f*(q1.x*q1.z + q1.w*q1.y);
	this.f = 2.0f*(q1.y*q1.z - q1.w*q1.x);
	this.i = 1.0f - 2.0f*q1.x*q1.x - 2.0f*q1.y*q1.y;
    }

    /**
     * Sets the value of this matrix to the value of the Matrix3f 
     * argument. 
     * @param m1 the source matrix3f 
     */  
    public final void set(Matrix3f m1) {

        this.a = m1.a;
        this.b = m1.b;
        this.c = m1.c;

        this.d = m1.d;
        this.e = m1.e;
        this.f = m1.f;

        this.g = m1.g;
        this.h = m1.h;
        this.i = m1.i;

    }

    /**
     * Sets the value of this matrix to the matrix inverse
     * of the passed matrix m1.
     * @param m1 the matrix to be inverted
     */
    public final void invertHere(Matrix3f m1)
    {
	 invertGeneral( m1);
    }

    /**
     * General invert routine.  Inverts m1 and places the result in "this".
     * Note that this routine handles both the "this" version and the
     * non-"this" version.
     *
     * Also note that since this routine is slow anyway, we won't worry
     * about allocating a little bit of garbage.
     */
    private final void invertGeneral(Matrix3f  m1) {
	double temp[] = new double[9];
	double result[] = new double[9];
	int row_perm[] = new int[3];
	int i, r, c;

	// Use LU decomposition and backsubstitution code specifically
	// for floating-point 3x3 matrices.

	// Copy source matrix to t1tmp 
        temp[0] = (double)m1.a;
        temp[1] = (double)m1.b;
        temp[2] = (double)m1.c;
 
        temp[3] = (double)m1.d;
        temp[4] = (double)m1.e;
        temp[5] = (double)m1.f;
 
        temp[6] = (double)m1.g;
        temp[7] = (double)m1.h;
        temp[8] = (double)m1.i;
 

	// Calculate LU decomposition: Is the matrix singular? 
	if (!luDecomposition(temp, row_perm)) {
	    // Matrix has no inverse 
	    throw new IllegalArgumentException("singular matrix");
	}

	// Perform back substitution on the identity matrix 
        for(i=0;i<9;i++) result[i] = 0.0;
        result[0] = 1.0; result[4] = 1.0; result[8] = 1.0;
	luBacksubstitution(temp, row_perm, result);

        this.a = (float)result[0];
        this.b = (float)result[1];
        this.c = (float)result[2];

        this.d = (float)result[3];
        this.e = (float)result[4];
        this.f = (float)result[5];
 
        this.g = (float)result[6];
        this.h = (float)result[7];
        this.i = (float)result[8];

    }

    /**
     * Given a 3x3 array "matrix0", this function replaces it with the 
     * LU decomposition of a row-wise permutation of itself.  The input 
     * parameters are "matrix0" and "dimen".  The array "matrix0" is also 
     * an output parameter.  The vector "row_perm[3]" is an output 
     * parameter that contains the row permutations resulting from partial 
     * pivoting.  The output parameter "even_row_xchg" is 1 when the 
     * number of row exchanges is even, or -1 otherwise.  Assumes data 
     * type is always double.
     *
     * This function is similar to luDecomposition, except that it
     * is tuned specifically for 3x3 matrices.
     *
     * @return true if the matrix is nonsingular, or false otherwise.
     */
    //
    // Reference: Press, Flannery, Teukolsky, Vetterling, 
    //	      _Numerical_Recipes_in_C_, Cambridge University Press, 
    //	      1988, pp 40-45.
    //
    static boolean luDecomposition(double[] matrix0,
				   int[] row_perm) {

	double row_scale[] = new double[3];

	// Determine implicit scaling information by looping over rows 
	{
	    int i, j;
	    int ptr, rs;
	    double big, temp;

	    ptr = 0;
	    rs = 0;

	    // For each row ... 
	    i = 3;
	    while (i-- != 0) {
		big = 0.0;

		// For each column, find the largest element in the row 
		j = 3;
		while (j-- != 0) {
		    temp = matrix0[ptr++];
		    temp = Math.abs(temp);
		    if (temp > big) {
			big = temp;
		    }
		}

		// Is the matrix singular? 
		if (big == 0.0) {
		    return false;
		}
		row_scale[rs++] = 1.0 / big;
	    }
	}

	{
	    int j;
	    int mtx;

	    mtx = 0;

	    // For all columns, execute Crout's method 
	    for (j = 0; j < 3; j++) {
		int i, imax, k;
		int target, p1, p2;
		double sum, big, temp;

		// Determine elements of upper diagonal matrix U 
		for (i = 0; i < j; i++) {
		    target = mtx + (3*i) + j;
		    sum = matrix0[target];
		    k = i;
		    p1 = mtx + (3*i);
		    p2 = mtx + j;
		    while (k-- != 0) {
			sum -= matrix0[p1] * matrix0[p2];
			p1++;
			p2 += 3;
		    }
		    matrix0[target] = sum;
		}

		// Search for largest pivot element and calculate
		// intermediate elements of lower diagonal matrix L.
		big = 0.0;
		imax = -1;
		for (i = j; i < 3; i++) {
		    target = mtx + (3*i) + j;
		    sum = matrix0[target];
		    k = j;
		    p1 = mtx + (3*i);
		    p2 = mtx + j;
		    while (k-- != 0) {
			sum -= matrix0[p1] * matrix0[p2];
			p1++;
			p2 += 3;
		    }
		    matrix0[target] = sum;

		    // Is this the best pivot so far? 
		    if ((temp = row_scale[i] * Math.abs(sum)) >= big) {
			big = temp;
			imax = i;
		    }
		}

		if (imax < 0) {
		    throw new RuntimeException();
		}

		// Is a row exchange necessary? 
		if (j != imax) {
		    // Yes: exchange rows 
		    k = 3;
		    p1 = mtx + (3*imax);
		    p2 = mtx + (3*j);
		    while (k-- != 0) {
			temp = matrix0[p1];
			matrix0[p1++] = matrix0[p2];
			matrix0[p2++] = temp;
		    }

		    // Record change in scale factor 
		    row_scale[imax] = row_scale[j];
		}

		// Record row permutation 
		row_perm[j] = imax;

		// Is the matrix singular 
		if (matrix0[(mtx + (3*j) + j)] == 0.0) {
		    return false;
		}

		// Divide elements of lower diagonal matrix L by pivot 
		if (j != (3-1)) {
		    temp = 1.0 / (matrix0[(mtx + (3*j) + j)]);
		    target = mtx + (3*(j+1)) + j;
		    i = 2 - j;
		    while (i-- != 0) {
			matrix0[target] *= temp;
			target += 3;
		    }
		}
	    }
	}

	return true;
    }

    /**
     * Solves a set of linear equations.  The input parameters "matrix1",
     * and "row_perm" come from luDecompostionD3x3 and do not change
     * here.  The parameter "matrix2" is a set of column vectors assembled
     * into a 3x3 matrix of floating-point values.  The procedure takes each
     * column of "matrix2" in turn and treats it as the right-hand side of the
     * matrix equation Ax = LUx = b.  The solution vector replaces the
     * original column of the matrix.
     *
     * If "matrix2" is the identity matrix, the procedure replaces its contents
     * with the inverse of the matrix from which "matrix1" was originally
     * derived.
     */
    //
    // Reference: Press, Flannery, Teukolsky, Vetterling, 
    //	      _Numerical_Recipes_in_C_, Cambridge University Press, 
    //	      1988, pp 44-45.
    //
    static void luBacksubstitution(double[] matrix1,
				   int[] row_perm,
				   double[] matrix2) {

	int i, ii, ip, j, k;
	int rp;
	int cv, rv;
	
	//	rp = row_perm;
	rp = 0;

	// For each column vector of matrix2 ... 
	for (k = 0; k < 3; k++) {
	    //	    cv = &(matrix2[0][k]);
	    cv = k;
	    ii = -1;

	    // Forward substitution 
	    for (i = 0; i < 3; i++) {
		double sum;

		ip = row_perm[rp+i];
		sum = matrix2[cv+3*ip];
		matrix2[cv+3*ip] = matrix2[cv+3*i];
		if (ii >= 0) {
		    //		    rv = &(matrix1[i][0]);
		    rv = i*3;
		    for (j = ii; j <= i-1; j++) {
			sum -= matrix1[rv+j] * matrix2[cv+3*j];
		    }
		}
		else if (sum != 0.0) {
		    ii = i;
		}
		matrix2[cv+3*i] = sum;
	    }

	    // Backsubstitution 
	    //	    rv = &(matrix1[3][0]);
	    rv = 2*3;
	    matrix2[cv+3*2] /= matrix1[rv+2];

	    rv -= 3;
	    matrix2[cv+3*1] = (matrix2[cv+3*1] -
			    matrix1[rv+2] * matrix2[cv+3*2]) / matrix1[rv+1];

	    rv -= 3;
	    matrix2[cv+4*0] = (matrix2[cv+3*0] -
			    matrix1[rv+1] * matrix2[cv+3*1] -
			    matrix1[rv+2] * matrix2[cv+3*2]) / matrix1[rv+0];

	}
    }
    
    /**
     * Sets the value of this matrix to a counter clockwise rotation 
     * about the x axis.
     * @param angle the angle to rotate about the X axis in radians
     */
    public final void rotX(float angle)
    {
	float	sinAngle, cosAngle;

	sinAngle = (float) Math.sin((double) angle);
	cosAngle = (float) Math.cos((double) angle);

	this.a = (float) 1.0;
	this.b = (float) 0.0;
	this.c = (float) 0.0;

	this.d = (float) 0.0;
	this.e = cosAngle;
	this.f = -sinAngle;

	this.g = (float) 0.0;
	this.h = sinAngle;
	this.i = cosAngle;
    }

   /**
     * Sets the value of this matrix to the result of multiplying itself
     * with matrix m1.
     * @param m1 the other matrix
     */  
    public final void mult(Matrix3f m1)
    {
          float       m00, m01, m02,
                      m10, m11, m12,
                      m20, m21, m22;

            m00 = this.a*m1.a + this.b*m1.d + this.c*m1.g;
            m01 = this.a*m1.b + this.b*m1.e + this.c*m1.h;
            m02 = this.a*m1.c + this.b*m1.f + this.c*m1.i;
 
            m10 = this.d*m1.a + this.e*m1.d + this.f*m1.g;
            m11 = this.d*m1.b + this.e*m1.e + this.f*m1.h;
            m12 = this.d*m1.c + this.e*m1.f + this.f*m1.i;
 
            m20 = this.g*m1.a + this.h*m1.d + this.i*m1.g;
            m21 = this.g*m1.b + this.h*m1.e + this.i*m1.h;
            m22 = this.g*m1.c + this.h*m1.f + this.i*m1.i;
 
            this.a = m00; this.b = m01; this.c = m02;
            this.d = m10; this.e = m11; this.f = m12;
            this.g = m20; this.h = m21; this.i = m22;
    }

    /**
     * Sets the value of this matrix to the result of multiplying
     * the two argument matrices together.
     * @param m1 the first matrix
     * @param m2 the second matrix
     */
    public final void multHere(Matrix3f m1, Matrix3f m2)
    {
	if (this != m1 && this != m2) {
            this.a = m1.a*m2.a + m1.b*m2.d + m1.c*m2.g;
            this.b = m1.a*m2.b + m1.b*m2.e + m1.c*m2.h;
            this.c = m1.a*m2.c + m1.b*m2.f + m1.c*m2.i;

            this.d = m1.d*m2.a + m1.e*m2.d + m1.f*m2.g;
            this.e = m1.d*m2.b + m1.e*m2.e + m1.f*m2.h;
            this.f = m1.d*m2.c + m1.e*m2.f + m1.f*m2.i;

            this.g = m1.g*m2.a + m1.h*m2.d + m1.i*m2.g;
            this.h = m1.g*m2.b + m1.h*m2.e + m1.i*m2.h;
            this.i = m1.g*m2.c + m1.h*m2.f + m1.i*m2.i;
	} else {
	    float	m00, m01, m02,
			m10, m11, m12,
			m20, m21, m22;

            m00 = m1.a*m2.a + m1.b*m2.d + m1.c*m2.g; 
            m01 = m1.a*m2.b + m1.b*m2.e + m1.c*m2.h; 
            m02 = m1.a*m2.c + m1.b*m2.f + m1.c*m2.i; 
 
            m10 = m1.d*m2.a + m1.e*m2.d + m1.f*m2.g; 
            m11 = m1.d*m2.b + m1.e*m2.e + m1.f*m2.h; 
            m12 = m1.d*m2.c + m1.e*m2.f + m1.f*m2.i; 
 
            m20 = m1.g*m2.a + m1.h*m2.d + m1.i*m2.g; 
            m21 = m1.g*m2.b + m1.h*m2.e + m1.i*m2.h; 
            m22 = m1.g*m2.c + m1.h*m2.f + m1.i*m2.i; 

            this.a = m00; this.b = m01; this.c = m02;
            this.d = m10; this.e = m11; this.f = m12;
            this.g = m20; this.h = m21; this.i = m22;
	}
    }

   /**
     * Returns true if all of the data members of Matrix3f m1 are
     * equal to the corresponding data members in this Matrix3f.
     * @param m1  the matrix with which the comparison is made
     * @return  true or false
     */  
    public boolean equals(Matrix3f m1)
    {
      try {

        return(this.a == m1.a && this.b == m1.b && this.c == m1.c
            && this.d == m1.d && this.e == m1.e && this.f == m1.f
            && this.g == m1.g && this.h == m1.h && this.i == m1.i);
      }  
      catch (NullPointerException e2) { return false; }

    }

   /**
     * Returns true if the Object o1 is of type Matrix3f and all of the
     * data members of o1 are equal to the corresponding data members in
     * this Matrix3f.
     * @param o1  the object with which the comparison is made
     * @return  true or false
     */  
    public boolean equals(Object o1)
    {
      try { 

           Matrix3f m2 = (Matrix3f) o1;
           return(this.a == m2.a && this.b == m2.b && this.c == m2.c
             && this.d == m2.d && this.e == m2.e && this.f == m2.f
             && this.g == m2.g && this.h == m2.h && this.i == m2.i);
        }
        catch (ClassCastException   e1) { return false; } 
        catch (NullPointerException e2) { return false; }
    }

  /**
    *  Sets this matrix to all zeros.
    */
   public final void setZero()
   {
        a = 0.0f;
        b = 0.0f;
        c = 0.0f;
 
        d = 0.0f;
        e = 0.0f;
        f = 0.0f;
 
        g = 0.0f;
        h = 0.0f;
        i = 0.0f;

   }

   /**
    * Multiply this matrix by the tuple t and place the result
    * back into the tuple (t = this*t).
    * @param t  the tuple to be multiplied by this matrix and then replaced
    */
    public final void transform(Vector3f t) {
     float x,y,z;
     x = a* t.x + b*t.y + c*t.z; 
     y = d* t.x + e*t.y + f*t.z; 
     z = g* t.x + h*t.y + i*t.z; 
     t.set(x,y,z);
    }

   /**
    * Multiply this matrix by the tuple t and and place the result 
    * into the tuple "result" (result = this*t).
    * @param t  the tuple to be multiplied by this matrix
    * @param result  the tuple into which the product is placed
    */
    public final void transform(Vector3f t, Vector3f result) { 
     float x,y,z;
     x = a* t.x + b*t.y + c*t.z; 
     y = d* t.x + e*t.y + f*t.z;
     result.z = g* t.x + h*t.y + i*t.z; 
     result.x = x;
     result.y = y;
    }
}
