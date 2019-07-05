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

/**
 * A single precision floating point 3 by 3 matrix.
 * Primarily to support 3D rotations.
 *
 */
public class Matrix3f implements java.io.Serializable, Cloneable {
  /** 
    * The first matrix element in the first row.
    */
    public	float	m00;

  /** 
    * The second matrix element in the first row.
    */
    public	float	m01;

  /** 
    * The third matrix element in the first row.
    */
    public	float	m02;

  /** 
    * The first matrix element in the second row.
    */
    public	float	m10;

  /** 
    * The second matrix element in the second row.
    */
    public	float	m11;

  /** 
    * The third matrix element in the second row.
    */
    public	float	m12;

  /** 
    * The first matrix element in the third row.
    */
    public	float	m20;

  /** 
    * The second matrix element in the third row.
    */
    public	float	m21;

  /** 
    * The third matrix element in the third row.
    */
    public	float	m22;
  
   /**
     *  Constructs a new matrix with the same values as the
     *  Matrix3f parameter.
     *  @param m1  the source matrix
     */  
   public Matrix3f(Matrix3f m1)
   {
        this.m00 = m1.m00;
        this.m01 = m1.m01;
        this.m02 = m1.m02;
 
        this.m10 = m1.m10;
        this.m11 = m1.m11;
        this.m12 = m1.m12;
 
        this.m20 = m1.m20;
        this.m21 = m1.m21;
        this.m22 = m1.m22;

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
	this.m00 + ", " + this.m01 + ", " + this.m02 + "\n" +
	this.m10 + ", " + this.m11 + ", " + this.m12 + "\n" +
	this.m20 + ", " + this.m21 + ", " + this.m22 + "\n";
    }

    /**
     * Sets this Matrix3f to identity.
     */
    public final void setIdentity()
    {
	this.m00 = (float) 1.0;
	this.m01 = (float) 0.0;
	this.m02 = (float) 0.0;

	this.m10 = (float) 0.0;
	this.m11 = (float) 1.0;
	this.m12 = (float) 0.0;

	this.m20 = (float) 0.0;
	this.m21 = (float) 0.0;
	this.m22 = (float) 1.0;
    }

    /**
     * Sets the specified element of this matrix3f to the value provided.
     * @param row the row number to be modified (zero indexed)
     * @param column the column number to be modified (zero indexed)
     * @param value the new value
     */
    public final void setElement(int row, int column, float value)
    {
	switch (row) 
	  {
	  case 0:
	    switch(column)
	      {
	      case 0:
		this.m00 = value;
		break;
	      case 1:
		this.m01 = value;
		break;
	      case 2:
		this.m02 = value;
		break;
	      default:
		throw new ArrayIndexOutOfBoundsException();
	      }
	    break;

	  case 1:
	    switch(column) 
	      {
	      case 0:
		this.m10 = value;
		break;
	      case 1:
		this.m11 = value;
		break;
	      case 2:
		this.m12 = value;
		break;
	      default:
		throw new ArrayIndexOutOfBoundsException();
	      }
	    break;
	  
	  case 2:
	    switch(column) 
	      {
	      case 0:
		this.m20 = value;
		break;
	      case 1:
		this.m21 = value;
		break;
	      case 2:
		this.m22 = value;
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
           v.x = m00;
           v.y = m01;
           v.z = m02;
        } else if(row == 1) {
           v.x = m10;
           v.y = m11;
           v.z = m12;
        } else if(row == 2) {
           v.x = m20;
           v.y = m21;
           v.z = m22;
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
           v.x = m00;
           v.y = m10;
           v.z = m20;
        } else if(column == 1) {
           v.x = m01;
           v.y = m11;
           v.z = m21;
        }else if(column == 2){
           v.x = m02;
           v.y = m12;
           v.z = m22;
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
    public final float getElement(int row, int column)
    {
	switch (row) 
	  {
	  case 0:
	    switch(column)
	      {
	      case 0:
		return(this.m00);
	      case 1:
		return(this.m01);
	      case 2:
		return(this.m02);
	      default:
                break;
	      }
	    break;
	  case 1:
	    switch(column) 
	      {
	      case 0:
		return(this.m10);
	      case 1:
		return(this.m11);
	      case 2:
		return(this.m12);
	      default:
                break;
	      }
	    break;
	  
	  case 2:
	    switch(column) 
	      {
	      case 0:
		return(this.m20);
	      case 1:
		return(this.m21);
	      case 2:
		return(this.m22);
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
	    this.m00 = x;
	    this.m01 = y;
	    this.m02 = z;
	    break;

	case 1:
	    this.m10 = x;
	    this.m11 = y;
	    this.m12 = z;
	    break;

	case 2:
	    this.m20 = x;
	    this.m21 = y;
	    this.m22 = z;
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
	    this.m00 = v.x;
	    this.m01 = v.y;
	    this.m02 = v.z;
	    break;

	case 1:
	    this.m10 = v.x;
	    this.m11 = v.y;
	    this.m12 = v.z;
	    break;

	case 2:
	    this.m20 = v.x;
	    this.m21 = v.y;
	    this.m22 = v.z;
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
        this.m00 += m1.m00;
        this.m01 += m1.m01;
        this.m02 += m1.m02;
 
        this.m10 += m1.m10;
        this.m11 += m1.m11;
        this.m12 += m1.m12;
 
        this.m20 += m1.m20;
        this.m21 += m1.m21;
        this.m22 += m1.m22;
    }  
    
    /**
     * Sets the value of this matrix to its transpose.
     */
    public final void transpose()
    {
	float temp;

	temp = this.m10;
	this.m10 = this.m01;
	this.m01 = temp;

	temp = this.m20;
	this.m20 = this.m02;
	this.m02 = temp;

	temp = this.m21;
	this.m21 = this.m12;
	this.m12 = temp;
    }

    /**
     * Sets the value of this matrix to the transpose of the argument matrix.
     * @param m1 the matrix to be transposed
     */
    public final void transpose(Matrix3f m1)
    {
	if (this != m1) {
	    this.m00 = m1.m00;
	    this.m01 = m1.m10;
	    this.m02 = m1.m20;

	    this.m10 = m1.m01;
	    this.m11 = m1.m11;
	    this.m12 = m1.m21;

	    this.m20 = m1.m02;
	    this.m21 = m1.m12;
	    this.m22 = m1.m22;
	} else
	    this.transpose();
    }

    /**
     * Sets the value of this matrix to the matrix conversion of the
     * (single precision) quaternion argument.
     * @param q1 the quaternion to be converted
     */
    public final void set(Quat4f q1)
    {
	this.m00 = 1.0f - 2.0f*q1.y*q1.y - 2.0f*q1.z*q1.z;
	this.m10 = 2.0f*(q1.x*q1.y + q1.w*q1.z);
	this.m20 = 2.0f*(q1.x*q1.z - q1.w*q1.y);

	this.m01 = 2.0f*(q1.x*q1.y - q1.w*q1.z);
	this.m11 = 1.0f - 2.0f*q1.x*q1.x - 2.0f*q1.z*q1.z;
	this.m21 = 2.0f*(q1.y*q1.z + q1.w*q1.x);

	this.m02 = 2.0f*(q1.x*q1.z + q1.w*q1.y);
	this.m12 = 2.0f*(q1.y*q1.z - q1.w*q1.x);
	this.m22 = 1.0f - 2.0f*q1.x*q1.x - 2.0f*q1.y*q1.y;
    }

    /**
     * Sets the value of this matrix to the value of the Matrix3f 
     * argument. 
     * @param m1 the source matrix3f 
     */  
    public final void set(Matrix3f m1) {

        this.m00 = m1.m00;
        this.m01 = m1.m01;
        this.m02 = m1.m02;

        this.m10 = m1.m10;
        this.m11 = m1.m11;
        this.m12 = m1.m12;

        this.m20 = m1.m20;
        this.m21 = m1.m21;
        this.m22 = m1.m22;

    }

    /**
     * Sets the value of this matrix to the matrix inverse
     * of the passed matrix m1.
     * @param m1 the matrix to be inverted
     */
    public final void invert(Matrix3f m1)
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
        temp[0] = (double)m1.m00;
        temp[1] = (double)m1.m01;
        temp[2] = (double)m1.m02;
 
        temp[3] = (double)m1.m10;
        temp[4] = (double)m1.m11;
        temp[5] = (double)m1.m12;
 
        temp[6] = (double)m1.m20;
        temp[7] = (double)m1.m21;
        temp[8] = (double)m1.m22;
 

	// Calculate LU decomposition: Is the matrix singular? 
	if (!luDecomposition(temp, row_perm)) {
	    // Matrix has no inverse 
	    throw new SingularMatrixException();
	}

	// Perform back substitution on the identity matrix 
        for(i=0;i<9;i++) result[i] = 0.0;
        result[0] = 1.0; result[4] = 1.0; result[8] = 1.0;
	luBacksubstitution(temp, row_perm, result);

        this.m00 = (float)result[0];
        this.m01 = (float)result[1];
        this.m02 = (float)result[2];

        this.m10 = (float)result[3];
        this.m11 = (float)result[4];
        this.m12 = (float)result[5];
 
        this.m20 = (float)result[6];
        this.m21 = (float)result[7];
        this.m22 = (float)result[8];

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

	this.m00 = (float) 1.0;
	this.m01 = (float) 0.0;
	this.m02 = (float) 0.0;

	this.m10 = (float) 0.0;
	this.m11 = cosAngle;
	this.m12 = -sinAngle;

	this.m20 = (float) 0.0;
	this.m21 = sinAngle;
	this.m22 = cosAngle;
    }

   /**
     * Sets the value of this matrix to the result of multiplying itself
     * with matrix m1.
     * @param m1 the other matrix
     */  
    public final void mul(Matrix3f m1)
    {
          float       m00, m01, m02,
                      m10, m11, m12,
                      m20, m21, m22;

            m00 = this.m00*m1.m00 + this.m01*m1.m10 + this.m02*m1.m20;
            m01 = this.m00*m1.m01 + this.m01*m1.m11 + this.m02*m1.m21;
            m02 = this.m00*m1.m02 + this.m01*m1.m12 + this.m02*m1.m22;
 
            m10 = this.m10*m1.m00 + this.m11*m1.m10 + this.m12*m1.m20;
            m11 = this.m10*m1.m01 + this.m11*m1.m11 + this.m12*m1.m21;
            m12 = this.m10*m1.m02 + this.m11*m1.m12 + this.m12*m1.m22;
 
            m20 = this.m20*m1.m00 + this.m21*m1.m10 + this.m22*m1.m20;
            m21 = this.m20*m1.m01 + this.m21*m1.m11 + this.m22*m1.m21;
            m22 = this.m20*m1.m02 + this.m21*m1.m12 + this.m22*m1.m22;
 
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
    }

    /**
     * Sets the value of this matrix to the result of multiplying
     * the two argument matrices together.
     * @param m1 the first matrix
     * @param m2 the second matrix
     */
    public final void mul(Matrix3f m1, Matrix3f m2)
    {
	if (this != m1 && this != m2) {
            this.m00 = m1.m00*m2.m00 + m1.m01*m2.m10 + m1.m02*m2.m20;
            this.m01 = m1.m00*m2.m01 + m1.m01*m2.m11 + m1.m02*m2.m21;
            this.m02 = m1.m00*m2.m02 + m1.m01*m2.m12 + m1.m02*m2.m22;

            this.m10 = m1.m10*m2.m00 + m1.m11*m2.m10 + m1.m12*m2.m20;
            this.m11 = m1.m10*m2.m01 + m1.m11*m2.m11 + m1.m12*m2.m21;
            this.m12 = m1.m10*m2.m02 + m1.m11*m2.m12 + m1.m12*m2.m22;

            this.m20 = m1.m20*m2.m00 + m1.m21*m2.m10 + m1.m22*m2.m20;
            this.m21 = m1.m20*m2.m01 + m1.m21*m2.m11 + m1.m22*m2.m21;
            this.m22 = m1.m20*m2.m02 + m1.m21*m2.m12 + m1.m22*m2.m22;
	} else {
	    float	m00, m01, m02,
			m10, m11, m12,
			m20, m21, m22;

            m00 = m1.m00*m2.m00 + m1.m01*m2.m10 + m1.m02*m2.m20; 
            m01 = m1.m00*m2.m01 + m1.m01*m2.m11 + m1.m02*m2.m21; 
            m02 = m1.m00*m2.m02 + m1.m01*m2.m12 + m1.m02*m2.m22; 
 
            m10 = m1.m10*m2.m00 + m1.m11*m2.m10 + m1.m12*m2.m20; 
            m11 = m1.m10*m2.m01 + m1.m11*m2.m11 + m1.m12*m2.m21; 
            m12 = m1.m10*m2.m02 + m1.m11*m2.m12 + m1.m12*m2.m22; 
 
            m20 = m1.m20*m2.m00 + m1.m21*m2.m10 + m1.m22*m2.m20; 
            m21 = m1.m20*m2.m01 + m1.m21*m2.m11 + m1.m22*m2.m21; 
            m22 = m1.m20*m2.m02 + m1.m21*m2.m12 + m1.m22*m2.m22; 

            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
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

        return(this.m00 == m1.m00 && this.m01 == m1.m01 && this.m02 == m1.m02
            && this.m10 == m1.m10 && this.m11 == m1.m11 && this.m12 == m1.m12
            && this.m20 == m1.m20 && this.m21 == m1.m21 && this.m22 == m1.m22);
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
           return(this.m00 == m2.m00 && this.m01 == m2.m01 && this.m02 == m2.m02
             && this.m10 == m2.m10 && this.m11 == m2.m11 && this.m12 == m2.m12
             && this.m20 == m2.m20 && this.m21 == m2.m21 && this.m22 == m2.m22);
        }
        catch (ClassCastException   e1) { return false; } 
        catch (NullPointerException e2) { return false; }
    }

  /**
    *  Sets this matrix to all zeros.
    */
   public final void setZero()
   {
        m00 = 0.0f;
        m01 = 0.0f;
        m02 = 0.0f;
 
        m10 = 0.0f;
        m11 = 0.0f;
        m12 = 0.0f;
 
        m20 = 0.0f;
        m21 = 0.0f;
        m22 = 0.0f;

   }

   /**
    * Multiply this matrix by the tuple t and place the result
    * back into the tuple (t = this*t).
    * @param t  the tuple to be multiplied by this matrix and then replaced
    */
    public final void transform(Vector3f t) {
     float x,y,z;
     x = m00* t.x + m01*t.y + m02*t.z; 
     y = m10* t.x + m11*t.y + m12*t.z; 
     z = m20* t.x + m21*t.y + m22*t.z; 
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
     x = m00* t.x + m01*t.y + m02*t.z; 
     y = m10* t.x + m11*t.y + m12*t.z;
     result.z = m20* t.x + m21*t.y + m22*t.z; 
     result.x = x;
     result.y = y;
    }
}
