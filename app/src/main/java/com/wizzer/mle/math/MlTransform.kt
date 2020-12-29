// COPYRIGHT_BEGIN
//
// The MIT License (MIT)
//
// Copyright (c) 2020-2021 Wizzer Works
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//  For information concerning this header file, contact Mark S. Millard,
//  of Wizzer Works at msm@wizzerworks.com.
//
//  More information concerning Wizzer Works may be found at
//
//      http://www.wizzerworks.com
//
// COPYRIGHT_END

// Declare package.
package com.wizzer.mle.math

// Import Magic Lantern math classes.
import com.wizzer.mle.math.MlAngle.Companion.angleToDegrees
import com.wizzer.mle.math.MlAngle.Companion.degreesToAngle
import com.wizzer.mle.math.MlMath.Companion.mlAsin
import com.wizzer.mle.math.MlMath.Companion.mlAtan2
import com.wizzer.mle.math.MlMath.Companion.mlCos
import com.wizzer.mle.math.MlMath.Companion.mlSin


/**
* This class implements a 4x3 affine matrix of floating-point elements.
*/
class MlTransform
{
    // The internal matrix.
    var mMatrix =
        Array(4) { FloatArray(3) }

    /**
     * The default constructor.
     */
    constructor() {
        setZero()
    }

    /**
     * A constructor given all 12 elements in row-major order.
     *
     * @param a11 Element for row 0, column 0.
     * @param a12 Element for row 0, column 1.
     * @param a13 Element for row 0, column 2.
     * @param a21 Element for row 1, column 0.
     * @param a22 Element for row 1, column 1.
     * @param a23 Element for row 1, column 2.
     * @param a31 Element for row 2, column 0.
     * @param a32 Element for row 2, column 1.
     * @param a33 Element for row 2, column 2.
     * @param a41 Element for row 3, column 0.
     * @param a42 Element for row 3, column 1.
     * @param a43 Element for row 3, column 2.
     */
    constructor(
        a11: Float, a12: Float, a13: Float,
        a21: Float, a22: Float, a23: Float,
        a31: Float, a32: Float, a33: Float,
        a41: Float, a42: Float, a43: Float
    ) {
        mMatrix[0][0] = a11
        mMatrix[0][1] = a12
        mMatrix[0][2] = a13
        mMatrix[1][0] = a21
        mMatrix[1][1] = a22
        mMatrix[1][2] = a23
        mMatrix[2][0] = a31
        mMatrix[2][1] = a32
        mMatrix[2][2] = a33
        mMatrix[3][0] = a41
        mMatrix[3][1] = a42
        mMatrix[3][2] = a43
    }

    /**
     * A constructor given an array of 4x3 floating-point
     * values.
     *
     * @param m The array of elements.
     */
    constructor(m: Array<FloatArray>) {
        setValue(m)
    }

    /**
     * Set the value of the transform based on an array of
     * 4 x 3 floating-point values.
     *
     * @param m The array of elements to set.
     */
    fun setValue(m: Array<FloatArray>) {
        mMatrix[0][0] = m[0][0]
        mMatrix[0][1] = m[0][1]
        mMatrix[0][2] = m[0][2]
        mMatrix[1][0] = m[1][0]
        mMatrix[1][1] = m[1][1]
        mMatrix[1][2] = m[1][2]
        mMatrix[2][0] = m[2][0]
        mMatrix[2][1] = m[2][1]
        mMatrix[2][2] = m[2][2]
        mMatrix[3][0] = m[3][0]
        mMatrix[3][1] = m[3][1]
        mMatrix[3][2] = m[3][2]
    }

    /**
     * Set the value of the transform based on the specified `MlTransform`.
     *
     * @param m The tranform to use to set **this**'s value.
     */
    fun setValue(m: MlTransform) {
        mMatrix[0][0] = m.mMatrix[0][0]
        mMatrix[0][1] = m.mMatrix[0][1]
        mMatrix[0][2] = m.mMatrix[0][2]
        mMatrix[1][0] = m.mMatrix[1][0]
        mMatrix[1][1] = m.mMatrix[1][1]
        mMatrix[1][2] = m.mMatrix[1][2]
        mMatrix[2][0] = m.mMatrix[2][0]
        mMatrix[2][1] = m.mMatrix[2][1]
        mMatrix[2][2] = m.mMatrix[2][2]
        mMatrix[3][0] = m.mMatrix[3][0]
        mMatrix[3][1] = m.mMatrix[3][1]
        mMatrix[3][2] = m.mMatrix[3][2]
    }

    /**
     * Get the value of the tranform.
     *
     * @return The value is returned as an array of
     * 4 x 3 floating-point values.
     */
    fun getValue(): Array<FloatArray> {
        val m =
            Array(4) { FloatArray(3) }
        m[0][0] = mMatrix[0][0]
        m[0][1] = mMatrix[0][1]
        m[0][2] = mMatrix[0][2]
        m[1][0] = mMatrix[1][0]
        m[1][1] = mMatrix[1][1]
        m[1][2] = mMatrix[1][2]
        m[2][0] = mMatrix[2][0]
        m[2][1] = mMatrix[2][1]
        m[2][2] = mMatrix[2][2]
        m[3][0] = mMatrix[3][0]
        m[3][1] = mMatrix[3][1]
        m[3][2] = mMatrix[3][2]
        return m
    }

    /**
     * Sets matrix to be identity.
     */
    fun makeIdentity() {
        mMatrix[0][0] = 1f
        mMatrix[0][1] = 0f
        mMatrix[0][2] = 0f
        mMatrix[1][0] = 0f
        mMatrix[1][1] = 1f
        mMatrix[1][2] = 0f
        mMatrix[2][0] = 0f
        mMatrix[2][1] = 0f
        mMatrix[2][2] = 1f
        mMatrix[3][0] = 0f
        mMatrix[3][1] = 0f
        mMatrix[3][2] = 0f
    }

    companion object {
        /**
         * Create an identity transform.
         *
         * @return Returns an identity matrix.
         */
        @JvmStatic
        fun identity(): MlTransform {
            val trans = MlTransform()
            trans.makeIdentity()
            return trans
        }
    }

    // Check matrix m for identity.
    private fun checkIdentity(m: Array<FloatArray>): Boolean {
        if (m[0][0] == 1.0f && m[0][1] == 0.0f && m[0][2] == 0.0f &&
            m[1][0] == 0.0f && m[1][1] == 1.0f && m[1][2] == 0.0f &&
            m[2][0] == 0.0f && m[2][1] == 0.0f && m[2][2] == 1.0f &&
            m[3][0] == 0.0f && m[3][1] == 0.0f && m[3][2] == 0.0f
        )
            return true

        return false
    }

    /**
     * Returns whether matrix is identity.
     *
     * @return **true** is returned if **this** is an identity
     * matrix. Otherwise, **false** will be returned.
     */
    fun isIdentity(): Boolean {
        return checkIdentity(mMatrix)
    }

    /**
     * Set all components to zero.
     */
    fun setZero() {
        mMatrix[0][0] = 0f
        mMatrix[0][1] = 0f
        mMatrix[0][2] = 0f
        mMatrix[1][0] = 0f
        mMatrix[1][1] = 0f
        mMatrix[1][2] = 0f
        mMatrix[2][0] = 0f
        mMatrix[2][1] = 0f
        mMatrix[2][2] = 0f
        mMatrix[3][0] = 0f
        mMatrix[3][1] = 0f
        mMatrix[3][2] = 0f
    }

    /**
     * Returns whether matrix is all zeros.
     *
     * @return **true** is returned if **this** is a zero-ed out
     * matrix. Otherwise, **false** will be returned.
     */
    fun isZero(): Boolean {
        if (mMatrix[0][0] === 0f && mMatrix[0][1] === 0f && mMatrix[0][2] === 0f &&
            mMatrix[1][0] === 0f && mMatrix[1][1] === 0f && mMatrix[1][2] === 0f &&
            mMatrix[2][0] === 0f && mMatrix[2][1] === 0f && mMatrix[2][2] === 0f &&
            mMatrix[3][0] === 0f && mMatrix[3][1] === 0f && mMatrix[3][2] === 0f
        )
            return true

        return false
    }

    // Returns determinant of 3x3 submatrix composed of given row indices (0-3).
    private fun det3(r1: Int, r2: Int, r3: Int): Float {
        return ((mMatrix[r1][0] * mMatrix[r2][1] * mMatrix[r3][2]
                + mMatrix[r1][1] * mMatrix[r2][2] * mMatrix[r3][0]
                + mMatrix[r1][2] * mMatrix[r2][0] * mMatrix[r3][1])
                - mMatrix[r1][0] * mMatrix[r2][2] * mMatrix[r3][1]
                - mMatrix[r1][1] * mMatrix[r2][0] * mMatrix[r3][2]
                - mMatrix[r1][2] * mMatrix[r2][1] * mMatrix[r3][0])
    }

    /**
     * Returns determinant of upper-left 3x3 matrix.
     *
     * @return The dterminant is returned.
     */
    fun determinant(): Float {
        return det3(0, 1, 2)
    }

    /**
     * Factors a matrix m into 5 pieces: m = r s r^ u t, where r^
     * means transpose of r, and r and u are rotations, s is a scale,
     * and t is a translation. Any projection information is returned
     * in proj.
     *
     * @param r The rotation of the transform.
     * @param s The scale of the transform.
     * @param u The rotation of the transform.
     * @param t The translation of the transform.
     * @param proj The projection of the transform.
     *
     * @return **true** is returned if **this** tranform was successfully
     * factored. Otherwise **false<>/b will be returned upon failer.
     ** */
    fun factor(
        r: MlTransform, s: MlVector3, u: MlTransform,
        t: MlVector3, proj: MlTransform
    ): Boolean {
        /*
         * Variable declarations from the original source:
         *
         * n    : order of matrix A
         * eivec: true if eigenvectors are desired, false otherwise.
         * a    : Array [1:n, 1:n] of numbers, assumed symmetric!
         *
         * a    : Superdiagonal elements of the original array a are destroyed.
         *        Diagonal and subdiagonal elements are untouched.
         * d    : Array [1:n] of eigenvalues of a.
         * v    : Array [1:n, 1:n] containing (if eivec = TRUE), the eigenvectors of
         *        a, with the kth column being the normalized eigenvector with
         *        eigenvalue d[k].
         * rot  : The number of jacobi rotations required to perform the operation.
         */
        val det: Float /* Determinant of matrix A */
        val det_sign: Float /* -1 if det < 0, 1 if det > 0 */
        val scratch: Float
        var i: Int
        var j: Int
        val junk = IntArray(0)
        val a = MlTransform()
        val b = MlTransform()
        val si = MlTransform()
        val evalues = FloatArray(3)
        val evectors = arrayOfNulls<MlVector3>(3)
        a.setValue(this)
        proj.makeIdentity()
        scratch = MlScalar.ML_SCALAR_ONE
        i = 0
        while (i < 3) {
            j = 0
            while (j < 3) {
                a.mMatrix[i][j] = a.mMatrix[i][j] * scratch
                j++
            }
            t.mVector[i] = mMatrix[3][i] * scratch
            a.mMatrix[3][i] = MlScalar.ML_SCALAR_ZERO
            i++
        }

        /* (3) Compute det A. If negative, set sign = -1, else sign = 1 */det = a.determinant()
        det_sign =
            if (det < MlScalar.ML_SCALAR_ZERO) -MlScalar.ML_SCALAR_ONE else MlScalar.ML_SCALAR_ONE
        if (det_sign * det < 1e-12) return false // Singular.

        /* (4) B = A * A^  (here A^ means A transpose) */b.setValue(a.mul(a.transpose()))
        b.jacobi3(evalues, evectors, junk)

        // Find min / max eigenvalues and do ratio test to determine singularity.
        r.setValue(
            MlTransform(
                evectors[0]!!.mVector[0],
                evectors[0]!!.mVector[1],
                evectors[0]!!.mVector[2],
                evectors[1]!!.mVector[0],
                evectors[1]!!.mVector[1],
                evectors[1]!!.mVector[2],
                evectors[2]!!.mVector[0],
                evectors[2]!!.mVector[1],
                evectors[2]!!.mVector[2],
                MlScalar.ML_SCALAR_ZERO,
                MlScalar.ML_SCALAR_ZERO,
                MlScalar.ML_SCALAR_ZERO
            )
        )

        /* Compute s = sqrt(evalues), with sign. Set si = s-inverse */si.makeIdentity()
        i = 0
        while (i < 3) {
            s.mVector[i] = det_sign * Math.sqrt(evalues[i].toDouble()).toFloat()
            si.mMatrix[i][i] = 1 / s.mVector[i]
            i++
        }

        /* (5) Compute U = R^ S! R A. */
        //u.setValue(r.mul(si.mul(r.transpose().mul(a))));
        u.setValue(r.mul(si).mul(r.transpose()).mul(a))
        return true
    }

    // Diagonalizes 3x3 matrix. See comment for factor().
    private fun jacobi3(
        evalues: FloatArray,
        evectors: Array<MlVector3?>,
        rots: IntArray
    ) {
        var sm: Float // smallest entry
        var theta: Float // angle for Jacobi rotation
        var c: Float
        var s: Float
        var t: Float // cosine, sine, tangent of theta
        var tau: Float // sine / (1 + cos)
        var h: Float
        var g: Float // two scrap values
        var thresh: Float // threshold below which no rotation done
        val b = FloatArray(3) // more scratch
        val z = FloatArray(3) // more scratch
        var p: Int
        var q: Int
        var i: Int
        var j: Int
        val a =
            Array(3) { FloatArray(3) }

        // Initializations.
        i = 0
        while (i < 3) {
            evalues[i] = mMatrix[i][i]
            b[i] = evalues[i]
            z[i] = MlScalar.ML_SCALAR_ZERO
            j = 0
            while (j < 3) {
                evectors[i]!!.mVector[j] =
                    if (i == j) MlScalar.ML_SCALAR_ONE else MlScalar.ML_SCALAR_ZERO
                a[i][j] = mMatrix[i][j]
                j++
            }
            i++
        }
        rots[0] = 0

        // Why 50? I don't know--it's the way the folks who wrote the
        // algorithm did it:
        i = 0
        while (i < 50) {
            sm = MlScalar.ML_SCALAR_ZERO
            p = 0
            while (p < 3 - 1) {
                q = p + 1
                while (q < 3) {
                    sm += Math.abs(a[p][q])
                    q++
                }
                p++
            }
            if (sm == MlScalar.ML_SCALAR_ZERO) return
            thresh = if (i < 3) sm * 0.022222222222f else MlScalar.ML_SCALAR_ZERO
            p = 0
            while (p < 3 - 1) {
                q = p + 1
                while (q < 3) {
                    g = (100.0 * Math.abs(a[p][q])).toFloat()
                    if (i > 3 && Math.abs(evalues[p]) + g == Math.abs(
                            evalues[p]
                        ) &&
                        Math.abs(evalues[q]) + g == Math.abs(
                            evalues[q]
                        )
                    ) a[p][q] = MlScalar.ML_SCALAR_ZERO else if (Math.abs(
                            a[p][q]
                        ) > thresh
                    ) {
                        h = evalues[q] - evalues[p]
                        if (Math.abs(h) + g == Math.abs(h)) t =
                            a[p][q] / h else {
                            theta = MlScalar.ML_SCALAR_HALF * h / a[p][q]
                            t =
                                (1 / (Math.abs(theta) + Math.sqrt(MlScalar.ML_SCALAR_ONE + (theta * theta).toDouble()))).toFloat()
                            if (theta < MlScalar.ML_SCALAR_ZERO) t = -t
                        }
                        // End of computing tangent of rotation angle
                        c =
                            (1 / Math.sqrt(MlScalar.ML_SCALAR_ONE + (t * t).toDouble())).toFloat()
                        s = t * c
                        tau = s / (MlScalar.ML_SCALAR_ONE + c)
                        h = t * a[p][q]
                        z[p] -= h
                        z[q] += h
                        evalues[p] -= h
                        evalues[q] += h
                        a[p][q] = MlScalar.ML_SCALAR_ZERO
                        j = 0
                        while (j < p) {
                            g = a[j][p]
                            h = a[j][q]
                            a[j][p] = g - s * (h + g * tau)
                            a[j][q] = h + s * (g - h * tau)
                            j++
                        }
                        j = p + 1
                        while (j < q) {
                            g = a[p][j]
                            h = a[j][q]
                            a[p][j] = g - s * (h + g * tau)
                            a[j][q] = h + s * (g - h * tau)
                            j++
                        }
                        j = q + 1
                        while (j < 3) {
                            g = a[p][j]
                            h = a[q][j]
                            a[p][j] = g - s * (h + g * tau)
                            a[q][j] = h + s * (g - h * tau)
                            j++
                        }
                        j = 0
                        while (j < 3) {
                            g = evectors[j]!!.mVector[p]
                            h = evectors[j]!!.mVector[q]
                            evectors[j]!!.mVector[p] = g - s * (h + g * tau)
                            evectors[j]!!.mVector[q] = h + s * (g - h * tau)
                            j++
                        }
                    }
                    rots[0]++
                    q++
                }
                p++
            }
            p = 0
            while (p < 3) {
                b[p] += z[p]
                evalues[p] = b[p]
                z[p] = MlScalar.ML_SCALAR_ZERO
                p++
            }
            i++
        }
    }

    /**
     * This method finds the inverse of an affine matrix.
     * The last column MUST be [0 0 0 1] for this to work.
     * This is taken from graphics gems 2, page 603
     *
     * computes the inverse of a 3d affine matrix; i.e. a matrix with a
     * dimensionality of 4 where the right column has the entries (0,0,0,1).
     *
     * This procedure treats the 4 by 4 matrix as a block matrix and calculates
     * the inverse of one submatrix for a significant performance
     * improvement over a general procedure that can invert any nonsingular matrix.
     *
     * -1
     * -1   |    |      |  -1    |
     * M   = |A  0|  =   | A     0|
     * |    |      |        |
     * |    |      |   -1   |
     * |C  1|      |-CA    1|
     *
     * where   M is a 4 by 4 matrix,
     * A is the 3 by 3 upper left submatrix of M,
     * C is the 1 by 3 lower left submatrix of M.
     * Input:
     * in - 3D affine matrix
     * Output:
     * out - inverse of 3D affine matrix
     * Returned Value:
     * inverse matrix if input matrix is nonsingular and affine
     * unchanged otherwise
     *
     * @return A new tranform is returned.
     */
    fun inverse(): MlTransform {
        // Trivial case
        if (checkIdentity(mMatrix)) return MlTransform.identity()
        val result =
            Array(4) { FloatArray(3) }

        // Calculate the determinant of submatrix A and determine if the matrix
        // is singular as limited by the double precision floating
        // point data representation.
        var det_1: Float
        var pos: Float
        var neg: Float
        var temp: Float
        neg = MlScalar.ML_SCALAR_ZERO
        pos = neg
        temp = mMatrix[0][0] * mMatrix[1][1] * mMatrix[2][2]
        if (temp >= MlScalar.ML_SCALAR_ZERO) pos += temp else neg += temp
        temp = mMatrix[0][1] * mMatrix[1][2] * mMatrix[2][0]
        if (temp >= MlScalar.ML_SCALAR_ZERO) pos += temp else neg += temp
        temp = mMatrix[0][2] * mMatrix[1][0] * mMatrix[2][1]
        if (temp >= MlScalar.ML_SCALAR_ZERO) pos += temp else neg += temp
        temp = -(mMatrix[0][2] * mMatrix[1][1] * mMatrix[2][0])
        if (temp >= MlScalar.ML_SCALAR_ZERO) pos += temp else neg += temp
        temp = -(mMatrix[0][1] * mMatrix[1][0] * mMatrix[2][2])
        if (temp >= MlScalar.ML_SCALAR_ZERO) pos += temp else neg += temp
        temp = -(mMatrix[0][0] * mMatrix[1][2] * mMatrix[2][1])
        if (temp >= MlScalar.ML_SCALAR_ZERO) pos += temp else neg += temp
        det_1 = pos + neg

        // XXX - May want to make this a variable.
        val PRECISION_LIMIT = 1.0e-15.toFloat()

        // Is the submatrix A singular?
        temp = det_1 / (pos - neg)
        if (Math.abs(temp) < PRECISION_LIMIT) return MlTransform(mMatrix)

        // Calculate inverse(A) = adj(A) / det(A)
        det_1 = 1 / det_1
        result[0][0] = mMatrix[1][1] * mMatrix[2][2] - mMatrix[1][2] * mMatrix[2][1] * det_1
        result[1][0] =
            -(mMatrix[1][0] * mMatrix[2][2] - mMatrix[1][2] * mMatrix[2][0] * det_1)
        result[2][0] = mMatrix[1][0] * mMatrix[2][1] - mMatrix[1][1] * mMatrix[2][0] * det_1
        result[0][1] =
            -(mMatrix[0][1] * mMatrix[2][2] - mMatrix[0][2] * mMatrix[2][1] * det_1)
        result[1][1] = mMatrix[0][0] * mMatrix[2][2] - mMatrix[0][2] * mMatrix[2][0] * det_1
        result[2][1] =
            -(mMatrix[0][0] * mMatrix[2][1] - mMatrix[0][1] * mMatrix[2][0] * det_1)
        result[0][2] = mMatrix[0][1] * mMatrix[1][2] - mMatrix[0][2] * mMatrix[1][1] * det_1
        result[1][2] =
            -(mMatrix[0][0] * mMatrix[1][2] - mMatrix[0][2] * mMatrix[1][0] * det_1)
        result[2][2] = mMatrix[0][0] * mMatrix[1][1] - mMatrix[0][1] * mMatrix[1][0] * det_1

        // Calculate -C * inverse(A)
        result[3][0] = -(mMatrix[3][0] * result[0][0] +
                mMatrix[3][1] * result[1][0] +
                mMatrix[3][2] * result[2][0])
        result[3][1] = -(mMatrix[3][0] * result[0][1] +
                mMatrix[3][1] * result[1][1] +
                mMatrix[3][2] * result[2][1])
        result[3][2] = -(mMatrix[3][0] * result[0][2] +
                mMatrix[3][1] * result[1][2] +
                mMatrix[3][2] * result[2][2])

        return MlTransform(result)
    }

    // Returns transpose of matrix
    fun transpose(): MlTransform {
        return MlTransform(
            mMatrix[0][0], mMatrix[1][0], mMatrix[2][0],
            mMatrix[0][1], mMatrix[1][1], mMatrix[2][1],
            mMatrix[0][2], mMatrix[1][2], mMatrix[2][2],
            0f, 0f, 0f
        )
    }

    //  Multiplies matrix by given matrix on right. this = this * trans.
    fun mulRight(trans: MlTransform): MlTransform {
        // Trivial cases.
        if (checkIdentity(trans.mMatrix))
            return this
        else if (checkIdentity(mMatrix)) {
            setValue(trans.mMatrix)
            return this
        }
        val tmp = Array(4) { FloatArray(3) }

        for (i in 0..2)
            for (j in 0..2)
                tmp[i][j] = mMatrix[i][0] * trans.mMatrix[0][j] +
                            mMatrix[i][1] * trans.mMatrix[1][j] +
                            mMatrix[i][2] * trans.mMatrix[2][j]
        for (i in 0..2)
            tmp[3][i] = mMatrix[3][0] * trans.mMatrix[0][i] +
                        mMatrix[3][1] * trans.mMatrix[1][i] +
                        mMatrix[3][2] * trans.mMatrix[2][i] +
                        trans.mMatrix[3][i]

        // Save calculated value.
        setValue(tmp)

        return this
    }

    //  Multiplies matrix by given matrix on left. this = trans * this.
    fun mulLeft(trans: MlTransform): MlTransform {
        // Trivial cases.
        if (checkIdentity(trans.mMatrix))
            return this
        else if (checkIdentity(mMatrix)) {
            setValue(trans.mMatrix)
            return this
        }

        val tmp = Array(4) { FloatArray(3) }

        for (i in 0..2)
            for (j in 0..2)
                tmp[i][j] = trans.mMatrix[i][0] * mMatrix[0][j] +
                            trans.mMatrix[i][1] * mMatrix[1][j] +
                            trans.mMatrix[i][2] * mMatrix[2][j]
        for (i in 0..2)
            tmp[3][i] = trans.mMatrix[3][0] * mMatrix[0][i] +
                        trans.mMatrix[3][1] * mMatrix[1][i] +
                        trans.mMatrix[3][2] * mMatrix[2][i] +
                        mMatrix[3][i]

        // Save calculated value.
        setValue(tmp)

        return this
    }

    // Multiplies matrix by given column vector, giving vector result
    fun mulMatrixVec(src: MlVector3, dst: MlVector3) {
        val x: Float
        val y: Float
        val z: Float
        val v = FloatArray(3)
        src.getValue(v)

        x = mMatrix[0][0] * v[0] + mMatrix[0][1] * v[1] +
            mMatrix[0][2] * v[2]
        y = mMatrix[1][0] * v[0] + mMatrix[1][1] * v[1] +
            mMatrix[1][2] * v[2]
        z = mMatrix[2][0] * v[0] + mMatrix[2][1] * v[1] +
            mMatrix[2][2] * v[2]
        dst.setValue(x, y, z)
    }

    // Multiplies given row vector by matrix, giving vector result
    fun mulVecMatrix(src: MlVector3, dst: MlVector3) {
        val x: Float
        val y: Float
        val z: Float
        val v = FloatArray(3)
        src.getValue(v)
        x = v[0] * mMatrix[0][0] + v[1] * mMatrix[1][0] +
            v[2] * mMatrix[2][0] + mMatrix[3][0]
        y = v[0] * mMatrix[0][1] + v[1] * mMatrix[1][1] +
            v[2] * mMatrix[2][1] + mMatrix[3][1]
        z = v[0] * mMatrix[0][2] + v[1] * mMatrix[1][2] +
            v[2] * mMatrix[2][2] + mMatrix[3][2]
        dst.setValue(x, y, z)
    }

    // Multiplies given row vector by matrix, giving vector result
    // src is assumed to be a direction vector, so translation part of
    // matrix is ignored.
    fun mulDirMatrix(src: MlVector3, dst: MlVector3) {
        val x: Float
        val y: Float
        val z: Float
        val v = FloatArray(3)
        src.getValue(v)

        x = v[0] * mMatrix[0][0] + v[1] * mMatrix[1][0] +
            v[2] * mMatrix[2][0]
        y = v[0] * mMatrix[0][1] + v[1] * mMatrix[1][1] +
            v[2] * mMatrix[2][1]
        z = v[0] * mMatrix[0][2] + v[1] * mMatrix[1][2] +
            v[2] * mMatrix[2][2]

        dst.setValue(x, y, z)
    }

    fun mul(m: MlTransform?): MlTransform {
        return mulRight(m!!)
    }

    fun mul(left: MlTransform, right: MlTransform): MlTransform {
        val m = MlTransform()
        m.setValue(left)
        m.mul(right)

        return m
    }

    // Equality comparison within given tolerance, for each component
    fun equals(trans: MlTransform, tolerance: Float): Boolean {
        var i: Int
        var j: Int
        var d: Float
        i = 0
        while (i < 4) {
            j = 0
            while (j < 3) {
                d = mMatrix[i][j] - trans.mMatrix[i][j]
                if (Math.abs(d) > tolerance) return false
                j++
            }
            i++
        }
        return true
    }

    // Decomposes the matrix into a translation, rotation, scale,
    // and scale orientation.  Any projection information is discarded.
    // The decomposition depends upon choice of center point for
    // rotation and scaling, which is optional as the last parameter.
    // Note that if the center is 0, decompose() is the same as
    // factor() where "t" is translation, "u" is rotation, "s" is scaleFactor,
    // and "r" is ScaleOrientattion.
    fun getTransform(
        translation: MlVector3?,
        rotation: MlRotation,
        scaleFactor: MlVector3?,
        scaleOrientation: MlRotation,
        center: MlVector3
    ) {
        val so = MlTransform()
        val rot = MlTransform()
        val proj = MlTransform()
        if (!center.equals(
                MlVector3(
                    MlScalar.ML_SCALAR_ZERO,
                    MlScalar.ML_SCALAR_ZERO,
                    MlScalar.ML_SCALAR_ZERO
                )
            )
        ) {
            // To get fields for a non-0 center, we
            // need to decompose a new matrix "m" such
            // that [-center][m][center] = [this]
            // i.e., [m] = [center][this][-center]
            val m = MlTransform()
            val c = MlTransform()
            m.setTranslation(center.unaryNegate())
            m.mulLeft(this)
            c.setTranslation(center)
            m.mulLeft(c)
            m.factor(so, scaleFactor!!, rot, translation!!, proj)
        } else factor(so, scaleFactor!!, rot, translation!!, proj)

        // Have to transpose because factor
        // gives us transpose of correct answer.
        scaleOrientation.setValue(so.transpose())
        rotation.setValue(rot)
    }

    fun getTransform(
        t: MlVector3?, r: MlRotation,
        s: MlVector3?, so: MlRotation
    ) {
        getTransform(
            t,
            r,
            s,
            so,
            MlVector3(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO)
        )
    }

    // Sets the given transform to the matrix constructed from the given
    // translation, fixed rotation, and scale vectors
    // Note: Uses scale - rotate - translate order with rotation order Z-Y-X
    fun setTransform(
        translation: MlVector3, rotation: MlVector3,
        scale: MlVector3
    ) {
        val t = FloatArray(3)
        translation.getValue(t)
        val r = FloatArray(3)
        rotation.getValue(r)
        val s = FloatArray(3)
        scale.getValue(s)
        setTransform(t, r, s)
    }

    private fun setTransform(
        translation: FloatArray,
        rotation: FloatArray,
        scale: FloatArray
    ) {
        setScale(scale)
        applyRotation(rotation)
        setTranslationOnly(translation)
    }

    //  Sets the given transform to the matrix constructed from the given
    //  translation, fixed angle rotation, and nonuniformScale*scale vectors
    //  Note: Uses scale - rotate - translate order with rotation order Z-Y-X
    fun setTransform(
        translation: MlVector3,
        rotation: MlVector3, nonuniformScale: MlVector3, scale: Float
    ) {
        val t = FloatArray(3)
        translation.getValue(t)
        val r = FloatArray(3)
        rotation.getValue(r)
        val s = FloatArray(3)
        nonuniformScale.getValue(s)
        val newScale = FloatArray(3)
        for (i in 0..2) newScale[i] = scale * s[i]
        setTransform(t, r, newScale)
    }

    // Composes the matrix from translation, rotation, scale, etc.
    fun setTransform(
        translation: MlVector3, rotation: MlRotation,
        scaleFactor: MlVector3, scaleOrientation: MlRotation, center: MlVector3
    ) {
        //#define TRANSLATE(vec) m.setTranslation(vec), mulLeft(m)
        //#define ROTATE(rot)    rot.getValue(m), mulLeft(m)
        val m = MlTransform()
        makeIdentity()
        if (!translation.equals(
                MlVector3(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO)
            )
        ) {
            m.setTranslation(translation)
            mulLeft(m)
        }
        if (!center.equals(
                MlVector3(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO)
            )
        ) {
            m.setTranslation(center)
            mulLeft(m)
        }
        if (!rotation.equals(
                MlRotation(
                    MlScalar.ML_SCALAR_ZERO,
                    MlScalar.ML_SCALAR_ZERO,
                    MlScalar.ML_SCALAR_ZERO,
                    MlScalar.ML_SCALAR_ONE
                )
            )
        ) {
            rotation.getValue(m)
            mulLeft(m)
        }
        if (!scaleFactor.equals(
                MlVector3(MlScalar.ML_SCALAR_ONE, MlScalar.ML_SCALAR_ONE, MlScalar.ML_SCALAR_ONE)
            )
        ) {
            val so: MlRotation = scaleOrientation
            if (!so.equals(
                    MlRotation(
                        MlScalar.ML_SCALAR_ZERO,
                        MlScalar.ML_SCALAR_ZERO,
                        MlScalar.ML_SCALAR_ZERO,
                        MlScalar.ML_SCALAR_ONE
                    )
                )
            ) {
                so.getValue(m)
                mulLeft(m)
            }
            m.setScale(scaleFactor)
            mulLeft(m)
            if (!so.equals(
                    MlRotation(
                        MlScalar.ML_SCALAR_ZERO,
                        MlScalar.ML_SCALAR_ZERO,
                        MlScalar.ML_SCALAR_ZERO,
                        MlScalar.ML_SCALAR_ONE
                    )
                )
            ) {
                so.invert()
                so.getValue(m)
                mulLeft(m)
            }
        }
        if (!center.equals(
                MlVector3(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO)
            )
        ) {
            m.setTranslation(center.unaryNegate())
            mulLeft(m)
        }
    }

    fun setTransform(t: MlVector3, r: MlRotation, s: MlVector3) {
        setTransform(t, r, s,
            MlRotation(
                MlScalar.ML_SCALAR_ZERO,
                MlScalar.ML_SCALAR_ZERO,
                MlScalar.ML_SCALAR_ZERO,
                MlScalar.ML_SCALAR_ONE
            ),
            MlVector3(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO)
        )
    }

    fun setTransform(t: MlVector3, r: MlRotation, s: MlVector3, so: MlRotation) {
        setTransform(t, r, s, so,
            MlVector3(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO)
        )
    }

    /**
     * Get the nonuniform scales (x, y, z) of the transformation
     * as a vector of 3 components.
     *
     * @param scale The result is returned in this Vector.
     */
    fun getScale(scale: MlVector3) {
        val s = FloatArray(3)

        /*
        for (int i = 0; i < 3; i++)
        {
            s[i] = (m_matrix[i][0] * m_matrix[i][0]) +
                   (m_matrix[i][1] * m_matrix[i][1]) +
                   (m_matrix[i][2] * m_matrix[i][2]);
            s[i] = (float)Math.sqrt(s[i]);
        }
        */
        getScale(s)

        scale.setValue(s)
    }

// Gets the X, Y, and Z nonuniform scales of the transformation
    // Gets the X, Y, and Z nonuniform scales of the transformation
    /**
     * Get the nonuniform scales (x, y, z) of the transformation
     * as a array of 3 components.
     *
     * @param scale The result is returned in this array.
     */
    fun getScale(scale: FloatArray) {
        for (i in 0..2) {
            scale[i] = mMatrix[i][0] * mMatrix[i][0] +
                    mMatrix[i][1] * mMatrix[i][1] +
                    mMatrix[i][2] * mMatrix[i][2]
            scale[i] = Math.sqrt(scale[i].toDouble()).toFloat()
        }
    }

    // Sets matrix to scale by given uniform factor
    fun setScale(scale: Float) {
        mMatrix[0][0] = scale
        mMatrix[0][1] = 0f
        mMatrix[0][2] = 0f
        mMatrix[1][0] = 0f
        mMatrix[1][1] = scale
        mMatrix[1][2] = 0f
        mMatrix[2][0] = 0f
        mMatrix[2][1] = 0f
        mMatrix[2][2] = scale
        mMatrix[3][0] = 0f
        mMatrix[3][1] = 0f
        mMatrix[3][2] = 0f
    }

    // Sets matrix to scale by given vector.
    fun setScale(scale: MlVector3) {
        val v = FloatArray(3)
        scale.getValue(v)
        setScale(v)
    }

    private fun setScale(v: FloatArray) {
        mMatrix[0][0] = v[0]
        mMatrix[0][1] = 0f
        mMatrix[0][2] = 0f
        mMatrix[1][0] = 0f
        mMatrix[1][1] = v[1]
        mMatrix[1][2] = 0f
        mMatrix[2][0] = 0f
        mMatrix[2][1] = 0f
        mMatrix[2][2] = v[2]
        mMatrix[3][0] = 0f
        mMatrix[3][1] = 0f
        mMatrix[3][2] = 0f
    }

    //  Sets the scales of the given transform to the X,Y,Z nonuniform scales
    //  contained in the given new scale vector
    fun setScaleOnly(scale: MlVector3) {
        // Get translation.
        val translation = FloatArray(3)
        getTranslation(translation)

        // Get rotation.
        val rotation = FloatArray(3)
        getRotation(rotation)

        // Set transform.
        val s = FloatArray(3)
        scale.getValue(s)
        setTransform(translation, rotation, s)
    }

    // Gets the the X,Y,Z fixed rotations from the transformation matrix
    // and returns it in the given rotation vector.
    // Note: normalizes angles to positive degrees
    fun getRotation(rotation: MlVector3) {
        val r = FloatArray(3)
        getRotation(r)

        rotation.setValue(r)
    }

    /**
     * Get the rotation as an array of 3 components (theta x, theta y, theta z).
     *
     *
     * This algorithm can result in Gimbal lock. Use quaternion rotations to avoid
     * this.
     *
     *
     * @param r The result is returned in this array.
     */
    fun getRotation(r: FloatArray) {
        val t =
            Array(4) { FloatArray(3) }
        var i: Int
        var j: Int

        // Normalize the rotation matrix portion of the transform.
        i = 0
        while (i < 3) {
            var total = MlScalar.ML_SCALAR_ZERO
            j = 0
            while (j < 3) {
                t[i][j] = mMatrix[i][j]
                total += t[i][j] * t[i][j]
                j++
            }
            if (total != MlScalar.ML_SCALAR_ZERO) {
                total = Math.sqrt(total.toDouble()).toFloat()
                j = 0
                while (j < 3) {
                    t[i][j] = t[i][j] / total
                    j++
                }
            }
            i++
        }

        // Get Y rotation
        r[1] = angleToDegrees(mlAsin(t[2][0]))
        if (Math.abs(t[2][0] - MlScalar.ML_SCALAR_ONE) > 0.001) {
            // Get X and Z rotations
            r[0] = angleToDegrees(
                mlAtan2(
                    -t[2][1],
                    t[2][2]
                )
            )
            r[2] = angleToDegrees(
                mlAtan2(
                    -t[1][0],
                    t[0][0]
                )
            )
        } else {
            // Have Gimbal lock -- lost Z degree of freedom, so
            // express rotation as only a X rotation.
            // This can be avoided by moving to quaternion rotations!
            r[0] = angleToDegrees(
                mlAtan2(
                    t[0][1],
                    t[2][1]
                )
            )
            r[2] = MlScalar.ML_SCALAR_ZERO
        }

        i = 0
        while (i < 3) {
            if (r[i] < MlScalar.ML_SCALAR_ZERO) r[i] += 360f
            i++
        }
    }

    /**
     * Get rotation component of transform.
     *
     * Gets the quaternion rotation contained in the transform matrix.
     *
     * @param rotation The output parameter containing the rotation.
     */
    fun getRotation(rotation: MlRotation) {
        rotation.setValue(this)
    }

    /**
     * Get the rotation as an array of 4 components (angle, theta x, theta y, theta z).
     *
     *
     * This algorithm can result in Gimbal lock. Use quaternion rotations to avoid
     * this.
     *
     *
     * @param r The result is returned in this array.
     */
    fun getAxisAngleRotation(r: FloatArray) {
        val t =
            Array(4) { FloatArray(3) }
        var i: Int
        var j: Int

        // Normalize the rotation matrix portion of the transform.
        i = 0
        while (i < 3) {
            var total = MlScalar.ML_SCALAR_ZERO
            j = 0
            while (j < 3) {
                t[i][j] = mMatrix[i][j]
                total += t[i][j] * t[i][j]
                j++
            }
            if (total != MlScalar.ML_SCALAR_ZERO) {
                total = Math.sqrt(total.toDouble()).toFloat()
                j = 0
                while (j < 3) {
                    t[i][j] = t[i][j] / total
                    j++
                }
            }
            i++
        }
        val m =
            Array(3) { FloatArray(3) }
        val axisAngle: DoubleArray = toAxisAngle(m)
        r[0] = axisAngle[0].toFloat()
        r[1] = axisAngle[1].toFloat()
        r[2] = axisAngle[2].toFloat()
        r[3] = axisAngle[3].toFloat()
    }

    /*
     * This requires a pure rotation matrix 'm' as input.
     */
    private fun toAxisAngle(m: Array<FloatArray>): DoubleArray {
        val angle: Double
        val x: Double
        val y: Double
        val z: Double // variables for result
        val epsilon = 0.01 // margin to allow for rounding errors
        val epsilon2 = 0.1 // margin to distinguish between 0 and 180 degrees
        assert(isRotationMatrix(m)) {
            "not valid rotation matrix" // for debugging
        }
        if (Math.abs(m[0][1] - m[1][0]) < epsilon
            && Math.abs(m[0][2] - m[2][0]) < epsilon
            && Math.abs(m[1][2] - m[2][1]) < epsilon
        ) {
            // Singularity found.

            // First check for identity matrix which must have +1 for all terms
            // in leading diagonal and zero in other terms.
            if (Math.abs(m[0][1] + m[1][0]) < epsilon2
                && Math.abs(m[0][2] + m[2][0]) < epsilon2
                && Math.abs(m[1][2] + m[2][1]) < epsilon2
                && Math.abs(
                    m[0][0] + m[1][1] + m[2][2] - 3
                ) < epsilon2
            ) {
                // This singularity is identity matrix so angle = 0
                val axisAngle =
                    DoubleArray(4) // zero angle, arbitrary axis
                axisAngle[0] = 0.0
                axisAngle[1] = 1.0
                axisAngle[2] = 0.0
                axisAngle[3] = 0.0
                return axisAngle
            }

            // Otherwise this singularity is angle = 180.
            angle = Math.PI
            val xx = (m[0][0] + 1) / 2.toDouble()
            val yy = (m[1][1] + 1) / 2.toDouble()
            val zz = (m[2][2] + 1) / 2.toDouble()
            val xy = (m[0][1] + m[1][0]) / 4.toDouble()
            val xz = (m[0][2] + m[2][0]) / 4.toDouble()
            val yz = (m[1][2] + m[2][1]) / 4.toDouble()
            if (xx > yy && xx > zz) { // m[0][0] is the largest diagonal term
                if (xx < epsilon) {
                    x = 0.0
                    y = 0.7071
                    z = 0.7071
                } else {
                    x = Math.sqrt(xx)
                    y = xy / x
                    z = xz / x
                }
            } else if (yy > zz) { // m[1][1] is the largest diagonal term
                if (yy < epsilon) {
                    x = 0.7071
                    y = 0.0
                    z = 0.7071
                } else {
                    y = Math.sqrt(yy)
                    x = xy / y
                    z = yz / y
                }
            } else { // m[2][2] is the largest diagonal term so base result on this
                if (zz < epsilon) {
                    x = 0.7071
                    y = 0.7071
                    z = 0.0
                } else {
                    z = Math.sqrt(zz)
                    x = xz / z
                    y = yz / z
                }
            }
            val axisAngle = DoubleArray(4) // return 180 deg rotation
            axisAngle[0] = angle
            axisAngle[1] = x
            axisAngle[2] = y
            axisAngle[3] = z
            return axisAngle
        }
        // As we have reached here there are no singularities so we can handle normally.
        var s = Math.sqrt(
            (m[2][1] - m[1][2]) * (m[2][1] - m[1][2]) + (m[0][2] - m[2][0]) * (m[0][2] - m[2][0]) + ((m[1][0] - m[0][1]) * (m[1][0] - m[0][1])).toDouble()
        ) // used to normalise
        if (Math.abs(s) < 0.001) s = 1.0
        // Prevent divide by zero, should not happen if matrix is orthogonal and should be
        // caught by singularity test above, but I've left it in just in case
        angle = Math.acos(
            (m[0][0] + m[1][1] + m[2][2] - 1) / 2.toDouble()
        )
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
        val axisAngle = DoubleArray(4)
        axisAngle[0] = angle
        axisAngle[1] = x
        axisAngle[2] = y
        axisAngle[3] = z

        return axisAngle
    }

    /*
     * This checks that the input is a pure rotation matrix 'm'.
     * The condition for this is:
     * R' * R = I
     * and
     * det(R) =1
     */
    private fun isRotationMatrix(m: Array<FloatArray>): Boolean {
        val epsilon = 0.01 // margin to allow for rounding errors
        if (Math.abs(
                m[0][0] * m[0][1] + m[0][1] * m[1][1] + m[0][2] * m[1][2]
            ) > epsilon
        ) return false
        if (Math.abs(
                m[0][0] * m[2][0] + m[0][1] * m[2][1] + m[0][2] * m[2][2]
            ) > epsilon
        ) return false
        if (Math.abs(
                m[1][0] * m[2][0] + m[1][1] * m[2][1] + m[1][2] * m[2][2]
            ) > epsilon
        ) return false
        if (Math.abs(
                m[0][0] * m[0][0] + m[0][1] * m[0][1] + m[0][2] * m[0][2] - 1
            ) > epsilon
        ) return false
        if (Math.abs(
                m[1][0] * m[1][0] + m[1][1] * m[1][1] + m[1][2] * m[1][2] - 1
            ) > epsilon
        ) return false
        return if (Math.abs(
                m[2][0] * m[2][0] + m[2][1] * m[2][1] + m[2][2] * m[2][2] - 1
            ) > epsilon
        ) false else Math.abs(det(m) - 1) < epsilon
        // det is defined here:
        // http://www.euclideanspace.com/maths/algebra/matrix/functions/determinant/threeD/
    }

    // Assumes matrix indices start from 0 (0, 1 and 2)
    // The value of the determinant for a 3×3 matrix is:
    //    det = m11 m22 m33 + m12 m23 m31 + m13 m21 m32 - m11 m23 m32 - m12 m 21 m33 - m13 m22 m31
    fun det(m: Array<FloatArray>): Double {
        return m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1] - m[0][0] * m[1][2] * m[2][1] - m[0][1] * m[1][0] * m[2][2] - (m[0][2] * m[1][1] * m[2][0]).toDouble()
    }

    /**
     * Set the rotation component of the transform.
     *
     * Sets matrix to the rotation matrix given by the new rotation.
     *
     * @param newRotation The rotation to set.
     */
    fun setRotation(newRotation: MlRotation?) {
        if (newRotation != null) {
            newRotation.getValue(this)
        }
    }

    /**
     * Set the rotation component of the transform.
     *
     * Sets only the rotation matrix of the transform to the rotation specified
     * by the given rotation quaternion, without otherwise disturbing the rest
     * of the transformation matrix.
     *
     * @param rotation The rotation matrix to set.
     */
    fun setRotationOnly(rotation: MlRotation?) {
        if (rotation != null) {
            val translation = MlVector3()
            val scale = MlVector3()
            getTranslation(translation)
            getScale(scale)
            setTransform(translation, rotation, scale)
        }
    }

    /**
     * Set the rotation component of the transform.
     *
     * Sets only the rotation matrix of the transform to the X,Y,Z fixed angle
     * rotations specified by the given rotation vector, without otherwise
     * disturbing the rest of the transformation matrix.
     *
     * @param rotation The rotation vector to set.
     */
    fun setRotationOnly(rotation: MlVector3?) {
        if (rotation != null) {
            val translation = MlVector3()
            val scale = MlVector3()
            getTranslation(translation)
            getScale(scale)
            setTransform(translation, rotation, scale)
        }
    }

    // Applies the fixed angle rotation to the matrix
    fun applyRotation(rotation: MlVector3) {
        val r = FloatArray(3)
        rotation.getValue(r)

        applyRotation(r)
    }

    fun applyRotation(r: FloatArray) {
        val mat = MlTransform()
        mat.makeIdentity()

        // Apply Z Rotation
        if (r[2] != MlScalar.ML_SCALAR_ZERO) {
            val angle = degreesToAngle(r[2])
            val sz = mlSin(angle)
            val cz = mlCos(angle)
            mat.mMatrix[0][0] = cz
            mat.mMatrix[0][1] = sz
            mat.mMatrix[1][0] = -sz
            mat.mMatrix[1][1] = cz
            mat.mMatrix[2][2] = MlScalar.ML_SCALAR_ONE
            mat.mMatrix[3][2] = MlScalar.ML_SCALAR_ZERO
            mat.mMatrix[3][1] = mat.mMatrix[3][2]
            mat.mMatrix[3][0] = mat.mMatrix[3][1]
            mat.mMatrix[2][1] = mat.mMatrix[3][0]
            mat.mMatrix[2][0] = mat.mMatrix[2][1]
            mat.mMatrix[1][2] = mat.mMatrix[2][0]
            mat.mMatrix[0][2] = mat.mMatrix[1][2]
            mulRight(mat)
        }

        // Apply Y Rotation
        if (r[1] != MlScalar.ML_SCALAR_ZERO) {
            val angle = degreesToAngle(r[1])
            val sy = mlSin(angle)
            val cy = mlCos(angle)
            mat.mMatrix[0][0] = cy
            mat.mMatrix[0][2] = -sy
            mat.mMatrix[1][1] = MlScalar.ML_SCALAR_ONE
            mat.mMatrix[2][0] = sy
            mat.mMatrix[2][2] = cy
            mat.mMatrix[3][2] = MlScalar.ML_SCALAR_ZERO
            mat.mMatrix[3][1] = mat.mMatrix[3][2]
            mat.mMatrix[3][0] = mat.mMatrix[3][1]
            mat.mMatrix[2][1] = mat.mMatrix[3][0]
            mat.mMatrix[1][2] = mat.mMatrix[2][1]
            mat.mMatrix[1][0] = mat.mMatrix[1][2]
            mat.mMatrix[0][1] = mat.mMatrix[1][0]
            mulRight(mat)
        }

        // Apply X Rotation
        if (r[0] != MlScalar.ML_SCALAR_ZERO) {
            val angle = degreesToAngle(r[0])
            val sx = mlSin(angle)
            val cx = mlCos(angle)
            mat.mMatrix[0][0] = MlScalar.ML_SCALAR_ONE
            mat.mMatrix[1][1] = cx
            mat.mMatrix[1][2] = sx
            mat.mMatrix[2][1] = -sx
            mat.mMatrix[2][2] = cx
            mat.mMatrix[3][2] = MlScalar.ML_SCALAR_ZERO
            mat.mMatrix[3][1] = mat.mMatrix[3][2]
            mat.mMatrix[3][0] = mat.mMatrix[3][1]
            mat.mMatrix[2][0] = mat.mMatrix[3][0]
            mat.mMatrix[1][0] = mat.mMatrix[2][0]
            mat.mMatrix[0][2] = mat.mMatrix[1][0]
            mat.mMatrix[0][1] = mat.mMatrix[0][2]
            mulRight(mat)
        }
    }

    // Sets the given translation vector to the X,Y,Z translations contained
    // in the given transformation matrix
    fun getTranslation(translation: MlVector3) {
        val v = FloatArray(3)
        getTranslation(v)
        translation.setValue(v)
    }

    fun getTranslation(v: FloatArray) {
        for (i in 0..2) v[i] = mMatrix[3][i]
    }

    // Sets matrix to the translation matrix given by the vector
    fun setTranslation(translation: MlVector3) {
        val t = FloatArray(3)
        translation.getValue(t)
        mMatrix[0][0] = 1f
        mMatrix[0][1] = 0f
        mMatrix[0][2] = 0f
        mMatrix[1][0] = 0f
        mMatrix[1][1] = 1f
        mMatrix[1][2] = 0f
        mMatrix[2][0] = 0f
        mMatrix[2][1] = 0f
        mMatrix[2][2] = 1f
        mMatrix[3][0] = t[0]
        mMatrix[3][1] = t[1]
        mMatrix[3][2] = t[2]
    }

    // Sets the translation of the given transform to the X,Y,Z translations
    // contained in the given new translation vector.
    fun setTranslationOnly(translation: MlVector3) {
        val t = FloatArray(3)
        translation.getValue(t)
        setTranslationOnly(t)
    }

    private fun setTranslationOnly(t: FloatArray) {
        for (i in 0..2) mMatrix[3][i] = t[i]
    }

    // Adds the given X,Y,Z translations to the existing translation
    // contained in the transform
    fun applyTranslation(translation: MlVector3) {
        val t = FloatArray(3)
        translation.getValue(t)
        for (i in 0..2) mMatrix[3][i] = mMatrix[3][i] + t[i]
    }
}

