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
import com.wizzer.mle.math.MlAngle.Companion.angleToRadians
import com.wizzer.mle.math.MlAngle.Companion.radiansToAngle
import com.wizzer.mle.math.MlMath.Companion.mlAcos
import com.wizzer.mle.math.MlMath.Companion.mlCos
import com.wizzer.mle.math.MlMath.Companion.mlSin

/**
* This class specifies a rotation, used to define the orientation of a 3d object.
*/
class MlRotation
{
    // The internal quaternion representation.
    var mQuat = FloatArray(4)

    /**
     * The default constructor. The quaternion is created as an
     * identity or null rotation.
     */
    constructor() {
        setValue(0f, 0f, 0f, 1f)
    }

    /**
     * A constructor given 4 individual components of a quaternion.
     *
     * @param q1 The first component.
     * @param q2 The second component.
     * @param q3 The third component.
     * @param q4 The fourth component.
     */
    constructor(
        q1: Float,
        q2: Float,
        q3: Float,
        q4: Float
    ) {
        setValue(q1, q2, q3, q4)
    }

    /**
     * A constructor given a quaternion as an array of 4 components.
     *
     * @param v The array of components.
     */
    constructor(v: FloatArray) {
        setValue(v)
    }

    /**
     * A constructor given a rotation matrix.
     *
     * @param q The rotation matrix.
     */
    constructor(q: MlTransform) {
        setValue(q)
    }

    /**
     * A constructor given 3D rotation axis vector and angle in radians.
     *
     * @param axis A reference to the 3D rotation axis.
     * @param radians The angle in radians.
     */
     constructor(axis: MlVector3, radians: Float) {
        setValue(axis, radians)
    }

    companion object {
        /**
         * Create a null rotation.
         *
         * @return An identity quaternion is returned.
         */
        @JvmStatic
        fun identity(): MlRotation {
            return MlRotation(
                MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO,
                MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ONE
            )
        }
    }

    /**
     * Returns 4 individual components of a rotation quaternion.
     *
     * @param v The array the components will be returned in.
     */
    fun getValue(v: FloatArray) {
        v[0] = mQuat[0]
        v[1] = mQuat[1]
        v[2] = mQuat[2]
        v[3] = mQuat[3]
    }

    /**
     * Returns corresponding 3D rotation axis vector and angle in radians.
     *
     * @param axis The rotation axis vector.
     * @param radians The angle, in radians.
     */
    fun getValue(axis: MlVector3, radians: FloatArray) {
        var axis = axis
        var len: Float

        val q = MlVector3()
        q.mVector[0] = mQuat[0]
        q.mVector[1] = mQuat[1]
        q.mVector[2] = mQuat[2]

        if (q.length().also { len = it } > 0.00001f) {
            axis = q.mul(1 / len)
            radians[0] =
                angleToRadians(2 * mlAcos(mQuat[3]))
        } else {
            axis.setValue(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ONE)
            radians[0] = MlScalar.ML_SCALAR_ZERO
        }
    }

    /**
     * Returns corresponding 4x3 rotation matrix.
     *
     * @param matrix The rotation matrix that is returned
     */
    fun getValue(matrix: MlTransform) {
        val m = MlTransform()
        m.mMatrix[0][0] =
            MlScalar.ML_SCALAR_ONE - 2 * (mQuat[1] * mQuat[1] + mQuat[2] * mQuat[2])
        m.mMatrix[0][1] =
            2 * (mQuat[0] * mQuat[1] + mQuat[2] * mQuat[3])
        m.mMatrix[0][2] =
            2 * (mQuat[2] * mQuat[0] - mQuat[1] * mQuat[3])
        m.mMatrix[1][0] =
            2 * (mQuat[0] * mQuat[1] - mQuat[2] * mQuat[3])
        m.mMatrix[1][1] =
            MlScalar.ML_SCALAR_ONE - 2 * (mQuat[2] * mQuat[2] + mQuat[0] * mQuat[0])
        m.mMatrix[1][2] =
            2 * (mQuat[1] * mQuat[2] + mQuat[0] * mQuat[3])
        m.mMatrix[2][0] =
            2 * (mQuat[2] * mQuat[0] + mQuat[1] * mQuat[3])
        m.mMatrix[2][1] =
            2 * (mQuat[1] * mQuat[2] - mQuat[0] * mQuat[3])
        m.mMatrix[2][2] =
            MlScalar.ML_SCALAR_ONE - 2 * (mQuat[1] * mQuat[1] + mQuat[0] * mQuat[0])
        m.mMatrix[3][0] = MlScalar.ML_SCALAR_ZERO
        m.mMatrix[3][1] = MlScalar.ML_SCALAR_ZERO
        m.mMatrix[3][2] = MlScalar.ML_SCALAR_ZERO

        //matrix = m;
        for (i in 0..3)
            for (j in 0..2)
                matrix.mMatrix[i][j] = m.mMatrix[i][j]
    }

    /**
     * Sets the value of the rotation from 4 individual components of a
     * quaternion.
     *
     * @param q1 The first component.
     * @param q2 The second component.
     * @param q3 The third component.
     * @param q4 The fourth component.
     *
     * @return **this** is returned.
     */
    fun setValue(
        q1: Float,
        q2: Float,
        q3: Float,
        q4: Float
    ): MlRotation {
        mQuat[0] = q1
        mQuat[1] = q2
        mQuat[2] = q3
        mQuat[3] = q4

        normalize()

        return this
    }

    /**
     * Sets the value of the rotation from array of 4 components of a
     * quaternion.
     *
     * @param q The array of components to set.
     *
     * @return **this** is returned.
     */
    fun setValue(q: FloatArray): MlRotation {
        mQuat[0] = q[0]
        mQuat[1] = q[1]
        mQuat[2] = q[2]
        mQuat[3] = q[3]

        normalize()

        return this
    }

    /**
     * Sets the value of the rotation from a transform matrix.
     *
     * @param m The transform matrix to set,
     *
     * @return **this** is returned.
     */
    fun setValue(m: MlTransform): MlRotation {
        val i: Int
        val j: Int
        val k: Int

        // First, find largest diagonal in matrix:
        if (m.isZero()) {
            mQuat[3] = MlScalar.ML_SCALAR_ONE
            mQuat[0] = MlScalar.ML_SCALAR_ZERO
            mQuat[1] = MlScalar.ML_SCALAR_ZERO
            mQuat[2] = MlScalar.ML_SCALAR_ZERO

            return this
        }

        i = if (m.mMatrix[0][0] > m.mMatrix[1][1]) {
            if (m.mMatrix[0][0] > m.mMatrix[2][2]) 0 else 2
        } else {
            if (m.mMatrix[1][1] > m.mMatrix[2][2]) 1 else 2
        }

        if (m.mMatrix[0][0] + m.mMatrix[1][1] + m.mMatrix[2][2] > m.mMatrix[i][i]
        ) {
            // Compute w first:
            mQuat[3] = Math.sqrt(
                (m.mMatrix[0][0] + m.mMatrix[1][1] +
                        m.mMatrix[2][2] + MlScalar.ML_SCALAR_ONE).toDouble()
            ).toFloat() * MlScalar.ML_SCALAR_HALF

            // And compute other values:
            mQuat[0] =
                (m.mMatrix[1][2] - m.mMatrix[2][1]) / (4 * mQuat[3])
            mQuat[1] =
                (m.mMatrix[2][0] - m.mMatrix[0][2]) / (4 * mQuat[3])
            mQuat[2] =
                (m.mMatrix[0][1] - m.mMatrix[1][0]) / (4 * mQuat[3])
        } else {
            // Compute x, y, or z first:
            j = (i + 1) % 3
            k = (i + 2) % 3

            // Compute first value:
            mQuat[i] = Math.sqrt(
                m.mMatrix[i][i] - m.mMatrix[j][j] -
                        m.mMatrix[k][k] + MlScalar.ML_SCALAR_ONE.toDouble()
            ).toFloat() * MlScalar.ML_SCALAR_HALF

            // And the others:
            mQuat[j] =
                (m.mMatrix[i][j] + m.mMatrix[j][i]) / (4 * mQuat[i])
            mQuat[k] =
                (m.mMatrix[i][k] + m.mMatrix[k][i]) / (4 * mQuat[i])
            mQuat[3] =
                (m.mMatrix[j][k] - m.mMatrix[k][j]) / (4 * mQuat[i])
        }

        return this
    }

    /**
     * Sets the value of the rotation from 3D rotation axis vector and angle in radians.
     *
     * @param axis The rotation axis vector.
     * @param radians The angle, in radians.
     *
     * @return **this** is returned.
     */
    fun setValue(axis: MlVector3?, radians: Float): MlRotation {
        val q = MlVector3()

        q.setValue(axis!!)
        q.normalize()

        q.mul(mlSin(radiansToAngle(radians * MlScalar.ML_SCALAR_HALF)))

        mQuat[0] = q.mVector[0]
        mQuat[1] = q.mVector[1]
        mQuat[2] = q.mVector[2]

        mQuat[3] =
            mlCos(radiansToAngle(radians * MlScalar.ML_SCALAR_HALF))

        return this
    }

    /**
     * Sets rotation to rotate from one direction vector to another.
     *
     * @param rotateFrom The vector to rotate from.
     * @param rotateTo The vector to rotate to.
     *
     * @return **this** is returned.
     */
    fun setValue(rotateFrom: MlVector3, rotateTo: MlVector3?): MlRotation {
        val from = MlVector3(rotateFrom)
        val to = MlVector3(rotateTo!!)
        var axis = MlVector3()
        val cost: Float
        from.normalize()
        to.normalize()
        cost = from.dot(to)

        // Check for degeneracies.
        if (cost > 0.99999f) {
            // Vectors are parallel.
            mQuat[2] = MlScalar.ML_SCALAR_ZERO
            mQuat[1] = mQuat[2]
            mQuat[0] = mQuat[1]
            mQuat[3] = MlScalar.ML_SCALAR_ONE
            return this
        } else if (cost < -0.99999f) {
            // Vectors are opposite.

            // Find an axis to rotate around, which should be
            // perpendicular to the original axis.
            //
            // Try cross product with (1,0,0) first, if that's one of our
            // original vectors then try  (0,1,0).
            var tmp = from.cross(
                MlVector3(
                    MlScalar.ML_SCALAR_ONE,
                    MlScalar.ML_SCALAR_ZERO,
                    MlScalar.ML_SCALAR_ZERO
                )
            )
            if (tmp.length() < 0.00001f) tmp = from.cross(
                MlVector3(
                    MlScalar.ML_SCALAR_ZERO,
                    MlScalar.ML_SCALAR_ONE,
                    MlScalar.ML_SCALAR_ZERO
                )
            )
            tmp.normalize()
            setValue(
                tmp.mVector[0],
                tmp.mVector[1],
                tmp.mVector[2],
                MlScalar.ML_SCALAR_ZERO
            )
            return this
        }
        axis = rotateFrom.cross(rotateTo!!)
        axis.normalize()

        // Use half-angle formulae.
        // sin^2 t = ( 1 - cos (2t) ) / 2
        axis.mul(
            Math.sqrt((MlScalar.ML_SCALAR_HALF * (MlScalar.ML_SCALAR_ONE - cost)).toDouble())
                .toFloat()
        )

        // Scale the axis by the sine of half the rotation angle to get
        // the normalized quaternion.
        mQuat[0] = axis.mVector[0]
        mQuat[1] = axis.mVector[1]
        mQuat[2] = axis.mVector[2]

        // cos^2 t = ( 1 + cos (2t) ) / 2
        // w part is cosine of half the rotation angle.
        mQuat[3] =
            Math.sqrt((MlScalar.ML_SCALAR_HALF * (MlScalar.ML_SCALAR_ONE + cost)).toDouble())
                .toFloat()

        return this
    }

    /*
         * Calculate the norm of the rotation vector.
         *
         * @return Returns the norm (square of the 4D length) of the quaternion
         * defining the rotation.
         */
    private fun norm(): Float {
        return mQuat[0] * mQuat[0] +
               mQuat[1] * mQuat[1] +
               mQuat[2] * mQuat[2] +
               mQuat[3] * mQuat[3]
    }

    /*
         * Normalizes a rotation quaternion to unit 4D length.
         */
    private fun normalize() {
        val dist = 1 / Math.sqrt(norm().toDouble()).toFloat()

        mQuat[0] = mQuat[0] * dist
        mQuat[1] = mQuat[1] * dist
        mQuat[2] = mQuat[2] * dist
        mQuat[3] = mQuat[3] * dist
    }

    /**
     * Changes a rotation to be its inverse.
     *
     * @return **this** is returned.
     */
    fun invert(): MlRotation {
        val invNorm = 1 / norm()

        mQuat[0] = -(mQuat[0] * invNorm)
        mQuat[1] = -(mQuat[1] * invNorm)
        mQuat[2] = -(mQuat[2] * invNorm)
        mQuat[3] = mQuat[3] * invNorm

        return this
    }

    /**
     * Calculates the inverse of a rotation.
     *
     * @return **this** is returned.
     */
    fun inverse(): MlRotation {
        val q = this
        return q.invert()
    }

    /**
     * Multiplies this rotation by another rotation;
     * results in product of rotations.
     *
     * @param q The rotation to multiply with.
     *
     * @return **this** is returned.
     */
    fun mul(q: MlRotation): MlRotation {
        val p0: Float
        val p1: Float
        val p2: Float
        val p3: Float

        p0 = q.mQuat[3] * mQuat[0] + q.mQuat[0] * mQuat[3] +
             q.mQuat[1] * mQuat[2] - q.mQuat[2] * mQuat[1]
        p1 = q.mQuat[3] * mQuat[1] + q.mQuat[1] * mQuat[3] +
             q.mQuat[2] * mQuat[0] - q.mQuat[0] * mQuat[2]
        p2 = q.mQuat[3] * mQuat[2] + q.mQuat[2] * mQuat[3] +
             q.mQuat[0] * mQuat[1] - q.mQuat[1] * mQuat[0]
        p3 = q.mQuat[3] * mQuat[3] - q.mQuat[0] * mQuat[0] -
             q.mQuat[1] * mQuat[1] - q.mQuat[2] * mQuat[2]
        mQuat[0] = p0
        mQuat[1] = p1
        mQuat[2] = p2
        mQuat[3] = p3

        normalize()

        return this
    }

    /**
     * The binary multiplication of two rotations.
     *
     * @param q1 The first rotation.
     * @param q2 The second rotation.
     *
     * @return A new rotation is created and returned.
     */
    fun mul(q1: MlRotation, q2: MlRotation): MlRotation {
        val q = MlRotation(
            q2.mQuat[3] * q1.mQuat[0] + q2.mQuat[0] * q1.mQuat[3] +
                q2.mQuat[1] * q1.mQuat[2] - q2.mQuat[2] * q1.mQuat[1],
            q2.mQuat[3] * q1.mQuat[1] + q2.mQuat[1] * q1.mQuat[3] +
                q2.mQuat[2] * q1.mQuat[0] - q2.mQuat[0] * q1.mQuat[2],
            q2.mQuat[3] * q1.mQuat[2] + q2.mQuat[2] * q1.mQuat[3] +
                q2.mQuat[0] * q1.mQuat[1] - q2.mQuat[1] * q1.mQuat[0],
            q2.mQuat[3] * q1.mQuat[3] - q2.mQuat[0] * q1.mQuat[0] -
                q2.mQuat[1] * q1.mQuat[1] - q2.mQuat[2] * q1.mQuat[2]
        )

        q.normalize()

        return q
    }

    /**
     * Puts the given vector through this rotation.
     * Multiplies the given vector by the matrix of this rotation.
     *
     * @param src The source vector.
     * @param dst The result of the rotation.
     */
    fun multVec(src: MlVector3?, dst: MlVector3?) {
        val myMat = MlTransform()
        getValue(myMat)

        myMat.mulVecMatrix(src!!, dst!!)
    }

    /**
     * Scale the angle of rotation.
     *
     *
     * Keep the axis the same. Multiply the angle of rotation by
     * the amount **scaleFactor**.
     *
     *
     * @param scaleFactor The amount to scale by.
     */
    fun scaleAngle(scaleFactor: Float) {
        val myAxis = MlVector3()
        val myAngle = FloatArray(1)

        // Get the Axis and angle.
        getValue(myAxis, myAngle)

        setValue(myAxis, myAngle[0] * scaleFactor)
    }

    /**
     * Spherical linear interpolation: as **t** goes from 0 to 1, returned
     * value goes from **rot0** to **rot1**.
     *
     * @param rot0 The first rotation.
     * @param rot1 The second rotation.
     * @param t The weight.
     *
     * @return A new rotation is returned.
     */
    fun slerp(rot0: MlRotation, rot1: MlRotation, t: Float): MlRotation {
        val r1q = FloatArray(4)
        rot1.getValue(r1q)
        val rot = MlRotation()
        val rot1q = FloatArray(4)
        val omega: Float
        var cosom: Float
        val sinom: Float
        val scalerot0: Float
        val scalerot1: Float
        var i: Int

        // Calculate the cosine.
        cosom =
            (rot0.mQuat[0] * rot1.mQuat[0] + rot0.mQuat[1] * rot1.mQuat[1]
                    + rot0.mQuat[2] * rot1.mQuat[2] + rot0.mQuat[3] * rot1.mQuat[3])

        // Adjust signs if necessary.
        if (cosom < MlScalar.ML_SCALAR_ZERO) {
            cosom = -cosom
            for (j in 0..3) rot1q[j] = -r1q[j]
        } else {
            for (j in 0..3) rot1q[j] = r1q[j]
        }

        // Calculate interpolating coeffs.
        if (MlScalar.ML_SCALAR_ONE - cosom > 0.00001f) {
            // standard case.
            omega = mlAcos(cosom)
            sinom = mlSin(omega)
            scalerot0 =
                (mlSin((MlScalar.ML_SCALAR_ONE - t) * omega) / sinom)
            scalerot1 = (mlSin(t * omega) / sinom)
        } else {
            // rot0 and rot1 very close - just do linear interp.
            scalerot0 = MlScalar.ML_SCALAR_ONE - t
            scalerot1 = t
        }

        // Build the new quarternion.
        i = 0
        while (i < 4) {
            rot.mQuat[i] = scalerot0 * rot0.mQuat[i] + scalerot1 * rot1q[i]
            i++
        }
        return rot
    }

    /**
     * Equality comparison.
     *
     * @param q The rotation to test against.
     *
     * @return **true** is returned if the rotations are equal.
     * Otherwise, **false** is returned.
     */
    fun equals(q: MlRotation): Boolean {
        return mQuat[0] === q.mQuat[0] && mQuat[1] === q.mQuat[1] && mQuat[2] === q.mQuat[2] && mQuat[3] === q.mQuat[3]
    }

    /**
     * Equality comparison operator within given tolerance - the square
     * of the length of the maximum distance between the two vectors.
     *
     * @param q The rotation to test against.
     * @param tolerance The tolerance.
     *
     * @return **true** is returned if the rotations are equal within.
     * the specified tolerance. Otherwise, **false** is returned.
     */
    fun equals(q: MlRotation, tolerance: Float): Boolean {
        return MlVector4(mQuat).equals(MlVector4(q.mQuat), tolerance)
    }
}