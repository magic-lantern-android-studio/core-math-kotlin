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

/**
 * 3D vector used to represent points or directions.  Each component of
 * the vector is a single-precision floating-point value.
 */
class MlVector3
{
    // The internal vector representation.
    var mVector = FloatArray(3)

    /**
     * The default constructor. All elements in the vector are
     * initialized to zero (0.0f).
     */
    constructor() {
        mVector[0] = 0.0f
        mVector[1] = 0.0f
        mVector[2] = 0.0f
    }

    /**
     * A constructor given an array of 3 components
     * to initialize from.
     *
     * @param v The array of components to initialize.
     */
    constructor(v: FloatArray) {
        mVector[0] = v[0]
        mVector[1] = v[1]
        mVector[2] = v[2]
    }

    /**
     * A constructor given 3 individual components
     * to initialize from.
     *
     * @param x The x element.
     * @param y The y element.
     * @param z The z element.
     */
    constructor(x: Float, y: Float, z: Float) {
        mVector[0] = x
        mVector[1] = y
        mVector[2] = z
    }

    /** A constructor given another vector to
     * initialize from.
     *
     * @param v The other vector.
     */
    constructor(v: MlVector3) {
        mVector[0] = v.mVector[0]
        mVector[1] = v.mVector[1]
        mVector[2] = v.mVector[2]
    }

    /**
     * Determine if the vector is set to [0 0 0].
     *
     * @return Returns <b>true</b> if all elements are zero.
     * Otherwise, returns <b>false</b>.
     */
    fun isZero(): Boolean {
        return ((mVector[0] == 0.0f) &&
                (mVector[1] == 0.0f) &&
                (mVector[2] == 0.0f))
    }

    /**
     * Calculate the cross-product of this vector and the passed
     * argument <b>v</b>.
     *
     * @param v The other vector.
     *
     * @return Returns the right-handed cross product of this vector
     * and another vector, v.
     */
    fun cross(v: MlVector3): MlVector3 {
        var result = MlVector3(
            (mVector[1] * v.mVector[2]) - (mVector[2] * v.mVector[1]),
            (mVector[2] * v.mVector[0]) - (mVector[0] * v.mVector[2]),
            (mVector[0] * v.mVector[1]) - (mVector[1] * v.mVector[0])
        );
        return result;
    }

    /**
     * Calculate the dot-product of this vector and the passed
     * argument **v**.
     *
     * @param v The other vector.
     *
     * @return Returns the dot (inner) product of this vector
     * and another vector, v.
     */
    fun dot(v: MlVector3): Float {
        return mVector[0] * v.mVector[0] +
                mVector[1] * v.mVector[1] +
                mVector[2] * v.mVector[2]
    }

    /**
     * Get the value of the vector.
     *
     * @param v Returns an array of 3 components.
     */
    fun getValue(v: FloatArray) {
        v[0] = mVector[0]
        v[1] = mVector[1]
        v[2] = mVector[2]
    }

    /**
     * Set the value of the vector.
     *
     * @param v An array of 3 components.
     *
     * @return **this** is returned.
     */
    fun setValue(v: FloatArray): MlVector3 {
        mVector[0] = v[0]
        mVector[1] = v[1]
        mVector[2] = v[2]
        return this
    }

    /**
     * Set the value of the vector.
     *
     * @param v The vector to copy from.
     *
     * @return **this** is returned.
     */
    fun setValue(v: MlVector3): MlVector3 {
        mVector[0] = v.mVector[0]
        mVector[1] = v.mVector[1]
        mVector[2] = v.mVector[2]
        return this
    }

    /**
     * Sets value of vector from 3 individual components.
     *
     * @param x The x element to set.
     * @param y The y element to set.
     * @param z The z element to set.
     *
     * @return **this** is returned.
     */
    fun setValue(x: Float, y: Float, z: Float): MlVector3 {
        mVector[0] = x
        mVector[1] = y
        mVector[2] = z
        return this
    }

    /**
     * Sets value of vector to be convex combination of 3 other
     * vectors, using barycentic coordinates.
     *
     * @param barycentic Barycentic coordinate vector.
     * @param v0 The first vector.
     * @param v1 The second vector.
     * @param v2 The third vector.
     *
     * @return **this** is returned.
     */
    fun setValue(
        barycentic: MlVector3,
        v0: MlVector3, v1: MlVector3, v2: MlVector3
    ): MlVector3 {
        mVector[0] = v0.mVector[0] * barycentic.mVector[0] +
                v1.mVector[0] * barycentic.mVector[1] +
                v2.mVector[0] * barycentic.mVector[2]
        mVector[1] = v0.mVector[1] * barycentic.mVector[0] +
                v1.mVector[1] * barycentic.mVector[1] +
                v2.mVector[1] * barycentic.mVector[2]
        mVector[2] = v0.mVector[2] * barycentic.mVector[0] +
                v1.mVector[2] * barycentic.mVector[1] +
                v2.mVector[2] * barycentic.mVector[2]
        return this
    }

    /**
     * Get the length of the vector.
     *
     * @return Returns geometric length of vector.
     */
    fun length(): Float {
        return Math.sqrt(
            (mVector[0] * mVector[0] +
                    mVector[1] * mVector[1] +
                    mVector[2] * mVector[2]).toDouble()
        ).toFloat()
    }

    /**
     * Returns approximate length of vector (+/- 7%).
     *
     * Fast approximation to the length of a vector.  3-D Euclidean distance
     * approximation: c1 = 15/16 , c2 = c3 = 3/8 and max(e) = 7.7 %.  Based on
     * *"Fast Linear Approximations of Euclidean Distance in Higher Dimensions"*
     * by Yoshikazu Ohashi, yoshi@cognex.com in "Graphics Gems IV", Academic
     * Press, 1994.
     *
     * @return The length is returned.
     */
    fun approximateLength(): Float {
        var a = mVector[0]
        var b = mVector[1]
        var c = mVector[2]

        // Set a, b, and c to absolute values of x, y, and z coordinates.
        a = Math.abs(a)
        b = Math.abs(b)
        c = Math.abs(c)

        // Test and swap so that a is the largest coordinate.
        if (a < b) {
            val x = a
            a = b
            b = x
        }
        if (a < c) {
            val x = a
            a = c
            c = x
        }
        return a * 0.9375f + (b + c) * 0.375f
    }

    /**
     * Normalize the vector.
     *
     * Changes vector to be unit length.
     *
     * @return The original length is returned.
     */
    fun normalize(): Float {
        if (isZero()) {
            return MlScalar.ML_SCALAR_ZERO
        }
        val len = length()
        if (len != MlScalar.ML_SCALAR_ZERO) {
            mVector[0] = mVector[0] * (1 / len)
            mVector[1] = mVector[1] * (1 / len)
            mVector[2] = mVector[2] * (1 / len)
        } else setValue(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO)
        return len
    }

    /**
     * Normalize the vector.
     *
     * Changes vector to be approximately unit length.
     *
     * @return The original length is returned.
     */
    fun approximateNormalize(): Float {
        val length = approximateLength()
        if (length != MlScalar.ML_SCALAR_ZERO) {
            mVector[0] = mVector[0] / length
            mVector[1] = mVector[1] / length
            mVector[2] = mVector[2] / length
        }
        return length
    }

    /**
     * Negates each component of vector in place.
     */
    fun negate() {
        mVector[0] = -mVector[0]
        mVector[1] = -mVector[1]
        mVector[2] = -mVector[2]
    }

    /**
     * Nondestructive unary negation.
     *
     * @return A new vector is returned.
     */
    fun unaryNegate(): MlVector3 {
        return MlVector3(-mVector[0], -mVector[1], -mVector[2])
    }

    /**
     * Changes vector to have length of input scale factor.
     *
     * @param newScale The new scale to use.
     *
     * @return **this** is returned.
     */
    fun scaleTo(newScale: Float): MlVector3 {
        var oldScale = length()
        if (oldScale != MlScalar.ML_SCALAR_ZERO) {
            oldScale = newScale / oldScale
            mVector[0] = mVector[0] * oldScale
            mVector[1] = mVector[1] * oldScale
            mVector[2] = mVector[2] * oldScale
        }
        return this
    }

    // Calculate the scale.
    companion object {
        @JvmStatic
        private fun calcScale(
            v: FloatArray, numElements: Int,
            scale: FloatArray, recipScale: FloatArray
        ) {
            // Don't touch the vector.
            scale[0] = MlScalar.ML_SCALAR_ONE
            recipScale[0] = MlScalar.ML_SCALAR_ONE
            return
        }

        /**
         * Linear interpolation, aka lerp.  Set "result" to the interpolation
         * by "weight" from "v0" (when weight=0) to "v1" (when weight=1).
         *
         * @param weight The factor.
         * @param v0 The first vector.
         * @param v1 The second vector.
         * @param result The result of the interpolation.
         */
        @JvmStatic
        fun interpolate(
            weight: Float,
            v0: MlVector3, v1: MlVector3, result: MlVector3
        ) {
            result.mVector[0] = v0.mVector[0] * (MlScalar.ML_SCALAR_ONE - weight) +
                    v1.mVector[0] * weight
            result.mVector[1] = v0.mVector[1] * (MlScalar.ML_SCALAR_ONE - weight) +
                    v1.mVector[1] * weight
            result.mVector[2] = v0.mVector[2] * (MlScalar.ML_SCALAR_ONE - weight) +
                    v1.mVector[2] * weight
        }
    }

    /**
     * Finds the scale factor to put max value between 1 and TBD.
     *
     * @param scale The retrieved scale, an array of length 1.
     * @param recipScale The reciprocal of the retrieved scale, an arary of length 1.
     */
    fun getScale(scale: FloatArray, recipScale: FloatArray) {
        calcScale(mVector, 3, scale, recipScale)
    }

    /**
     * Determine if the passed vector **v** is equal to **this**.
     *
     * @param v The vector to test against.
     *
     * @return **true** is returned if the vectors are equal.
     * **false** is returned if the vectors are not equal.
     */
    fun equals(v: MlVector3): Boolean {
        return mVector[0] === v.mVector[0] &&
                mVector[1] === v.mVector[1] &&
                mVector[2] === v.mVector[2]
    }

    /**
     * Equality comparison within given tolerance - the square of the
     * length of the maximum distance between the two vectors.
     *
     * @param v The vector to test against.
     * @param tolerance The specified tolerance.
     *
     * @return **true** is returned if the vectors are equal.
     * **false** is returned if the vectors are not equal.
     */
    fun equals(v: MlVector3, tolerance: Float): Boolean {
        val diff = MlVector3(
            mVector[0] - v.mVector[0],
            mVector[1] - v.mVector[1],
            mVector[2] - v.mVector[2]
        )
        return diff.dot(diff) <= tolerance
    }

    /**
     * Returns principal axis that is closest (based on maximum dot
     * product) to this vector.
     *
     * @return The principal axis is returned as a vector.
     */
    fun getClosestAxis(): MlVector3 {
        val axis =
            MlVector3(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO)
        var bestAxis = MlVector3()
        var d: Float
        var max = -21.234f
        axis.mVector[0] = MlScalar.ML_SCALAR_ONE // +x axis
        if (dot(axis).also { d = it } > max) {
            max = d
            bestAxis = axis
        }
        axis.mVector[0] = -MlScalar.ML_SCALAR_ONE // -x axis
        if (dot(axis).also { d = it } > max) {
            max = d
            bestAxis = axis
        }
        axis.mVector[0] = MlScalar.ML_SCALAR_ZERO
        axis.mVector[1] = MlScalar.ML_SCALAR_ONE // +y axis
        if (dot(axis).also { d = it } > max) {
            max = d
            bestAxis = axis
        }
        axis.mVector[1] = -MlScalar.ML_SCALAR_ONE // -y axis
        if (dot(axis).also { d = it } > max) {
            max = d
            bestAxis = axis
        }
        axis.mVector[1] = MlScalar.ML_SCALAR_ZERO
        axis.mVector[2] = MlScalar.ML_SCALAR_ONE // +z axis
        if (dot(axis).also { d = it } > max) {
            max = d
            bestAxis = axis
        }
        axis.mVector[2] = -MlScalar.ML_SCALAR_ONE // -z axis
        if (dot(axis).also { d = it } > max) {
            max = d
            bestAxis = axis
        }
        return bestAxis
    }

    /**
     * Component-wise scalar multiplication.
     *
     * @param d The scalar value to multiply by.
     *
     * @return **this** vector is returned.
     */
    fun mul(d: Float): MlVector3 {
        mVector[0] = mVector[0] * d
        mVector[1] = mVector[1] * d
        mVector[2] = mVector[2] * d
        return this
    }

    /**
     * Component-wise binary scalar multiplication.
     *
     * @param v The vector to multiply.
     * @param d The scalar value to multiply by.
     *
     * @return A new vector containing the result is returned.
     */
    fun mul(v: MlVector3, d: Float): MlVector3 {
        return MlVector3(
            v.mVector[0] * d,
            v.mVector[1] * d,
            v.mVector[2] * d
        )
    }

    /**
     * Component-wise scalar division.
     *
     * @param d The scalar value to divide by.
     *
     * @return **this** vector is returned.
     */
    operator fun div(d: Float): MlVector3 {
        mul(1 / d)
        return this
    }

    /**
     * Component-wise vector addition.
     *
     * @param v The vector to add.
     *
     * @return A new vector containing the result is returned.
     */
    fun add(v: MlVector3): MlVector3 {
        mVector[0] += v.mVector[0]
        mVector[1] += v.mVector[1]
        mVector[2] += v.mVector[2]
        return this
    }

    /**
     * Component-wise binary vector addition.
     *
     * @param v1 The first vector.
     * @param v2 The second vector.
     *
     * @return A new vector containing the result is returned.
     */
    fun add(v1: MlVector3, v2: MlVector3): MlVector3 {
        return MlVector3(
            v1.mVector[0] + v2.mVector[0],
            v1.mVector[1] + v2.mVector[1],
            v1.mVector[2] + v2.mVector[2]
        )
    }

    /**
     * Component-wise vector subtraction.
     *
     * @param v The vector to subtract.
     *
     * @return A new vector containing the result is returned.
     */
    fun sub(v: MlVector3): MlVector3 {
        mVector[0] -= v.mVector[0]
        mVector[1] -= v.mVector[1]
        mVector[2] -= v.mVector[2]
        return this
    }

    /**
     * Component-wise binary vector subtraction.
     *
     * @param v1 The first vector.
     * @param v2 The second vector.
     *
     * @return A new vector containing the result is returned.
     */
    fun sub(v1: MlVector3, v2: MlVector3): MlVector3 {
        return MlVector3(
            v1.mVector[0] - v2.mVector[0],
            v1.mVector[1] - v2.mVector[1],
            v1.mVector[2] - v2.mVector[2]
        )
    }
}