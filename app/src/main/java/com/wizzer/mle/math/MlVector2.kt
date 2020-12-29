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
 * 2D vector used to represent points or directions.  Each component of
 * the vector is a single-precision floating-point value.
 */
class MlVector2
{
    // The internal vector representation.
    var mVector = FloatArray(2)

    /**
     * The default constructor. All elements in the vector are
     * initialized to zero (0.0f).
     */
    constructor() {
        mVector[0] = 0.0f
        mVector[1] = 0.0f
    }

    /**
    * A constructor given an array of 2 components
    * to initialize from.
    *
    * @param v The array of components to initialize.
    */
    constructor(v: FloatArray)
    {
        mVector[0] = v[0];
        mVector[1] = v[1];
    }

    /**
     * A constructor given 2 individual components
     * to initialize from.
     *
     * @param x The x element.
     * @param y The y element.
     */
    constructor(x: Float, y: Float)
    {
        mVector[0] = x;
        mVector[1] = y;
    }

    /**
     * A constructor given another vector to
     * initialize from.
     *
     * @param v The other vector.
     */
    constructor(v: MlVector2) {
        mVector[0] = v.mVector[0]
        mVector[1] = v.mVector[1]
    }

    /**
     * Determine if the vector is set to [0 0 0 0].
     *
     * @return Returns **true** if all elements are zero.
     * Otherwise, returns **false**.
     */
    fun isZero(): Boolean {
        return mVector[0] === 0.0F &&
                mVector[1] === 0.0F
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
    fun dot(v: MlVector2): Float {
        return mVector[0] * v.mVector[0] +
                mVector[1] * v.mVector[1]
    }

    /**
     * Get the value of the vector.
     *
     * @param v Returns an array of 2 components.
     */
    fun getValue(v: FloatArray) {
        v[0] = mVector[0]
        v[1] = mVector[1]
    }

    /**
     * Set the value of the vector.
     *
     * @param v An array of 2 components.
     *
     * @return **this** is returned.
     */
    fun setValue(v: FloatArray): MlVector2 {
        mVector[0] = v[0]
        mVector[1] = v[1]
        return this
    }

    /**
     * Set the value of the vector.
     *
     * @param v The vector to copy from.
     *
     * @return **this** is returned.
     */
    fun setValue(v: MlVector2): MlVector2 {
        mVector[0] = v.mVector[0]
        mVector[1] = v.mVector[1]
        return this
    }

    /**
     * Sets value of vector from 2 individual components.
     *
     * @param x The x element to set.
     * @param y The y element to set.
     *
     * @return **this** is returned.
     */
    fun setValue(x: Float, y: Float): MlVector2 {
        mVector[0] = x
        mVector[1] = y
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
                    mVector[1] * mVector[1]).toDouble()
        ).toFloat()
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
        } else setValue(MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ZERO)
        return len
    }

    /**
     * Negates each component of vector in place.
     */
    fun negate() {
        mVector[0] = -mVector[0]
        mVector[1] = -mVector[1]
    }

    /**
     * Nondestructive unary negation.
     *
     * @return A new vector is returned.
     */
    fun unaryNegate() : MlVector2 {
        return MlVector2(-mVector[0], -mVector[1])
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
    }

    /**
     * Finds the scale factor to put max value between 1 and TBD.
     *
     * @param scale The retrieved scale, an array of length 1.
     * @param recipScale The reciprocal of the retrieved scale, an arary of length 1.
     */
    fun getScale(scale: FloatArray, recipScale: FloatArray) {
        calcScale(mVector, 2, scale, recipScale)
    }

    /**
     * Determine if the passed vector **v** is equal to **this**.
     *
     * @param v The vector to test against.
     *
     * @return **true** is returned if the vectors are equal.
     * **false** is returned if the vectors are not equal.
     */
    fun equals(v: MlVector2): Boolean {
        return mVector[0] === v.mVector[0] &&
                mVector[1] === v.mVector[1]
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
    fun equals(v: MlVector2, tolerance: Float): Boolean {
        val diff = MlVector2(
            mVector[0] - v.mVector[0],
            mVector[1] - v.mVector[1]
        )
        return diff.dot(diff) <= tolerance
    }

    /**
     * Component-wise scalar multiplication.
     *
     * @param d The scalar value to multiply by.
     *
     * @return **this** vector is returned.
     */
    fun mul(d: Float): MlVector2 {
        mVector[0] = mVector[0] * d
        mVector[1] = mVector[1] * d
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
    fun mul(v: MlVector2, d: Float): MlVector2 {
        return MlVector2(
            v.mVector[0] * d,
            v.mVector[1] * d
        )
    }

    /**
     * Component-wise scalar division.
     *
     * @param d The scalar value to divide by.
     *
     * @return **this** vector is returned.
     */
    operator fun div(d: Float): MlVector2 {
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
    fun add(v: MlVector2): MlVector2 {
        mVector[0] += v.mVector[0]
        mVector[1] += v.mVector[1]
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
    fun add(v1: MlVector2, v2: MlVector2): MlVector2 {
        return MlVector2(
            v1.mVector[0] + v2.mVector[0],
            v1.mVector[1] + v2.mVector[1]
        )
    }

    /**
     * Component-wise vector subtraction.
     *
     * @param v The vector to subtract.
     *
     * @return A new vector containing the result is returned.
     */
    fun sub(v: MlVector2): MlVector2 {
        mVector[0] -= v.mVector[0]
        mVector[1] -= v.mVector[1]
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
    fun sub(v1: MlVector2, v2: MlVector2): MlVector2 {
        return MlVector2(
            (v1.mVector[0] - v2.mVector[0]),
            (v1.mVector[1] - v2.mVector[1])
        )
    }
}