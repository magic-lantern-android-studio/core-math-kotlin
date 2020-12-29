// COPYRIGHT_BEGIN
//
// The MIT License (MIT)
//
// Copyright (c) 2000 - 2021 Wizzer Works
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

// Import JUnit classes.
import org.junit.Assert.assertEquals
import org.junit.Test

class MlRotationUnitTest
{
    @Test
    @Throws(Exception::class)
    fun testConstructors() {
        // Test default constructor.
        val rot = MlRotation()
        assertEquals(0.0f, rot.mQuat[0], 0.0f)
        assertEquals(0.0f, rot.mQuat[1], 0.0f)
        assertEquals(0.0f, rot.mQuat[2], 0.0f)
        assertEquals(1.0f, rot.mQuat[3], 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testGetValue() {
        // Test default constructor.
        val rot = MlRotation()
        assertEquals(0.0f, rot.mQuat[0], 0.0f)
        assertEquals(0.0f, rot.mQuat[1], 0.0f)
        assertEquals(0.0f, rot.mQuat[2], 0.0f)
        assertEquals(1.0f, rot.mQuat[3], 0.0f)
        val v = FloatArray(4)
        rot.getValue(v)
        assertEquals(0.0f, v[0], 0.0f)
        assertEquals(0.0f, v[1], 0.0f)
        assertEquals(0.0f, v[2], 0.0f)
        assertEquals(1.0f, v[3], 0.0f)
        val axis = MlVector3()
        val angle = FloatArray(1)
        rot.getValue(axis, angle)
        assertEquals(0.0f, axis.mVector[0], 0.0f)
        assertEquals(0.0f, axis.mVector[1], 0.0f)
        assertEquals(1.0f, axis.mVector[2], 0.0f)
        assertEquals(0.0f, angle[0], 0.0f)
        val m = MlTransform()
        rot.getValue(m)
        assertEquals(1.0f, m.mMatrix[0][0], 0.0f)
        assertEquals(0.0f, m.mMatrix[0][1], 0.0f)
        assertEquals(0.0f, m.mMatrix[0][2], 0.0f)
        assertEquals(0.0f, m.mMatrix[1][0], 0.0f)
        assertEquals(1.0f, m.mMatrix[1][1], 0.0f)
        assertEquals(0.0f, m.mMatrix[1][2], 0.0f)
        assertEquals(0.0f, m.mMatrix[2][0], 0.0f)
        assertEquals(0.0f, m.mMatrix[2][1], 0.0f)
        assertEquals(1.0f, m.mMatrix[2][2], 0.0f)
        assertEquals(0.0f, m.mMatrix[3][0], 0.0f)
        assertEquals(0.0f, m.mMatrix[3][1], 0.0f)
        assertEquals(0.0f, m.mMatrix[3][2], 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun tesMultiplication1() {
        // Test default constructor.
        val rot = MlRotation()
        assertEquals(0.0f, rot.mQuat[0], 0.0f)
        assertEquals(0.0f, rot.mQuat[1], 0.0f)
        assertEquals(0.0f, rot.mQuat[2], 0.0f)
        assertEquals(1.0f, rot.mQuat[3], 0.0f)
        val delta = MlRotation(
            MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ONE,
            MlScalar.ML_SCALAR_ZERO, 0.035f
        )
        rot.mul(delta)
        assertEquals(0.0f, rot.mQuat[0], 0.0f)
        assertEquals(0.9993880987167358f, rot.mQuat[1], 0.0f)
        assertEquals(0.0f, rot.mQuat[2], 0.0f)
        assertEquals(0.034978583455085754f, rot.mQuat[3], 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun tesMultiplication2() {
        // Test default constructor.
        val rot = MlRotation()
        assertEquals(0.0f, rot.mQuat[0], 0.0f)
        assertEquals(0.0f, rot.mQuat[1], 0.0f)
        assertEquals(0.0f, rot.mQuat[2], 0.0f)
        assertEquals(1.0f, rot.mQuat[3], 0.0f)
        val delta = MlRotation(
            MlVector3(
                MlScalar.ML_SCALAR_ZERO, MlScalar.ML_SCALAR_ONE,
                MlScalar.ML_SCALAR_ZERO
            ), 0.035f
        )
        rot.mul(delta)
        assertEquals(0.0f, rot.mQuat[0], 0.0f)
        assertEquals(0.017499107867479324f, rot.mQuat[1], 0.0f)
        assertEquals(0.0f, rot.mQuat[2], 0.0f)
        assertEquals(0.9998469948768616f, rot.mQuat[3], 0.0f)
    }
}
