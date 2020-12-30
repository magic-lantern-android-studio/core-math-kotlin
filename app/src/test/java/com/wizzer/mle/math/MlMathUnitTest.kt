// COPYRIGHT_BEGIN
//
// The MIT License (MIT)
//
// Copyright (c) 2000 - 2016 Wizzer Works
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
import org.junit.Assert.*
import org.junit.Test
import java.nio.ByteOrder

/**
 * This class is used to unit test the <code>MlMath</code> class.
 */
class MlMathUnitTest
{
    @Test
    @Throws(Exception::class)
    fun testIntegerToByteArrayConversion() {
        val i = 5
        val b = ByteArray(4)
        val offset = 0
        val type: ByteOrder = ByteOrder.BIG_ENDIAN

        MlMath.convertIntegerToByteArray(i, b, offset, type)
        assertEquals(4, b.size)
    }

    @Test
    @Throws(Exception::class)
    fun testByteArrayToIntegerConversion() {
        val i = 5
        val b = ByteArray(4)
        val offset = 0
        val type: ByteOrder = ByteOrder.BIG_ENDIAN

        MlMath.convertIntegerToByteArray(i, b, offset, type)
        assertEquals(4, b.size)

        val value = MlMath.convertByteArrayToInteger(b, offset, type)
        assertEquals(i, value)
    }

    @Test
    @Throws(Exception::class)
    fun testFloatToByteArrayConversion() {
        val f = 5.0f
        val b = ByteArray(8)
        val offset = 0
        val type: ByteOrder = ByteOrder.BIG_ENDIAN

        MlMath.convertFloatToByteArray(f, b, offset, type)
        assertEquals(8, b.size)
    }

    @Test
    @Throws(Exception::class)
    fun testByteArrayToFloatConversion() {
        val f = 5.0f
        val b = ByteArray(8)
        val offset = 0
        val type: ByteOrder = ByteOrder.BIG_ENDIAN

        MlMath.convertFloatToByteArray(f, b, offset, type)
        assertEquals(8, b.size)

        val value = MlMath.convertByteArrayToFloat(b, offset, type)
        assertEquals(f, value, 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testDoubleToByteArrayConversion() {
        val d = 5.0
        val b = ByteArray(16)
        val offset = 0
        val type: ByteOrder = ByteOrder.BIG_ENDIAN

        MlMath.convertDoubleToByteArray(d, b, offset, type)
        assertEquals(16, b.size)
    }

    @Test
    @Throws(Exception::class)
    fun testByteArrayToDoubleConversion() {
        val d = 5.0
        val b = ByteArray(16)
        val offset = 0
        val type: ByteOrder = ByteOrder.BIG_ENDIAN

        MlMath.convertDoubleToByteArray(d, b, offset, type)
        assertEquals(16, b.size)

        val value = MlMath.convertByteArrayToDouble(b, offset, type)
        assertEquals(d, value, (0.0).toDouble())
    }

    @Test
    @Throws(Exception::class)
    fun testVector2ToByteArrayConversion() {
        val b = ByteArray(8)
        val offset = 0

        val v = MlVector2(1.0f, 1.0f)

        MlMath.convertVector2ToByteArray(offset, b, v)
        assertEquals(8, b.size)
    }

    @Test
    @Throws(Exception::class)
    fun testByteArrayToVector2Conversion() {
        val b = ByteArray(8)
        val offset = 0

        val v = MlVector2(1.0f, 1.0f)

        MlMath.convertVector2ToByteArray(offset, b, v)
        assertEquals(8, b.size)

        val value = MlVector2()
        MlMath.convertByteArrayToVector2(offset, b, value)
        assertEquals(1.0f, value.mVector[0])
        assertEquals(1.0f, value.mVector[1])
    }

    @Test
    @Throws(Exception::class)
    fun testVector3ToByteArrayConversion() {
            val b = ByteArray(12)
            val offset = 0

            val v = MlVector3(1.0f, 1.0f, 1.0f)

        MlMath.convertVector3ToByteArray(offset, b, v)
        assertEquals(12, b.size)
    }

    @Test
    @Throws(Exception::class)
    fun testByteArrayToVector3Conversion() {
        val b = ByteArray(12)
        val offset = 0

        val v = MlVector3(1.0f, 1.0f, 1.0f)

        MlMath.convertVector3ToByteArray(offset, b, v)
        assertEquals(12, b.size)

        val value = MlVector3()
        MlMath.convertByteArrayToVector3(offset, b, value)
        assertEquals(1.0f, value.mVector[0], 0.0f)
        assertEquals(1.0f, value.mVector[1], 0.0f)
        assertEquals(1.0f, value.mVector[2], 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testVector4ToByteArrayConversion() {
        val b = ByteArray(16)
        val offset = 0

        val v = MlVector4(1.0f, 1.0f, 1.0f, 1.0f)

        MlMath.convertVector4ToByteArray(offset, b, v)
        assertEquals(16, b.size)
    }

    @Test
    @Throws(Exception::class)
    fun testByteArrayToVector4Conversion() {
        val b = ByteArray(16)
        val offset = 0

        val v = MlVector4(1.0f, 1.0f, 1.0f, 1.0f)

        MlMath.convertVector4ToByteArray(offset, b, v)
        assertEquals(16, b.size)

        val value = MlVector4()
        MlMath.convertByteArrayToVector4(offset, b, value)
        assertEquals(1.0f, value.mVector[0], 0.0f)
        assertEquals(1.0f, value.mVector[1], 0.0f)
        assertEquals(1.0f, value.mVector[2], 0.0f)
        assertEquals(1.0f, value.mVector[3], 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testRotationToByteArrayConversion() {
        val b = ByteArray(16)
        val offset = 0

        val r = MlRotation(0.0f, 0.0f, 0.0f, 1.0f)

        MlMath.convertRotationToByteArray(offset, b, r)
        assertEquals(16, b.size)
    }

    @Test
    @Throws(Exception::class)
    fun testByteArrayToRotationConversion() {
        val b = ByteArray(16)
        val offset = 0

        val r = MlRotation(0.0f, 0.0f, 0.0f, 1.0f)

        MlMath.convertRotationToByteArray(offset, b, r)
        assertEquals(16, b.size)

        val value = MlRotation()
        MlMath.convertByteArrayToRotation(offset, b, value)
        assertEquals(0.0f, value.mQuat[0], 0.0f)
        assertEquals(0.0f, value.mQuat[1], 0.0f)
        assertEquals(0.0f, value.mQuat[2], 0.0f)
        assertEquals(1.0f, value.mQuat[3], 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testTransformToByteArrayConversion() {
        val b = ByteArray(48)
        val offset = 0

        val a11 = 0f; val a12 = 0f; val a13 = 0f
        val a21 = 0f; val a22 = 0f; val a23 = 0f
        val a31 = 0f; val a32 = 0f; val a33 = 0f
        val a41 = 0f; val a42 = 0f; val a43 = 0f
        val t = MlTransform(a11, a12, a13, a21, a22, a23, a31, a32, a33, a41, a42, a43)

        MlMath.convertTransforrmToByteArray(offset, b, t)
        assertEquals(48, b.size)
    }

    @Test
    @Throws(Exception::class)
    fun testByteArrayToTransformConversion() {
        val b = ByteArray(48)
        val offset = 0

        val a11 = 0f; val a12 = 0f; val a13 = 0f
        val a21 = 0f; val a22 = 0f; val a23 = 0f
        val a31 = 0f; val a32 = 0f; val a33 = 0f
        val a41 = 0f; val a42 = 0f; val a43 = 0f
        val t = MlTransform(a11, a12, a13, a21, a22, a23, a31, a32, a33, a41, a42, a43)

        MlMath.convertTransforrmToByteArray(offset, b, t)
        assertEquals(48, b.size)

        val value = MlTransform()
        MlMath.convertByteArrayToTransform(offset, b, value)
        assertEquals(a11, value.mMatrix[0][0], 0.0f)
        assertEquals(a12, value.mMatrix[0][1], 0.0f)
        assertEquals(a13, value.mMatrix[0][2], 0.0f)
        assertEquals(a21, value.mMatrix[1][0], 0.0f)
        assertEquals(a22, value.mMatrix[1][1], 0.0f)
        assertEquals(a23, value.mMatrix[1][2], 0.0f)
        assertEquals(a31, value.mMatrix[2][0], 0.0f)
        assertEquals(a32, value.mMatrix[2][1], 0.0f)
        assertEquals(a33, value.mMatrix[2][2], 0.0f)
        assertEquals(a41, value.mMatrix[3][0], 0.0f)
        assertEquals(a42, value.mMatrix[3][1], 0.0f)
        assertEquals(a43, value.mMatrix[3][2], 0.0f)
    }
}