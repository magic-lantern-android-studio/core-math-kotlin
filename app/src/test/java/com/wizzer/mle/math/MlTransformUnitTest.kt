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
import org.junit.Assert.assertTrue
import org.junit.Test

// Import Magic Lantern classes.
import com.wizzer.mle.math.MlAngle.Companion.angleToRadians
import com.wizzer.mle.math.MlTransform.Companion.identity

class MlTransformUnitTest
{
    @Test
    @Throws(Exception::class)
    fun testConstructors() {
        // Test default constructor.
        var transform = MlTransform()
        assertTrue(transform.isZero())

        // Test constructor with individual elements initialization.
        val a11 = 0f
        val a12 = 0f
        val a13 = 0f
        val a21 = 0f
        val a22 = 0f
        val a23 = 0f
        val a31 = 0f
        val a32 = 0f
        val a33 = 0f
        val a41 = 0f
        val a42 = 0f
        val a43 = 0f
        transform = MlTransform(a11, a12, a13, a21, a22, a23, a31, a32, a33, a41, a42, a43)
        assertTrue(transform.isZero())

        // Test constructor with array initialization.
        val a = Array(4) { FloatArray(3) }
        a[0][0] = 0f
        a[0][1] = 0f
        a[0][2] = 0f
        a[1][0] = 0f
        a[1][1] = 0f
        a[1][2] = 0f
        a[2][0] = 0f
        a[2][1] = 0f
        a[2][2] = 0f
        a[3][0] = 0f
        a[3][1] = 0f
        a[3][2] = 0f
        transform = MlTransform(a)
        assertTrue(transform.isZero())
    }

    @Test
    @Throws(Exception::class)
    fun testIdentity() {
        // Test default constructor.
        var transform = MlTransform()
        assertTrue(transform.isZero())
        transform.makeIdentity()
        assertTrue(transform.isIdentity())
        assertEquals(1.0f, transform.mMatrix[0][0], 0.0f)
        assertEquals(0.0f, transform.mMatrix[0][1], 0.0f)
        assertEquals(0.0f, transform.mMatrix[0][2], 0.0f)
        assertEquals(0.0f, transform.mMatrix[1][0], 0.0f)
        assertEquals(1.0f, transform.mMatrix[1][1], 0.0f)
        assertEquals(0.0f, transform.mMatrix[1][2], 0.0f)
        assertEquals(0.0f, transform.mMatrix[2][0], 0.0f)
        assertEquals(0.0f, transform.mMatrix[2][1], 0.0f)
        assertEquals(1.0f, transform.mMatrix[2][2], 0.0f)
        assertEquals(0.0f, transform.mMatrix[3][0], 0.0f)
        assertEquals(0.0f, transform.mMatrix[3][1], 0.0f)
        assertEquals(0.0f, transform.mMatrix[3][2], 0.0f)
        transform = identity()
        assertTrue(transform.isIdentity())
    }

    @Test
    @Throws(Exception::class)
    fun testDeterminant() {
        // Test default constructor.
        val transform = identity()
        assertTrue(transform.isIdentity())
        var v = transform.determinant()
        assertEquals(1.0f, v, 0.0f)
        val a = Array(4) { FloatArray(3) }
        a[0][0] = 6f
        a[0][1] = 1f
        a[0][2] = 1f
        a[1][0] = 4f
        a[1][1] = (-2).toFloat()
        a[1][2] = 5f
        a[2][0] = 2f
        a[2][1] = 8f
        a[2][2] = 7f
        a[3][0] = 0f
        a[3][1] = 0f
        a[3][2] = 0f
        transform.setValue(a)
        v = transform.determinant()
        assertEquals(-306.0f, v, 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testUniformScale() {
        // Test default constructor.
        val transform = identity()
        assertTrue(transform.isIdentity())
        val s = 5f
        transform.setScale(s)
        assertEquals(5.0f, transform.mMatrix[0][0], 0.0f)
        assertEquals(0.0f, transform.mMatrix[0][1], 0.0f)
        assertEquals(0.0f, transform.mMatrix[0][2], 0.0f)
        assertEquals(0.0f, transform.mMatrix[1][0], 0.0f)
        assertEquals(5.0f, transform.mMatrix[1][1], 0.0f)
        assertEquals(0.0f, transform.mMatrix[1][2], 0.0f)
        assertEquals(0.0f, transform.mMatrix[2][0], 0.0f)
        assertEquals(0.0f, transform.mMatrix[2][1], 0.0f)
        assertEquals(5.0f, transform.mMatrix[2][2], 0.0f)
        assertEquals(0.0f, transform.mMatrix[3][0], 0.0f)
        assertEquals(0.0f, transform.mMatrix[3][1], 0.0f)
        assertEquals(0.0f, transform.mMatrix[3][2], 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testNonUniformScale() {
        // Test default constructor.
        val transform = identity()
        assertTrue(transform.isIdentity())
        val s = MlVector3()
        s.mVector[0] = 2f
        s.mVector[1] = 4f
        s.mVector[2] = 8f
        transform.setScale(s)
        assertEquals(2.0f, transform.mMatrix[0][0], 0.0f)
        assertEquals(0.0f, transform.mMatrix[0][1], 0.0f)
        assertEquals(0.0f, transform.mMatrix[0][2], 0.0f)
        assertEquals(0.0f, transform.mMatrix[1][0], 0.0f)
        assertEquals(4.0f, transform.mMatrix[1][1], 0.0f)
        assertEquals(0.0f, transform.mMatrix[1][2], 0.0f)
        assertEquals(0.0f, transform.mMatrix[2][0], 0.0f)
        assertEquals(0.0f, transform.mMatrix[2][1], 0.0f)
        assertEquals(8.0f, transform.mMatrix[2][2], 0.0f)
        assertEquals(0.0f, transform.mMatrix[3][0], 0.0f)
        assertEquals(0.0f, transform.mMatrix[3][1], 0.0f)
        assertEquals(0.0f, transform.mMatrix[3][2], 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testSetScaleOnly() {
        // Initialize the transform.
        val a = Array(4) { FloatArray(3) }
        a[0][0] = 6f
        a[0][1] = 1f
        a[0][2] = 1f
        a[1][0] = 4f
        a[1][1] = (-2).toFloat()
        a[1][2] = 5f
        a[2][0] = 2f
        a[2][1] = 8f
        a[2][2] = 7f
        a[3][0] = 0f
        a[3][1] = 0f
        a[3][2] = 0f
        val transform = MlTransform(a)

        // Set only the nonuniform scale values, not touching translation and rotation.
        val s = MlVector3()
        s.mVector[0] = 2f
        s.mVector[1] = 4f
        s.mVector[2] = 8f
        transform.setScaleOnly(s)
        assertEquals(1.6760088205337524f, transform.mMatrix[0][0], 0.0f)
        assertEquals(-0.9253019690513611f, transform.mMatrix[0][1], 0.0f)
        assertEquals(0.5786280632019043f, transform.mMatrix[0][2], 0.0f)
        assertEquals(2.053525686264038f, transform.mMatrix[1][0], 0.0f)
        assertEquals(1.9552825689315796f, transform.mMatrix[1][1], 0.0f)
        assertEquals(-2.8213295936584473f, transform.mMatrix[1][2], 0.0f)
        assertEquals(1.4792004823684692f, transform.mMatrix[2][0], 0.0f)
        assertEquals(5.916801452636719f, transform.mMatrix[2][1], 0.0f)
        assertEquals(5.1772027015686035f, transform.mMatrix[2][2], 0.0f)
        assertEquals(0f, transform.mMatrix[3][0], 0.0f)
        assertEquals(0f, transform.mMatrix[3][1], 0.0f)
        assertEquals(0f, transform.mMatrix[3][2], 0.0f)

        // Check whether extracted nonuniform scale values are the same as the one set above.
        val result = MlVector3()
        transform.getScale(result)
        assertEquals(2f, result.mVector[0], 0.000001f)
        assertEquals(4f, result.mVector[1], 0.000001f)
        assertEquals(8f, result.mVector[2], 0.000001f)
    }

    @Test
    @Throws(Exception::class)
    fun testSetTranslation() {
        val t = identity()

        // Set only the translation values, not touching scale and rotation.
        val v = MlVector3()
        v.mVector[0] = (-1).toFloat() // Translate along x.
        v.mVector[1] = (-25).toFloat() // Translate along y.
        v.mVector[2] = 50f // Translate along z
        t.setTranslation(v)
        assertEquals(1f, t.mMatrix[0][0], 0f)
        assertEquals(0f, t.mMatrix[0][1], 0f)
        assertEquals(0f, t.mMatrix[0][2], 0f)
        assertEquals(0f, t.mMatrix[1][0], 0f)
        assertEquals(1f, t.mMatrix[1][1], 0f)
        assertEquals(0f, t.mMatrix[1][2], 0f)
        assertEquals(0f, t.mMatrix[2][0], 0f)
        assertEquals(0f, t.mMatrix[2][1], 0f)
        assertEquals(1f, t.mMatrix[2][2], 0f)
        assertEquals(-1f, t.mMatrix[3][0], 0f)
        assertEquals(-25f, t.mMatrix[3][1], 0f)
        assertEquals(50f, t.mMatrix[3][2], 0f)

        // Check whether extracted translation values are the same as the one set above.
        val result = MlVector3()
        t.getTranslation(result)
        assertEquals(-1f, result.mVector[0], 0f)
        assertEquals(-25f, result.mVector[1], 0f)
        assertEquals(50f, result.mVector[2], 0f)
    }

    @Test
    @Throws(Exception::class)
    fun testSetTranslationOnly() {
        val a = Array(4) { FloatArray(3) }
        a[0][0] = 6f
        a[0][1] = 1f
        a[0][2] = 1f
        a[1][0] = 4f
        a[1][1] = (-2).toFloat()
        a[1][2] = 5f
        a[2][0] = 2f
        a[2][1] = 8f
        a[2][2] = 7f
        a[3][0] = 0f
        a[3][1] = 0f
        a[3][2] = 0f
        val t = MlTransform(a)

        // Set only the translation values, not touching scale and rotation.
        val v = MlVector3()
        v.mVector[0] = (-1).toFloat() // Translate along x
        v.mVector[1] = (-25).toFloat() // Translate along y
        v.mVector[2] = 50f // Translate along z
        t.setTranslationOnly(v)
        assertEquals(6f, t.mMatrix[0][0], 0f)
        assertEquals(1f, t.mMatrix[0][1], 0f)
        assertEquals(1f, t.mMatrix[0][2], 0f)
        assertEquals(4f, t.mMatrix[1][0], 0f)
        assertEquals(-2f, t.mMatrix[1][1], 0f)
        assertEquals(5f, t.mMatrix[1][2], 0f)
        assertEquals(2f, t.mMatrix[2][0], 0f)
        assertEquals(8f, t.mMatrix[2][1], 0f)
        assertEquals(7f, t.mMatrix[2][2], 0f)
        assertEquals(-1f, t.mMatrix[3][0], 0f)
        assertEquals(-25f, t.mMatrix[3][1], 0f)
        assertEquals(50f, t.mMatrix[3][2], 0f)

        // Check whether extracted translation values are the same as the one set above.
        val result = MlVector3()
        t.getTranslation(result)
        assertEquals(-1f, result.mVector[0], 0f)
        assertEquals(-25f, result.mVector[1], 0f)
        assertEquals(50f, result.mVector[2], 0f)
    }

    @Test
    @Throws(Exception::class)
    fun testSetRotationOnly() {
        val t = identity()

        // Set only the rotation values, not touching scale and translation.
        val v = MlVector3()
        v.mVector[0] = 10f // Rotate 10 degrees around the x axis.
        v.mVector[1] = 10f // Rotate 10 degrees around the y axis.
        v.mVector[2] = 10f // Rotate 10 degrees around the z axis.
        t.setRotationOnly(v)
        assertEquals(0.969846248626709f, t.mMatrix[0][0], 0f)
        assertEquals(0.2007056623697281f, t.mMatrix[0][1], 0f)
        assertEquals(-0.13825835287570953f, t.mMatrix[0][2], 0f)
        assertEquals(-0.1710100769996643f, t.mMatrix[1][0], 0f)
        assertEquals(0.9646100997924805f, t.mMatrix[1][1], 0f)
        assertEquals(0.20070567727088928f, t.mMatrix[1][2], 0f)
        assertEquals(0.1736481934785843f, t.mMatrix[2][0], 0f)
        assertEquals(-0.1710100769996643f, t.mMatrix[2][1], 0f)
        assertEquals(0.969846248626709f, t.mMatrix[2][2], 0f)
        assertEquals(0f, t.mMatrix[3][0], 0f)
        assertEquals(0f, t.mMatrix[3][1], 0f)
        assertEquals(0f, t.mMatrix[3][2], 0f)

        // Check whether extracted nonuniform scale values are the same as the one set above.
        val result = MlVector3()
        t.getScale(result)
        assertEquals(1f, result.mVector[0], 0.000001f)
        assertEquals(1f, result.mVector[1], 0.000001f)
        assertEquals(1f, result.mVector[2], 0.000001f)

        // Check whether extracted translation values are the same as the one set above.
        t.getTranslation(result)
        assertEquals(0f, result.mVector[0], 0.000001f)
        assertEquals(0f, result.mVector[1], 0.000001f)
        assertEquals(0f, result.mVector[2], 0.000001f)

        // Check whether extracted translation values are the same as the one set above.
        t.getRotation(result)
        assertEquals(10f, result.mVector[0], 0.000001f)
        assertEquals(10f, result.mVector[1], 0.000001f)
        assertEquals(10f, result.mVector[2], 0.000001f)
    }

    @Test
    @Throws(Exception::class)
    fun testSetRotation() {
        val t = MlTransform(0f, 0f, 0f,
            0f, 0f, 0f, 0f, 0f, 0f,
            0f, 0f, 0f)
        val rot = MlRotation()
        val axis = MlVector3(1f, 1f, 1f)
        val angle = angleToRadians(10f)
        rot.setValue(axis, angle)
        t.setRotation(rot)
        assertEquals(1f, t.mMatrix[0][0], 0f)
        assertEquals(1.560076611895056E-6f, t.mMatrix[0][1], 0f)
        assertEquals(-1.560074110784626E-6f, t.mMatrix[0][2], 0f)
        assertEquals(-1.560074110784626E-6f, t.mMatrix[1][0], 0f)
        assertEquals(1f, t.mMatrix[1][1], 0f)
        assertEquals(1.560076611895056E-6f, t.mMatrix[1][2], 0f)
        assertEquals(1.560076611895056E-6f, t.mMatrix[2][0], 0f)
        assertEquals(-1.560074110784626E-6f, t.mMatrix[2][1], 0f)
        assertEquals(1f, t.mMatrix[2][2], 0f)
        assertEquals(0f, t.mMatrix[3][0], 0f)
        assertEquals(0f, t.mMatrix[3][1], 0f)
        assertEquals(0f, t.mMatrix[3][2], 0f)

        // Check whether extracted rotation values are the same as the one set above.
        val rotation = MlVector3()
        t.getRotation(rotation)
        assertEquals(8.938566315919161E-5f, rotation.mVector[0], 0f)
        assertEquals(8.938580140238628E-5f, rotation.mVector[1], 0f)
        assertEquals(8.938566315919161E-5f, rotation.mVector[2], 0f)
    }
}