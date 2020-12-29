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
import org.junit.Assert.*
import org.junit.Test


/**
 * To work on unit tests, switch the Test Artifact in the Build Variants view.
 */
class MlVector2UnitTest
{
    @Test
    @Throws(Exception::class)
    fun testConstructors() {
        var vector = MlVector2()
        vector.mVector[0] = 1.0.toFloat()
        vector.mVector[1] = 1.0.toFloat()
        assertEquals(1.0f, vector.mVector[0], 0.0f)
        assertEquals(1.0f, vector.mVector[1], 0.0f)
        vector = MlVector2(2.0.toFloat(), 2.0.toFloat())
        assertEquals(2.0f, vector.mVector[0], 0.0f)
        assertEquals(2.0f, vector.mVector[1], 0.0f)
        val v = FloatArray(2)
        v[0] = 3.0.toFloat()
        v[1] = 3.0.toFloat()
        vector = MlVector2(v)
        assertEquals(3.0f, vector.mVector[0], 0.0f)
        assertEquals(3.0f, vector.mVector[1], 0.0f)
        val vectorCopy = MlVector2(vector)
        assertEquals(3.0f, vectorCopy.mVector[0], 0.0f)
        assertEquals(3.0f, vectorCopy.mVector[1], 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testIsZero() {
        val vector = MlVector2()
        assertTrue(vector.isZero())
        vector.mVector[0] = 1.0.toFloat()
        assertFalse(vector.isZero())
    }

    @Test
    @Throws(Exception::class)
    fun testDotProduct() {
        val vector1 = MlVector2()
        val vector2 = MlVector2(1.0.toFloat(), 1.0.toFloat())
        assertEquals(0.0f, vector1.dot(vector2), 0.0f)
        vector1.mVector[0] = 1f
        vector1.mVector[1] = 1f
        assertEquals(2.0f, vector1.dot(vector2), 0.0f)
        vector1.mVector[0] = 2f
        vector1.mVector[1] = 2f
        assertEquals(4.0f, vector1.dot(vector2), 0.0f)
        vector2.mVector[0] = 3f
        vector2.mVector[1] = 3f
        assertEquals(12.0f, vector1.dot(vector2), 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testLength() {
        val vector1 = MlVector2()
        val vector2 = MlVector2(1.0.toFloat(), 1.0.toFloat())
        assertEquals(0.0f, vector1.length(), 0.0f)
        assertEquals(1.4142135381698608f, vector2.length(), 0.0f)
    }

}