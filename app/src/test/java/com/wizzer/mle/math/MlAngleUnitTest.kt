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

/**
 * This class is used to unit test the <code>MlAngle</code> class.
 */
class MlAngleUnitTest
{
    @Test
    @Throws(Exception::class)
    fun testAngleToDegreesConversion() {
        var degrees = MlAngle.angleToDegrees(MlAngle.ML_ANGLE_ZERO)
        assertEquals(0.0f, degrees, 0.0f)

        degrees = MlAngle.angleToDegrees(MlAngle.ML_ANGLE_PI)
        assertEquals(180.0f, degrees, 0.0f)

        degrees = MlAngle.angleToDegrees(MlAngle.ML_ANGLE_PI_HALF)
        assertEquals(90.0f, degrees, 0.0f)

        degrees = MlAngle.angleToDegrees(MlAngle.ML_ANGLE_TWO_PI)
        assertEquals(360.0f, degrees, 0.0f)

        degrees = MlAngle.angleToDegrees(MlAngle.ML_ANGLE_PI_FOURTH)
        assertEquals(45.0f, degrees, 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testAngleToRadiansConversion() {
        var radians = MlAngle.angleToRadians(MlAngle.ML_ANGLE_ZERO)
        assertEquals(0.0f, radians, 0.0f)

        radians = MlAngle.angleToRadians(MlAngle.ML_ANGLE_PI)
        assertEquals(3.1415927f, radians, 0.0f)

        radians = MlAngle.angleToRadians(MlAngle.ML_ANGLE_PI_HALF)
        assertEquals(1.5707964f, radians, 0.0f)

        radians = MlAngle.angleToRadians(MlAngle.ML_ANGLE_TWO_PI)
        assertEquals(6.2831855f, radians, 0.0f)

        radians = MlAngle.angleToRadians(MlAngle.ML_ANGLE_PI_FOURTH)
        assertEquals(0.7853982f, radians, 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testDegreesToAngleConversion() {
        var angle = MlAngle.degreesToAngle(0.0f)
        assertEquals(MlAngle.ML_ANGLE_ZERO, angle, 0.0f)

        angle = MlAngle.degreesToAngle(180.0f)
        assertEquals(MlAngle.ML_ANGLE_PI, angle, 0.0f)

        angle = MlAngle.degreesToAngle(90.0f)
        assertEquals(MlAngle.ML_ANGLE_PI_HALF, angle, 0.0f)

        angle = MlAngle.degreesToAngle(360.0f)
        assertEquals(MlAngle.ML_ANGLE_TWO_PI, angle, 0.0f)

        angle = MlAngle.degreesToAngle(45.0f)
        assertEquals(MlAngle.ML_ANGLE_PI_FOURTH, angle, 0.0f)
    }

    @Test
    @Throws(Exception::class)
    fun testRadiansToAngleConversion() {
        var angle = MlAngle.radiansToAngle(0.0f)
        assertEquals(MlAngle.ML_ANGLE_ZERO, angle, 0.0f)

        angle = MlAngle.radiansToAngle(3.1415927f)
        assertEquals(MlAngle.ML_ANGLE_PI , angle, 0.0f)

        angle = MlAngle.radiansToAngle(1.5707964f)
        assertEquals(MlAngle.ML_ANGLE_PI_HALF, angle, 0.0f)

        angle = MlAngle.radiansToAngle(6.2831855f)
        assertEquals(MlAngle.ML_ANGLE_TWO_PI, angle, 0.0f)

        angle = MlAngle.radiansToAngle(0.7853982f)
        assertEquals(MlAngle.ML_ANGLE_PI_FOURTH, angle, 0.0f)
    }
}