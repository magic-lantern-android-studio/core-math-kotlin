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
 * Utility class to help process angles.
 */
class MlAngle
{
    companion object {
        /** The angle: 0.  */
        const val ML_ANGLE_ZERO = 0f

        /** The angle: 2 * PI.  */
        const val ML_ANGLE_TWO_PI = 1f

        /** The angle: PI.  */
        const val ML_ANGLE_PI = 0.5f

        /** The angle: PI / 2.  */
        const val ML_ANGLE_PI_HALF = 0.25f

        /** The angle: PI / 4.  */
        const val ML_ANGLE_PI_FOURTH = 0.125f

        // Functions for doing conversions to angles, which use 2PI == 0x10000
        // in fixed and regular floating point values in float mode.

        /**
         * Convert angle to degrees.
         *
         * @param a The angle to convert.
         *
         * @return The corresponding angle is returned, in degrees.
         */
        @JvmStatic
        fun angleToDegrees(a: Float): Float {
            return 360 * a
        }

        /**
         * Convert angle to radians.
         *
         * @param a The angle to convert.
         *
         * @return The corresponding angle is returned, in radians.
         */
        @JvmStatic
        fun angleToRadians(a: Float): Float {
            return MlScalar.ML_SCALAR_TWO_PI * a
        }

        /**
         * Convert degrees to angle.
         *
         * @param s The degrees to convert.
         *
         * @return The corresponding angle is returned.
         */
        @JvmStatic
        fun degreesToAngle(s: Float): Float {
            // Divide by 360 as 1.42/512.
            return s * 2.77777777777777777e-3.toFloat()
        }

        /**
         * Convert radians to angle.
         *
         * @param s The radians to convert.
         *
         * @return The corresponding angle is returned.
         */
        @JvmStatic
        fun radiansToAngle(s: Float): Float {
            // Divide by 2PI as 1/2PI = 5.09/32
            return s * 0.15915494309189533619f
        }
    }
}