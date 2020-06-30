/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Limits.hpp
 *
 * Limiting / constrain helper functions
 */

#pragma once

#include <float.h>
#include <math.h>
#include <stdint.h>

#ifndef MATH_PI
#define MATH_PI		3.141592653589793238462643383280
#define M_TWOPI		6.28318531
#endif

namespace math
{

template<typename _Tp>
constexpr _Tp min(_Tp a, _Tp b)
{
	return (a < b) ? a : b;
}

template<typename _Tp>
constexpr _Tp max(_Tp a, _Tp b) {
	return (a > b) ? a : b;
}

template<typename _Tp>
constexpr _Tp constrain(_Tp val, _Tp min_val, _Tp max_val) {
	return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

/** Constrain float values to valid values for int16_t.
 * Invalid values are just clipped to be in the range for int16_t. */
constexpr int16_t constrainFloatToInt16(float value) {
	return (int16_t)math::constrain(value, (float)INT16_MIN, (float)INT16_MAX);
}

template<typename _Tp>
constexpr bool isInRange(_Tp val, _Tp min_val, _Tp max_val) {
	return (min_val <= val) && (val <= max_val);
}

template<typename T>
constexpr T radians(T degrees) {
	return degrees * (static_cast<T>(MATH_PI) / static_cast<T>(180));
}

template<typename T>
constexpr T degrees(T radians) {
	return radians * (static_cast<T>(180) / static_cast<T>(MATH_PI));
}

/** Safe way to check if float is zero */
inline bool isZero(float val) {
	return fabsf(val - 0.0f) < FLT_EPSILON;
}

/** Safe way to check if double is zero */
inline bool isZero(double val) {
	return fabs(val - 0.0) < DBL_EPSILON;
}

template<typename Type>
Type wrap(Type x, Type low, Type high) {
    // already in range
    if (low <= x && x < high) {
        return x;
    }

    const Type range = high - low;
    const Type inv_range = Type(1) / range; // should evaluate at compile time, multiplies below at runtime
    const Type num_wraps = floor((x - low) * inv_range);
    return x - range * num_wraps;
}

template<typename Type>
Type wrapPi(Type x) {
    return wrap(x, Type(-MATH_PI), Type(MATH_PI));
}

/**
 * Wrap value in range [0, 2π)
 */
template<typename Type>
Type wrap2Pi(Type x) {
    return wrap(x, Type(0), Type(M_TWOPI));
}

}
