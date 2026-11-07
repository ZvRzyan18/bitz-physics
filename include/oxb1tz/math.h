#ifndef BZ_MATH_H
#define BZ_MATH_H

#include "oxb1tz/hint.h"
#include "oxb1tz/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
	float f;
 uint32_t i;
 int32_t s;
} oxb_floatbits;


oxb_const oxb_inline float oxb_nan(void) {
 oxb_floatbits bits;
 bits.i = 0x7F800001;
 return bits.f;
}



oxb_const oxb_inline float oxb_inf(void) {
 oxb_floatbits bits;
 bits.i = 0x7F800000;
 return bits.f;
}

/*
 not usually a fastest approach, but still usefull
*/
oxb_const oxb_inline float oxb_copysign(float a, float b) {
 oxb_floatbits a_bits, b_bits;
 a_bits.f = a;
 b_bits.f = b;
 a_bits.i = (a_bits.i & 0x7FFFFFFF) | (b_bits.i & 0x80000000); 
 return a_bits.f;
}

/*
 use std fmaf instead?
*/
oxb_const oxb_inline float oxb_fma(const float a, const float b, const float c) {
#if defined(__aarch64__)
 float result;
 __asm__ volatile(
 "fmadd %s0, %s1, %s2, %s3"
 : "=w"(result)
 : "w"(a), "w"(b), "w"(c)
 );
 return result;
#else
 return a * b + c;
#endif
}

/*
 use std fabsf instead?
*/
oxb_const oxb_inline float oxb_abs(const float x) {
#if defined(__aarch64__)
 float result;
 __asm__ volatile(
 "fabs %s0, %s1"
 : "=w"(result)
 : "w"(x)
 );
 return result;
#else
 __f_floatbits f;
 f.f = x;
 f.i &= 0x7FFFFFFF;
 return f.f;
#endif
}


oxb_const oxb_inline float oxb_min(const float a, const float b) {
#if defined(__aarch64__)
 float result;
 __asm__ volatile(
 "fmin %s0, %s1, %s2"
 : "=w"(result)
 : "w"(a), "w"(b)
 );
 return result;
#else
 return (a < b) ? a : b;
#endif
}

oxb_const oxb_inline float oxb_max(const float a, const float b) {
#if defined(__aarch64__)
 float result;
 __asm__ volatile(
 "fmax %s0, %s1, %s2"
 : "=w"(result)
 : "w"(a), "w"(b)
 );
 return result;
#else
 return (a > b) ? a : b;
#endif
}



/*
 use std math instead? 
 for round, ceil, floor
*/

oxb_inline float oxb_round(const float x) {
 int a = (int)(x + (x >= 0.0f ? 0.5f : -0.5f));
 return (float)a;
}

/*
 every fractional digits, rounds up towards positive infinity
*/
oxb_inline float oxb_ceil(const float x) {
 int i = (int)x;
 return (float)(x > 0 && x != (float)i) ? i + 1 : i;
}

/*
 every fractional digits, rounds up towards negative infinity
*/
oxb_inline float oxb_floor(const float x) {
 int i = (int)x;
 return (float)(x < 0 && x != (float)i) ? i - 1 : i;
}


oxb_const uintptr_t oxb_alignto(const uintptr_t x, const uint8_t alignment);
oxb_const uint32_t oxb_nextp2(const uint32_t x);


/*
 trigs
*/
void oxb_sincos(const float theta, float *const oxb_restrict s, float *const oxb_restrict c);
oxb_pure float oxb_sin(const float theta);
oxb_pure float oxb_cos(const float theta);
float oxb_tan(const float theta);

/*
 inv trig
*/
oxb_pure float oxb_asin(const float x);
oxb_pure float oxb_acos(const float x);
oxb_pure float oxb_atan(const float x);
oxb_pure float oxb_atan2(const float y, const float x);

/*
 exponential and logarithm
*/
oxb_pure float oxb_log2(const float x);
oxb_pure float oxb_exp2(const float x);
oxb_pure float oxb_tanh(const float x);
oxb_pure float oxb_log(const float x);
oxb_pure float oxb_exp(const float x);

#if defined(__aarch64__)

oxb_inline float oxb_rsqrt(const float x) {
 float o;
 __asm__ volatile (
  "fsqrt %s0, %s1\n"
  : "=w" (o)
  : "w" (x)
 );
 return 1.0f / o;
}

oxb_inline float oxb_sqrt(const float x) {
 float o;
 __asm__ volatile (
  "fsqrt %s0, %s1\n"
  : "=w" (o)
  : "w" (x)
 );
 return o;
}

#elif defined(__x86_64__)

oxb_inline float oxb_rsqrt(const float x) {
 float o;
 __asm__ volatile (
  "movss %[in], %%xmm0 \n"
  "sqrtss %%xmm0, %%xmm0 \n"
  "movss %%xmm0, %[out]"
  : [out] "=m" (o)
  : [in] "m" (x)
  : "xmm0"
 );
 return 1.0f / out;
}

oxb_inline float oxb_sqrt(const float x) {
 float o;
 __asm__ volatile (
  "movss %[in], %%xmm0 \n"
  "sqrtss %%xmm0, %%xmm0 \n"
  "movss %%xmm0, %[out]"
  : [out] "=m" (o)
  : [in] "m" (x)
  : "xmm0"
 );
 return o;
}

#else

/*
 Quake III fast inv sqrt

 magic number total mean error input [1, 100]
 <MAE>       <magic>
 0.00421 --> 0x5f3759df (original magic number)
 0.00428 --> 0x5F378171 (some random guess)
 0.00324 --> 0x5f33cea4 (linear search optimized coeffs ≈23.1% free accuracy increase)
*/
oxb_inline float oxb_rsqrt(const float x) {
 float f, mx, x_half;
 oxb_floatbits bits, x_bits;
 x_bits.f = x;
 bits.i =   0x5f33cea4 - (x_bits.i >> 1);
 mx =       bits.f;
 x_half =   x * 0.5f;
 f =        mx;
 f =        (f * (1.5f - (x_half * f * f)));
 return f;
}


oxb_inline float oxb_sqrt(const float x) {
 float f, mx, x_half;
 oxb_floatbits bits, x_bits;
 x_bits.f = x;
 bits.i =   0x5f33cea4 - (x_bits.i >> 1);
 mx =       bits.f;
 x_half =   x * 0.5f;
 f =        mx;
 f =        (f * (1.5f - (x_half * f * f)));
 return x * f;
}

#endif



#ifdef __cplusplus
}
#endif

#endif


