#include "oxb1tz/math.h"

/*

 Deterministic math. i guess?

*/


/*
 Robert Jenkins' 32 bit integer hash function
 src : https://gist.github.com/badboy/6267743
*/
oxb_const uint32_t oxb_hash(uint32_t a) {
 a = (a + 0x7ed55d16) + (a << 12);
 a = (a ^ 0xc761c23c) ^ (a >> 19);
 a = (a + 0x165667b1) + (a << 5);
 a = (a + 0xd3a2646c) ^ (a << 9);
 a = (a + 0xfd7046c5) + (a << 3);
 a = (a ^ 0xb55a4f09) ^ (a >> 16);
 return a;
}

/*
 next multiple of
*/
oxb_const uintptr_t oxb_alignto(const uintptr_t x, const uint8_t alignment) {
 return (x + (uintptr_t)alignment - 1) & ~((uintptr_t)alignment - 1);
}

/*
 next power of 2
*/
oxb_const uint32_t oxb_nextp2(const uint32_t x) {
 uint32_t mx = x;
 mx--;
 mx |= mx >> 1;
 mx |= mx >> 2;
 mx |= mx >> 4;
 mx |= mx >> 8;
 mx |= mx >> 16;
 return ++mx;
}



/*
 coeffs polynomial with lowest degree
 just acceptable enough for rough estimation
*/
/* original coeffs */
/*
  3.515696e-02f,
 -4.909662e-01f,
  1.00000000e-00f,
*/

static const float COSINE[3] = {

//  scaled coeffs
 2.97037352e-18f,
-4.51285593e-09f,
 1.00000000e-00f,
};


/*
 scaled by the 
 COSINE[0] *= powf(M_PI*2 / 65536.0, 4)
 COSINE[1] *= powf(M_PI*2 / 65536.0, 2)

*/

/*-------------------------------------------------------------------*/

/*

               point in unit circle
               *
              / |
             /  |
  radius -> /   |
           /    | -> sin
          /     |
         / |    |
 origin *------ *
       angle  | -> cos

  cos(theta) = x-axis of theta angle
  sin(theta) = y-axis of theta angle
*/ 
 /*
  convert to fixed point of (65536 / TAU)
  and use intentional overflow so we dont have to
  use remainder operation.
 */
 
void oxb_sincos(const float theta, float *const oxb_restrict s, float *const oxb_restrict c) {

 uint16_t tx = (uint16_t)(uint32_t)(oxb_abs(theta) * 10430.378350f);
 int16_t ix = (int16_t)tx;
 uint8_t fp = (tx >= 0x3FFF) && (tx <= 0xBFFF);
 ix = fp ? 0x7FFF - ix : ix;
 // convert it back
 // np_real t = ((np_real)ix) * 0.0000958738f;
 
 float t = ((float)ix);

 float x2 = t * t;
 float mx = oxb_fma(oxb_fma(COSINE[0], x2, COSINE[1]), x2, COSINE[2]);
 float mx1 = oxb_sqrt(1.0f - mx * mx);
 *c = fp ? -mx : mx;
 *s = ((theta < 0.0f) ^ (tx > 0x7FFF)) ? -mx1 : mx1;
}

oxb_pure float oxb_sin(const float theta) { 
 uint16_t tx = 0x3FFF - (uint16_t)(uint32_t)(theta * 10430.378350f);
 int16_t ix = tx;
 uint8_t fp = (tx >= 0x3FFF) && (tx <= 0xBFFF);
 ix = fp ? 0x7FFF - ix : ix;
 float t = ((float)ix);
 float x2 = t * t;
 float mx = oxb_fma(oxb_fma(COSINE[0], x2, COSINE[1]), x2, COSINE[2]);
 return fp ? -mx : mx;
}



oxb_pure float oxb_cos(const float theta) { 
 uint16_t tx = (uint16_t)(uint32_t)(theta * 10430.378350f);
 int16_t ix = tx;
 uint8_t fp = (tx >= 0x3FFF) && (tx <= 0xBFFF);
 ix = fp ? 0x7FFF - ix : ix;
 float t = ((float)ix);
 float x2 = t * t;
 float mx = oxb_fma(oxb_fma(COSINE[0], x2, COSINE[1]), x2, COSINE[2]);
 return fp ? -mx : mx;
}


/*
 approximation 1 degree
 interval : [0.0:0.785398163397448309615660845820]
 p(x) = (cos(sqrt(x))-1) / x
 cos(x) = 1 + x2 * p(x2)
 
 coeffs
  0.040591002033280692f,
 -0.49989513679207137f,
 
 scaled 
 0.040591002033280692f * powf(M_PI*2 / 65536.0, 4)
 -0.49989513679207137f * powf(M_PI*2 / 65536.0, 2)
 
*/


/*
 new version of trig function.
 slightly more expensive but it encreases the accuracy
*/
/*
static float COSINE[2] = {
  3.429489862425e-18f,
 -4.594928810527e-09f,
};

void oxb_sincos(const float theta, float *const oxb_restrict s, float *const oxb_restrict c) {
 uint16_t tx = (uint16_t)(uint32_t)(oxb_abs(theta) * 10430.378350f);
 
 int16_t r = (int16_t)tx;
 uint8_t fp = (tx >= 0x3FFF) && (tx <= 0xBFFF);
 r = fp ? 0x7FFF - r : r;
 uint8_t flip = r > 8192;
 r = flip ? (16384 - r) : r;
 
 float mx = ((float)r);
 float x2 = mx * mx;
 float cosine = oxb_fma(x2, oxb_fma(COSINE[0], x2, COSINE[1]), 1.0f);
 float sine = oxb_sqrt(1.0f - cosine * cosine);
 
 float os, oc;
 os = flip ? cosine : sine;
 oc = flip ? sine : cosine;
 os = ((theta < 0.0f) ^ (tx > 0x7FFF)) ? -os : os;
 oc = fp ? -oc : oc;
 *s = os;
 *c = oc;
}



oxb_pure float oxb_sin(const float theta) { 
 uint16_t tx = (uint16_t)(uint32_t)(oxb_abs(theta) * 10430.378350f);
 
 int16_t r = (int16_t)tx;
 uint8_t fp = (tx >= 0x3FFF) && (tx <= 0xBFFF);
 r = fp ? 0x7FFF - r : r;
 uint8_t flip = r > 8192;
 r = flip ? (16384 - r) : r;
 
 float mx = ((float)r);
 float x2 = mx * mx;
 float cosine = oxb_fma(x2, oxb_fma(COSINE[0], x2, COSINE[1]), 1.0f);
 float sine = oxb_sqrt(1.0f - cosine * cosine);
 
 float os;
 os = flip ? cosine : sine;
 return ((theta < 0.0f) ^ (tx > 0x7FFF)) ? -os : os;
}



oxb_pure float oxb_cos(const float theta) { 
 uint16_t tx = (uint16_t)(uint32_t)(oxb_abs(theta) * 10430.378350f);
 
 int16_t r = (int16_t)tx;
 uint8_t fp = (tx >= 0x3FFF) && (tx <= 0xBFFF);
 r = fp ? 0x7FFF - r : r;
 uint8_t flip = r > 8192;
 r = flip ? (16384 - r) : r;
 
 float mx = ((float)r);
 float x2 = mx * mx;
 float cosine = oxb_fma(x2, oxb_fma(COSINE[0], x2, COSINE[1]), 1.0f);
 float sine = oxb_sqrt(1.0f - cosine * cosine);
 
 float oc;
 oc = flip ? sine : cosine;
 return fp ? -oc : oc;
}
*/


float oxb_tan(const float theta) {
	float s, c;
 oxb_sincos(theta, &s, &c);
 return s / c;
}


/*
 arc sine
          • (x, y) = normalized value
         /|
        / |
       /  |
      /   |
     /__  |
    /   | |
   •-------
   ^
  angle
 
  angle = asin(y)
*/

/*
 approximation

 acos(x) = acos(x)/sqrt(1.0 - x), interval : [0, 1]
 asin(x) = pi/2 - acos(x)
*/
static const float ASIN[3] = {
  0.46707077e-01f,
 -0.20212058e-00f,
  1.57021169e-00f,
};

oxb_pure float oxb_asin(const float x) {
 float mx, mx1;
 mx = oxb_min(oxb_abs(x), 1.0f);
 mx1 = oxb_fma(ASIN[0], mx, ASIN[1]);
 mx1 = oxb_fma(mx1, mx, ASIN[2]);
 mx1 = mx1 * oxb_sqrt(1.0f - mx);
 return (1.570796f - ((x < 0.0f) ? 3.14159f - mx1 : mx1));
}

/*
 arc cosine
          • (x, y) = normalized value
         /|
        / |
       /  |
      /   |
     /__  |
    /   | |
   •-------
   ^
  angle
 
  angle = acos(x)
*/
/*
 approxiamtion

 acos(x) = acos(x)/sqrt(1.0 - x), interval : [0, 1]
*/

static const float ACOS[2] = {
 -0.15658276e-00f,
  1.56496682e-00f,
};

oxb_pure float oxb_acos(const float x) {
 float mx, mx1;
 mx = oxb_min(oxb_abs(x), 1.0f);
 mx1 = oxb_fma(ACOS[0], mx, ACOS[1]);
 mx1 = mx1 * oxb_sqrt(1.0f - mx);
 return ((x < 0.0f) ? 3.14159f - mx1 : mx1);
}

/*
 arc tangent
          • (x, y) normalized value
         /|
        / |
       /  |
      /   |
     /__  |
    /   | |
   •-------
     ^
    angle
 
  angle = atan(y/x)
*/

static const float ATAN[2] = {
/*
 -6.038020e-02f,
  1.770501e-01f,
 -3.323025e-01f,
*/
  1.187314e-01f,
 -3.257921e-01f
};

/*
 # arc tangent
                     ________
 atan(x) = asin(x / √1 + x*x ) 

 • alternative (only works on positive x):
                     ________
 atan(x) = acos(1 / √1 + x*x )
*/

/*
 implementation
 
 for |x| <= 1.0
 
 atanPoly(x) = c0 * x + c1 * x² + c2 * x³....
 
 for |x| >= 1.0
 
 atan(x) pi_half - atanPoly(1.0 / x)
*/

oxb_pure float oxb_atan(const float x) {
 float x2, mx, out;
 
 mx = oxb_abs(x);
 if(mx < 1.0f) {
  x2 = mx * mx;
  out = oxb_fma((mx * x2), oxb_fma(ATAN[0], x2, ATAN[1]), mx);
  return oxb_copysign(mx, x);
 }
 mx = 1.0f / mx;
 x2 = mx * mx;
 out = oxb_fma((mx * x2), oxb_fma(ATAN[0], x2, ATAN[1]), mx);
 out = 1.570796f - out;
 return oxb_copysign(out, x);
}

/*
 arc tangent but with separate sine and cosine input and quadrant correction
 because sin/cos = tan
*/
oxb_pure float oxb_atan2(const float y, const float x) {
 float ratio, mx, x2;
 float ax = oxb_abs(x);
 float ay = oxb_abs(y);

 if(ax >= ay) {
  ratio = ay / ax;
  mx = ratio;
  x2 = mx * mx;
  mx = oxb_fma((mx * x2), oxb_fma(ATAN[0], x2, ATAN[1]), mx);
 } else {
  ratio = ax / ay;
  mx = ratio;
  x2 = mx * mx;
  mx = 1.570796f - oxb_fma((mx * x2), oxb_fma(ATAN[0], x2, ATAN[1]), mx);
 }
    
 mx = (x < 0.0f) ? (3.14159f - mx) : mx;
 mx = oxb_copysign(mx, y);
 return mx;
}


/*-------------------------------------------------------------------*/
/*
 log base 2 coeffs
 accurate enough for at least 2-3 decimal digits.
*/
static const float LOG2[5] = {
 -8.161580e-02f,
  6.451423e-01f,
 -2.120675e-00f,
  4.070090e-00f,
 -2.512854e-00f,
};
/*

 
                                        ______
                                    __ -
  logarithmic growth ->         __ -
                            _ -
                           /
                          -
                         /
    udefined negatives  |
      -------|-------|--}* -> point exactly to one
                       /
                       | -> below 1.0 are negative values
                       /
   
-> gradually goes to zero with negative sign


  log2(x) = ln(x) / ln(2)
 
*/
oxb_pure float oxb_log2(const float x) {
 oxb_floatbits mantissa, bits;
 float ma, mx;
 
 if(x >= 1.0f && x < 2.0f) {
  ma = x;
  return oxb_fma(oxb_fma(oxb_fma(oxb_fma(LOG2[0], ma, LOG2[1]), ma, LOG2[2]), ma, LOG2[3]), ma, LOG2[4]);
 } else if(x < 1.0f) {
 	bits.f = 1.0f / x;
  mantissa.i = 1065353216U | (bits.i & 0x007FFFFF);
  ma = mantissa.f;
  mx = oxb_fma(oxb_fma(oxb_fma(oxb_fma(LOG2[0], ma, LOG2[1]), ma, LOG2[2]), ma, LOG2[3]), ma, LOG2[4]);
  mx = (float)((int32_t)(bits.i >> 23) - 127) + mx;
  return -mx;
 }
 
 bits.f = x;
 mantissa.i = 1065353216U | (bits.i & 0x007FFFFF);
 ma = mantissa.f;
 mx = oxb_fma(oxb_fma(oxb_fma(oxb_fma(LOG2[0], ma, LOG2[1]), ma, LOG2[2]), ma, LOG2[3]), ma, LOG2[4]);
 mx = (float)((int32_t)(bits.i >> 23) - 127) + mx;
 return mx;
}

/*-------------------------------------------------------------------*/
/*
 exponential base 2 coeffs
 accurate enough for at least 2-3 decimal digits.
*/
static const float EXP2[5] = {
 0.96857114e-02f,
 0.55922025e-01f,
 0.24021865e-00f,
 0.69312103e-00f,
 1.00000015e-00f,
};

/*

 
                exponential growth

                        /
                       |
                      /   
                     |
                    /
                ---*  <- point exactly to one
              __
    _______---      <- negative values are always below one

    exp2(x) = exp(x * ln(2))
*/

oxb_pure float oxb_exp2(const float x) { 
 float mx, a;
	oxb_floatbits bits;
 int32_t whole;

 if(oxb_abs(x) < 0.5f) {
 	a = x;
  return oxb_fma(oxb_fma(oxb_fma(oxb_fma(EXP2[0], a, EXP2[1]), a, EXP2[2]), a, EXP2[3]), a, EXP2[4]);
 }
	a = x;
 whole = (float)(a - 0.5f);
 a = a - ((float)whole);
 bits.i = (uint32_t)((int32_t)(127 + whole) << 23);
 mx = oxb_fma(oxb_fma(oxb_fma(oxb_fma(EXP2[0], a, EXP2[1]), a, EXP2[2]), a, EXP2[3]), a, EXP2[4]);
 return bits.f * mx;
}

/*-------------------------------------------------------------------*/

/*
 hyperbolic tangent 

 1.0 ---------------------------------
                    ____
                ___
            ___
         __
       __
     _
   -
  -
-------------------------------------
 for negative values it is just '-tanh(abs(x))'
 
 as the value increases it slowly approaches 1.0
 useful for smooth transition with clamped limits.

 tanh(x) = (e ^ x - 1.0) / (e ^ x + 1.0);

*/
/*
oxb_pure float oxb_tanh(const float x) {
 if(oxb_abs(x) >= 45.0f)
  return oxb_copysign(1.0f, x);
 const float log2_e = 1.44269504f;
 float exponential = oxb_exp2(2.0 * x * log2_e);
 return (exponential - 1.0) / (exponential + 1.0);
}
*/
static const float TANH[6] = {
 9.49274892e-02f,
-3.29312635e-01f,

 4.78187872e-02f,
-3.72573327e-01f,
 9.85379375e-01f,
 1.03154878e-01f,
};

oxb_pure float oxb_tanh(const float x) {
 float mx = oxb_abs(x);
 if(mx < 1.0) {
  float x2 = mx * mx;
  float mm = oxb_fma((x2 * mx), oxb_fma(TANH[0], x2, TANH[1]), mx);
  return oxb_copysign(mm, x);
 } else if(mx < 3.0) {
  float mm = oxb_fma(oxb_fma(oxb_fma(TANH[2], mx, TANH[3]), mx, TANH[4]), mx, TANH[5]);
  return oxb_copysign(mm, x);
 } 
 return oxb_copysign(1.0f, x);
}


/*
 base e log and exp
*/
oxb_pure float oxb_log(const float x) {
 const float inv_log2_e = 0.69314718f;
 return oxb_log2(x) * inv_log2_e;
}

oxb_pure float oxb_exp(const float x) {
 const float log2_e = 1.44269504f;
 return oxb_exp2(x * log2_e);
}

