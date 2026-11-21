#ifndef OXB_COMPLEX_H
#define OXB_COMPLEX_H

#include "oxb1tz/vector.h"

extern const oxb_complex OXB_COMPLEX_IDENTITY;

oxb_inline float oxb_complex_real(const oxb_complex c) {
 return oxb_vec2_getx(c);
}

oxb_inline float oxb_complex_imag(const oxb_complex c) {
 return oxb_vec2_gety(c);
}

oxb_inline oxb_complex oxb_complex_setreal(oxb_complex c, const float x) {
 return oxb_vec2_setx(c, x);
}

oxb_inline oxb_complex oxb_complex_setimag(oxb_complex c, const float x) {
 return oxb_vec2_sety(c, x);
}

/*
 //https://www.mathsisfun.com/algebra/complex-number-multiply.html
 Foil method:
  (a + bi)(c + di) = ac + adi + bci + bdi^2

 Quicker way to multiply complex numbers : 
  (a + bi)(c + di) = (ac − bd) + (ad + bc)i
*/
oxb_inline oxb_complex oxb_complex_mul(const oxb_complex a, const oxb_complex b) {
#ifdef OXB_NEON
 static const uint32x2_t sign = {0x80000000u, 0x00000000u};

 float32x2_t vreal = vdup_n_f32(oxb_complex_real(a));
 float32x2_t vimag = vdup_n_f32(oxb_complex_imag(a));
  
 float32x2_t lm = vmul_f32(vimag, vrev64_f32(b));
 float32x2_t c = vreinterpret_f32_u32(veor_u32(vreinterpret_u32_f32(lm), sign));
 return oxb_vec2_fma(vreal, b, c);
#else
 oxb_complex out = OXB_VEC2_INIT(0.0f, 0.0f);
 out = oxb_complex_setreal(out, oxb_complex_real(a) * oxb_complex_real(b) - oxb_complex_imag(a) * oxb_complex_imag(b) );
 out = oxb_complex_setimag(out, oxb_complex_real(a) * oxb_complex_imag(b) + oxb_complex_imag(a) * oxb_complex_real(b) );
 return out;
#endif
}

/*
 conj(a + bi) = a - bi
*/
oxb_inline oxb_complex oxb_complex_conj(const oxb_complex a) {
#ifdef OXB_NEON
 static const uint32x2_t sign = {0x00000000u, 0x80000000u};
 return vreinterpret_f32_u32(veor_u32(vreinterpret_u32_f32(a), sign));
#else
 oxb_complex out = a;
 out = oxb_complex_setimag(out, -oxb_complex_imag(a));
 return out;
#endif
}

#ifdef __cplusplus
extern "C" {
#endif

oxb_complex oxb_complex_rotate(const float a);
float oxb_complex_angle(const oxb_complex a);

#ifdef __cplusplus
}
#endif

#endif

