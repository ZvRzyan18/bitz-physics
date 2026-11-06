#ifndef OXB_VECTOR_H
#define OXB_VECTOR_H

#include "oxb1tz/types.h"
#include "oxb1tz/math.h"
#include "oxb1tz/hint.h"


/**********************************************
*                   Initializers
**********************************************/
#ifdef OXB_NEON
#define OXB_VEC2_INIT(a, b) ((float32x2_t){a, b})
#define OXB_VEC3_INIT(a, b, c) ((float32x4_t){a, b, c, 0.0f})
#define OXB_VEC4_INIT(a, b, c, d) ((float32x4_t){a, b, c, d})
#else
#define OXB_VEC2_INIT(a, b) ((oxb_vec2){a, b})
#define OXB_VEC3_INIT(a, b, c) ((oxb_vec3){a, b, c})
#define OXB_VEC4_INIT(a, b, c, d) ((oxb_vec4){a, b, c, d})
#endif


/**********************************************
*                   Vector 2
**********************************************/
oxb_inline oxb_vec2 oxb_vec2_splat(const float x) {
#ifdef OXB_NEON
 return vdup_n_f32(x);
#else
 oxb_vec2 m;
 m._arr[0] = x;
 m._arr[1] = x;
 return m;
#endif
}

oxb_inline float oxb_vec2_get(const oxb_vec2 m, const size_t s) {
#ifdef OXB_NEON
 switch(s) {
  case 0:
   return vget_lane_f32(m, 0);
  break;
  case 1:
   return vget_lane_f32(m, 1);
  break;
 }
 assert(0);
 oxb_unreachable();
 return oxb_nan(); /* return something to remove a warning */
#else
 assert(s < 2);
 return m._arr[s];
#endif
}

oxb_inline float oxb_vec2_getx(const oxb_vec2 m) {
#ifdef OXB_NEON
 return vget_lane_f32(m, 0);
#else
 return oxb_vec2_get(m, 0);
#endif
}

oxb_inline float oxb_vec2_gety(const oxb_vec2 m) {
#ifdef OXB_NEON
 return vget_lane_f32(m, 1);
#else
 return oxb_vec2_get(m, 1);
#endif
}


oxb_inline oxb_vec2 oxb_vec2_set(oxb_vec2 m, const size_t s, const float x) {
#ifdef OXB_NEON
 switch(s) {
  case 0:
   return vset_lane_f32(x, m, 0);
  break;
  case 1:
   return vset_lane_f32(x, m, 1);
  break;
 }
 assert(0);
 oxb_unreachable();
 return oxb_vec2_splat(oxb_nan()); /* return something to remove a warning */
#else
 assert(s < 2);
 m._arr[s] = x;
 return m;
#endif
}


oxb_inline oxb_vec2 oxb_vec2_setx(const oxb_vec2 m, const float x) {
#ifdef OXB_NEON
 return vset_lane_f32(x, m, 0);
#else
 return oxb_vec2_set(m, 0, x);
#endif
}


oxb_inline oxb_vec2 oxb_vec2_sety(const oxb_vec2 m, const float x) {
#ifdef OXB_NEON
 return vset_lane_f32(x, m, 1);
#else
 return oxb_vec2_set(m, 1, x);
#endif
}


oxb_inline oxb_vec2 oxb_vec2_load(const float *const ptr) {
#ifdef OXB_NEON
 return vld1_f32(ptr);
#else
 oxb_vec2 m;
 m._arr[0] = ptr[0];
 m._arr[1] = ptr[1];
 return m;
#endif
}


oxb_inline void oxb_vec2_store(float *const ptr, const oxb_vec2 v) {
#ifdef OXB_NEON
 vst1_f32(ptr, v);
#else
 ptr[0] = oxb_vec2_get(v, 0);
 ptr[1] = oxb_vec2_get(v, 1);
#endif
}

oxb_inline oxb_vec2 oxb_vec2_swaplane(const oxb_vec2 x) {
#ifdef OXB_NEON
 return vrev64_f32(x);
#else
 oxb_vec2 m;
 m._arr[1] = oxb_vec2_getx(x);
 m._arr[0] = oxb_vec2_gety(x);
 return m;
#endif
}

oxb_inline oxb_vec2 oxb_vec2_neg(const oxb_vec2 a) {
#ifdef OXB_NEON
 return vneg_f32(a);
#else
 oxb_vec2 out;
 out._arr[0] = -oxb_vec2_getx(a);
 out._arr[1] = -oxb_vec2_gety(a);
 return out;
#endif
}

oxb_inline oxb_vec2 oxb_vec2_abs(const oxb_vec2 a) {
#ifdef OXB_NEON
 return vabs_f32(a);
#else
 oxb_vec2 out;
 out._arr[0] = oxb_abs(oxb_vec2_getx(a));
 out._arr[1] = oxb_abs(oxb_vec2_gety(a));
 return out;
#endif
}

oxb_inline oxb_vec2 oxb_vec2_min(const oxb_vec2 a, const oxb_vec2 b) {
#ifdef OXB_NEON
 return vmin_f32(a, b);
#else
 oxb_vec2 out;
 out._arr[0] = oxb_min(oxb_vec2_getx(a), oxb_vec2_getx(b));
 out._arr[1] = oxb_min(oxb_vec2_gety(a), oxb_vec2_gety(b));
 return out;
#endif
}

oxb_inline oxb_vec2 oxb_vec2_max(const oxb_vec2 a, const oxb_vec2 b) {
#ifdef OXB_NEON
 return vmax_f32(a, b);
#else
 oxb_vec2 out;
 out._arr[0] = oxb_max(oxb_vec2_getx(a), oxb_vec2_getx(b));
 out._arr[1] = oxb_max(oxb_vec2_gety(a), oxb_vec2_gety(b));
 return out;
#endif
}


oxb_inline oxb_vec2 oxb_vec2_add(const oxb_vec2 a, const oxb_vec2 b) {
#ifdef OXB_NEON
 return vadd_f32(a, b);
#else
 oxb_vec2 out;
 out._arr[0] = oxb_vec2_getx(a) + oxb_vec2_getx(b);
 out._arr[1] = oxb_vec2_gety(a) + oxb_vec2_gety(b);
 return out;
#endif
}


oxb_inline oxb_vec2 oxb_vec2_sub(const oxb_vec2 a, const oxb_vec2 b) {
#ifdef OXB_NEON
 return vsub_f32(a, b);
#else
 oxb_vec2 out;
 out._arr[0] = oxb_vec2_getx(a) - oxb_vec2_getx(b);
 out._arr[1] = oxb_vec2_gety(a) - oxb_vec2_gety(b);
 return out;
#endif
}

oxb_inline oxb_vec2 oxb_vec2_mul(const oxb_vec2 a, const oxb_vec2 b) {
#ifdef OXB_NEON
 return vmul_f32(a, b);
#else
 oxb_vec2 out;
 out._arr[0] = oxb_vec2_getx(a) * oxb_vec2_getx(b);
 out._arr[1] = oxb_vec2_gety(a) * oxb_vec2_gety(b);
 return out;
#endif
}

oxb_inline oxb_vec2 oxb_vec2_fma(const oxb_vec2 a, const oxb_vec2 b, const oxb_vec2 c) {
#ifdef OXB_NEON
 /* vfma(64-bit),,, vmla(32-bit) */
 return oxb_neon_fma(c, a, b);
#else
 oxb_vec2 out;
 out._arr[0] = oxb_vec2_getx(a) * oxb_vec2_getx(b) + oxb_vec2_getx(c);
 out._arr[1] = oxb_vec2_gety(a) * oxb_vec2_gety(b) + oxb_vec2_gety(c);
 return out;
#endif
}


oxb_inline oxb_vec2 oxb_vec2_div(const oxb_vec2 a, const oxb_vec2 b) {
#ifdef OXB_NEON
#ifdef OXB_NEON_64
 return vdiv_f32(a, b);
#else
 float32x2_t reciprocal = vrecpe_f32(b);
 reciprocal = vmul_f32(reciprocal, vrecps_f32(b, reciprocal));
 reciprocal = vmul_f32(reciprocal, vrecps_f32(b, reciprocal));
 return vmul_f32(a, reciprocal);
#endif
#else
 oxb_vec2 out;
 out._arr[0] = oxb_vec2_getx(a) / oxb_vec2_getx(b);
 out._arr[1] = oxb_vec2_gety(a) / oxb_vec2_gety(b);
 return out;
#endif
}


oxb_inline float oxb_vec2_dot(const oxb_vec2 a, const oxb_vec2 b) {
#ifdef OXB_NEON
#ifdef OXB_NEON_64
 return vaddv_f32(vmul_f32(a, b));
#else
 float32x2_t p = vmul_f32(a, b);
 return vget_lane_f32(vpadd_f32(p, p), 0);
#endif
#else
 return oxb_vec2_getx(a) * oxb_vec2_getx(b) + oxb_vec2_gety(a) * oxb_vec2_gety(b);
#endif
}


oxb_inline oxb_vec2 oxb_vec2_normalize(const oxb_vec2 a) {
 float inv_len = oxb_rsqrt(oxb_vec2_dot(a, a));
 return oxb_vec2_mul(a, oxb_vec2_splat(inv_len));
}


oxb_inline oxb_vec2 oxb_vec2_rperp(const oxb_vec2 a) {
#ifdef OXB_NEON
 static const uint32x2_t sign = {0x00000000u, 0x80000000u};
 return vreinterpret_f32_u32(veor_u32(vreinterpret_u32_f32(vrev64_f32(a)), sign));
#else
 oxb_vec2 out;
 out._arr[0] = oxb_vec2_gety(a);
 out._arr[1] = -oxb_vec2_getx(a);
 return out;
#endif
}

oxb_inline oxb_vec2 oxb_vec2_lperp(const oxb_vec2 a) {
#ifdef OXB_NEON
 static const uint32x2_t sign = {0x80000000u, 0x00000000u};
 return vreinterpret_f32_u32(veor_u32(vreinterpret_u32_f32(vrev64_f32(a)), sign));
#else
 oxb_vec2 out;
 out._arr[0] = -oxb_vec2_gety(a);
 out._arr[1] = oxb_vec2_getx(a);
 return out;
#endif
}

oxb_inline float oxb_vec2_cross(const oxb_vec2 a, const oxb_vec2 b) {
#ifdef OXB_NEON
 float32x2_t p = vmul_f32(a, vrev64_f32(b));
 return vget_lane_f32(vsub_f32(p, vrev64_f32(p)), 0);
#else
 return oxb_fma(oxb_vec2_getx(a), oxb_vec2_gety(b), -oxb_vec2_gety(a) * oxb_vec2_getx(b));
#endif
}

oxb_inline oxb_vec2 oxb_vec2_scross(const float a, const oxb_vec2 b) {
#ifdef OXB_NEON
 static const uint32x2_t sign = {0x80000000u, 0x00000000u};
 float32x2_t av = vreinterpret_f32_u32(veor_u32(vreinterpret_u32_f32(vdup_n_f32(a)), sign));
 return vmul_f32(vrev64_f32(b), av);
#else
 oxb_vec2 out;
 out._arr[0] = -a * oxb_vec2_gety(b);
 out._arr[1] = a * oxb_vec2_getx(b);
 return out;
#endif
}



/**********************************************
*                   Vector 3
**********************************************/

oxb_inline oxb_vec3 oxb_vec3_splat(const float x) {
#ifdef OXB_NEON
 return vdupq_n_f32(x);
#else
 oxb_vec3 m;
 m._arr[0] = x;
 m._arr[1] = x;
 m._arr[2] = x;
 return m;
#endif
}


oxb_inline float oxb_vec3_get(const oxb_vec3 m, const size_t s) {
#ifdef OXB_NEON
 switch(s) {
  case 0:
   return vgetq_lane_f32(m, 0);
  break;
  case 1:
   return vgetq_lane_f32(m, 1);
  break;
  case 2:
   return vgetq_lane_f32(m, 2);
  break;
 }
 assert(0);
 oxb_unreachable();
 return oxb_nan(); /* return something to remove a warning */
#else
 assert(s < 3);
 return m._arr[s];
#endif
}

oxb_inline float oxb_vec3_getx(const oxb_vec3 m) {
#ifdef OXB_NEON
 return vgetq_lane_f32(m, 0);
#else
 return oxb_vec3_get(m, 0);
#endif
}

oxb_inline float oxb_vec3_gety(const oxb_vec3 m) {
#ifdef OXB_NEON
 return vgetq_lane_f32(m, 1);
#else
 return oxb_vec3_get(m, 1);
#endif
}


oxb_inline float oxb_vec3_getz(const oxb_vec3 m) {
#ifdef OXB_NEON
 return vgetq_lane_f32(m, 2);
#else
 return oxb_vec3_get(m, 2);
#endif
}

oxb_inline oxb_vec3 oxb_vec3_set(oxb_vec3 m, const size_t s, const float x) {
#ifdef OXB_NEON
 switch(s) {
  case 0:
   return vsetq_lane_f32(x, m, 0);
  break;
  case 1:
   return vsetq_lane_f32(x, m, 1);
  break;
  case 2:
   return vsetq_lane_f32(x, m, 2);
  break;
 }
 assert(0);
 oxb_unreachable();
 return oxb_vec3_splat(oxb_nan()); /* return something to remove a warning */
#else
 assert(s < 3);
 m._arr[s] = x;
 return m;
#endif
}


oxb_inline oxb_vec3 oxb_vec3_setx(const oxb_vec3 m, const float x) {
#ifdef OXB_NEON
 return vsetq_lane_f32(x, m, 0);
#else
 return oxb_vec3_set(m, 0, x);
#endif
}


oxb_inline oxb_vec3 oxb_vec3_sety(const oxb_vec3 m, const float x) {
#ifdef OXB_NEON
 return vsetq_lane_f32(x, m, 1);
#else
 return oxb_vec3_set(m, 1, x);
#endif
}


oxb_inline oxb_vec3 oxb_vec3_setz(const oxb_vec3 m, const float x) {
#ifdef OXB_NEON
 return vsetq_lane_f32(x, m, 2);
#else
 return oxb_vec3_set(m, 2, x);
#endif
}



oxb_inline oxb_vec3 oxb_vec3_load(const float *const ptr) {
#ifdef OXB_NEON
 float32x2_t low = vld1_f32(ptr);
 float32x2_t high = {0.0f, 0.0f};
 high = vset_lane_f32(ptr[2], high, 0);
 return vcombine_f32(low, high);
#else
 oxb_vec3 m;
 m._arr[0] = ptr[0];
 m._arr[1] = ptr[1];
 m._arr[2] = ptr[2];
 return m;
#endif
}

oxb_inline oxb_vec3 oxb_vec3_paddedload(const float *const ptr) {
#ifdef OXB_NEON
 return vld1q_f32(ptr);
#else
 oxb_vec3 m;
 m._arr[0] = ptr[0];
 m._arr[1] = ptr[1];
 m._arr[2] = ptr[2];
 return m;
#endif
}

oxb_inline void oxb_vec3_store(float *const ptr, const oxb_vec3 v) {
#ifdef OXB_NEON
 vst1_f32(ptr, vget_low_f32(v));
 vst1_lane_f32(&ptr[2], vget_high_f32(v), 0);
#else
 ptr[0] = oxb_vec3_get(v, 0);
 ptr[1] = oxb_vec3_get(v, 1);
 ptr[2] = oxb_vec3_get(v, 2);
#endif
}

oxb_inline void oxb_vec3_paddedstore(float *const ptr, const oxb_vec3 v) {
#ifdef OXB_NEON
 vst1q_f32(ptr, v);
#else
 ptr[0] = oxb_vec3_get(v, 0);
 ptr[1] = oxb_vec3_get(v, 1);
 ptr[2] = oxb_vec3_get(v, 2);
#endif
}


oxb_inline oxb_vec3 oxb_vec3_perm(const oxb_vec3 x, const uint8_t a, const uint8_t b, const uint8_t c) {
 assert(a < 3);
 assert(b < 3);
 assert(c < 3);
#ifdef OXB_NEON
static const uint32_t perm_tbl[4] = {
 0x03020100,
 0x07060504,
 0x0B0A0908,
 0x0F0E0D0C,
};
#ifdef OXB_NEON_64
 const uint8x16_t bytes = vreinterpretq_u8_f32(x);
 uint32x4_t tbl = {perm_tbl[a], perm_tbl[b], perm_tbl[c], perm_tbl[3]};
 uint8x16_t idx = vreinterpretq_u8_u32(tbl);
 return vreinterpretq_f32_u8(vqtbl1q_u8(bytes, idx));
#else
 const uint8x16_t bytes = vreinterpretq_u8_f32(x);
 uint32x4_t tbl = {perm_tbl[a], perm_tbl[b], perm_tbl[c], perm_tbl[3]};
 const uint8x16_t idx = vreinterpretq_u8_u32(tbl);

 uint8x8x2_t table;
 table.val[0] = vget_low_u8(bytes);
 table.val[1] = vget_high_u8(bytes);

 const uint8x8_t lo = vtbl2_u8(table, vget_low_u8(idx));
 const uint8x8_t hi = vtbl2_u8(table, vget_high_u8(idx));
 const uint8x16_t result = vcombine_u8(lo, hi);
 return vreinterpretq_f32_u8(result);
#endif
#else
 return OXB_VEC3_INIT(oxb_vec3_get(x, a), oxb_vec3_get(x, b), oxb_vec3_get(x, c));
#endif
}


oxb_inline oxb_vec2 oxb_vec3_low(const oxb_vec3 a) {
#ifdef OXB_NEON
 return vget_low_f32(a);
#else
 oxb_vec2 m;
 m._arr[0] = oxb_vec3_getx(a);
 m._arr[1] = oxb_vec3_gety(a);
 return m;
#endif
}


oxb_inline oxb_vec3 oxb_vec3_neg(const oxb_vec3 a) {
#ifdef OXB_NEON
 return vnegq_f32(a);
#else
 oxb_vec3 out;
 out._arr[0] = -oxb_vec3_getx(a);
 out._arr[1] = -oxb_vec3_gety(a);
 out._arr[2] = -oxb_vec3_getz(a);
 return out;
#endif
}

oxb_inline oxb_vec3 oxb_vec3_abs(const oxb_vec3 a) {
#ifdef OXB_NEON
 return vabsq_f32(a);
#else
 oxb_vec3 out;
 out._arr[0] = oxb_abs(oxb_vec3_getx(a));
 out._arr[1] = oxb_abs(oxb_vec3_gety(a));
 out._arr[2] = oxb_abs(oxb_vec3_getz(a));
 return out;
#endif
}

oxb_inline oxb_vec3 oxb_vec3_add(const oxb_vec3 a, const oxb_vec3 b) {
#ifdef OXB_NEON
 return vaddq_f32(a, b);
#else
 oxb_vec3 out;
 out._arr[0] = oxb_vec3_getx(a) + oxb_vec3_getx(b);
 out._arr[1] = oxb_vec3_gety(a) + oxb_vec3_gety(b);
 out._arr[2] = oxb_vec3_getz(a) + oxb_vec3_getz(b);
 return out;
#endif
}


oxb_inline oxb_vec3 oxb_vec3_sub(const oxb_vec3 a, const oxb_vec3 b) {
#ifdef OXB_NEON
 return vsubq_f32(a, b);
#else
 oxb_vec3 out;
 out._arr[0] = oxb_vec3_getx(a) - oxb_vec3_getx(b);
 out._arr[1] = oxb_vec3_gety(a) - oxb_vec3_gety(b);
 out._arr[2] = oxb_vec3_getz(a) - oxb_vec3_getz(b);
 return out;
#endif
}

oxb_inline oxb_vec3 oxb_vec3_mul(const oxb_vec3 a, const oxb_vec3 b) {
#ifdef OXB_NEON
 return vmulq_f32(a, b);
#else
 oxb_vec3 out;
 out._arr[0] = oxb_vec3_getx(a) * oxb_vec3_getx(b);
 out._arr[1] = oxb_vec3_gety(a) * oxb_vec3_gety(b);
 out._arr[2] = oxb_vec3_getz(a) * oxb_vec3_getz(b); 
 return out;
#endif
}

oxb_inline oxb_vec3 oxb_vec3_fma(const oxb_vec3 a, const oxb_vec3 b, const oxb_vec3 c) {
#ifdef OXB_NEON
 /* vfma(64-bit),,, vmla(32-bit) */
 return oxb_neon_fmaq(c, a, b);
#else
 oxb_vec3 out;
 out._arr[0] = oxb_vec3_getx(a) * oxb_vec3_getx(b) + oxb_vec3_getx(c);
 out._arr[1] = oxb_vec3_gety(a) * oxb_vec3_gety(b) + oxb_vec3_gety(c);
 out._arr[2] = oxb_vec3_getz(a) * oxb_vec3_getz(b) + oxb_vec3_getz(c);
 return out;
#endif
}


oxb_inline oxb_vec3 oxb_vec3_div(const oxb_vec3 a, const oxb_vec3 b) {
#ifdef OXB_NEON
#ifdef OXB_NEON_64
 return vdivq_f32(a, b);
#else
 float32x4_t reciprocal = vrecpeq_f32(b);
 reciprocal = vmulq_f32(reciprocal, vrecpsq_f32(b, reciprocal));
 reciprocal = vmulq_f32(reciprocal, vrecpsq_f32(b, reciprocal));
 return vmulq_f32(a, reciprocal);
#endif
#else
 oxb_vec3 out;
 out._arr[0] = oxb_vec3_getx(a) / oxb_vec3_getx(b);
 out._arr[1] = oxb_vec3_gety(a) / oxb_vec3_gety(b);
 out._arr[2] = oxb_vec3_getz(a) / oxb_vec3_getz(b);
 return out;
#endif
}

oxb_inline float oxb_vec3_dot(const oxb_vec3 a, const oxb_vec3 b) {
#ifdef OXB_NEON
 float32x4_t ma = vsetq_lane_f32(0.0f, a, 3);
#ifdef OXB_NEON_64
 float32x4_t p = vmulq_f32(ma, b);
 return vaddvq_f32(p);
#else
 float32x4_t p = vmulq_f32(ma, b);
 p = vpaddq_f32(p, p);
 return vgetq_lane_f32(vpaddq_f32(p, p), 0);
#endif
#else
 return oxb_vec3_getx(a) * oxb_vec3_getx(b) + oxb_vec3_gety(a) * oxb_vec3_gety(b) + oxb_vec3_getz(a) * oxb_vec3_getz(b);
#endif
}


oxb_inline oxb_vec3 oxb_vec3_normalize(const oxb_vec3 a) {
 float inv_len = oxb_rsqrt(oxb_vec3_dot(a, a));
 return oxb_vec3_mul(a, oxb_vec3_splat(inv_len));
}


oxb_inline oxb_vec3 oxb_vec3_cross(const oxb_vec3 a, const oxb_vec3 b) {
 oxb_vec3 ayzx = oxb_vec3_perm(a, 1, 2, 0);
 oxb_vec3 azxy = oxb_vec3_perm(a, 2, 0, 1);
 oxb_vec3 byzx = oxb_vec3_perm(b, 1, 2, 0);
 oxb_vec3 bzxy = oxb_vec3_perm(b, 2, 0, 1);
 return oxb_vec3_fma(ayzx, bzxy, oxb_vec3_neg(oxb_vec3_mul(azxy, byzx)));
}


/**********************************************
*                   Vector 4
**********************************************/

oxb_inline oxb_vec4 oxb_vec4_splat(const float x) {
#ifdef OXB_NEON
 return vdupq_n_f32(x);
#else
 oxb_vec4 m;
 m._arr[0] = x;
 m._arr[1] = x;
 m._arr[2] = x;
 m._arr[3] = x;
 return m;
#endif
}


oxb_inline float oxb_vec4_get(const oxb_vec4 m, const size_t s) {
#ifdef OXB_NEON
 switch(s) {
  case 0:
   return vgetq_lane_f32(m, 0);
  break;
  case 1:
   return vgetq_lane_f32(m, 1);
  break;
  case 2:
   return vgetq_lane_f32(m, 2);
  break;
  case 3:
   return vgetq_lane_f32(m, 3);
  break;
 }
 assert(0);
 oxb_unreachable();
 return oxb_nan(); /* return something to remove a warning */
#else
 assert(s < 4);
 return m._arr[s];
#endif
}

oxb_inline float oxb_vec4_getx(const oxb_vec4 m) {
#ifdef OXB_NEON
 return vgetq_lane_f32(m, 0);
#else
 return oxb_vec4_get(m, 0);
#endif
}

oxb_inline float oxb_vec4_gety(const oxb_vec4 m) {
#ifdef OXB_NEON
 return vgetq_lane_f32(m, 1);
#else
 return oxb_vec4_get(m, 1);
#endif
}


oxb_inline float oxb_vec4_getz(const oxb_vec4 m) {
#ifdef OXB_NEON
 return vgetq_lane_f32(m, 2);
#else
 return oxb_vec4_get(m, 2);
#endif
}

oxb_inline float oxb_vec4_getw(const oxb_vec4 m) {
#ifdef OXB_NEON
 return vgetq_lane_f32(m, 3);
#else
 return oxb_vec4_get(m, 3);
#endif
}

oxb_inline oxb_vec4 oxb_vec4_set(oxb_vec4 m, const size_t s, const float x) {
#ifdef OXB_NEON
 switch(s) {
  case 0:
   return vsetq_lane_f32(x, m, 0);
  break;
  case 1:
   return vsetq_lane_f32(x, m, 1);
  break;
  case 2:
   return vsetq_lane_f32(x, m, 2);
  break;
  case 3:
   return vsetq_lane_f32(x, m, 3);
  break;
 }
 assert(0);
 oxb_unreachable();
 return oxb_vec4_splat(oxb_nan()); /* return something to remove a warning */
#else
 assert(s < 4);
 m._arr[s] = x;
 return m;
#endif
}


oxb_inline oxb_vec4 oxb_vec4_setx(const oxb_vec4 m, const float x) {
#ifdef OXB_NEON
 return vsetq_lane_f32(x, m, 0);
#else
 return oxb_vec4_set(m, 0, x);
#endif
}


oxb_inline oxb_vec4 oxb_vec4_sety(const oxb_vec4 m, const float x) {
#ifdef OXB_NEON
 return vsetq_lane_f32(x, m, 1);
#else
 return oxb_vec4_set(m, 1, x);
#endif
}


oxb_inline oxb_vec4 oxb_vec4_setz(const oxb_vec4 m, const float x) {
#ifdef OXB_NEON
 return vsetq_lane_f32(x, m, 2);
#else
 return oxb_vec4_set(m, 2, x);
#endif
}

oxb_inline oxb_vec4 oxb_vec4_setw(const oxb_vec4 m, const float x) {
#ifdef OXB_NEON
 return vsetq_lane_f32(x, m, 3);
#else
 return oxb_vec4_set(m, 3, x);
#endif
}


oxb_inline oxb_vec4 oxb_vec4_load(const float *const ptr) {
#ifdef OXB_NEON
 return vld1q_f32(ptr);
#else
 oxb_vec4 m;
 m._arr[0] = ptr[0];
 m._arr[1] = ptr[1];
 m._arr[2] = ptr[2];
 m._arr[3] = ptr[3];
 return m;
#endif
}

oxb_inline void oxb_vec4_store(float *const ptr, const oxb_vec4 v) {
#ifdef OXB_NEON
 vst1q_f32(ptr, v);
#else
 ptr[0] = oxb_vec4_get(v, 0);
 ptr[1] = oxb_vec4_get(v, 1);
 ptr[2] = oxb_vec4_get(v, 2);
 ptr[3] = oxb_vec4_get(v, 3);
#endif
}

oxb_inline oxb_vec2 oxb_vec4_low(const oxb_vec4 a) {
#ifdef OXB_NEON
 return vget_low_f32(a);
#else
 oxb_vec2 m;
 m._arr[0] = oxb_vec4_getx(a);
 m._arr[1] = oxb_vec4_gety(a);
 return m;
#endif
}

oxb_inline oxb_vec2 oxb_vec4_high(const oxb_vec4 a) {
#ifdef OXB_NEON
 return vget_high_f32(a);
#else
 oxb_vec2 m;
 m._arr[0] = oxb_vec4_getz(a);
 m._arr[1] = oxb_vec4_getw(a);
 return m;
#endif
}


oxb_inline oxb_vec4 oxb_vec4_fromvec3(const oxb_vec3 a) {
#ifdef OXB_NEON
 return a;
#else
 oxb_vec4 m;
 m._arr[0] = oxb_vec3_getx(a);
 m._arr[1] = oxb_vec3_gety(a);
 m._arr[2] = oxb_vec3_getz(a);
 return m;
#endif
}

oxb_inline oxb_vec3 oxb_vec4_tovec3(const oxb_vec4 a) {
#ifdef OXB_NEON
 return a;
#else
 oxb_vec3 m;
 m._arr[0] = oxb_vec4_getx(a);
 m._arr[1] = oxb_vec4_gety(a);
 m._arr[2] = oxb_vec4_getz(a);
 return m;
#endif
}

oxb_inline oxb_vec4 oxb_vec4_combine(const oxb_vec2 a, const oxb_vec2 b) {
#ifdef OXB_NEON
 return vcombine_f32(a, b);
#else
 oxb_vec4 m;
 m._arr[0] = oxb_vec2_getx(a);
 m._arr[1] = oxb_vec2_gety(a);
 m._arr[2] = oxb_vec2_getx(b);
 m._arr[3] = oxb_vec2_gety(b);
 return m;
#endif
}


oxb_inline oxb_vec4 oxb_vec4_perm(const oxb_vec4 x, const uint8_t a, const uint8_t b, const uint8_t c, const uint8_t d) {
 assert(a < 4);
 assert(b < 4);
 assert(c < 4);
 assert(d < 4);
#ifdef OXB_NEON
static const uint32_t perm_tbl[4] = {
 0x03020100,
 0x07060504,
 0x0B0A0908,
 0x0F0E0D0C,
};
#ifdef OXB_NEON_64
 const uint8x16_t bytes = vreinterpretq_u8_f32(x);
 uint32x4_t tbl = {perm_tbl[a], perm_tbl[b], perm_tbl[c], perm_tbl[d]};
 uint8x16_t idx = vreinterpretq_u8_u32(tbl);
 return vreinterpretq_f32_u8(vqtbl1q_u8(bytes, idx));
#else
 const uint8x16_t bytes = vreinterpretq_u8_f32(x);
 uint32x4_t tbl = {perm_tbl[a], perm_tbl[b], perm_tbl[c], perm_tbl[d]};
 const uint8x16_t idx = vreinterpretq_u8_u32(tbl);

 uint8x8x2_t table;
 table.val[0] = vget_low_u8(bytes);
 table.val[1] = vget_high_u8(bytes);

 const uint8x8_t lo = vtbl2_u8(table, vget_low_u8(idx));
 const uint8x8_t hi = vtbl2_u8(table, vget_high_u8(idx));
 const uint8x16_t result = vcombine_u8(lo, hi);
 return vreinterpretq_f32_u8(result);
#endif
#else
 return OXB_VEC4_INIT(oxb_vec4_get(x, a), oxb_vec4_get(x, b), oxb_vec4_get(x, c), oxb_vec4_get(x, d));
#endif
}

oxb_inline oxb_vec4 oxb_vec4_neg(const oxb_vec4 a) {
#ifdef OXB_NEON
 return vnegq_f32(a);
#else
 oxb_vec4 out;
 out._arr[0] = -oxb_vec4_getx(a);
 out._arr[1] = -oxb_vec4_gety(a);
 out._arr[2] = -oxb_vec4_getz(a);
 out._arr[3] = -oxb_vec4_getw(a);
 return out;
#endif
}

oxb_inline oxb_vec4 oxb_vec4_abs(const oxb_vec4 a) {
#ifdef OXB_NEON
 return vabsq_f32(a);
#else
 oxb_vec4 out;
 out._arr[0] = oxb_abs(oxb_vec4_getx(a));
 out._arr[1] = oxb_abs(oxb_vec4_gety(a));
 out._arr[2] = oxb_abs(oxb_vec4_getz(a));
 out._arr[3] = oxb_abs(oxb_vec4_getw(a));
 return out;
#endif
}

oxb_inline oxb_vec4 oxb_vec4_min(const oxb_vec4 a, const oxb_vec4 b) {
#ifdef OXB_NEON
 return vminq_f32(a, b);
#else
 oxb_vec4 out;
 out._arr[0] = oxb_min(oxb_vec4_getx(a), oxb_vec4_getx(b));
 out._arr[1] = oxb_min(oxb_vec4_gety(a), oxb_vec4_gety(b));
 out._arr[2] = oxb_min(oxb_vec4_getz(a), oxb_vec4_getz(b));
 out._arr[3] = oxb_min(oxb_vec4_getw(a), oxb_vec4_getw(b));
 return out;
#endif
}

oxb_inline oxb_vec4 oxb_vec4_max(const oxb_vec4 a, const oxb_vec4 b) {
#ifdef OXB_NEON
 return vmaxq_f32(a, b);
#else
 oxb_vec4 out;
 out._arr[0] = oxb_max(oxb_vec4_getx(a), oxb_vec4_getx(b));
 out._arr[1] = oxb_max(oxb_vec4_gety(a), oxb_vec4_gety(b));
 out._arr[2] = oxb_max(oxb_vec4_getz(a), oxb_vec4_getz(b));
 out._arr[3] = oxb_max(oxb_vec4_getw(a), oxb_vec4_getw(b));
 return out;
#endif
}


oxb_inline oxb_vec4 oxb_vec4_add(const oxb_vec4 a, const oxb_vec4 b) {
#ifdef OXB_NEON
 return vaddq_f32(a, b);
#else
 oxb_vec4 out;
 out._arr[0] = oxb_vec4_getx(a) + oxb_vec4_getx(b);
 out._arr[1] = oxb_vec4_gety(a) + oxb_vec4_gety(b);
 out._arr[2] = oxb_vec4_getz(a) + oxb_vec4_getz(b);
 out._arr[3] = oxb_vec4_getw(a) + oxb_vec4_getw(b);
 return out;
#endif
}


oxb_inline oxb_vec4 oxb_vec4_sub(const oxb_vec4 a, const oxb_vec4 b) {
#ifdef OXB_NEON
 return vsubq_f32(a, b);
#else
 oxb_vec4 out;
 out._arr[0] = oxb_vec4_getx(a) - oxb_vec4_getx(b);
 out._arr[1] = oxb_vec4_gety(a) - oxb_vec4_gety(b);
 out._arr[2] = oxb_vec4_getz(a) - oxb_vec4_getz(b);
 out._arr[3] = oxb_vec4_getw(a) - oxb_vec4_getw(b);
 return out;
#endif
}


oxb_inline oxb_vec4 oxb_vec4_mul(const oxb_vec4 a, const oxb_vec4 b) {
#ifdef OXB_NEON
 return vmulq_f32(a, b);
#else
 oxb_vec4 out;
 out._arr[0] = oxb_vec4_getx(a) * oxb_vec4_getx(b);
 out._arr[1] = oxb_vec4_gety(a) * oxb_vec4_gety(b);
 out._arr[2] = oxb_vec4_getz(a) * oxb_vec4_getz(b);
 out._arr[3] = oxb_vec4_getw(a) * oxb_vec4_getw(b);
 return out;
#endif
}


oxb_inline oxb_vec4 oxb_vec4_fma(const oxb_vec4 a, const oxb_vec4 b, const oxb_vec4 c) {
#ifdef OXB_NEON
 return oxb_neon_fmaq(c, a, b);
#else
 oxb_vec4 out;
 out._arr[0] = oxb_vec4_getx(a) * oxb_vec4_getx(b) + oxb_vec4_getx(c);
 out._arr[1] = oxb_vec4_gety(a) * oxb_vec4_gety(b) + oxb_vec4_gety(c);
 out._arr[2] = oxb_vec4_getz(a) * oxb_vec4_getz(b) + oxb_vec4_getz(c);
 out._arr[3] = oxb_vec4_getw(a) * oxb_vec4_getw(b) + oxb_vec4_getw(c);
 return out;
#endif
}


oxb_inline oxb_vec4 oxb_vec4_div(const oxb_vec4 a, const oxb_vec4 b) {
#ifdef OXB_NEON
#ifdef OXB_NEON_64
 return vdivq_f32(a, b);
#else
 float32x4_t reciprocal = vrecpeq_f32(b);
 reciprocal = vmulq_f32(reciprocal, vrecpsq_f32(b, reciprocal));
 reciprocal = vmulq_f32(reciprocal, vrecpsq_f32(b, reciprocal));
 return vmulq_f32(a, reciprocal);
#endif
#else
 oxb_vec4 out;
 out._arr[0] = oxb_vec4_getx(a) / oxb_vec4_getx(b);
 out._arr[1] = oxb_vec4_gety(a) / oxb_vec4_gety(b);
 out._arr[2] = oxb_vec4_getz(a) / oxb_vec4_getz(b);
 out._arr[3] = oxb_vec4_getw(a) / oxb_vec4_getw(b); 
 return out;
#endif
}


oxb_inline float oxb_vec4_dot(const oxb_vec4 a, const oxb_vec4 b) {
#ifdef OXB_NEON
#ifdef OXB_NEON_64
 return vaddvq_f32(vmulq_f32(a, b));
#else
 float32x4_t p = vmulq_f32(a, b);
 p = vpaddq_f32(p, p);
 return vgetq_lane_f32(vpaddq_f32(p, p), 0);
#endif
#else
 return oxb_vec4_getx(a) * oxb_vec4_getx(b) + oxb_vec4_gety(a) * oxb_vec4_gety(b) + oxb_vec4_getz(a) * oxb_vec4_getz(b) + oxb_vec4_getw(a) * oxb_vec4_getw(b);
#endif
}


oxb_inline oxb_vec4 oxb_vec4_normalize(const oxb_vec4 a) {
 float inv_len = oxb_rsqrt(oxb_vec4_dot(a, a));
 return oxb_vec4_mul(a, oxb_vec4_splat(inv_len));
}


#endif


