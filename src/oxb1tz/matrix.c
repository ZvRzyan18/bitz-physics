#include "oxb1tz/matrix.h"
#include "oxb1tz/vector.h"
#include "oxb1tz/complex.h"
#include "oxb1tz/mask.h"


const oxb_mat2x2 oxb_mat2x2_identity = {
 ._basis[0] = OXB_VEC2_INIT(1.0f, 0.0f),
 ._basis[1] = OXB_VEC2_INIT(0.0f, 1.0f),
};

const oxb_mat3x3 oxb_mat3x3_identity = {
 ._basis[0] = OXB_VEC3_INIT(1.0f, 0.0f, 0.0f),
 ._basis[1] = OXB_VEC3_INIT(0.0f, 1.0f, 0.0f),
 ._basis[2] = OXB_VEC3_INIT(0.0f, 0.0f, 1.0f),
};



oxb_hot void oxb_mat2x2_store(float *oxb_restrict const arr, const oxb_mat2x2 m) {
 oxb_vec2_store(&arr[0], m._basis[0]);
 oxb_vec2_store(&arr[2], m._basis[1]);
}


oxb_hot oxb_mat2x2 oxb_mat2x2_load(const float *oxb_restrict const arr) {
 oxb_mat2x2 out;
 out._basis[0] = oxb_vec2_load(arr);
 out._basis[1] = oxb_vec2_load(arr + 2); 
 return out;
}


oxb_hot oxb_mat2x2 oxb_mat2x2_transpose(const oxb_mat2x2 a) {
#ifdef OXB_NEON
 oxb_mat2x2 out;
 float32x2x2_t result = vtrn_f32(a._basis[0], a._basis[1]);
 out._basis[0] = result.val[0];
 out._basis[1] = result.val[1];
 return out;
#else
 oxb_mat2x2 out = {0};
 out._basis[0] = oxb_vec2_setx(out._basis[0], oxb_vec2_getx(a._basis[0]));
 out._basis[1] = oxb_vec2_setx(out._basis[1], oxb_vec2_gety(a._basis[0]));

 out._basis[0] = oxb_vec2_sety(out._basis[0], oxb_vec2_getx(a._basis[1]));
 out._basis[1] = oxb_vec2_sety(out._basis[1], oxb_vec2_gety(a._basis[1]));
 return out;
#endif
}



oxb_hot oxb_mat2x2 oxb_mat2x2_mul(const oxb_mat2x2 a, const oxb_mat2x2 b) {
 oxb_mat2x2 out;
 
 for(int i = 0; i < 2; i++) {
  const oxb_vec2 b0 = oxb_vec2_splat(oxb_vec2_getx(b._basis[i]));
  const oxb_vec2 b1 = oxb_vec2_splat(oxb_vec2_gety(b._basis[i]));
 
  out._basis[i] = oxb_vec2_mul(a._basis[0], b0);
  out._basis[i] = oxb_vec2_fma(a._basis[1], b1, out._basis[i]);
 }
 return out;
}

oxb_hot oxb_mat2x2 oxb_mat2x2_inv(const oxb_mat2x2 a) {
 /* vec2 cross product : a.x * b.y - a.y * b.x */
 float det = oxb_vec2_cross(a._basis[0], a._basis[1]);
 oxb_vec4 inv_det = oxb_vec4_splat(1.0f / det);
 
 static const oxb_mask4 sign = OXB_MASK4_INIT(0x00000000, 0x80000000, 0x80000000, 0x00000000);
 
 oxb_vec4 combined = oxb_vec4_combine(a._basis[0], a._basis[1]);
 combined = oxb_vec4_mul(combined, inv_det);
 combined = oxb_mask4_tovec4(oxb_mask4_eor(oxb_mask4_fromvec4(combined), sign));
 combined = oxb_vec4_perm(combined, 3, 1, 2, 0);
 
 oxb_mat2x2 out;
 out._basis[0] = oxb_vec4_low(combined);
 out._basis[1] = oxb_vec4_high(combined);
 return out;
}


oxb_hot oxb_vec2 oxb_mat2x2_mulvec(const oxb_vec2 a, const oxb_mat2x2 b) {
 oxb_vec2 x, y;
 x = oxb_vec2_splat(oxb_vec2_getx(a));
 y = oxb_vec2_splat(oxb_vec2_gety(a));
 return oxb_vec2_fma(b._basis[0], x, oxb_vec2_mul(b._basis[1], y));
}






oxb_hot void oxb_mat3x3_store(float *oxb_restrict const arr, const oxb_mat3x3 m) {
 oxb_vec3_store(&arr[0], m._basis[0]);
 oxb_vec3_store(&arr[3], m._basis[1]);
 oxb_vec3_store(&arr[6], m._basis[2]);
}


oxb_hot oxb_mat3x3 oxb_mat3x3_load(const float *oxb_restrict const arr) {
 oxb_mat3x3 out;
 out._basis[0] = oxb_vec3_load(arr);
 out._basis[1] = oxb_vec3_load(arr + 3);
 out._basis[2] = oxb_vec3_load(arr + 6);
 return out;
}


oxb_hot oxb_mat3x3 oxb_mat3x3_transpose(const oxb_mat3x3 a) {
#ifdef OXB_NEON
 oxb_mat3x3 out;

 float32x4_t c0 = a._basis[0];
 float32x4_t c1 = a._basis[1];
 float32x4_t c2 = a._basis[2];
 float32x4x2_t t = vtrnq_f32(c0, c1);
 float32x2_t zero = vdup_n_f32(0.0f);
 out._basis[0] = vsetq_lane_f32(vgetq_lane_f32(c2, 0), vcombine_f32(vget_low_f32(t.val[0]), zero), 2);
 out._basis[1] = vsetq_lane_f32(vgetq_lane_f32(c2, 1), vcombine_f32(vget_low_f32(t.val[1]), zero), 2);
 out._basis[2] = vsetq_lane_f32(vgetq_lane_f32(c2, 2), vcombine_f32(vget_high_f32(t.val[0]), zero), 2);

 return out;
#else
 oxb_mat3x3 out = {0};
 out._basis[0] = oxb_vec3_setx(out._basis[0], oxb_vec3_getx(a._basis[0]));
 out._basis[1] = oxb_vec3_setx(out._basis[1], oxb_vec3_gety(a._basis[0]));
 out._basis[2] = oxb_vec3_setx(out._basis[2], oxb_vec3_getz(a._basis[0]));

 out._basis[0] = oxb_vec3_sety(out._basis[0], oxb_vec3_getx(a._basis[1]));
 out._basis[1] = oxb_vec3_sety(out._basis[1], oxb_vec3_gety(a._basis[1]));
 out._basis[2] = oxb_vec3_sety(out._basis[2], oxb_vec3_getz(a._basis[1]));

 out._basis[0] = oxb_vec3_setz(out._basis[0], oxb_vec3_getx(a._basis[2]));
 out._basis[1] = oxb_vec3_setz(out._basis[1], oxb_vec3_gety(a._basis[2]));
 out._basis[2] = oxb_vec3_setz(out._basis[2], oxb_vec3_getz(a._basis[2]));
 return out;
#endif
}


oxb_hot oxb_mat3x3 oxb_mat3x3_mul(const oxb_mat3x3 a, const oxb_mat3x3 b) {
 oxb_mat3x3 out;
 
 for(int i = 0; i < 3; i++) {
  const oxb_vec3 b0 = oxb_vec3_splat(oxb_vec3_getx(b._basis[i]));
  const oxb_vec3 b1 = oxb_vec3_splat(oxb_vec3_gety(b._basis[i]));
  const oxb_vec3 b2 = oxb_vec3_splat(oxb_vec3_getz(b._basis[i]));

  out._basis[i] = oxb_vec3_mul(a._basis[0], b0);
  out._basis[i] = oxb_vec3_fma(a._basis[1], b1, out._basis[i]);
  out._basis[i] = oxb_vec3_fma(a._basis[2], b2, out._basis[i]);
 }
 return out;
}



oxb_hot oxb_mat3x3 oxb_mat3x3_inv(const oxb_mat3x3 a) {
 oxb_mat3x3 out;

 oxb_vec3 c0 = a._basis[0];
 oxb_vec3 c1 = a._basis[1];
 oxb_vec3 c2 = a._basis[2];

 oxb_vec3 r0 = oxb_vec3_cross(c1, c2);
 oxb_vec3 r1 = oxb_vec3_cross(c2, c0);
 oxb_vec3 r2 = oxb_vec3_cross(c0, c1);

 float det = oxb_vec3_dot(c0, r0);
 oxb_vec3 inv_det = oxb_vec3_splat(1.0f / det);

 out._basis[0] = oxb_vec3_mul(r0, inv_det);
 out._basis[1] = oxb_vec3_mul(r1, inv_det);
 out._basis[2] = oxb_vec3_mul(r2, inv_det);
 return out;
}


oxb_hot oxb_mat3x3 oxb_mat3x3_transform(const oxb_complex rot, const oxb_vec2 pos) {
	oxb_mat3x3 out;
	oxb_vec2 zero = OXB_VEC2_INIT(0.0f, 0.0f);
	oxb_vec2 one = OXB_VEC2_INIT(0.0f, 1.0f);
	out._basis[0] = oxb_vec4_tovec3(oxb_vec4_combine(rot, zero));
 out._basis[1] = oxb_vec4_tovec3(oxb_vec4_combine(oxb_vec2_swaplane(oxb_complex_conj(rot)), zero));
 out._basis[2] = oxb_vec4_tovec3(oxb_vec4_combine(pos, one));
 return out;
}


oxb_hot oxb_vec3 oxb_mat3x3_mulvec(const oxb_vec3 a, const oxb_mat3x3 b) {
 oxb_vec3 x, y, z;
 x = oxb_vec3_splat(oxb_vec3_getx(a));
 y = oxb_vec3_splat(oxb_vec3_gety(a));
 z = oxb_vec3_splat(oxb_vec3_getz(a));
 return oxb_vec3_fma(b._basis[0], x, oxb_vec3_fma(b._basis[1], y, oxb_vec3_mul(b._basis[2], z)));
}

