#ifndef OXB_SIMD_H
#define OXB_SIMD_H

#include "oxb1tz/vector.h"
#include "oxb1tz/mask.h"
#include "oxb1tz/core2d.h"

/*
 simd implemented functions
*/
oxb_inline oxb_vec2 oxb_vec2_log2(oxb_vec2 x) {
	oxb_vec2 out;
#ifdef OXB_NEON
 static const float32x2_t one = {1.0f, 1.0f};
 static const uint32x2_t bits1 = {1065353216U, 1065353216U};
 static const uint32x2_t bits2 = {0x007FFFFF, 0x007FFFFF};
 static const int32x2_t bias = {127, 127};

 static const float32x2_t LOG2[5] = {
  {-8.161580e-02f, -8.161580e-02f},
  { 6.451423e-01f,  6.451423e-01f},
  {-2.120675e-00f, -2.120675e-00f},
  { 4.070090e-00f,  4.070090e-00f},
  {-2.512854e-00f, -2.512854e-00f}
 };

 uint32x2_t small = vclt_f32(x, one);
 
 float32x2_t initial_val = vbsl_f32(small, oxb_vec2_div(one, x), x);
 uint32x2_t bits = vreinterpret_u32_f32(initial_val);
 float32x2_t mantissa = vreinterpret_f32_u32(vorr_u32(vand_u32(bits2, bits), bits1));
 int32x2_t signed_value = vreinterpret_s32_u32(vshr_n_u32(bits, 23));
 
 mantissa = oxb_vec2_fma(oxb_vec2_fma(oxb_vec2_fma(oxb_vec2_fma(LOG2[0], mantissa, LOG2[1]), mantissa, LOG2[2]), mantissa, LOG2[3]), mantissa, LOG2[4]);
 signed_value = vsub_s32(signed_value, bias);
 out = vcvt_f32_u32(signed_value);
 out = vadd_f32(out, mantissa);
 out = vbsl_f32(small, vneg_f32(out), out);
#else
 out._arr[0] = oxb_log2(oxb_vec2_getx(x));
 out._arr[1] = oxb_log2(oxb_vec2_gety(x));
#endif
 return out;
}

oxb_inline oxb_vec4 oxb_vec4_log2(oxb_vec4 x) {
	oxb_vec4 out;
#ifdef OXB_NEON
 static const float32x4_t one = {1.0f, 1.0f, 1.0f, 1.0f};
 static const uint32x4_t bits1 = {1065353216U, 1065353216U, 1065353216U, 1065353216U};
 static const uint32x4_t bits2 = {0x007FFFFF, 0x007FFFFF, 0x007FFFFF, 0x007FFFFF};
 static const int32x4_t bias = {127, 127, 127, 127};

 static const float32x4_t LOG2[5] = {
  {-8.161580e-02f, -8.161580e-02f, -8.161580e-02f, -8.161580e-02f},
  { 6.451423e-01f,  6.451423e-01f,  6.451423e-01f,  6.451423e-01f},
  {-2.120675e-00f, -2.120675e-00f, -2.120675e-00f, -2.120675e-00f},
  { 4.070090e-00f,  4.070090e-00f,  4.070090e-00f,  4.070090e-00f},
  {-2.512854e-00f, -2.512854e-00f, -2.512854e-00f, -2.512854e-00f}
 };

 uint32x4_t small = vcltq_f32(x, one);
 
 float32x4_t initial_val = vbslq_f32(small, oxb_vec4_div(one, x), x);
 uint32x4_t bits = vreinterpretq_u32_f32(initial_val);
 float32x4_t mantissa = vreinterpretq_f32_u32(vorrq_u32(vandq_u32(bits2, bits), bits1));
 int32x4_t signed_value = vreinterpretq_s32_u32(vshrq_n_u32(bits, 23));
 
 mantissa = oxb_vec4_fma(oxb_vec4_fma(oxb_vec4_fma(oxb_vec4_fma(LOG2[0], mantissa, LOG2[1]), mantissa, LOG2[2]), mantissa, LOG2[3]), mantissa, LOG2[4]);
 signed_value = vsubq_s32(signed_value, bias);
 out = vcvtq_f32_u32(signed_value);
 out = vaddq_f32(out, mantissa);
 out = vbslq_f32(small, vnegq_f32(out), out);
#else
 out._arr[0] = oxb_log2(oxb_vec4_getx(x));
 out._arr[1] = oxb_log2(oxb_vec4_gety(x));
 out._arr[2] = oxb_log2(oxb_vec4_getz(x));
 out._arr[3] = oxb_log2(oxb_vec4_getw(x));

#endif
 return out;
}



oxb_inline oxb_vec2 oxb_vec2_exp2(oxb_vec2 x) {
	oxb_vec2 out;
#ifdef OXB_NEON
 static const float32x2_t half = {0.5f, 0.5f};
 static const int32x2_t bias = {127, 127};

 static const float32x2_t EXP2[5] = {
  {0.96857114e-02f, 0.96857114e-02f},
  {0.55922025e-01f, 0.55922025e-01f},
  {0.24021865e-00f, 0.24021865e-00f},
  {0.69312103e-00f, 0.69312103e-00f},
  {1.00000015e-00f, 1.00000015e-00f}
 };

 int32x2_t whole = vcvt_s32_f32(vsub_f32(x, half));
 float32x2_t fractional = vsub_f32(x, vcvt_f32_s32(whole));
 uint32x2_t bits = vreinterpret_u32_s32(vshl_n_u32(vadd_s32(whole, bias), 23));
 float32x2_t mx = oxb_vec2_fma(oxb_vec2_fma(oxb_vec2_fma(oxb_vec2_fma(EXP2[0], fractional, EXP2[1]), fractional, EXP2[2]), fractional, EXP2[3]), fractional, EXP2[4]);
 out = vmul_f32(mx, vreinterpret_f32_u32(bits));
#else
 out._arr[0] = oxb_exp2(oxb_vec2_getx(x));
 out._arr[1] = oxb_exp2(oxb_vec2_gety(x));
#endif
 return out;
}


oxb_inline oxb_vec4 oxb_vec4_exp2(oxb_vec4 x) {
	oxb_vec4 out;
#ifdef OXB_NEON
 static const float32x4_t half = {0.5f, 0.5f, 0.5f, 0.5f};
 static const int32x4_t bias = {127, 127, 127, 127};

 static const float32x4_t EXP2[5] = {
  {0.96857114e-02f, 0.96857114e-02f, 0.96857114e-02f, 0.96857114e-02f},
  {0.55922025e-01f, 0.55922025e-01f, 0.55922025e-01f, 0.55922025e-01f},
  {0.24021865e-00f, 0.24021865e-00f, 0.24021865e-00f, 0.24021865e-00f},
  {0.69312103e-00f, 0.69312103e-00f, 0.69312103e-00f, 0.69312103e-00f},
  {1.00000015e-00f, 1.00000015e-00f, 1.00000015e-00f, 1.00000015e-00f}
 };

 int32x4_t whole = vcvtq_s32_f32(vsubq_f32(x, half));
 float32x4_t fractional = vsubq_f32(x, vcvtq_f32_s32(whole));
 uint32x4_t bits = vreinterpretq_u32_s32(vshlq_n_u32(vaddq_s32(whole, bias), 23));
 float32x4_t mx = oxb_vec4_fma(oxb_vec4_fma(oxb_vec4_fma(oxb_vec4_fma(EXP2[0], fractional, EXP2[1]), fractional, EXP2[2]), fractional, EXP2[3]), fractional, EXP2[4]);
 out = vmulq_f32(mx, vreinterpretq_f32_u32(bits));
#else
 out._arr[0] = oxb_exp2(oxb_vec4_getx(x));
 out._arr[1] = oxb_exp2(oxb_vec4_gety(x));
 out._arr[2] = oxb_exp2(oxb_vec4_getz(x));
 out._arr[3] = oxb_exp2(oxb_vec4_getw(x));
#endif
 return out;
}





oxb_inline oxb_vec2 oxb_vec2_sqrt(oxb_vec2 x) {
	oxb_vec2 out;
#ifdef OXB_NEON
#ifdef OXB_NEON_64
 out = vsqrt_f32(x);
#else
 float32x2_t rsqrt = vrsqrte_f32(x);
 rsqrt = vmul_f32(vrsqrts_f32(vmul_f32(x, rsqrt), rsqrt), rsqrt);
 out = vmul_f32(x, rsqrt);
#endif
#else
 out._arr[0] = oxb_sqrt(oxb_vec2_getx(x));
 out._arr[1] = oxb_sqrt(oxb_vec2_gety(x));
#endif
 return out;
}




oxb_inline oxb_vec4 oxb_vec4_rsqrt(oxb_vec4 x) {
	oxb_vec4 out;
#ifdef OXB_NEON
 float32x4_t rsqrt = vrsqrteq_f32(x);
#ifndef __FAST_MATH__
 rsqrt = vmulq_f32(vrsqrtsq_f32(vmulq_f32(x, rsqrt), rsqrt), rsqrt);
#endif
 out = rsqrt;
#else
 out._arr[0] = oxb_rsqrt(oxb_vec4_getx(x));
 out._arr[1] = oxb_rsqrt(oxb_vec4_gety(x));
 out._arr[2] = oxb_rsqrt(oxb_vec4_getz(x));
 out._arr[3] = oxb_rsqrt(oxb_vec4_getw(x));
#endif
 return out;
}



/*
 batch integration, has to be performed with
 gather + scatter.
*/
typedef struct {
 oxb_vec4 _position_x;
 oxb_vec4 _position_y;

 oxb_vec4 _rotation_real;
 oxb_vec4 _rotation_imag;

 oxb_vec4 _velocity_x;
 oxb_vec4 _velocity_y;

 oxb_vec4 _omega;
 oxb_vec4 _linear_damping;
 oxb_vec4 _angular_damping;
 oxb_mask4 _flags;
} oxb_dynamic2d_batch4;

oxb_inline void oxb_integrate2d_dynamic4(oxb_dynamic2d_batch4 *batch, oxb_vec2 gravity, oxb_vec4 dt_vec, float max_omega) {
 oxb_vec4 gravity_x = oxb_vec4_splat(oxb_vec2_getx(gravity));
 oxb_vec4 gravity_y = oxb_vec4_splat(oxb_vec2_gety(gravity));

 batch->_velocity_x = oxb_vec4_fma(gravity_x, dt_vec, batch->_velocity_x);
 batch->_velocity_y = oxb_vec4_fma(gravity_y, dt_vec, batch->_velocity_y);
 
 oxb_vec4 linear_damp = oxb_vec4_exp2(oxb_vec4_mul(batch->_linear_damping, dt_vec));
 batch->_velocity_x = oxb_vec4_mul(batch->_velocity_x, linear_damp);
 batch->_velocity_y = oxb_vec4_mul(batch->_velocity_y, linear_damp);
 batch->_omega = oxb_vec4_mul(batch->_omega, oxb_vec4_exp2(oxb_vec4_mul(batch->_angular_damping, dt_vec)));
 batch->_omega = oxb_vec4_max(oxb_vec4_min(batch->_omega, oxb_vec4_splat(max_omega)), oxb_vec4_splat(-max_omega));

 /* lock mask */
 static const oxb_mask4 posx_lock = OXB_MASK4_INIT((uint32_t)OXB_FLAG_LOCK_POSX_2D, (uint32_t)OXB_FLAG_LOCK_POSX_2D, (uint32_t)OXB_FLAG_LOCK_POSX_2D, (uint32_t)OXB_FLAG_LOCK_POSX_2D);
 static const oxb_mask4 posy_lock = OXB_MASK4_INIT((uint32_t)OXB_FLAG_LOCK_POSY_2D, (uint32_t)OXB_FLAG_LOCK_POSY_2D, (uint32_t)OXB_FLAG_LOCK_POSY_2D, (uint32_t)OXB_FLAG_LOCK_POSY_2D);
 static const oxb_mask4 rot_lock = OXB_MASK4_INIT((uint32_t)OXB_FLAG_LOCK_ROT_2D, (uint32_t)OXB_FLAG_LOCK_ROT_2D, (uint32_t)OXB_FLAG_LOCK_ROT_2D, (uint32_t)OXB_FLAG_LOCK_ROT_2D);

 static const oxb_mask4 zero_mask = OXB_MASK4_INIT(0, 0, 0, 0);

 oxb_mask4 lock_x = oxb_mask4_equal(oxb_mask4_and(batch->_flags, posx_lock), zero_mask);
 oxb_mask4 lock_y = oxb_mask4_equal(oxb_mask4_and(batch->_flags, posy_lock), zero_mask);
 oxb_mask4 lock_rot = oxb_mask4_equal(oxb_mask4_and(batch->_flags, rot_lock), zero_mask);

 batch->_velocity_x = oxb_mask4_tovec4(oxb_mask4_and(oxb_mask4_fromvec4(batch->_velocity_x), lock_x));
 batch->_velocity_y = oxb_mask4_tovec4(oxb_mask4_and(oxb_mask4_fromvec4(batch->_velocity_y), lock_y));
 batch->_omega = oxb_mask4_tovec4(oxb_mask4_and(oxb_mask4_fromvec4(batch->_omega), lock_rot));


 oxb_vec4 delta_omega = oxb_vec4_mul(batch->_omega, dt_vec);
 static const oxb_vec4 one = OXB_VEC4_INIT(1.0f, 1.0f, 1.0f, 1.0f);
 static const oxb_vec4 half = OXB_VEC4_INIT(0.5f, 0.5f, 0.5f, 0.5f);
 
 /* first order of taylor series */
 oxb_vec4 real = oxb_vec4_sub(one, oxb_vec4_mul(oxb_vec4_mul(half, delta_omega), delta_omega));
 oxb_vec4 imag = delta_omega;


 batch->_rotation_real = oxb_vec4_sub(oxb_vec4_mul(batch->_rotation_real, real), oxb_vec4_mul(batch->_rotation_imag, imag));
 batch->_rotation_imag = oxb_vec4_add(oxb_vec4_mul(batch->_rotation_real, imag), oxb_vec4_mul(batch->_rotation_imag, real));
 
 oxb_vec4 real_squared = oxb_vec4_mul(batch->_rotation_real, batch->_rotation_real);
 oxb_vec4 imag_squared = oxb_vec4_mul(batch->_rotation_imag, batch->_rotation_imag);
 oxb_vec4 len_squared = oxb_vec4_add(real_squared, imag_squared);
 /*
 oxb_vec4 inv_len = oxb_vec4_div(one, oxb_vec4_sqrt(len_squared));
 */
 oxb_vec4 inv_len = oxb_vec4_rsqrt(len_squared);
 
 batch->_rotation_real = oxb_vec4_mul(batch->_rotation_real, inv_len);
 batch->_rotation_imag = oxb_vec4_mul(batch->_rotation_imag, inv_len);

 batch->_position_x = oxb_vec4_fma(batch->_velocity_x, dt_vec, batch->_position_x);   
 batch->_position_y = oxb_vec4_fma(batch->_velocity_y, dt_vec, batch->_position_y);   
}




typedef struct {
 oxb_vec4    _r1_x[2];
 oxb_vec4    _r1_y[2];
 oxb_vec4    _r2_x[2];
 oxb_vec4    _r2_y[2];

 oxb_vec4    _normal_x[2];
 oxb_vec4    _normal_y[2];
 oxb_vec4    _depth[2];
 oxb_vec4    _bias[2];
 oxb_vec4    _mass_normal[2];
 oxb_vec4    _mass_tangent[2];
 oxb_vec4    _normal_lambda[2];
 oxb_vec4    _tangent_lambda[2];
 
 oxb_vec4    _static_friction;
 oxb_vec4    _dynamic_friction;
 oxb_vec4    _velocity_x[2];
 oxb_vec4    _velocity_y[2];
 oxb_vec4    _omega[2];
 oxb_vec4    _inv_mass[2];
 oxb_vec4    _inv_inertia[2];
} oxb_manifold2d_batch4;


oxb_inline void oxb_contact2d_solve4(oxb_manifold2d_batch4 *m, oxb_vec4 bias_coeff, oxb_vec4 mass_coeff, oxb_vec4 impulse_coeff, oxb_vec4 slop) {
 static const oxb_vec4 zero = OXB_VEC4_INIT(0.0f, 0.0f, 0.0f, 0.0f);
 
 for(size_t i = 0; i < 2; i++) {
 	
 	oxb_vec4 aocr1_x, aocr1_y;
 	oxb_vec4 bocr2_x, bocr2_y;
 	{
 	 aocr1_x = oxb_vec4_mul(oxb_vec4_neg(m->_omega[0]), m->_r1_y[i]);
 	 aocr1_y = oxb_vec4_mul(m->_omega[0], m->_r1_x[i]);

   bocr2_x = oxb_vec4_mul(oxb_vec4_neg(m->_omega[1]), m->_r2_y[i]);
 	 bocr2_y = oxb_vec4_mul(m->_omega[1], m->_r2_x[i]);
 	}
 	
 	oxb_vec4 va_x = oxb_vec4_add(m->_velocity_x[0], aocr1_x);
 	oxb_vec4 va_y = oxb_vec4_add(m->_velocity_y[0], aocr1_y);

 	oxb_vec4 vb_x = oxb_vec4_add(m->_velocity_x[1], bocr2_x);
 	oxb_vec4 vb_y = oxb_vec4_add(m->_velocity_y[1], bocr2_y);
 	
 	oxb_vec4 relative_vel_x = oxb_vec4_sub(vb_x, va_x);
 	oxb_vec4 relative_vel_y = oxb_vec4_sub(vb_y, va_y);
 	oxb_vec4 vn = oxb_vec4_fma(relative_vel_x, m->_normal_x[i], oxb_vec4_mul(relative_vel_y, m->_normal_y[i]));

  oxb_vec4 penetration_error;

  oxb_vec4 error = oxb_vec4_min(oxb_vec4_add(m->_depth[i], slop), zero);
  penetration_error = oxb_vec4_mul(oxb_vec4_neg(error), bias_coeff);
  
  /*
   logarithmic damping
  */
  oxb_vec4 inv_log2_e = oxb_vec4_splat(0.69314718f);
  penetration_error = oxb_vec4_mul(oxb_vec4_log2(oxb_vec4_add(penetration_error, oxb_vec4_splat(1.0f))), inv_log2_e);
  
  penetration_error = oxb_vec4_add(oxb_vec4_neg(oxb_vec4_sub(vn, m->_bias[i])), penetration_error);



  oxb_vec4 j;
  {
   oxb_vec4 mass = oxb_vec4_mul(m->_mass_normal[i], mass_coeff);
	  j = oxb_vec4_mul(mass, penetration_error);
   j = oxb_vec4_fma(oxb_vec4_neg(impulse_coeff), m->_normal_lambda[i], j);
  }
  {
		 oxb_vec4 accumulated_normal = m->_normal_lambda[i];
	 	m->_normal_lambda[i] = oxb_vec4_max(oxb_vec4_add(accumulated_normal, j), zero);
		 j = oxb_vec4_sub(m->_normal_lambda[i], accumulated_normal);
  }
  oxb_vec4 impulse_x, impulse_y;
  oxb_vec4 tmp;
  {
   impulse_x = oxb_vec4_mul(m->_normal_x[i], j);
   impulse_y = oxb_vec4_mul(m->_normal_y[i], j);
   m->_velocity_x[0] = oxb_vec4_fma(m->_inv_mass[0], impulse_x, m->_velocity_x[0]);
   m->_velocity_y[0] = oxb_vec4_fma(m->_inv_mass[0], impulse_y, m->_velocity_y[0]);
   m->_velocity_x[1] = oxb_vec4_fma(m->_inv_mass[1], impulse_x, m->_velocity_x[1]);
   m->_velocity_y[1] = oxb_vec4_fma(m->_inv_mass[1], impulse_y, m->_velocity_y[1]);
   tmp = oxb_vec4_fma(m->_r1_x[i], impulse_y, oxb_vec4_mul(oxb_vec4_neg(m->_r1_y[i]), impulse_x));
	  m->_omega[0] = oxb_vec4_fma(m->_inv_inertia[0], tmp, m->_omega[0]);
   tmp = oxb_vec4_fma(m->_r2_x[i], impulse_y, oxb_vec4_mul(oxb_vec4_neg(m->_r2_y[i]), impulse_x));
	  m->_omega[1] = oxb_vec4_fma(m->_inv_inertia[1], tmp, m->_omega[1]);
  }
  
  oxb_vec4 tangent_x = m->_normal_y[i];
  oxb_vec4 tangent_y = oxb_vec4_neg(m->_normal_x[i]);
 	{
 	 aocr1_x = oxb_vec4_mul(oxb_vec4_neg(m->_omega[0]), m->_r1_y[i]);
 	 aocr1_y = oxb_vec4_mul(m->_omega[0], m->_r1_x[i]);

   bocr2_x = oxb_vec4_mul(oxb_vec4_neg(m->_omega[1]), m->_r2_y[i]);
 	 bocr2_y = oxb_vec4_mul(m->_omega[1], m->_r2_x[i]);
 	}
 	
  va_x = oxb_vec4_add(m->_velocity_x[0], aocr1_x);
  va_y = oxb_vec4_add(m->_velocity_y[0], aocr1_y);

  vb_x = oxb_vec4_add(m->_velocity_x[1], bocr2_x);
  vb_y = oxb_vec4_add(m->_velocity_y[1], bocr2_y);
 	
  relative_vel_x = oxb_vec4_sub(vb_x, va_x);
  relative_vel_y = oxb_vec4_sub(vb_y, va_y);
  oxb_vec4 vt = oxb_vec4_fma(relative_vel_x, tangent_x, oxb_vec4_mul(relative_vel_y, tangent_y));

  j = oxb_vec4_mul(m->_mass_tangent[i], oxb_vec4_neg(vt));

  oxb_vec4 dynamic_limit = oxb_vec4_mul(m->_dynamic_friction, m->_normal_lambda[i]);
  oxb_vec4 static_limit = oxb_vec4_mul(m->_static_friction, m->_normal_lambda[i]);

  oxb_vec4 prev_tangent = m->_tangent_lambda[i];
  oxb_vec4 target_tangent = oxb_vec4_add(prev_tangent, j);
  
  oxb_mask4 switch_friction = oxb_mask4_fgt(oxb_vec4_abs(target_tangent), static_limit);

  oxb_vec4 ta = oxb_vec4_min(oxb_vec4_max(target_tangent, oxb_vec4_neg(dynamic_limit)), dynamic_limit);
  oxb_vec4 tb = target_tangent;
  m->_tangent_lambda[i] = oxb_mask4_fselect(switch_friction, ta, tb);
  j = oxb_vec4_sub(m->_tangent_lambda[i], prev_tangent);

  {
   impulse_x = oxb_vec4_mul(tangent_x, j);
   impulse_y = oxb_vec4_mul(tangent_y, j);
   m->_velocity_x[0] = oxb_vec4_fma(m->_inv_mass[0], impulse_x, m->_velocity_x[0]);
   m->_velocity_y[0] = oxb_vec4_fma(m->_inv_mass[0], impulse_y, m->_velocity_y[0]);
   m->_velocity_x[1] = oxb_vec4_fma(m->_inv_mass[1], impulse_x, m->_velocity_x[1]);
   m->_velocity_y[1] = oxb_vec4_fma(m->_inv_mass[1], impulse_y, m->_velocity_y[1]);
   tmp = oxb_vec4_fma(m->_r1_x[i], impulse_y, oxb_vec4_mul(oxb_vec4_neg(m->_r1_y[i]), impulse_x));
	  m->_omega[0] = oxb_vec4_fma(m->_inv_inertia[0], tmp, m->_omega[0]);
   tmp = oxb_vec4_fma(m->_r2_x[i], impulse_y, oxb_vec4_mul(oxb_vec4_neg(m->_r2_y[i]), impulse_x));
	  m->_omega[1] = oxb_vec4_fma(m->_inv_inertia[1], tmp, m->_omega[1]);
  }
  
 }
}



#endif


