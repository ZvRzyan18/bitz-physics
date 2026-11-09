#include "vector.h"
#include "oxb1tz/vector.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef OXB_TEST

oxb_inline float VECTOR_TEST_RANDOM(float min, float max) {
 return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

oxb_inline void VECTOR2_TEST() {
 const float epsilon = 1e-2f;
 const float min_range = -100.0f;
 const float max_range =  100.0f;
 const int loop_repeat = 100;
 
 printf("--splat test\n");
 /* splat test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  oxb_vec2 a = oxb_vec2_splat(tmp);
  assert(fabsf(oxb_vec2_getx(a) - tmp) < epsilon);
  assert(fabsf(oxb_vec2_gety(a) - tmp) < epsilon);  
 }
 
 printf("--setx sety, getx, gety test\n");
 /* setx sety, getx, gety test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp_a = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_b = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec2 a = oxb_vec2_splat(0.0f);
  a = oxb_vec2_setx(a, tmp_a);
  a = oxb_vec2_sety(a, tmp_b);
  assert(fabsf(oxb_vec2_getx(a) - tmp_a) < epsilon);
  assert(fabsf(oxb_vec2_gety(a) - tmp_b) < epsilon);  
 }

 printf("--set and get test\n");
 /* set and get test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp_a = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_b = VECTOR_TEST_RANDOM(min_range, max_range);
  oxb_vec2 a = oxb_vec2_splat(0.0f);
  a = oxb_vec2_set(a, 0, tmp_a);
  a = oxb_vec2_set(a, 1, tmp_b);
  assert(fabsf(oxb_vec2_get(a, 0) - tmp_a) < epsilon);
  assert(fabsf(oxb_vec2_get(a, 1) - tmp_b) < epsilon);  
 }

 printf("--load store test\n");
 /* load store test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2], tmp1[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[0] = oxb_nan();
  tmp1[1] = oxb_nan();
  oxb_vec2 a = oxb_vec2_load(tmp);
  
  assert(fabsf(oxb_vec2_get(a, 0) - tmp[0]) < epsilon);
  assert(fabsf(oxb_vec2_get(a, 1) - tmp[1]) < epsilon);  
  
  oxb_vec2_store(tmp1, a);

  assert(fabsf(tmp1[0] - tmp[0]) < epsilon);
  assert(fabsf(tmp1[1] - tmp[1]) < epsilon);  

 }

 printf("--laneswap test\n");
 /* laneswap test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp_a = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_b = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec2 a = oxb_vec2_splat(0.0f);
  a = oxb_vec2_setx(a, tmp_a);
  a = oxb_vec2_sety(a, tmp_b);
  a = oxb_vec2_swaplane(a);
  assert(fabsf(oxb_vec2_getx(a) - tmp_b) < epsilon);
  assert(fabsf(oxb_vec2_gety(a) - tmp_a) < epsilon);  
 }
 
 printf("--neg test\n");
 /* neg test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp1 = VECTOR_TEST_RANDOM(min_range, max_range);
  oxb_vec2 a = oxb_vec2_neg(OXB_VEC2_INIT(tmp, tmp1));
  assert(fabsf(oxb_vec2_getx(a) - -tmp) < epsilon);
  assert(fabsf(oxb_vec2_gety(a) - -tmp1) < epsilon);  
 }

 printf("--abs test\n");
 /* abs test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp1 = VECTOR_TEST_RANDOM(min_range, max_range);
  oxb_vec2 a = oxb_vec2_abs(OXB_VEC2_INIT(tmp, tmp1));
  assert(fabsf(oxb_vec2_getx(a) - fabsf(tmp)) < epsilon);
  assert(fabsf(oxb_vec2_gety(a) - fabsf(tmp1)) < epsilon);  
 }
 
 printf("--add test\n");
 /* add test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2], tmp1[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 mb = oxb_vec2_load(tmp1);
  oxb_vec2 r = oxb_vec2_add(ma, mb);
  
  float sum_a = tmp[0] + tmp1[0];
  float sum_b = tmp[1] + tmp1[1];
  assert(fabsf(oxb_vec2_getx(r) - sum_a) < epsilon);
  assert(fabsf(oxb_vec2_gety(r) - sum_b) < epsilon);  
 }

 printf("--sub test\n");
 /* sub test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2], tmp1[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 mb = oxb_vec2_load(tmp1);
  oxb_vec2 r = oxb_vec2_sub(ma, mb);
  
  float difference_a = tmp[0] - tmp1[0];
  float difference_b = tmp[1] - tmp1[1];
  assert(fabsf(oxb_vec2_getx(r) - difference_a) < epsilon);
  assert(fabsf(oxb_vec2_gety(r) - difference_b) < epsilon);  
 }

 printf("--mul test\n");
 /* mul test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2], tmp1[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 mb = oxb_vec2_load(tmp1);
  oxb_vec2 r = oxb_vec2_mul(ma, mb);
  
  float product_a = tmp[0] * tmp1[0];
  float product_b = tmp[1] * tmp1[1];
  assert(fabsf(oxb_vec2_getx(r) - product_a) < epsilon);
  assert(fabsf(oxb_vec2_gety(r) - product_b) < epsilon);  
 }

 printf("--div test\n");
 /* div test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2], tmp1[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  /* avoid division by zero */
  for(int l = 0; l < 2; l++) {
   if(fabsf(tmp1[l]) < epsilon)
    tmp1[l] = copysignf(epsilon, tmp1[l]);
  }
  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 mb = oxb_vec2_load(tmp1);
  oxb_vec2 r = oxb_vec2_div(ma, mb);
  
  float quotient_a = tmp[0] / tmp1[0];
  float quotient_b = tmp[1] / tmp1[1];
  assert(fabsf(oxb_vec2_getx(r) - quotient_a) < epsilon);
  assert(fabsf(oxb_vec2_gety(r) - quotient_b) < epsilon);  
 }

 printf("--fma test\n");
 /* fma test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2], tmp1[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 mb = oxb_vec2_load(tmp1);
  oxb_vec2 r = oxb_vec2_fma(ma, mb, ma);
  
  float ra = tmp[0] * tmp1[0] + tmp[0];
  float rb = tmp[1] * tmp1[1] + tmp[1];
  assert(fabsf(oxb_vec2_getx(r) - ra) < epsilon);
  assert(fabsf(oxb_vec2_gety(r) - rb) < epsilon);  
 }

 printf("--dot test\n");
 /* dot test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2], tmp1[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 mb = oxb_vec2_load(tmp1);
  float r = oxb_vec2_dot(ma, mb);
  
  float ra = tmp[0] * tmp1[0] + tmp[1] * tmp1[1];
  assert(fabsf(r - ra) < epsilon);
 }

 printf("--normalize test\n");
 /* nornalize test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 r = oxb_vec2_normalize(ma);
  float length = oxb_vec2_dot(r, r);
  assert(fabsf(length - 1.0f) < epsilon);
 }

 printf("--rperp test\n");
 /* perp test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 r = oxb_vec2_rperp(ma);
  float ra = tmp[1];
  float rb = -tmp[0];
  assert(fabsf(oxb_vec2_getx(r) - ra) < epsilon);
  assert(fabsf(oxb_vec2_gety(r) - rb) < epsilon);  
 }

 printf("--lperp test\n");
 /* perp test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 r = oxb_vec2_lperp(ma);
  float ra = -tmp[1];
  float rb = tmp[0];
  assert(fabsf(oxb_vec2_getx(r) - ra) < epsilon);
  assert(fabsf(oxb_vec2_gety(r) - rb) < epsilon);  
 }
 
 printf("--cross test\n");
 /* cross test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2], tmp1[2];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 mb = oxb_vec2_load(tmp1);
  float r = oxb_vec2_cross(ma, mb);
  float cc = tmp[0] * tmp1[1] - tmp[1] * tmp1[0];
  assert(fabsf(r - cc) < epsilon);
 }

 printf("--scross test\n");
 /* scross test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[2], tmp1;
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1 = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec2 ma = oxb_vec2_load(tmp);
  oxb_vec2 r = oxb_vec2_scross(tmp1, ma);
  float ra = -tmp1 * tmp[1];
  float rb = tmp1 * tmp[0]; 
  assert(fabsf(oxb_vec2_getx(r) - ra) < epsilon);
  assert(fabsf(oxb_vec2_gety(r) - rb) < epsilon);
 }
}


oxb_inline void VECTOR3_TEST() {
 const float epsilon = 1e-2f;
 const float min_range = -100.0f;
 const float max_range =  100.0f;
 const int loop_repeat = 100;
 
 printf("--splat test\n");
 /* splat test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  oxb_vec3 a = oxb_vec3_splat(tmp);
  assert(fabsf(oxb_vec3_getx(a) - tmp) < epsilon);
  assert(fabsf(oxb_vec3_gety(a) - tmp) < epsilon);  
  assert(fabsf(oxb_vec3_getz(a) - tmp) < epsilon);  
 }

 printf("--setx sety, getx, gety test\n");
 /* setx sety, getx, gety test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp_a = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_b = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_c = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec3 a = oxb_vec3_splat(0.0f);
  a = oxb_vec3_setx(a, tmp_a);
  a = oxb_vec3_sety(a, tmp_b);
  a = oxb_vec3_setz(a, tmp_c);
  assert(fabsf(oxb_vec3_getx(a) - tmp_a) < epsilon);
  assert(fabsf(oxb_vec3_gety(a) - tmp_b) < epsilon);  
  assert(fabsf(oxb_vec3_getz(a) - tmp_c) < epsilon);  
 }

 printf("--set and get test\n");
 /* set and get test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp_a = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_b = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_c = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec3 a = oxb_vec3_splat(0.0f);
  a = oxb_vec3_set(a, 0, tmp_a);
  a = oxb_vec3_set(a, 1, tmp_b);
  a = oxb_vec3_set(a, 2, tmp_c);
  assert(fabsf(oxb_vec3_get(a, 0) - tmp_a) < epsilon);
  assert(fabsf(oxb_vec3_get(a, 1) - tmp_b) < epsilon);  
  assert(fabsf(oxb_vec3_get(a, 2) - tmp_c) < epsilon);  
 }

 printf("--load store test\n");
 /* load store test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[3], tmp1[3];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = oxb_nan();
  tmp1[1] = oxb_nan();
  tmp1[2] = oxb_nan();
  oxb_vec3 a = oxb_vec3_load(tmp);
  
  assert(fabsf(oxb_vec3_get(a, 0) - tmp[0]) < epsilon);
  assert(fabsf(oxb_vec3_get(a, 1) - tmp[1]) < epsilon);  
  assert(fabsf(oxb_vec3_get(a, 2) - tmp[2]) < epsilon);  
  
  oxb_vec3_store(tmp1, a);

  assert(fabsf(tmp1[0] - tmp[0]) < epsilon);
  assert(fabsf(tmp1[1] - tmp[1]) < epsilon);  
  assert(fabsf(tmp1[2] - tmp[2]) < epsilon);  
 }

 printf("--neg test\n");
 /* neg test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp1 = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp2 = VECTOR_TEST_RANDOM(min_range, max_range);
  
  oxb_vec3 a = oxb_vec3_neg(OXB_VEC3_INIT(tmp, tmp1, tmp2));
  assert(fabsf(oxb_vec3_getx(a) - -tmp) < epsilon);
  assert(fabsf(oxb_vec3_gety(a) - -tmp1) < epsilon);  
  assert(fabsf(oxb_vec3_getz(a) - -tmp2) < epsilon);  
 }

 printf("--abs test\n");
 /* abs test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp1 = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp2 = VECTOR_TEST_RANDOM(min_range, max_range);
  
  oxb_vec3 a = oxb_vec3_abs(OXB_VEC3_INIT(tmp, tmp1, tmp2));
  assert(fabsf(oxb_vec3_getx(a) - fabsf(tmp)) < epsilon);
  assert(fabsf(oxb_vec3_gety(a) - fabsf(tmp1)) < epsilon);  
  assert(fabsf(oxb_vec3_getz(a) - fabsf(tmp2)) < epsilon);  
 }
 
 printf("--add test\n");
 /* add test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[3], tmp1[3];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec3 ma = oxb_vec3_load(tmp);
  oxb_vec3 mb = oxb_vec3_load(tmp1);
  oxb_vec3 r = oxb_vec3_add(ma, mb);
  
  float sum_a = tmp[0] + tmp1[0];
  float sum_b = tmp[1] + tmp1[1];
  float sum_c = tmp[2] + tmp1[2];
  assert(fabsf(oxb_vec3_getx(r) - sum_a) < epsilon);
  assert(fabsf(oxb_vec3_gety(r) - sum_b) < epsilon);  
  assert(fabsf(oxb_vec3_getz(r) - sum_c) < epsilon);  
 }

 printf("--sub test\n");
 /* sub test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[3], tmp1[3];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec3 ma = oxb_vec3_load(tmp);
  oxb_vec3 mb = oxb_vec3_load(tmp1);
  oxb_vec3 r = oxb_vec3_sub(ma, mb);
  
  float difference_a = tmp[0] - tmp1[0];
  float difference_b = tmp[1] - tmp1[1];
  float difference_c = tmp[2] - tmp1[2];
  assert(fabsf(oxb_vec3_getx(r) - difference_a) < epsilon);
  assert(fabsf(oxb_vec3_gety(r) - difference_b) < epsilon);  
  assert(fabsf(oxb_vec3_getz(r) - difference_c) < epsilon);  
 }

 printf("--mul test\n");
 /* mul test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[3], tmp1[3];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec3 ma = oxb_vec3_load(tmp);
  oxb_vec3 mb = oxb_vec3_load(tmp1);
  oxb_vec3 r = oxb_vec3_mul(ma, mb);
  
  float product_a = tmp[0] * tmp1[0];
  float product_b = tmp[1] * tmp1[1];
  float product_c = tmp[2] * tmp1[2];
  assert(fabsf(oxb_vec3_getx(r) - product_a) < epsilon);
  assert(fabsf(oxb_vec3_gety(r) - product_b) < epsilon);  
  assert(fabsf(oxb_vec3_getz(r) - product_c) < epsilon);  
 }

 printf("--fma test\n");
 /* fma test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[3], tmp1[3], tmp2[3];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);

  tmp2[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp2[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp2[2] = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec3 ma = oxb_vec3_load(tmp);
  oxb_vec3 mb = oxb_vec3_load(tmp1);
  oxb_vec3 mc = oxb_vec3_load(tmp2);
  oxb_vec3 r = oxb_vec3_fma(ma, mb, mc);
  
  float ra = tmp[0] * tmp1[0] + tmp2[0];
  float rb = tmp[1] * tmp1[1] + tmp2[1];
  float rc = tmp[2] * tmp1[2] + tmp2[2];
  assert(fabsf(oxb_vec3_getx(r) - ra) < epsilon);
  assert(fabsf(oxb_vec3_gety(r) - rb) < epsilon);  
  assert(fabsf(oxb_vec3_getz(r) - rc) < epsilon);  
 }

 printf("--div test\n");
 /* div test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[3], tmp1[3];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);

  /* avoid division by zero */
  for(int l = 0; l < 3; l++) {
   if(fabsf(tmp1[l]) < epsilon)
    tmp1[l] = copysignf(epsilon, tmp1[l]);
  }
  oxb_vec3 ma = oxb_vec3_load(tmp);
  oxb_vec3 mb = oxb_vec3_load(tmp1);
  oxb_vec3 r = oxb_vec3_div(ma, mb);

  float quotient_a = tmp[0] / tmp1[0];
  float quotient_b = tmp[1] / tmp1[1];
  float quotient_c = tmp[2] / tmp1[2];
  assert(fabsf(oxb_vec3_getx(r) - quotient_a) < epsilon);
  assert(fabsf(oxb_vec3_gety(r) - quotient_b) < epsilon);  
  assert(fabsf(oxb_vec3_getz(r) - quotient_c) < epsilon);  
 }

 printf("--dot test\n");
 /* dot test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[3], tmp1[3];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec3 ma = oxb_vec3_load(tmp);
  oxb_vec3 mb = oxb_vec3_load(tmp1);
  float r = oxb_vec3_dot(ma, mb);
  
  float ra = tmp[0] * tmp1[0] + tmp[1] * tmp1[1] + tmp[2] * tmp1[2];
  assert(fabsf(r - ra) < epsilon);
 }
 
 printf("--normalize test\n");
 /* nornalize test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[3];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  oxb_vec3 ma = oxb_vec3_load(tmp);
  oxb_vec3 r = oxb_vec3_normalize(ma);
  float length = oxb_vec3_dot(r, r);
  assert(fabsf(length - 1.0f) < epsilon);
 }

 printf("--cross test\n");
 /* cross test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[3], tmp1[3];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec3 ma = oxb_vec3_load(tmp);
  oxb_vec3 mb = oxb_vec3_load(tmp1);
  oxb_vec3 r = oxb_vec3_cross(ma, mb);
  
  float ca = tmp[1] * tmp1[2] - tmp[2] * tmp1[1];
  float cb = tmp[2] * tmp1[0] - tmp[0] * tmp1[2];
  float cc = tmp[0] * tmp1[1] - tmp[1] * tmp1[0];
  assert(fabsf(oxb_vec3_getx(r) - ca) < epsilon);
  assert(fabsf(oxb_vec3_gety(r) - cb) < epsilon);  
  assert(fabsf(oxb_vec3_getz(r) - cc) < epsilon);  
 }
}


void VECTOR4_TEST() {
 const float epsilon = 1e-2f;
 const float min_range = -100.0f;
 const float max_range =  100.0f;
 const int loop_repeat = 100;
 
 printf("--splat test\n");
 /* splat test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  oxb_vec4 a = oxb_vec4_splat(tmp);
  assert(fabsf(oxb_vec4_getx(a) - tmp) < epsilon);
  assert(fabsf(oxb_vec4_gety(a) - tmp) < epsilon);  
  assert(fabsf(oxb_vec4_getz(a) - tmp) < epsilon);
  assert(fabsf(oxb_vec4_getw(a) - tmp) < epsilon);  
 }

 printf("--setx sety, getx, gety test\n");
 /* setx sety, getx, gety test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp_a = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_b = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_c = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_d = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec4 a = oxb_vec4_splat(0.0f);
  a = oxb_vec4_setx(a, tmp_a);
  a = oxb_vec4_sety(a, tmp_b);
  a = oxb_vec4_setz(a, tmp_c);
  a = oxb_vec4_setw(a, tmp_d);
  assert(fabsf(oxb_vec4_getx(a) - tmp_a) < epsilon);
  assert(fabsf(oxb_vec4_gety(a) - tmp_b) < epsilon);  
  assert(fabsf(oxb_vec4_getz(a) - tmp_c) < epsilon);  
  assert(fabsf(oxb_vec4_getw(a) - tmp_d) < epsilon);  
 }
 
 printf("--set and get test\n");
 /* set and get test */
 for(int i = 0; i < loop_repeat; i++) {
 float tmp_a = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_b = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_c = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp_d = VECTOR_TEST_RANDOM(min_range, max_range);

  oxb_vec4 a = oxb_vec4_splat(0.0f);
  a = oxb_vec4_set(a, 0, tmp_a);
  a = oxb_vec4_set(a, 1, tmp_b);
  a = oxb_vec4_set(a, 2, tmp_c);
  a = oxb_vec4_set(a, 3, tmp_d);
  assert(fabsf(oxb_vec4_get(a, 0) - tmp_a) < epsilon);
  assert(fabsf(oxb_vec4_get(a, 1) - tmp_b) < epsilon);  
  assert(fabsf(oxb_vec4_get(a, 2) - tmp_c) < epsilon);  
  assert(fabsf(oxb_vec4_get(a, 3) - tmp_d) < epsilon);  
 }

 printf("--load store test\n");
 /* load store test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[4], tmp1[4];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[3] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = oxb_nan();
  tmp1[1] = oxb_nan();
  tmp1[2] = oxb_nan();
  tmp1[3] = oxb_nan();
  oxb_vec4 a = oxb_vec4_load(tmp);
  
  assert(fabsf(oxb_vec4_get(a, 0) - tmp[0]) < epsilon);
  assert(fabsf(oxb_vec4_get(a, 1) - tmp[1]) < epsilon);  
  assert(fabsf(oxb_vec4_get(a, 2) - tmp[2]) < epsilon);  
  assert(fabsf(oxb_vec4_get(a, 3) - tmp[3]) < epsilon);  
  
  oxb_vec4_store(tmp1, a);

  assert(fabsf(tmp1[0] - tmp[0]) < epsilon);
  assert(fabsf(tmp1[1] - tmp[1]) < epsilon);  
  assert(fabsf(tmp1[2] - tmp[2]) < epsilon);  
  assert(fabsf(tmp1[3] - tmp[3]) < epsilon);  
 }

 printf("--neg test\n");
 /* neg test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp1 = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp2 = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp3 = VECTOR_TEST_RANDOM(min_range, max_range);
  
  oxb_vec4 a = oxb_vec4_neg(OXB_VEC4_INIT(tmp, tmp1, tmp2, tmp3));
  assert(fabsf(oxb_vec4_getx(a) - -tmp) < epsilon);
  assert(fabsf(oxb_vec4_gety(a) - -tmp1) < epsilon);  
  assert(fabsf(oxb_vec4_getz(a) - -tmp2) < epsilon);  
  assert(fabsf(oxb_vec4_getw(a) - -tmp3) < epsilon);  
 }

 printf("--abs test\n");
 /* abs test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp1 = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp2 = VECTOR_TEST_RANDOM(min_range, max_range);
  float tmp3 = VECTOR_TEST_RANDOM(min_range, max_range);
  
  oxb_vec4 a = oxb_vec4_abs(OXB_VEC4_INIT(tmp, tmp1, tmp2, tmp3));
  assert(fabsf(oxb_vec4_getx(a) - fabsf(tmp)) < epsilon);
  assert(fabsf(oxb_vec4_gety(a) - fabsf(tmp1)) < epsilon);  
  assert(fabsf(oxb_vec4_getz(a) - fabsf(tmp2)) < epsilon);  
  assert(fabsf(oxb_vec4_getw(a) - fabsf(tmp3)) < epsilon);  
 }
 
 printf("--add test\n");
 /* add test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[4], tmp1[4];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[3] = VECTOR_TEST_RANDOM(min_range, max_range);

  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[3] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec4 ma = oxb_vec4_load(tmp);
  oxb_vec4 mb = oxb_vec4_load(tmp1);
  oxb_vec4 r = oxb_vec4_add(ma, mb);
  
  float sum_a = tmp[0] + tmp1[0];
  float sum_b = tmp[1] + tmp1[1];
  float sum_c = tmp[2] + tmp1[2];
  float sum_d = tmp[3] + tmp1[3];
  assert(fabsf(oxb_vec4_getx(r) - sum_a) < epsilon);
  assert(fabsf(oxb_vec4_gety(r) - sum_b) < epsilon);  
  assert(fabsf(oxb_vec4_getz(r) - sum_c) < epsilon);  
  assert(fabsf(oxb_vec4_getw(r) - sum_d) < epsilon);  
 }

 printf("--sub test\n");
 /* sub test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[4], tmp1[4];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[3] = VECTOR_TEST_RANDOM(min_range, max_range);

  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[3] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec4 ma = oxb_vec4_load(tmp);
  oxb_vec4 mb = oxb_vec4_load(tmp1);
  oxb_vec4 r = oxb_vec4_sub(ma, mb);
  
  float difference_a = tmp[0] - tmp1[0];
  float difference_b = tmp[1] - tmp1[1];
  float difference_c = tmp[2] - tmp1[2];
  float difference_d = tmp[3] - tmp1[3];
  assert(fabsf(oxb_vec4_getx(r) - difference_a) < epsilon);
  assert(fabsf(oxb_vec4_gety(r) - difference_b) < epsilon);  
  assert(fabsf(oxb_vec4_getz(r) - difference_c) < epsilon);  
  assert(fabsf(oxb_vec4_getw(r) - difference_d) < epsilon);  
 }
 
 printf("--mul test\n");
 /* mul test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[4], tmp1[4];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[3] = VECTOR_TEST_RANDOM(min_range, max_range);

  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[3] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec4 ma = oxb_vec4_load(tmp);
  oxb_vec4 mb = oxb_vec4_load(tmp1);
  oxb_vec4 r = oxb_vec4_mul(ma, mb);
  
  float product_a = tmp[0] * tmp1[0];
  float product_b = tmp[1] * tmp1[1];
  float product_c = tmp[2] * tmp1[2];
  float product_d = tmp[3] * tmp1[3];
  assert(fabsf(oxb_vec4_getx(r) - product_a) < epsilon);
  assert(fabsf(oxb_vec4_gety(r) - product_b) < epsilon);  
  assert(fabsf(oxb_vec4_getz(r) - product_c) < epsilon);  
  assert(fabsf(oxb_vec4_getw(r) - product_d) < epsilon);  
 }

 printf("--div test\n");
 /* div test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[4], tmp1[4];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[3] = VECTOR_TEST_RANDOM(min_range, max_range);

  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[3] = VECTOR_TEST_RANDOM(min_range, max_range);

  /* avoid division by zero */
  for(int l = 0; l < 4; l++) {
   if(fabsf(tmp1[l]) < epsilon)
    tmp1[l] = copysignf(epsilon, tmp1[l]);
  }
  oxb_vec4 ma = oxb_vec4_load(tmp);
  oxb_vec4 mb = oxb_vec4_load(tmp1);
  oxb_vec4 r = oxb_vec4_div(ma, mb);
  
  float quotient_a = tmp[0] / tmp1[0];
  float quotient_b = tmp[1] / tmp1[1];
  float quotient_c = tmp[2] / tmp1[2];
  float quotient_d = tmp[3] / tmp1[3];
  assert(fabsf(oxb_vec4_getx(r) - quotient_a) < epsilon);
  assert(fabsf(oxb_vec4_gety(r) - quotient_b) < epsilon);  
  assert(fabsf(oxb_vec4_getz(r) - quotient_c) < epsilon);  
  assert(fabsf(oxb_vec4_getw(r) - quotient_d) < epsilon);  
 }

 printf("--fma test\n");
 /* fma test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[4], tmp1[4], tmp2[4];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[3] = VECTOR_TEST_RANDOM(min_range, max_range);

  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[3] = VECTOR_TEST_RANDOM(min_range, max_range);

  tmp2[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp2[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp2[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp2[3] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec4 ma = oxb_vec4_load(tmp);
  oxb_vec4 mb = oxb_vec4_load(tmp1);
  oxb_vec4 mc = oxb_vec4_load(tmp2);
  oxb_vec4 r = oxb_vec4_fma(ma, mb, mc);
  
  float ra = tmp[0] * tmp1[0] + tmp2[0];
  float rb = tmp[1] * tmp1[1] + tmp2[1];
  float rc = tmp[2] * tmp1[2] + tmp2[2];
  float rd = tmp[3] * tmp1[3] + tmp2[3];
  assert(fabsf(oxb_vec4_getx(r) - ra) < epsilon);
  assert(fabsf(oxb_vec4_gety(r) - rb) < epsilon);  
  assert(fabsf(oxb_vec4_getz(r) - rc) < epsilon);  
  assert(fabsf(oxb_vec4_getw(r) - rd) < epsilon);  
 }

 printf("--dot test\n");
 /* dot test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[4], tmp1[4];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[3] = VECTOR_TEST_RANDOM(min_range, max_range);
  
  tmp1[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp1[3] = VECTOR_TEST_RANDOM(min_range, max_range);
 
  oxb_vec4 ma = oxb_vec4_load(tmp);
  oxb_vec4 mb = oxb_vec4_load(tmp1);
  float r = oxb_vec4_dot(ma, mb);
  
  float ra = tmp[0] * tmp1[0] + tmp[1] * tmp1[1] + tmp[2] * tmp1[2] + tmp[3] * tmp1[3];
  assert(fabsf(r - ra) < epsilon);
 }
 
 printf("--normalize test\n");
 /* normalize test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp[4];
  tmp[0] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[1] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[2] = VECTOR_TEST_RANDOM(min_range, max_range);
  tmp[3] = VECTOR_TEST_RANDOM(min_range, max_range);
  oxb_vec4 ma = oxb_vec4_load(tmp);
  oxb_vec4 r = oxb_vec4_normalize(ma);
  float length = oxb_vec4_dot(r, r);
  assert(fabsf(length - 1.0f) < epsilon);
 }
 
}



void oxb_vector_test() {
 printf("[vector2] test..\n");
 VECTOR2_TEST();
 printf("\n\n[vector3] test..\n");
 VECTOR3_TEST();
 printf("\n\n[vector4] test..\n");
 VECTOR4_TEST();
 printf("\n");
}

#endif



