#include "test_math.h"
#include "oxb1tz/math.h"
#include <math.h>

#ifdef OXB_TEST

#define OXB_PI 3.141592f

oxb_inline float VECTOR_TEST_RANDOM(float min, float max) {
 return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

void oxb_math_test() {
 /* estimated math, accurate enough for physics */
 const float epsilon = 1.5f;
 const float trig_epsilon = 3.9f;
 const float min_range = -50.0f;
 const float max_range =  50.0f;
 const int loop_repeat = 100;

 printf("--sin test\n");
 /* sin test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float ra = oxb_sin(tmp);
  float rb = sinf(tmp);
  assert(fabsf(ra - rb) < trig_epsilon);
 }

 printf("--cos test\n");
 /* cos test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float ra = oxb_cos(tmp);
  float rb = cosf(tmp);
  assert(fabsf(ra - rb) < trig_epsilon);
 }

 printf("--sincos test\n");
 /* sincos test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float ra[2];
  oxb_sincos(tmp, &ra[0], &ra[1]);
  float rb[2];
  rb[0] = sinf(tmp);
  rb[1] = cosf(tmp);
  assert(fabsf(ra[0] - rb[0]) < trig_epsilon);
  assert(fabsf(ra[1] - rb[1]) < trig_epsilon);
 }

 printf("--asin test\n");
 /* asin test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(-1.0f, 1.0f);
  float ra = oxb_asin(tmp);
  float rb = asinf(tmp);
  assert(fabsf(ra - rb) < trig_epsilon);
 }

 printf("--acos test\n");
 /* acos test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(-1.0f, 1.0f);
  float ra = oxb_acos(tmp);
  float rb = acosf(tmp);
  assert(fabsf(ra - rb) < trig_epsilon);
 }

 printf("--atan test\n");
 /* atan test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float ra = oxb_atan(tmp);
  float rb = atanf(tmp);
  assert(fabsf(ra - rb) < trig_epsilon);
 }

 printf("--atan2 test\n");
 /* atan2 test */
 for(int i = 0; i < 360; i++) {
  float tmp = (float)i * (OXB_PI / 180.0f);
  float ra = oxb_atan2(sinf(tmp), cosf(tmp));
  float rb = atan2f(sinf(tmp), cosf(tmp));
  assert(fabsf(ra - rb) < trig_epsilon);
 }


 printf("--log2 test\n");
 /* log2 test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(0.0f, max_range);
  float ra = oxb_log2(tmp);
  float rb = log2f(tmp);
  assert(fabsf(ra - rb) < epsilon);
 }


 printf("--exp2 test\n");
 /* exp2 test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(-5.0f, 10.0f);
  float ra = oxb_exp2(tmp);
  float rb = exp2f(tmp);
  assert(fabsf(ra - rb) < epsilon);
 }

 printf("--tanh test\n");
 /* tanh test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(min_range, max_range);
  float ra = oxb_tanh(tmp);
  float rb = tanhf(tmp);
  assert(fabsf(ra - rb) < epsilon);
 }


 printf("--log test\n");
 /* log test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(0.0f, max_range);
  float ra = oxb_log(tmp);
  float rb = logf(tmp);
  assert(fabsf(ra - rb) < epsilon);
 }


 printf("--exp test\n");
 /* exp test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(-5.0f, 5.0f);
  float ra = oxb_exp(tmp);
  float rb = expf(tmp);
  assert(fabsf(ra - rb) < epsilon);
 }

 /* non deterministic implementation (may vary depends on cpu) */
 printf("--rsqrt test\n");
 /* rsqrt test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(0.0f, max_range);
  float ra = oxb_rsqrt(tmp);
  float rb = 1.0f / sqrtf(tmp);
  assert(fabsf(ra - rb) < 0.8f);
 }

 printf("--sqrt test\n");
 /* sqrt test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = VECTOR_TEST_RANDOM(0.0f, max_range);
  float ra = oxb_sqrt(tmp);
  float rb = sqrtf(tmp);
  assert(fabsf(ra - rb) < 0.8f);
 }

}



#endif


