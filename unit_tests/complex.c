#include "complex.h"
#include "oxb1tz/complex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef OXB_TEST
oxb_inline float COMPLEX_TEST_RANDOM(float min, float max) {
 return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

void oxb_complex_test() {
 const float epsilon = 1e-3f;
 const float min_range = -100.0f;
 const float max_range =  100.0f;
 const int loop_repeat = 10;
 
 printf("--mul test\n");
 /* mul test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = COMPLEX_TEST_RANDOM(min_range, max_range);
  float tmp1 = COMPLEX_TEST_RANDOM(min_range, max_range);

  float tmp2 = COMPLEX_TEST_RANDOM(min_range, max_range);
  float tmp3 = COMPLEX_TEST_RANDOM(min_range, max_range);

  oxb_complex a = OXB_VEC2_INIT(tmp, tmp1);
  oxb_complex b = OXB_VEC2_INIT(tmp2, tmp3);
  oxb_complex r = oxb_complex_mul(a, b);
  
  float real = tmp * tmp2 - tmp1 * tmp3;
  float imag = tmp * tmp3 + tmp1 * tmp2;

  assert(fabsf(oxb_complex_real(r) - real) < epsilon);
  assert(fabsf(oxb_complex_imag(r) - imag) < epsilon);  
 }
 
 printf("--conj test\n");
 /* conj test */
 for(int i = 0; i < loop_repeat; i++) {
  float tmp = COMPLEX_TEST_RANDOM(min_range, max_range);
  float tmp1 = COMPLEX_TEST_RANDOM(min_range, max_range);

  oxb_complex r = OXB_VEC2_INIT(tmp, tmp1);
  r = oxb_complex_conj(r);
  
  float real = tmp;
  float imag = -tmp1;
  assert(fabsf(oxb_complex_real(r) - real) < epsilon);
  assert(fabsf(oxb_complex_imag(r) - imag) < epsilon);  
 }
 

}


#endif
