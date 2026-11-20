#include "matrix.h"
#include "oxb1tz/matrix.h"
#include "oxb1tz/vector.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef OXB_TEST

oxb_inline float MATRIX_TEST_RANDOM(float min, float max) {
 return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

static void TEST_MAT2X2_INV(float out[4], const float m[4]) {
 const float det = m[0] * m[3] - m[1] * m[2];
 const float inv_det = 1.0f / det;
 out[0] =  m[3] * inv_det;
 out[1] = -m[1] * inv_det;
 out[2] = -m[2] * inv_det;
 out[3] =  m[0] * inv_det;
}

static void TEST_MAT3X3_INV(float inv[9], float m[9]) {
 float a00 = m[0], a01 = m[3], a02 = m[6];
 float a10 = m[1], a11 = m[4], a12 = m[7];
 float a20 = m[2], a21 = m[5], a22 = m[8];

 float c00 = a11 * a22 - a12 * a21;
 float c01 = a02 * a21 - a01 * a22;
 float c02 = a01 * a12 - a02 * a11;
 float det = a00 * c00 + a10 * c01 + a20 * c02;
 float inv_det = 1.0f / det;
 inv[0] = c00 * inv_det;
 inv[1] = c01 * inv_det;
 inv[2] = c02 * inv_det;

 inv[3] = (a12 * a20 - a10 * a22) * inv_det;
 inv[4] = (a00 * a22 - a02 * a20) * inv_det;
 inv[5] = (a02 * a10 - a00 * a12) * inv_det;

 inv[6] = (a10 * a21 - a11 * a20) * inv_det;
 inv[7] = (a01 * a20 - a00 * a21) * inv_det;
 inv[8] = (a00 * a11 - a01 * a10) * inv_det;
}

static void MAT3_MULV(float out[3], float a[3], float b[9]) {
 for(int row = 0; row < 3; row++) {
  out[row] = 0.0f;
  for(int col = 0; col < 3; col++) {
   out[row] += a[col] * b[col * 3 + row];
  }
 }
}


static void MAT2_MULV(float out[2], float a[2], float b[4]) {
 for(int row = 0; row < 2; row++) {
  out[row] = 0.0f;
  for(int col = 0; col < 2; col++) {
   out[row] += a[col] * b[col * 2 + row];
  }
 }
}


static void mat2x2_test() {
 const float epsilon = 1e-2f;
 const float min_range = -100.0f;
 const float max_range =  100.0f;
 const int loop_repeat = 10;

 printf("--transpose test\n");
 /* transpose test */
 for(int kk = 0; kk < loop_repeat; kk++) {
  float tmp[4];
  
  for(int i = 0; i < 4; i++) {
  	tmp[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  }
  
  oxb_mat2x2 aa;
  aa = oxb_mat2x2_load(tmp);
  aa = oxb_mat2x2_transpose(aa);
  
  float transposed1[4];
  float transposed2[4];
  oxb_mat2x2_store(transposed1, aa);  
  for(int i = 0; i < 2; i++) {
   for(int j = 0; j < 2; j++) {
    transposed2[j * 2 + i] = tmp[i * 2 + j];
   }
  }

  for(int i = 0; i < 4; i++) {
   assert(fabsf(transposed1[i] - transposed2[i]) < epsilon);
  }
 }
 
 printf("--mul test\n");
 /* mul test */
 for(int kk = 0; kk < loop_repeat; kk++) {
  float tmp[4], tmp1[4];
  
  for(int i = 0; i < 4; i++) {
  	tmp[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  	tmp1[i] = MATRIX_TEST_RANDOM(min_range, max_range);  
  }
  
  oxb_mat2x2 aa, bb;
  aa = oxb_mat2x2_load(tmp);
  bb = oxb_mat2x2_load(tmp1);
    
  aa = oxb_mat2x2_mul(aa, bb);
  
  float product1[4];
  float product2[4];
  oxb_mat2x2_store(product1, aa);
  
  
  for(int col = 0; col < 2; col++) {
   for(int row = 0; row < 2; row++) {
    product2[row + col * 2] = 0.0f;
    for(int k = 0; k < 2; k++) {
     product2[row + col * 2] = fmaf(tmp[row + k * 2], tmp1[k + col * 2], product2[row + col * 2]);
    }
   }
  }

  for(int i = 0; i < 4; i++) {
   assert(fabsf(product1[i] - product2[i]) < epsilon);
  }
 }



 printf("--inv test\n");
 /* inverse test */
 for(int kk = 0; kk < loop_repeat; kk++) {
  float tmp[4];
  
  for(int i = 0; i < 4; i++) {
  	tmp[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  }
  
  oxb_mat2x2 aa;
  aa = oxb_mat2x2_load(tmp);
  aa = oxb_mat2x2_inv(aa);
  
  float inv1[4];
  float inv2[4];
  oxb_mat2x2_store(inv1, aa);
  TEST_MAT2X2_INV(inv2, tmp);

  for(int i = 0; i < 4; i++) {
   if((isinf(inv1[i]) || isnan(inv1[i])) && (isinf(inv2[i]) || isnan(inv2[i])))
    continue;
   assert(fabsf(inv1[i] - inv2[i]) < epsilon);
  }
 }



 printf("--mul vec test\n");
 /* mul vec test */
 for(int kk = 0; kk < loop_repeat; kk++) {
  float tmp[4], tmp1[4];
  
  for(int i = 0; i < 4; i++) {
  	tmp[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  	tmp1[i] = MATRIX_TEST_RANDOM(min_range, max_range);  
  }
  
  oxb_mat2x2 aa;
  oxb_vec2 bb;
  aa = oxb_mat2x2_load(tmp);
  bb = oxb_vec2_load(tmp1);
  
  bb = oxb_mat2x2_mulvec(bb, aa);
  
  float product1[2];
  float product2[2];
  oxb_vec2_store(product1, bb);
  MAT2_MULV(product2, tmp1, tmp);
  
  for(int i = 0; i < 2; i++) {
   assert(fabsf(product1[i] - product2[i]) < epsilon);
  }
 }
}


static void mat3x3_test() {
 const float epsilon = 1e-2f;
 const float min_range = -100.0f;
 const float max_range =  100.0f;
 const int loop_repeat = 10;
 
 printf("--transpose test\n");
 /* transpose test */
 for(int kk = 0; kk < loop_repeat; kk++) {
  float tmp[9];
  
  for(int i = 0; i < 9; i++) {
  	tmp[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  }
  
  oxb_mat3x3 aa;
  aa = oxb_mat3x3_load(tmp);
  aa = oxb_mat3x3_transpose(aa);
  
  float transposed[9];
  float transposed1[9];
  oxb_mat3x3_store(transposed1, aa);
  for(int i = 0; i < 3; i++) {
   for(int j = 0; j < 3; j++) {
    transposed[j * 3 + i] = tmp[i * 3 + j];
   }
  }
  
  for(int i = 0; i < 9; i++) {
   assert(fabsf(transposed1[i] - transposed[i]) < epsilon);
  }
 }
 
 printf("--mul test\n");
 /* mul test */
 for(int kk = 0; kk < loop_repeat; kk++) {
  float tmp[9], tmp1[9];
  
  for(int i = 0; i < 9; i++) {
  	tmp[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  	tmp1[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  }

  oxb_mat3x3 aa;
  aa = oxb_mat3x3_load(tmp);

  oxb_mat3x3 bb;
  bb = oxb_mat3x3_load(tmp1);
  
  oxb_mat3x3 rr = oxb_mat3x3_mul(aa, bb);
  float product1[9], product2[9];
  oxb_mat3x3_store(product1, rr);

  for(int col = 0; col < 3; col++) {
   for(int row = 0; row < 3; row++) {
    product2[row + col * 3] = 0.0f;
    for(int k = 0; k < 3; k++) {
     product2[row + col * 3] = fmaf(tmp[row + k * 3], tmp1[k + col * 3], product2[row + col * 3]);
    }
   }
  }

  for(int i = 0; i < 9; i++) {
   assert(fabsf(product1[i] - product2[i]) < epsilon);
  }
 }
 
 printf("--inv test\n");
 /* inv test */
 for(int kk = 0; kk < loop_repeat; kk++) {
  float tmp[9];
  
  for(int i = 0; i < 9; i++) {
  	tmp[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  }
  
  oxb_mat3x3 aa;
  aa = oxb_mat3x3_load(tmp);
  aa = oxb_mat3x3_inv(aa);
  
  float r1[9];
  float r2[9];
  oxb_mat3x3_store(r1, aa);
  TEST_MAT3X3_INV(r2, tmp);
  
  for(int i = 0; i < 9; i++) {
   if((isinf(r1[i]) || isnan(r1[i])) && (isinf(r2[i]) || isnan(r2[i])))
    continue;
   assert(fabsf(r1[i] - r2[i]) < epsilon);
  }
 }
 
 printf("--vector matrix mul test\n");
 /* inv test */
 for(int kk = 0; kk < loop_repeat; kk++) {
  float tmp[9];
  float tmp1[3];
  for(int i = 0; i < 3; i++) {
  	tmp1[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  }
  for(int i = 0; i < 9; i++) {
  	tmp[i] = MATRIX_TEST_RANDOM(min_range, max_range);
  }
  
  oxb_mat3x3 aa;
  aa = oxb_mat3x3_load(tmp);
  oxb_vec3 bb = oxb_vec3_load(tmp1);
  bb = oxb_mat3x3_mulvec(bb, aa);
  
  float cc[3], dd[3];
  MAT3_MULV(cc, tmp1, tmp);
  oxb_vec3_store(dd, bb);
  for(int i = 0; i < 3; i++) {
   if((isinf(dd[i]) || isnan(dd[i])) && (isinf(cc[i]) || isnan(cc[i])))
    continue;
   assert(fabsf(dd[i] - cc[i]) < epsilon);
  }
 }
 
}

void oxb_matrix_test() {
 printf("[mat2x2] test...\n");
 mat2x2_test();
 printf("\n");
 printf("[mat3x3] test...\n");
 mat3x3_test();
 printf("\n");
}

#endif

