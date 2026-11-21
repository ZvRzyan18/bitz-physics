#ifndef OXB_MATRIX_H
#define OXB_MATRIX_H

#include "oxb1tz/types.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const oxb_mat2x2 OXB_MAT2X2_IDENTITY;
extern const oxb_mat3x3 OXB_MAT3X3_IDENTITY;

oxb_hot void oxb_mat2x2_store(float *oxb_restrict const arr, const oxb_mat2x2 m);
oxb_hot oxb_mat2x2 oxb_mat2x2_load(const float *oxb_restrict const arr);
oxb_hot oxb_mat2x2 oxb_mat2x2_transpose(const oxb_mat2x2 a);
oxb_hot oxb_mat2x2 oxb_mat2x2_mul(const oxb_mat2x2 a, const oxb_mat2x2 b);
oxb_hot oxb_mat2x2 oxb_mat2x2_inv(const oxb_mat2x2 a);
oxb_hot oxb_vec2 oxb_mat2x2_mulvec(const oxb_vec2 a, const oxb_mat2x2 b);


oxb_hot void oxb_mat3x3_store(float *oxb_restrict const arr, const oxb_mat3x3 m);
oxb_hot oxb_mat3x3 oxb_mat3x3_load(const float *oxb_restrict const arr);
oxb_hot oxb_mat3x3 oxb_mat3x3_transpose(const oxb_mat3x3 a);
oxb_hot oxb_mat3x3 oxb_mat3x3_mul(const oxb_mat3x3 a, const oxb_mat3x3 b);
oxb_hot oxb_mat3x3 oxb_mat3x3_inv(const oxb_mat3x3 a);
oxb_hot oxb_mat3x3 oxb_mat3x3_transform(const oxb_complex rot, const oxb_vec2 pos);
oxb_hot oxb_vec3 oxb_mat3x3_mulvec(const oxb_vec3 a, const oxb_mat3x3 b);


#ifdef __cplusplus
}
#endif

#endif


