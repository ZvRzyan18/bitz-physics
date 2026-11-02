#ifndef OXB_TYPES_H
#define OXB_TYPES_H

#include "oxb1tz/hint.h"
#include <stdbool.h>

/* primitive types of its engine */



/* can be aligned or padded */
/* 
 access through ._arr is strictly forbidden
 every lane access must go through setter and getter
*/
#ifdef OXB_NEON
typedef float32x2_t oxb_vec2;
typedef float32x4_t oxb_vec3;
typedef float32x4_t oxb_vec4;
#else
typedef struct {
 float _arr[2];
} oxb_vec2;
typedef struct {
 float _arr[3];
} oxb_vec3;
typedef struct {
 float _arr[4];
} oxb_vec4;
#endif


/*
 simd padded matrices
*/
typedef struct {
 oxb_vec2 _basis[2];
} oxb_mat2x2;
typedef struct {
 oxb_vec3 _basis[3];
} oxb_mat3x3;


/*
 complex number
*/
typedef oxb_vec2 oxb_complex;


/*
 packed variants
*/
typedef struct {
 float _arr[2];
} oxb_packedv2f;
typedef struct {
 float _arr[3];
} oxb_packedv3f;
typedef struct {
 float _arr[4];
} oxb_packedv4f;


/*
 id types
*/
#ifdef OXB_ID_32
typedef uint32_t oxb_id;
#define OXB_ID_MAX 0xFFFFFFFF
#else
typedef uint16_t oxb_id;
#define OXB_ID_MAX 0xFFFF
#endif

typedef struct {
 oxb_id  _id;
 uint8_t _type;
} oxb_handle;

static const oxb_handle OXB_HANDLE_NULL = {OXB_ID_MAX, 0xFF};
oxb_inline bool oxb_handle_isequal(oxb_handle a, oxb_handle b) {
 return a._type == b._type && a._id == b._id;
}

/*
 allocator type
*/
typedef struct {
 void* (*_alloc)(size_t m);
 void (*_free)(void *m);
 size_t _alignment;
} oxb_allocator;

#endif


