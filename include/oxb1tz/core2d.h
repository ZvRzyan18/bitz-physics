#ifndef OXB_CORE2D_H
#define OXB_CORE2D_H

#include "oxb1tz/types.h"

/* (object_type >> 26) */
typedef enum {
 OXB_OBJECT_DYNAMIC_2D =   0b00000000000000000000000000000000,
 OXB_OBJECT_KINEMATIC_2D = 0b00000100000000000000000000000000,
 OXB_OBJECT_STATIC_2D =    0b00001000000000000000000000000000,
 OXB_OBJECT_SENSOR_2D =    0b00001100000000000000000000000000,
 OXB_OBJECT_JOINT_2D =     0b00010000000000000000000000000000,
 OXB_OBJECT_MASK_2D =      0b11111100000000000000000000000000,
} oxb_object_type2d;


/* (shape_type >> 20) */
typedef enum {
 OXB_SHAPE_BOX_2D =        0b00000000000000000000000000000000,
 OXB_SHAPE_CIRCLE_2D =     0b00000000000000000000000000000001,
 OXB_SHAPE_CAPSULE_2D =    0b00000000000000000000000000000010,
 OXB_SHAPE_CONVEX_2D =     0b00000000000000000000000000000011,
 OXB_SHAPE_COMPOUND_2D =   0b00000000000000000000000000000100,
 OXB_SHAPE_MASK_2D =       0b00000000000000000000000000111111,
} oxb_shape_type2d;


typedef enum {
 OXB_FLAG_LOCK_POSX_2D =   0b00000000000000000000000001000000,
 OXB_FLAG_LOCK_POSY_2D =   0b00000000000000000000000010000000,
 OXB_FLAG_LOCK_ROT_2D =    0b00000000000000000000000100000000,
 OXB_FLAG_SLEEP_2D =       0b00000000000000000000001000000000,
 OXB_FLAG_REST_2D =        0b00000000000000000000010000000000, 
 OXB_FLAG_MASK_2D =        0b00000000000000000000011111000000,
} oxb_flag_type2d;



typedef struct oxb_world2d oxb_world2d;
typedef struct oxb_pool2d oxb_pool2d;


typedef struct {
 oxb_allocator _allocator;
 oxb_vec2 _gravity;
 uint8_t  _substeps;
} oxb_world2d_info;

typedef union {
 struct {
  oxb_vec2 _half_size;
 } _box;
 struct {
  float    _radius;
 } _circle;
} oxb_shape2d;

typedef struct {
 uint32_t    _flags;
 union {
  
  struct {
   oxb_shape2d _shape;
   oxb_vec2    _position;
   oxb_complex _rotation;
   oxb_vec2    _velocity;
   float       _omega;
   float       _density;
   
   float       _linear_damping;
   float       _angular_damping;
   float       _restitution;
   float       _static_friction;
   float       _dynamic_friction;
  } _dynamic_body;

  struct {
   oxb_shape2d _shape;
   oxb_vec2    _position;
   oxb_complex _rotation;
   oxb_vec2    _velocity;
   oxb_vec2    _up_direction;
   float       _omega;
   
   float       _linear_damping;
   float       _angular_damping;
   float       _restitution;
   float       _static_friction;
   float       _dynamic_friction;
   uint8_t     _priority;
  } _kinematic_body;

  struct {
   oxb_shape2d _shape;
   oxb_vec2    _position;
   oxb_complex _rotation;
   
   float       _restitution;
   float       _static_friction;
   float       _dynamic_friction;
  } _static_body;

  
 } _type;
} oxb_object2d_info;

typedef struct {
 oxb_vec2    _position;
 oxb_complex _rotation;
 oxb_vec2    _velocity;
 float       _omega;
 bool        _sleeping;
 bool        _resting;
} oxb_object2d_data;


#ifdef __cplusplus
extern "C" {
#endif



oxb_world2d* oxb_world2d_create(const oxb_world2d_info *oxb_restrict const info);
void oxb_world2d_destroy(oxb_world2d *oxb_restrict const world);
oxb_pool2d* oxb_world2d_getpool2d(oxb_world2d *oxb_restrict const world);
void oxb_world2d_step(oxb_world2d *oxb_restrict const world, const float dt);


oxb_handle oxb_pool2d_acquire(oxb_pool2d *oxb_restrict const pool, const oxb_object2d_info *oxb_restrict const info);
void oxb_pool2d_release(oxb_pool2d *oxb_restrict const pool, const oxb_handle id);
void oxb_pool2d_getdata(oxb_pool2d *oxb_restrict const pool, oxb_object2d_data *oxb_restrict const dat, const oxb_handle id);
void oxb_pool2d_setdata(oxb_pool2d *oxb_restrict const pool, const oxb_handle id, const oxb_object2d_data *oxb_restrict const dat);

void oxb_pool2d_addforce(oxb_pool2d *oxb_restrict const pool, const oxb_handle id, const oxb_vec2 v);
void oxb_pool2d_addtorque(oxb_pool2d *oxb_restrict const pool, const oxb_handle id, const float v);


#ifdef __cplusplus
}
#endif


#endif
 