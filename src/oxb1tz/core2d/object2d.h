#ifndef OXB_OBJECT2D_H
#define OXB_OBJECT2D_H

#include "oxb1tz/core2d.h"
#include "oxb1tz/types.h"
#include "oxb1tz/pool.h"
#include "oxb1tz/complex.h"
#include "oxb1tz/vector.h"



typedef struct {
 oxb_packedv2f _min, _max;
} oxb_aabb2d;



typedef union {
 struct {
  oxb_packedv2f _half_size;
 } _box;

 struct {
  float         _radius;
 } _circle;

 struct {
  float         _radius;
  float         _length;
 } _capsule;
} oxb_packedshape2d;


typedef struct {
 oxb_packedshape2d _shape;
 oxb_aabb2d        _aabb;
 oxb_packedv2f     _position;
 oxb_packedv2f     _rotation;
 oxb_packedv2f     _velocity;
 oxb_packedv2f     _damping;
 oxb_packedv2f     _friction;
 oxb_handle        _allocation;
 oxb_pool_id       _bvh_id;
 oxb_pool_id       _island_id;
 uint32_t          _color_bits;
 float             _omega;
 float             _inv_mass;
 float             _inv_rotational_inertia;
 float             _restitution;
 float             _resting_time;
 uint16_t          _flags;
} oxb_dynamic2d;


typedef struct {
 oxb_packedshape2d _shape;
 oxb_aabb2d        _aabb;
 oxb_packedv2f     _position;
 oxb_packedv2f     _rotation;
 oxb_packedv2f     _velocity;
 oxb_packedv2f     _damping;
 oxb_packedv2f     _friction;
 oxb_packedv2f     _up_direction;
 oxb_handle        _allocation;
 oxb_pool_id       _bvh_id;
 float             _omega;
 float             _restitution;
 uint32_t          _color_bits;
 uint16_t          _flags;
 uint8_t           _priority;
} oxb_kinematic2d;


typedef struct {
 oxb_packedshape2d _shape;
 oxb_aabb2d        _aabb;
 oxb_packedv2f     _position;
 oxb_packedv2f     _rotation;
 oxb_packedv2f     _friction;
 oxb_handle        _allocation;
 oxb_pool_id       _bvh_id;
 float             _restitution;
 uint8_t           _flags;
} oxb_static2d;



typedef struct {
 oxb_packedshape2d _shape;
 oxb_aabb2d        _aabb;
 oxb_packedv2f     _position;
 oxb_packedv2f     _rotation;
 oxb_handle        _allocation;
 uint8_t           _flags;
} oxb_collider2d;

void oxb_dynamic2d_init(oxb_dynamic2d *oxb_restrict const body, const oxb_object2d_info *oxb_restrict const info);
void oxb_dynamic2d_integrate(oxb_dynamic2d *oxb_restrict const body, oxb_vec2 gravity, const float dt);
void oxb_dynamic2d_collider(oxb_collider2d *oxb_restrict const collider, const oxb_dynamic2d *oxb_restrict const body);

void oxb_kinematic2d_init(oxb_kinematic2d *oxb_restrict const body, const oxb_object2d_info *oxb_restrict const info);
void oxb_kinematic2d_integrate(oxb_kinematic2d *oxb_restrict const body, const float dt);
void oxb_kinematic2d_collider(oxb_collider2d *oxb_restrict const collider, const oxb_kinematic2d *oxb_restrict const body);

void oxb_static2d_init(oxb_static2d *oxb_restrict const body, const oxb_object2d_info *oxb_restrict const info);
void oxb_static2d_collider(oxb_collider2d *oxb_restrict const collider, const oxb_static2d *oxb_restrict const body);

#endif

