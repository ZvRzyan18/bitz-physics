#ifndef OXB_POOL2D_H
#define OXB_POOL2D_H

#include "oxb1tz/container.h"
#include "oxb1tz/core2d/object2d.h"
#include "oxb1tz/core2d/joint2d.h"
#include "oxb1tz/core2d.h"

/*
 OXB_OBJECT_DYNAMIC_2D =   0,
 OXB_OBJECT_KINEMATIC_2D = 1,
 OXB_OBJECT_STATIC_2D =    2,
 OXB_OBJECT_SENSOR_2D =    3,
 OXB_OBJECT_JOINT_2D =     4,
*/
typedef enum {
 OXB_POOL2D_TYPE_DYNAMIC_2D =   0,
 OXB_POOL2D_TYPE_KINEMATIC_2D = 1,
 OXB_POOL2D_TYPE_STATIC_2D =    2,
 OXB_POOL2D_TYPE_SENSOR_2D =    3,
 OXB_POOL2D_TYPE_JOINT_2D =     4,
} oxb_pool2d_type;

struct oxb_pool2d {
 oxb_container _containers[5];
 void *_world;
};

void oxb_pool2d_create(oxb_pool2d *oxb_restrict const pool, oxb_allocator *oxb_restrict const alloc, const size_t initial_reserves[5]);
void oxb_pool2d_destroy(oxb_pool2d *oxb_restrict const pool);

oxb_handle oxb_pool2d_acquire(oxb_pool2d *oxb_restrict const pool, const oxb_object2d_info *oxb_restrict const info);
void oxb_pool2d_release(oxb_pool2d *oxb_restrict const pool, const oxb_handle id);
void oxb_pool2d_getdata(oxb_pool2d *oxb_restrict const pool, oxb_object2d_data *oxb_restrict const dat, const oxb_handle id);
void oxb_pool2d_setdata(oxb_pool2d *oxb_restrict const pool, const oxb_handle id, const oxb_object2d_data *oxb_restrict const dat);

void oxb_pool2d_addforce(oxb_pool2d *oxb_restrict const pool, const oxb_handle id, const oxb_vec2 v);
void oxb_pool2d_addtorque(oxb_pool2d *oxb_restrict const pool, const oxb_handle id, const float v);

oxb_inline void* oxb_pool2d_get(oxb_pool2d *oxb_restrict const pool, const oxb_handle id) {
 assert(id._type < 3);
 oxb_container_id container_id;
 container_id._val = id._id;
 return oxb_container_get(&pool->_containers[id._type], container_id);
}

oxb_inline void oxb_pool2d_collider(oxb_pool2d *oxb_restrict const pool, oxb_collider2d *oxb_restrict const coll, const oxb_handle id) {
 oxb_container_id lookup;
 lookup._val = id._id;
 switch(id._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
   oxb_dynamic2d_collider(coll, (oxb_dynamic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_DYNAMIC_2D], lookup));
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
   oxb_kinematic2d_collider(coll, (oxb_kinematic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_KINEMATIC_2D], lookup));
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
   oxb_static2d_collider(coll, (oxb_static2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_STATIC_2D], lookup));
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}

#endif
