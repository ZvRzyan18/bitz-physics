#include "oxb1tz/core2d/pool2d.h"
#include "oxb1tz/core2d/world2d.h"

/*
 multi type memory dense pool.
 see @container.c
*/


void oxb_pool2d_create(oxb_pool2d *oxb_restrict const pool, oxb_allocator *oxb_restrict const alloc, const size_t initial_reserve) {
 oxb_container_create(&pool->_containers[0], alloc, sizeof(oxb_dynamic2d), initial_reserve);
 oxb_container_create(&pool->_containers[1], alloc, sizeof(oxb_kinematic2d), initial_reserve);
 oxb_container_create(&pool->_containers[2], alloc, sizeof(oxb_static2d), initial_reserve);
}

void oxb_pool2d_destroy(oxb_pool2d *oxb_restrict const pool) {
 oxb_container_destroy(&pool->_containers[0]);
 oxb_container_destroy(&pool->_containers[1]);
 oxb_container_destroy(&pool->_containers[2]);
}


oxb_handle oxb_pool2d_acquire(oxb_pool2d *oxb_restrict const pool, const oxb_object2d_info *oxb_restrict const info) {
 size_t pool_index = (info->_flags & OXB_OBJECT_MASK_2D) >> 26;
 assert(pool_index < 3);
 oxb_container_id container_id = oxb_container_acquire(&pool->_containers[pool_index]);
 oxb_handle handle;
 handle._type = pool_index;
 handle._id = container_id._val;
 
 oxb_world2d *world = (oxb_world2d*)pool->_world;
 oxb_bvh2d *bvh = (oxb_bvh2d*)&world->_bvh2d;

 switch(handle._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *dynamic_body = (oxb_dynamic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_DYNAMIC_2D], container_id);
  	oxb_dynamic2d_init(dynamic_body, info);
  	dynamic_body->_allocation = handle;

  	dynamic_body->_bvh_id = oxb_bvh2d_insert(bvh, dynamic_body->_aabb, handle);
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *kinematic_body = (oxb_kinematic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_KINEMATIC_2D], container_id);
  	oxb_kinematic2d_init(kinematic_body, info);
  	kinematic_body->_allocation = handle;
  	
  	kinematic_body->_bvh_id = oxb_bvh2d_insert(bvh, kinematic_body->_aabb, handle);
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	oxb_static2d *static_body = (oxb_static2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_STATIC_2D], container_id);
  	oxb_static2d_init(static_body, info);
  	static_body->_allocation = handle;
  	
  	static_body->_bvh_id = oxb_bvh2d_insert(bvh, static_body->_aabb, handle);
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 return handle;
}


void oxb_pool2d_release(oxb_pool2d *oxb_restrict const pool, const oxb_handle id) {
 assert(id._id < 5);
 oxb_container_id container_id;
 container_id._val = id._id;
 oxb_container_release(&pool->_containers[id._type], container_id);

 oxb_world2d *world = (oxb_world2d*)pool->_world;
 oxb_bvh2d *bvh = (oxb_bvh2d*)&world->_bvh2d;

 switch(id._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *dynamic_body = (oxb_dynamic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_DYNAMIC_2D], container_id);
  	oxb_bvh2d_remove(bvh, dynamic_body->_bvh_id);
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *kinematic_body = (oxb_kinematic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_KINEMATIC_2D], container_id);
  	oxb_bvh2d_remove(bvh, kinematic_body->_bvh_id);
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	oxb_static2d *static_body = (oxb_static2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_STATIC_2D], container_id);
  	oxb_bvh2d_remove(bvh, static_body->_bvh_id);
  }
  break;
 }
}


void oxb_pool2d_getdata(oxb_pool2d *oxb_restrict const pool, oxb_object2d_data *oxb_restrict const dat, const oxb_handle id) {
 switch(id._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_container_id lookup;
  	lookup._val = id._id;
  	oxb_dynamic2d *body = (oxb_dynamic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_DYNAMIC_2D], lookup);
  	
  	dat->_position = oxb_vec2_load(body->_position._arr);
  	dat->_rotation = oxb_vec2_load(body->_rotation._arr);
  	dat->_velocity = oxb_vec2_load(body->_velocity._arr);
  	dat->_omega = body->_omega;
  	dat->_resting = (body->_flags & OXB_FLAG_REST_2D) != 0;
  	dat->_sleeping = (body->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_container_id lookup;
  	lookup._val = id._id;
  	oxb_kinematic2d *body = (oxb_kinematic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_KINEMATIC_2D], lookup);
  	
  	dat->_position = oxb_vec2_load(body->_position._arr);
  	dat->_rotation = oxb_vec2_load(body->_rotation._arr);
  	dat->_velocity = oxb_vec2_load(body->_velocity._arr);
  	dat->_omega = body->_omega;

  	dat->_resting = false;
  	dat->_sleeping = false;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	oxb_container_id lookup;
  	lookup._val = id._id;
  	oxb_static2d *body = (oxb_static2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_STATIC_2D], lookup);
  	
  	dat->_position = oxb_vec2_load(body->_position._arr);
  	dat->_rotation = oxb_vec2_load(body->_rotation._arr);
  	dat->_velocity = oxb_vec2_splat(0.0f);
  	dat->_omega = 0.0f;
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}


void oxb_pool2d_setdata(oxb_pool2d *oxb_restrict const pool, const oxb_handle id, const oxb_object2d_data *oxb_restrict const dat) {
 switch(id._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_container_id lookup;
  	lookup._val = id._id;
  	oxb_dynamic2d *body = (oxb_dynamic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_DYNAMIC_2D], lookup);
  	
  	oxb_vec2_store(body->_position._arr, dat->_position);
  	oxb_vec2_store(body->_rotation._arr, dat->_rotation);
  	oxb_vec2_store(body->_velocity._arr, dat->_velocity);
  	body->_omega = dat->_omega;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_container_id lookup;
  	lookup._val = id._id;
  	oxb_kinematic2d *body = (oxb_kinematic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_KINEMATIC_2D], lookup);
  	
  	oxb_vec2_store(body->_position._arr, dat->_position);
  	oxb_vec2_store(body->_rotation._arr, dat->_rotation);
  	oxb_vec2_store(body->_velocity._arr, dat->_velocity);
  	body->_omega = dat->_omega;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	oxb_container_id lookup;
  	lookup._val = id._id;
  	oxb_static2d *body = (oxb_static2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_STATIC_2D], lookup);
  	
  	oxb_vec2_store(body->_position._arr, dat->_position);
  	oxb_vec2_store(body->_rotation._arr, dat->_rotation);
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}


void oxb_pool2d_addforce(oxb_pool2d *oxb_restrict const pool, const oxb_handle id, const oxb_vec2 v) {
 oxb_world2d *world = (oxb_world2d*)pool->_world;

 switch(id._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_container_id lookup;
  	lookup._val = id._id;
  	oxb_dynamic2d *body = (oxb_dynamic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_DYNAMIC_2D], lookup);
  	
   oxb_vec2 velocity = oxb_vec2_load(body->_velocity._arr);
   oxb_vec2 inv_mass = oxb_vec2_splat(body->_inv_mass);
   oxb_vec2 acceleration = oxb_vec2_mul(v, inv_mass);
   velocity = oxb_vec2_fma(acceleration, oxb_vec2_splat(world->_dt), velocity);
   oxb_vec2_store(body->_velocity._arr, velocity);
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
   assert(0);
   oxb_unreachable();
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
   assert(0);
   oxb_unreachable();
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}


void oxb_pool2d_addtorque(oxb_pool2d *oxb_restrict const pool, const oxb_handle id, const float v) {
 oxb_world2d *world = (oxb_world2d*)pool->_world;

 switch(id._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_container_id lookup;
  	lookup._val = id._id;
  	oxb_dynamic2d *body = (oxb_dynamic2d*)oxb_container_get(&pool->_containers[OXB_POOL2D_TYPE_DYNAMIC_2D], lookup);
  	
   float omega = body->_omega;
   float inv_inertia = body->_inv_rotational_inertia;
   float acceleration = v * inv_inertia;
   omega = oxb_fma(acceleration, world->_dt, omega);
   body->_omega = omega;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
   assert(0);
   oxb_unreachable();
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
   assert(0);
   oxb_unreachable();
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}

