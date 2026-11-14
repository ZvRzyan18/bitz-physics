#include "oxb1tz/core2d/object2d.h"
#include "oxb1tz/core2d/world2d.h"
#include "oxb1tz/core2d/island2d.h"
#include "oxb1tz/memory.h"
#include "oxb1tz/simd.h"
#include "oxb1tz/mask.h"


oxb_noinline static oxb_aabb2d update_aabb(const oxb_packedshape2d shape, const oxb_vec2 pos, const oxb_complex rot, const uint8_t shape_type) {
 oxb_aabb2d out;
 switch(shape_type) {
  case OXB_SHAPE_BOX_2D:
  {
   oxb_vec2 r = oxb_vec2_abs(rot);
   oxb_vec2 hx = oxb_vec2_splat(shape._box._half_size._arr[0]);
   oxb_vec2 hy = oxb_vec2_splat(shape._box._half_size._arr[1]);
   oxb_vec2 extents = oxb_vec2_fma(hx, r, oxb_vec2_mul(hy, oxb_vec2_swaplane(r)));
   oxb_vec2_store(out._min._arr, oxb_vec2_sub(pos, extents));
   oxb_vec2_store(out._max._arr, oxb_vec2_add(pos, extents));
  }
  break;
  case OXB_SHAPE_CIRCLE_2D:
  {
   oxb_vec2 extents = oxb_vec2_splat(shape._circle._radius);
   oxb_vec2_store(out._min._arr, oxb_vec2_sub(pos, extents));
   oxb_vec2_store(out._max._arr, oxb_vec2_add(pos, extents));
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 return out;
}


/**********************************************
*                   dynamic
**********************************************/

void oxb_dynamic2d_init(oxb_dynamic2d *oxb_restrict const body, const oxb_object2d_info *oxb_restrict const info) {
 oxb_memory_set(body, 0, sizeof(oxb_dynamic2d));
 
 oxb_vec2_store(body->_position._arr, info->_type._dynamic_body._position);
 oxb_vec2_store(body->_rotation._arr, info->_type._dynamic_body._rotation);
 oxb_vec2_store(body->_velocity._arr, info->_type._dynamic_body._velocity);

 body->_omega = info->_type._dynamic_body._omega;
/*
 body->_damping._arr[0] = oxb_log2(info->_type._dynamic_body._linear_damping);
 body->_damping._arr[1] = oxb_log2(info->_type._dynamic_body._angular_damping);
*/
 oxb_vec2_store(body->_damping._arr, oxb_vec2_log2(OXB_VEC2_INIT(info->_type._dynamic_body._linear_damping, info->_type._dynamic_body._angular_damping)));
 
 body->_friction._arr[0] = info->_type._dynamic_body._static_friction;
 body->_friction._arr[1] = info->_type._dynamic_body._dynamic_friction; 
 body->_restitution = info->_type._dynamic_body._restitution;

 body->_flags = info->_flags & (OXB_SHAPE_MASK_2D | OXB_FLAG_MASK_2D);

 body->_island_id = OXB_POOL_NULL_ID;
 /* calculate mass, inertia and copy shape data */
 uint8_t shape_type = body->_flags & OXB_SHAPE_MASK_2D;
 switch(shape_type) {
  case OXB_SHAPE_BOX_2D:
  {
   oxb_vec2_store(body->_shape._box._half_size._arr, info->_type._dynamic_body._shape._box._half_size);
   float mx = body->_shape._box._half_size._arr[0] + body->_shape._box._half_size._arr[0];
   float my = body->_shape._box._half_size._arr[1] + body->_shape._box._half_size._arr[1];
   float mass = mx * my * info->_type._dynamic_body._density;
   body->_inv_mass = 1.0f / mass;
   body->_inv_rotational_inertia = 1.0f / (mass * (mx * mx + my * my) / 12.0f);
  }
  break;
  case OXB_SHAPE_CIRCLE_2D:
  {
   body->_shape._circle._radius = info->_type._dynamic_body._shape._circle._radius;
   float area = 3.14159f * body->_shape._circle._radius * body->_shape._circle._radius;
   float mass = area * info->_type._dynamic_body._density;
   body->_inv_mass = 1.0f / mass;
   
   float inertia = 0.5f * mass * body->_shape._circle._radius * body->_shape._circle._radius;
   body->_inv_rotational_inertia = 1.0f / inertia;
  }
  break;
  default :
   assert(0);
   oxb_unreachable();
 }
 body->_aabb = update_aabb(body->_shape, info->_type._dynamic_body._position, info->_type._dynamic_body._rotation, shape_type);
}



void oxb_dynamic2d_integrate(oxb_dynamic2d *oxb_restrict const body, void *w, const float dt) {
 oxb_world2d *world = (oxb_world2d*)w;
 body->_color_bits = 0;
 if(body->_flags & OXB_FLAG_SLEEP_2D)
  return;
 
 oxb_vec2 dt_vec = oxb_vec2_splat(dt);
 oxb_vec2 position = oxb_vec2_load(body->_position._arr);
 oxb_complex rotation = oxb_vec2_load(body->_rotation._arr);
 oxb_vec2 velocity = oxb_vec2_load(body->_velocity._arr);
 float omega = body->_omega;
 oxb_vec2 linear_acceleration = world->_gravity;
 
 oxb_vec2 damping = oxb_vec2_exp2(oxb_vec2_mul(oxb_vec2_load(body->_damping._arr), dt_vec));
 oxb_vec2 linear_damping = oxb_vec2_splat(oxb_vec2_getx(damping));
 float angular_damping = oxb_vec2_gety(damping);

 oxb_compiler_memory_barrier();
 
 velocity = oxb_vec2_fma(linear_acceleration, dt_vec, velocity);
 
 velocity = oxb_vec2_mul(velocity, linear_damping);
 omega *= angular_damping;
 omega = oxb_max(oxb_min(omega, world->_max_omega), -world->_max_omega);

 /* lock the position and rotation */
 static const oxb_mask4 poslock_mask = OXB_MASK4_INIT((uint32_t)OXB_FLAG_LOCK_POSX_2D, (uint32_t)OXB_FLAG_LOCK_POSY_2D, (uint32_t)OXB_FLAG_LOCK_ROT_2D, 0);
 static const oxb_mask4 zero_mask = OXB_MASK4_INIT(0, 0, 0, 0);
 oxb_mask4 locks = oxb_mask4_splat((uint32_t)body->_flags);
 locks = oxb_mask4_equal(oxb_mask4_and(locks, poslock_mask), zero_mask);
 oxb_vec4 movement_tmp = OXB_VEC4_INIT(oxb_vec2_getx(velocity), oxb_vec2_gety(velocity), omega, 0.0f);
 movement_tmp = oxb_mask4_tovec4(oxb_mask4_and(oxb_mask4_fromvec4(movement_tmp), locks));
 velocity = oxb_vec4_low(movement_tmp);
 omega = oxb_vec4_getz(movement_tmp);


 float delta_omega = omega * dt;
 /* first order of taylor series */
 float real = 1.0f - 0.5f * delta_omega * delta_omega;
 float imag = delta_omega;
 oxb_complex omega_complex = OXB_VEC2_INIT(real, imag);

 rotation = oxb_complex_mul(omega_complex, rotation);
 rotation = oxb_vec2_normalize(rotation);

 position = oxb_vec2_fma(velocity, dt_vec, position);   


 oxb_compiler_memory_barrier();

 oxb_vec2_store(body->_position._arr, position);
 oxb_vec2_store(body->_rotation._arr, rotation);
 oxb_vec2_store(body->_velocity._arr, velocity);
 body->_omega = omega;

 body->_aabb = update_aabb(body->_shape, position, rotation, body->_flags & OXB_SHAPE_MASK_2D);
}

void oxb_dynamic2d_integrate4(oxb_dynamic2d *oxb_restrict const body, void *w, const float dt) {
 oxb_world2d *world = (oxb_world2d*)w;
 
 oxb_dynamic2d_batch4 batch = {0};
 
 for(size_t i = 0; i < 4; i++) {
  body[i]._color_bits = 0;
  batch._position_x = oxb_vec4_set(batch._position_x, i, body[i]._position._arr[0]);
  batch._position_y = oxb_vec4_set(batch._position_y, i, body[i]._position._arr[1]);

  batch._rotation_real = oxb_vec4_set(batch._rotation_real, i, body[i]._rotation._arr[0]);
  batch._rotation_imag = oxb_vec4_set(batch._rotation_imag, i, body[i]._rotation._arr[1]);

  batch._velocity_x = oxb_vec4_set(batch._velocity_x, i, body[i]._velocity._arr[0]);
  batch._velocity_y = oxb_vec4_set(batch._velocity_y, i, body[i]._velocity._arr[1]);

  batch._omega = oxb_vec4_set(batch._omega, i, body[i]._omega);
  batch._linear_damping = oxb_vec4_set(batch._linear_damping, i, body[i]._damping._arr[0]);
  batch._angular_damping = oxb_vec4_set(batch._angular_damping, i, body[i]._damping._arr[1]);
  batch._flags = oxb_mask4_set(batch._flags, i, body[i]._flags);
 }
 
 
 oxb_integrate2d_dynamic4(&batch, world->_gravity, oxb_vec4_splat(dt), world->_max_omega);
  
 for(size_t i = 0; i < 4; i++) {
  if(body[i]._flags & OXB_FLAG_SLEEP_2D)
   continue;
  body[i]._position._arr[0] = oxb_vec4_get(batch._position_x, i);
  body[i]._position._arr[1] = oxb_vec4_get(batch._position_y, i);

  body[i]._rotation._arr[0] = oxb_vec4_get(batch._rotation_real, i);
  body[i]._rotation._arr[1] = oxb_vec4_get(batch._rotation_imag, i);

  body[i]._velocity._arr[0] = oxb_vec4_get(batch._velocity_x, i);
  body[i]._velocity._arr[1] = oxb_vec4_get(batch._velocity_y, i);

  body[i]._omega = oxb_vec4_get(batch._omega, i);   
  body[i]._aabb = update_aabb(body[i]._shape, oxb_vec2_load(body[i]._position._arr), oxb_vec2_load(body[i]._rotation._arr), body[i]._flags & OXB_SHAPE_MASK_2D);
 }
}



void oxb_dynamic2d_collider(oxb_collider2d *oxb_restrict const collider, const oxb_dynamic2d *oxb_restrict const body) {
 collider->_shape = body->_shape;
 collider->_aabb = body->_aabb;
 collider->_allocation = body->_allocation;
 collider->_position = body->_position;
 collider->_rotation = body->_rotation;
 collider->_flags = (uint8_t)(body->_flags & OXB_SHAPE_MASK_2D);
}



void oxb_dynamic2d_updaterest(oxb_dynamic2d *oxb_restrict const body, void *w, const float dt) {
 oxb_world2d *world = (oxb_world2d*)w;
 /* resting state */
 const float vt = world->_linear_rest_threshold;
 const float vo = world->_angular_rest_threshold;

 oxb_vec2 velocity = oxb_vec2_load(body->_velocity._arr);
 if(!(body->_flags & OXB_FLAG_REST_2D)) {
  if(oxb_vec2_dot(velocity, velocity) < vt && oxb_abs(body->_omega) < vo) {
   body->_resting_time += dt;
   if(body->_resting_time > world->_rest_time_threshold) {
    body->_flags |= OXB_FLAG_REST_2D;
    velocity = oxb_vec2_mul(velocity, oxb_vec2_splat(0.3f));
    body->_omega *= 0.3f;
    body->_resting_time = 0.0f;
   }
  } else {
   body->_resting_time = 0.0f;
  }
 } else {
  if(oxb_vec2_dot(velocity, velocity) > vt || oxb_abs(body->_omega) > vo) {
   body->_resting_time = 0.0f;
   body->_flags &= ~OXB_FLAG_REST_2D;
   velocity = oxb_vec2_mul(velocity, oxb_vec2_splat(0.3f * dt));
   body->_omega *= 0.3f * dt;

  } 
 }
 oxb_vec2_store(body->_velocity._arr, velocity);
}

void oxb_dynamic2d_updatesleep(oxb_dynamic2d *oxb_restrict const body, void *w) {
 oxb_world2d *world = (oxb_world2d*)w;
 if(oxb_pool_id_isnull(body->_island_id)) {
  bool resting = (body->_flags & OXB_FLAG_REST_2D) != 0;
  body->_island_id = oxb_island2d_acquire(&world->_island, resting);
 } else {
  if(oxb_island2d_sleeping(&world->_island, body->_island_id)) {
   body->_flags |= OXB_FLAG_SLEEP_2D;
  } else {
   body->_flags &= ~OXB_FLAG_SLEEP_2D;
  }
 }
}


void oxb_dynamic2d_wake(oxb_dynamic2d *oxb_restrict const body, void *w) {
 oxb_world2d *world = (oxb_world2d*)w;

 const float vt = world->_linear_rest_threshold;
 const float vo = world->_angular_rest_threshold;
 oxb_vec2 velocity = oxb_vec2_load(body->_velocity._arr);

 if(oxb_vec2_dot(velocity, velocity) > vt || oxb_abs(body->_omega) > vo) {
  body->_resting_time = 0.0f;
  body->_flags &= ~OXB_FLAG_REST_2D;
  body->_flags &= ~OXB_FLAG_SLEEP_2D;
    
  oxb_container_id group_id = oxb_island2d_groupid(&world->_island, body->_island_id);
  oxb_island2d_group *group = oxb_container_get(&world->_island._group_container, group_id);
  group->_sleeping = false;
 }
}

void oxb_dynamic2d_forcewake(oxb_dynamic2d *oxb_restrict const body, void *w) {
 oxb_world2d *world = (oxb_world2d*)w;

 body->_resting_time = 0.0f;
 body->_flags &= ~OXB_FLAG_REST_2D;
 body->_flags &= ~OXB_FLAG_SLEEP_2D;
    
 oxb_container_id group_id = oxb_island2d_groupid(&world->_island, body->_island_id);
 oxb_island2d_group *group = oxb_container_get(&world->_island._group_container, group_id);
 group->_sleeping = false;
}


/*
 array of dynamic body to integrate
*/
void oxb_dynamic2d_integrate_iterate(oxb_dynamic2d *oxb_restrict const body, const size_t begin, const size_t end, void *w, const float dt) {
 size_t i = begin;
 for(; (i + 32) <= end; i += 32) {
  oxb_dynamic2d_integrate4(body + i +  0, w, dt);
  oxb_dynamic2d_integrate4(body + i +  4, w, dt);
  oxb_dynamic2d_integrate4(body + i +  8, w, dt);
  oxb_dynamic2d_integrate4(body + i + 12, w, dt);
  oxb_dynamic2d_integrate4(body + i + 16, w, dt);
  oxb_dynamic2d_integrate4(body + i + 20, w, dt);
  oxb_dynamic2d_integrate4(body + i + 24, w, dt);
  oxb_dynamic2d_integrate4(body + i + 28, w, dt);
 }
 for(; (i + 4) <= end; i += 4) {
  oxb_dynamic2d_integrate4(body + i, w, dt);
 }
 for(; i < end; i++) {
  oxb_dynamic2d_integrate(body + i, w, dt);
 }
}


/**********************************************
*                   kinematic
**********************************************/


void oxb_kinematic2d_init(oxb_kinematic2d *oxb_restrict const body, const oxb_object2d_info *oxb_restrict const info) {
 oxb_memory_set(body, 0, sizeof(oxb_kinematic2d));

 oxb_vec2_store(body->_position._arr, info->_type._kinematic_body._position);
 oxb_vec2_store(body->_rotation._arr, info->_type._kinematic_body._rotation);
 oxb_vec2_store(body->_velocity._arr, info->_type._kinematic_body._velocity);

 body->_omega = info->_type._kinematic_body._omega;
 body->_damping._arr[0] = oxb_log2(info->_type._kinematic_body._linear_damping);
 body->_damping._arr[1] = oxb_log2(info->_type._kinematic_body._angular_damping);

 oxb_vec2_store(body->_up_direction._arr, info->_type._kinematic_body._up_direction);

 body->_friction._arr[0] = info->_type._kinematic_body._static_friction;
 body->_friction._arr[1] = info->_type._kinematic_body._dynamic_friction; 
 body->_restitution = info->_type._kinematic_body._restitution;

 
 body->_flags = info->_flags & (OXB_SHAPE_MASK_2D | OXB_FLAG_MASK_2D);
 body->_priority = info->_type._kinematic_body._priority;

 /* calculate mass, inertia and copy shape data */
 uint8_t shape_type = body->_flags & OXB_SHAPE_MASK_2D;
 switch(shape_type) {
  case OXB_SHAPE_BOX_2D:
  {
   oxb_vec2_store(body->_shape._box._half_size._arr, info->_type._kinematic_body._shape._box._half_size);
  }
  break;
  case OXB_SHAPE_CIRCLE_2D:
  {
   body->_shape._circle._radius = info->_type._dynamic_body._shape._circle._radius;
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 body->_aabb = update_aabb(body->_shape, info->_type._kinematic_body._position, info->_type._kinematic_body._rotation, shape_type);
}


void oxb_kinematic2d_integrate(oxb_kinematic2d *oxb_restrict const body, const float dt) {
 body->_color_bits = 0;
 oxb_vec2 dt_vec = oxb_vec2_splat(dt);
 oxb_vec2 position = oxb_vec2_load(body->_position._arr);
 oxb_complex rotation = oxb_vec2_load(body->_rotation._arr);
 oxb_vec2 velocity = oxb_vec2_load(body->_velocity._arr);
 float omega = body->_omega;
 
 oxb_vec2 damping = oxb_vec2_exp2(oxb_vec2_mul(oxb_vec2_load(body->_damping._arr), dt_vec));
 oxb_vec2 linear_damping = oxb_vec2_splat(oxb_vec2_getx(damping));
 float angular_damping = oxb_vec2_gety(damping);

 oxb_compiler_memory_barrier();
  
 velocity = oxb_vec2_mul(velocity, linear_damping);
 omega *= angular_damping;
 omega = oxb_max(oxb_min(omega, 2.6f), -2.6f);


 /* lock the position and rotation */
 static const oxb_mask4 poslock_mask = OXB_MASK4_INIT((uint32_t)OXB_FLAG_LOCK_POSX_2D, (uint32_t)OXB_FLAG_LOCK_POSY_2D, (uint32_t)OXB_FLAG_LOCK_ROT_2D, 0);
 static const oxb_mask4 zero_mask = OXB_MASK4_INIT(0, 0, 0, 0);
 oxb_mask4 locks = oxb_mask4_splat((uint32_t)body->_flags);
 locks = oxb_mask4_equal(oxb_mask4_and(locks, poslock_mask), zero_mask);
 oxb_vec4 movement_tmp = OXB_VEC4_INIT(oxb_vec2_getx(velocity), oxb_vec2_gety(velocity), omega, 0.0f);
 movement_tmp = oxb_mask4_tovec4(oxb_mask4_and(oxb_mask4_fromvec4(movement_tmp), locks));
 velocity = oxb_vec4_low(movement_tmp);
 omega = oxb_vec4_getz(movement_tmp);


 float delta_omega = omega * dt;
 /* first order of taylor series */
 float real = 1.0f - 0.5f * delta_omega * delta_omega;
 float imag = delta_omega;
 oxb_complex omega_complex = OXB_VEC2_INIT(real, imag);

 rotation = oxb_complex_mul(omega_complex, rotation);
 rotation = oxb_vec2_normalize(rotation);
 
 position = oxb_vec2_fma(velocity, dt_vec, position);   


 oxb_compiler_memory_barrier();

 oxb_vec2_store(body->_position._arr, position);
 oxb_vec2_store(body->_rotation._arr, rotation);
 oxb_vec2_store(body->_velocity._arr, velocity);
 body->_omega = omega;

 body->_aabb = update_aabb(body->_shape, position, rotation, body->_flags & OXB_SHAPE_MASK_2D);
}


void oxb_kinematic2d_collider(oxb_collider2d *oxb_restrict const collider, const oxb_kinematic2d *oxb_restrict const body) {
 collider->_shape = body->_shape;
 collider->_aabb = body->_aabb;
 collider->_allocation = body->_allocation;
 collider->_position = body->_position;
 collider->_rotation = body->_rotation;
 collider->_flags = (uint8_t)(body->_flags & OXB_SHAPE_MASK_2D);
}

/**********************************************
*                   static
**********************************************/

void oxb_static2d_init(oxb_static2d *oxb_restrict const body, const oxb_object2d_info *oxb_restrict const info) {
 oxb_memory_set(body, 0, sizeof(oxb_static2d));

 oxb_vec2_store(body->_position._arr, info->_type._static_body._position);
 oxb_vec2_store(body->_rotation._arr, info->_type._static_body._rotation);
 
 body->_friction._arr[0] = info->_type._static_body._static_friction;
 body->_friction._arr[1] = info->_type._static_body._dynamic_friction; 
 body->_restitution = info->_type._static_body._restitution;

 body->_flags = info->_flags & OXB_SHAPE_MASK_2D;
 uint8_t shape_type = body->_flags;
 switch(shape_type) {
  case OXB_SHAPE_BOX_2D:
  {
   oxb_vec2_store(body->_shape._box._half_size._arr, info->_type._static_body._shape._box._half_size);
  }
  break;
  case OXB_SHAPE_CIRCLE_2D:
  {
   body->_shape._circle._radius = info->_type._dynamic_body._shape._circle._radius;
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 body->_aabb = update_aabb(body->_shape, info->_type._static_body._position, info->_type._static_body._rotation, shape_type);
}


void oxb_static2d_collider(oxb_collider2d *oxb_restrict const collider, const oxb_static2d *oxb_restrict const body) {
 collider->_shape = body->_shape;
 collider->_aabb = body->_aabb;
 collider->_allocation = body->_allocation;
 collider->_position = body->_position;
 collider->_rotation = body->_rotation;
 collider->_flags = body->_flags;
}



