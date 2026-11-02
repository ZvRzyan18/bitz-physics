#include "oxb1tz/core2d/object2d.h"
#include "oxb1tz/memory.h"


oxb_noinline static oxb_aabb2d update_aabb(const oxb_packedshape2d shape, const oxb_vec2 pos, const oxb_complex rot, uint8_t shape_type) {
 oxb_aabb2d out;
 switch(shape_type & OXB_SHAPE_MASK_2D) {
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
 body->_damping._arr[0] = oxb_log2(info->_type._dynamic_body._linear_damping);
 body->_damping._arr[1] = oxb_log2(info->_type._dynamic_body._angular_damping);

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
  case OXB_SHAPE_CAPSULE_2D:
  break;
  case OXB_SHAPE_CONVEX_2D:
  break;
  case OXB_SHAPE_COMPOUND_2D:
  break;
  default :
   assert(0);
   oxb_unreachable();
 }
 body->_aabb = update_aabb(body->_shape, info->_type._dynamic_body._position, info->_type._dynamic_body._rotation, shape_type);
}



void oxb_dynamic2d_integrate(oxb_dynamic2d *oxb_restrict const body, oxb_vec2 gravity, const float dt) {

 body->_color_bits = 0;
 if(body->_flags & OXB_FLAG_SLEEP_2D)
  return;
 
 oxb_vec2 dt_vec = oxb_vec2_splat(dt);
 oxb_vec2 position = oxb_vec2_load(body->_position._arr);
 oxb_complex rotation = oxb_vec2_load(body->_rotation._arr);
 oxb_vec2 velocity = oxb_vec2_load(body->_velocity._arr);
 float omega = body->_omega;
 oxb_vec2 linear_acceleration = gravity;
 
 oxb_vec2 linear_damping = oxb_vec2_splat(oxb_exp2(body->_damping._arr[0] * dt));
 float angular_damping = oxb_exp2(body->_damping._arr[1] * dt);

 oxb_compiler_memory_barrier();
 
 velocity = oxb_vec2_fma(linear_acceleration, dt_vec, velocity);
 
 velocity = oxb_vec2_mul(velocity, linear_damping);
 omega *= angular_damping;


 /* resting state */
  const float vt = 1.2f;
  const float vo = 0.4f;

 if(!(body->_flags & OXB_FLAG_REST_2D)) {
  if(oxb_vec2_dot(velocity, velocity) < vt && oxb_abs(omega) < vo) {
   body->_resting_time += dt;
   if(body->_resting_time > 1.5f) {
    body->_flags |= OXB_FLAG_REST_2D;
    velocity = oxb_vec2_mul(velocity, oxb_vec2_splat(0.3f));
    omega *= 0.3f;
    body->_resting_time = 0.0f;
   }
  } else {
   body->_resting_time = 0.0f;
  }
 } else {
  if(oxb_vec2_dot(velocity, velocity) > vt || oxb_abs(omega) > vo) {
   body->_resting_time = 0.0f;
   body->_flags &= ~OXB_FLAG_REST_2D;
  }
 }
 


 float delta_omega = omega * dt;
 /* first order of taylor series */
 float real = 1.0f - 0.5f * delta_omega * delta_omega;
 float imag = delta_omega;
 oxb_complex omega_complex = OXB_VEC2_INIT(real, imag);

 rotation = oxb_complex_mul(omega_complex, rotation);
 rotation = oxb_vec2_normalize(rotation);
  
 position = oxb_vec2_fma(velocity, dt_vec, position);   


 oxb_compiler_memory_barrier();

 body->_aabb = update_aabb(body->_shape, position, rotation, body->_flags & OXB_SHAPE_MASK_2D);


 oxb_vec2_store(body->_position._arr, position);
 oxb_vec2_store(body->_rotation._arr, rotation);
 oxb_vec2_store(body->_velocity._arr, velocity);
 body->_omega = omega;
}


void oxb_dynamic2d_collider(oxb_collider2d *oxb_restrict const collider, const oxb_dynamic2d *oxb_restrict const body) {
 collider->_shape = body->_shape;
 collider->_aabb = body->_aabb;
 collider->_allocation = body->_allocation;
 collider->_position = body->_position;
 collider->_rotation = body->_rotation;
 collider->_flags = (uint8_t)(body->_flags & OXB_SHAPE_MASK_2D);
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
  case OXB_SHAPE_CAPSULE_2D:
  break;
  case OXB_SHAPE_CONVEX_2D:
  break;
  case OXB_SHAPE_COMPOUND_2D:
  break;
  default :
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
 
 oxb_vec2 linear_damping = oxb_vec2_splat(oxb_exp2(body->_damping._arr[0] * dt));
 float angular_damping = oxb_exp2(body->_damping._arr[1] * dt);

 oxb_compiler_memory_barrier();
  
 velocity = oxb_vec2_mul(velocity, linear_damping);
 omega *= angular_damping;


 float delta_omega = omega * dt;
 /* first order of taylor series */
 float real = 1.0f - 0.5f * delta_omega * delta_omega;
 float imag = delta_omega;
 oxb_complex omega_complex = OXB_VEC2_INIT(real, imag);

 rotation = oxb_complex_mul(omega_complex, rotation);
 rotation = oxb_vec2_normalize(rotation);
 
 position = oxb_vec2_fma(velocity, dt_vec, position);   


 oxb_compiler_memory_barrier();

 body->_aabb = update_aabb(body->_shape, position, rotation, body->_flags & OXB_SHAPE_MASK_2D);

 oxb_vec2_store(body->_position._arr, position);
 oxb_vec2_store(body->_rotation._arr, rotation);
 oxb_vec2_store(body->_velocity._arr, velocity);
 body->_omega = omega;
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
  case OXB_SHAPE_CAPSULE_2D:
  break;
  case OXB_SHAPE_CONVEX_2D:
  break;
  case OXB_SHAPE_COMPOUND_2D:
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



