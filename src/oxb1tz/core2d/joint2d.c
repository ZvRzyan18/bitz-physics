#include "oxb1tz/core2d/joint2d.h"
#include "oxb1tz/core2d/world2d.h"
#include "oxb1tz/core2d/solver2d.h"
#include "oxb1tz/core2d/island2d.h"
#include "oxb1tz/memory.h"
#include "oxb1tz/matrix.h"


/*
 mouse joint.


        • --------> point
        |
        |
        |------------> error line
  •_____|_______•
  |     |       |
  |     • - - - |--------> local anchor
  |             |
  |             |
  |             |
  |             | ----->  object (usually dyanmic2d)
  •_____________•
  
  it allows object to be draggable using mouse cursor or touch.
*/
/*
 TODO : handle kinematic and static ?
*/
oxb_cold static void oxb_mouse2d_init(oxb_joint2d *oxb_restrict const joint, const oxb_object2d_info *oxb_restrict const info) {
 oxb_memory_set(joint, 0, sizeof(oxb_joint2d));
 joint->_flags = info->_flags & OXB_JOINT_MASK_2D;
 oxb_vec2_store(joint->_type._mouse._anchor._arr, info->_type._mouse._anchor);
 oxb_vec2_store(joint->_type._mouse._point._arr, info->_type._mouse._point);
 joint->_type._mouse._object = info->_type._mouse._object;
 joint->_type._mouse._max_force = info->_type._mouse._max_force;
 joint->_type._mouse._previous_point = joint->_type._mouse._point;
 assert(info->_type._mouse._object._type == OXB_POOL2D_TYPE_DYNAMIC_2D);
}


oxb_hot static void oxb_mouse2d_presolve(oxb_joint2d *oxb_restrict const joint, const oxb_solver2d_input *oxb_restrict const input) {
 oxb_world2d *world = (oxb_world2d*)input->_ctx;
 void *obj;
 float inv_mass;
 float inv_inertia;
 oxb_vec2 rotation;
 oxb_vec2 velocity;
 float omega;
 
 switch(joint->_type._mouse._object._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
   obj = oxb_pool2d_get(&world->_pool2d, joint->_type._mouse._object);
   oxb_dynamic2d *body = (oxb_dynamic2d*)obj;
   inv_mass = body->_inv_mass;
   inv_inertia = body->_inv_rotational_inertia;
   rotation = oxb_vec2_load(body->_rotation._arr);
   velocity = oxb_vec2_load(body->_velocity._arr);
   omega = body->_omega;
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 
 oxb_vec2 anchor = oxb_vec2_load(joint->_type._mouse._anchor._arr);
 oxb_vec2 accumulated_lambda = oxb_vec2_load(joint->_type._mouse._accumulated_lambda._arr);
 
 oxb_compiler_memory_barrier();
 
 oxb_vec2 r = oxb_complex_mul(anchor, rotation);
 
 {
  float k11 = oxb_fma((inv_inertia * oxb_vec2_gety(r)), oxb_vec2_gety(r), inv_mass);
  float k12 = -inv_inertia * oxb_vec2_getx(r) * oxb_vec2_gety(r);
  float k22 = oxb_fma((inv_inertia * oxb_vec2_getx(r)), oxb_vec2_getx(r), inv_mass);
  oxb_mat2x2 mass_matrix;
  mass_matrix._basis[0] = OXB_VEC2_INIT(k11, k12);
  mass_matrix._basis[1] = OXB_VEC2_INIT(k12, k22);
  oxb_mat2x2_store(joint->_type._mouse._inv_mass, oxb_mat2x2_inv(mass_matrix));
 }
 
 /* warm start */
	velocity = oxb_vec2_add(velocity, oxb_vec2_mul(oxb_vec2_splat(inv_mass), accumulated_lambda));
	omega += inv_inertia * oxb_vec2_cross(r, accumulated_lambda);

 oxb_compiler_memory_barrier();
 oxb_vec2_store(joint->_type._mouse._global_anchor._arr, r);

 switch(joint->_type._mouse._object._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
   obj = oxb_pool2d_get(&world->_pool2d, joint->_type._mouse._object);
   oxb_dynamic2d *body = (oxb_dynamic2d*)obj;
   oxb_vec2_store(body->_velocity._arr, velocity);
   body->_omega = omega;

   /* force wake up if it is moving fast enough */
   const float vt = 1.2f;
   const float vo = 0.4f;

   if(oxb_vec2_dot(velocity, velocity) > vt || oxb_abs(omega) > vo) {
    body->_resting_time = 0.0f;
    body->_flags &= ~OXB_FLAG_REST_2D;
    body->_flags &= ~OXB_FLAG_SLEEP_2D;
    
    oxb_container_id group_id = oxb_island2d_groupid(&world->_island, body->_island_id);
    oxb_island2d_group *group = oxb_container_get(&world->_island._group_container, group_id);
    group->_sleeping = false;
   }
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}


oxb_hot static void oxb_mouse2d_solve(oxb_joint2d *oxb_restrict const joint, const oxb_solver2d_input *oxb_restrict const input) {
 oxb_world2d *world = (oxb_world2d*)input->_ctx;
 void *obj;
 float inv_mass;
 float inv_inertia;
 oxb_vec2 rotation;
 oxb_vec2 position;
 oxb_vec2 velocity;
 float omega;
 
 switch(joint->_type._mouse._object._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
   obj = oxb_pool2d_get(&world->_pool2d, joint->_type._mouse._object);
   oxb_dynamic2d *body = (oxb_dynamic2d*)obj;

   inv_mass = body->_inv_mass;
   inv_inertia = body->_inv_rotational_inertia;

   rotation = oxb_vec2_load(body->_rotation._arr);
   position = oxb_vec2_load(body->_position._arr);
   velocity = oxb_vec2_load(body->_velocity._arr);
   omega = body->_omega;
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }

 oxb_mat2x2 mass_coeff;
 mass_coeff._basis[0] = OXB_VEC2_INIT(input->_mass_coeff, 0.0f);
 mass_coeff._basis[1] = OXB_VEC2_INIT(0.0f, input->_mass_coeff);

 oxb_mat2x2 mass_matrix = oxb_mat2x2_load(joint->_type._mouse._inv_mass);


 oxb_vec2 accumulated_lambda = oxb_vec2_load(joint->_type._mouse._accumulated_lambda._arr);
 oxb_vec2 global_anchor = oxb_vec2_load(joint->_type._mouse._global_anchor._arr);
 oxb_vec2 point = oxb_vec2_load(joint->_type._mouse._point._arr);
 oxb_vec2 previous_point = oxb_vec2_load(joint->_type._mouse._previous_point._arr);

 oxb_compiler_memory_barrier();
 
 oxb_vec2 anchor = oxb_vec2_add(position, global_anchor);

 mass_matrix = oxb_mat2x2_mul(mass_coeff, mass_matrix);
 oxb_vec2 c = oxb_vec2_sub(anchor, point);
 c = oxb_vec2_mul(c, oxb_vec2_splat(input->_bias_coeff));

/*
 oxb_vec2 anchor_velocity = velocity + oxb_vec2_scross(omega, global_anchor);
 oxb_vec2 lambda = oxb_mat2x2_mulvec(oxb_vec2_neg(oxb_vec2_add(oxb_vec2_add(anchor_velocity, c), accumulated_lambda)), mass_matrix);
*/
 /* hinge but single object */
 oxb_vec2 target_velocity = oxb_vec2_mul(oxb_vec2_sub(point, previous_point), oxb_vec2_splat(input->_inv_dt) );
 oxb_vec2 anchor_velocity = oxb_vec2_add(velocity, oxb_vec2_scross(omega, global_anchor));
 oxb_vec2 relative_velocity = oxb_vec2_sub(anchor_velocity, target_velocity);
 oxb_vec2 lambda = oxb_mat2x2_mulvec(oxb_vec2_neg(oxb_vec2_add(relative_velocity, c)), mass_matrix);
 
 lambda = oxb_vec2_sub(lambda, oxb_vec2_mul(accumulated_lambda, oxb_vec2_splat(input->_impulse_coeff)));


 oxb_vec2 old_mpulse = accumulated_lambda;
 accumulated_lambda = oxb_vec2_add(accumulated_lambda, lambda);


 float max_force = joint->_type._mouse._max_force;
 float max_impulse = max_force * input->_dt;
 float len_squared = oxb_vec2_dot(accumulated_lambda, accumulated_lambda);

 if(len_squared > 1e-3f) {
  /* create smooth transition without hard clamp */
  float len = oxb_sqrt(len_squared);
  float ratio = len / max_impulse;
  float scale = max_impulse * oxb_tanh(ratio) / len;
  accumulated_lambda = oxb_vec2_mul(accumulated_lambda, oxb_vec2_splat(scale));
 }
 

 lambda = oxb_vec2_sub(accumulated_lambda, old_mpulse);

	velocity = oxb_vec2_add(velocity, oxb_vec2_mul(oxb_vec2_splat(inv_mass), lambda));
	omega += inv_inertia * oxb_vec2_cross(global_anchor, lambda);


 oxb_compiler_memory_barrier();
 
 oxb_vec2_store(joint->_type._mouse._accumulated_lambda._arr, accumulated_lambda);
 joint->_type._mouse._previous_point = joint->_type._mouse._point;

 switch(joint->_type._mouse._object._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
   obj = oxb_pool2d_get(&world->_pool2d, joint->_type._mouse._object);
   oxb_dynamic2d *body = (oxb_dynamic2d*)obj;
   oxb_vec2_store(body->_velocity._arr, velocity);
   body->_omega = omega;
  }
  break;
  default:
  assert(0);
  oxb_unreachable();
 }
}





void oxb_joint2d_init(oxb_joint2d *oxb_restrict const joint, const oxb_object2d_info *oxb_restrict const info) {
 switch(info->_flags & OXB_JOINT_MASK_2D) {
  case OXB_JOINT_MOUSE_2D:
   oxb_mouse2d_init(joint, info);
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}


void oxb_joint2d_presolve(oxb_joint2d *oxb_restrict const joint, const void *oxb_restrict const _input) {
 switch(joint->_flags & OXB_JOINT_MASK_2D) {
  case OXB_JOINT_MOUSE_2D:
   oxb_mouse2d_presolve(joint, (const oxb_solver2d_input*)_input);
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}

void oxb_joint2d_solve(oxb_joint2d *oxb_restrict const joint, const void *oxb_restrict const _input) {
 switch(joint->_flags & OXB_JOINT_MASK_2D) {
  case OXB_JOINT_MOUSE_2D:
   oxb_mouse2d_solve(joint, (const oxb_solver2d_input*)_input);
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}

