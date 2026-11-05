#include "oxb1tz/core2d/solver2d.h"
#include "oxb1tz/core2d/world2d.h"



/*
 Projected Gauss Seidel algorithm solver
 with soft constraints
*/
void oxb_solver2d_contact_prepare(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool, const oxb_solver2d_input *oxb_restrict const input) {
 uint32_t used_color = 0;
 bool both_kinematics = false;
 bool both_sleeping = true;
 
 void *a_obj, *b_obj;
 
 oxb_vec2 a_velocity;
 float a_inv_mass;
 float a_inv_inertia;
 float a_restitution;
 float a_omega;

 oxb_vec2 b_velocity;
 float b_inv_mass;
 float b_inv_inertia;
 float b_restitution;
 float b_omega;
  
 switch(m->_a._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	a_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)a_obj;
  	a_velocity = oxb_vec2_load(obj->_velocity._arr);
  	a_inv_mass = obj->_inv_mass;
  	a_inv_inertia = obj->_inv_rotational_inertia;
  	a_restitution = obj->_restitution;
  	a_omega = obj->_omega;
  	used_color |= obj->_color_bits;
  	both_sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	a_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)a_obj;
   switch(m->_b._type) {
   	case OXB_POOL2D_TYPE_DYNAMIC_2D:
    	a_velocity = oxb_vec2_splat(0.0f);
  	  a_inv_mass = 0.0f;
  	  a_restitution = obj->_restitution;
   	break;
   	case OXB_POOL2D_TYPE_KINEMATIC_2D:
   	 both_kinematics = true;
    	a_velocity = oxb_vec2_splat(0.0f);
  	  a_inv_mass = 0.0f;
  	  a_restitution = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_STATIC_2D:
    	a_velocity = oxb_vec2_load(obj->_velocity._arr);
  	  a_inv_mass = 1.0f;
  	  a_restitution = 0.0f;
   	break;
   }
   a_inv_inertia = 0.0f;
  	a_omega = 0.0f;
  	used_color |= obj->_color_bits;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	a_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_static2d *obj = (oxb_static2d*)a_obj;
  	a_velocity = oxb_vec2_splat(0.0f);
  	a_inv_mass = 0.0f;
  	a_inv_inertia = 0.0f;
  	a_restitution = obj->_restitution;
  	a_omega = 0.0f;
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }

 switch(m->_b._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	b_obj = oxb_pool2d_get(pool, m->_b);
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)b_obj;
  	b_velocity = oxb_vec2_load(obj->_velocity._arr);
  	b_inv_mass = obj->_inv_mass;
  	b_inv_inertia = obj->_inv_rotational_inertia;
  	b_restitution = obj->_restitution;
  	b_omega = obj->_omega;
  	used_color |= obj->_color_bits;
  	both_sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	b_obj = oxb_pool2d_get(pool, m->_b);
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)b_obj;
   switch(m->_a._type) {
   	case OXB_POOL2D_TYPE_DYNAMIC_2D:
    	b_velocity = oxb_vec2_splat(0.0f);
  	  b_inv_mass = 0.0f;
  	  b_restitution = obj->_restitution;
   	break;
   	case OXB_POOL2D_TYPE_KINEMATIC_2D:
   	 both_kinematics = true;
    	b_velocity = oxb_vec2_splat(0.0f);
  	  b_inv_mass = 0.0f;
  	  b_restitution = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_STATIC_2D:
    	b_velocity = oxb_vec2_load(obj->_velocity._arr);
  	  b_inv_mass = 1.0f;
  	  b_restitution = 0.0f;
   	break;
   }
   b_inv_inertia = 0.0f;
  	b_omega = 0.0f;
  	used_color |= obj->_color_bits;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	b_obj = oxb_pool2d_get(pool, m->_b);
  	oxb_static2d *obj = (oxb_static2d*)b_obj;
  	b_velocity = oxb_vec2_splat(0.0f);
  	b_inv_mass = 0.0f;
  	b_inv_inertia = 0.0f;
  	b_restitution = obj->_restitution;
  	b_omega = 0.0f;
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 
 if(both_sleeping)
  goto skip_solver;

 if(both_kinematics) {
 	oxb_kinematic2d *obj_a = (oxb_kinematic2d*)a_obj;
 	oxb_kinematic2d *obj_b = (oxb_kinematic2d*)b_obj;
  if(obj_a->_priority < obj_b->_priority) {
  	a_velocity = oxb_vec2_load(obj_a->_velocity._arr);
  	a_inv_mass = 1.0f;
  	a_omega = 1.0f;
  } else if(obj_a->_priority > obj_b->_priority) {
  	b_velocity = oxb_vec2_load(obj_b->_velocity._arr);
  	b_inv_mass = 1.0f;
  	b_omega = 1.0f;
  } else {
  	a_velocity = oxb_vec2_load(obj_a->_velocity._arr);
  	b_velocity = oxb_vec2_load(obj_b->_velocity._arr);
  	a_inv_mass = 0.5f;
  	b_inv_mass = 0.5f;
  	a_omega = 0.5f;
  	b_omega = 0.5f;
  }
 }

 float inv_mass = a_inv_mass + b_inv_mass;
 float e = oxb_max(a_restitution, b_restitution);

 
 for(uint8_t i = 0; i < m->_contact_count; i++) {
 	oxb_contact2d *const contact = m->_contacts + i;

  oxb_vec2 r1 = oxb_vec2_load(contact->_r1._arr);
  oxb_vec2 r2 = oxb_vec2_load(contact->_r2._arr);
  oxb_vec2 normal = oxb_vec2_load(contact->_normal._arr);
  oxb_vec2 tangent = oxb_vec2_rperp(normal);
  
  float r1_length_squared = oxb_vec2_dot(r1, r1);
  float r2_length_squared = oxb_vec2_dot(r2, r2);
 
  {
	  float rn1_squared = oxb_vec2_dot(r1, normal);
	  float rn2_squared = oxb_vec2_dot(r2, normal);
	  rn1_squared *= rn1_squared;
	  rn2_squared *= rn2_squared;
	 
		 float k_normal = oxb_fma(oxb_fma(a_inv_inertia, (r1_length_squared - rn1_squared), b_inv_inertia), (r2_length_squared - rn2_squared), inv_mass);
  	contact->_mass_normal = 1.0f / k_normal;
  }

  {
	  float rt1_squared = oxb_vec2_dot(r1, tangent);
	  float rt2_squared = oxb_vec2_dot(r2, tangent);
	  rt1_squared *= rt1_squared;
	  rt2_squared *= rt2_squared;
	 
	  float k_tangent = oxb_fma(oxb_fma(a_inv_inertia, (r1_length_squared - rt1_squared), b_inv_inertia), (r2_length_squared - rt2_squared), inv_mass);
	  contact->_mass_tangent = 1.0f / k_tangent;
  }
  /*
   Not correct. but by allowing only the linear relative velocity
   makes it more predictable and less likely to overshoot.
  */
  oxb_vec2 relative_vel = oxb_vec2_sub(b_velocity, a_velocity);
  float impact_speed = oxb_vec2_dot(relative_vel, normal);
  /* 
   hyperbolic tangent gives a smooth transition betweew low and max resitution coeffs


   tanh(x)
   1.0 ---------------------------------
                      ____
                  ___
              ___
           __
         __
       _
     -
    -
  -------------------------------------

  conditional by threshold
  -------------------------------------
                  |-------------------
                  |
                  |
                  |
                  |
                  |
   _______________
  -------------------------------------
  
  */
  float x = impact_speed;
  contact->_bias = (impact_speed * e) * oxb_tanh(oxb_min(x, 0.0f));
 }
 
 skip_solver:
 {
 }
 
 /* resting state */
 const float vt = 1.2f;
 const float vo = 0.4f;

 if(m->_a._type == OXB_POOL2D_TYPE_DYNAMIC_2D) {
 	oxb_dynamic2d *body = (oxb_dynamic2d*)a_obj;
  if(!(body->_flags & OXB_FLAG_REST_2D)) {
   if(oxb_vec2_dot(a_velocity, a_velocity) < vt && oxb_abs(a_omega) < vo) {
    body->_resting_time += input->_dt;
    if(body->_resting_time > 1.5f) {
     body->_flags |= OXB_FLAG_REST_2D;
     a_velocity = oxb_vec2_mul(a_velocity, oxb_vec2_splat(0.3f));
     a_omega *= 0.3f;
     body->_resting_time = 0.0f;
    }
   } else {
    body->_resting_time = 0.0f;
   }
  } else {
   if(oxb_vec2_dot(a_velocity, a_velocity) > vt || oxb_abs(a_omega) > vo) {
    body->_resting_time = 0.0f;
    body->_flags &= ~OXB_FLAG_REST_2D;
   } 
  }
 }



 if(m->_b._type == OXB_POOL2D_TYPE_DYNAMIC_2D) {
 	oxb_dynamic2d *body = (oxb_dynamic2d*)b_obj;
  if(!(body->_flags & OXB_FLAG_REST_2D)) {
   if(oxb_vec2_dot(b_velocity, b_velocity) < vt && oxb_abs(b_omega) < vo) {
    body->_resting_time += input->_dt;
    if(body->_resting_time > 1.5f) {
     body->_flags |= OXB_FLAG_REST_2D;
     b_velocity = oxb_vec2_mul(b_velocity, oxb_vec2_splat(0.3f));
     b_omega *= 0.3f;
     body->_resting_time = 0.0f;
    }
   } else {
    body->_resting_time = 0.0f;
   }
  } else {
   if(oxb_vec2_dot(b_velocity, b_velocity) > vt || oxb_abs(b_omega) > vo) {
    body->_resting_time = 0.0f;
    body->_flags &= ~OXB_FLAG_REST_2D;
   }
  }
 }
 
 
 
 /* color creation for parallel solver. */
 uint32_t color = 0;

 while(color < 32 && (used_color & (1u << color)))
  ++color;
 
 oxb_world2d *world = (oxb_world2d*)input->_ctx;

 m->_flags &= ~OXB_MANIFOLD2D_SOLO;
 m->_flags &= ~OXB_MANIFOLD2D_COLOR;
 uint32_t color_bit = 0;

 if(color >= 32) {
  m->_flags |= OXB_MANIFOLD2D_SOLO;
  world->_solo_manifolds++;
  color_bit = 0;
 } else {
  world->_colors[color]++;
  m->_flags |= (uint8_t)(color & OXB_MANIFOLD2D_COLOR);
  color_bit = 1u << color;
 }

 switch(m->_a._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)a_obj;
  	obj->_color_bits |= color_bit;
  	oxb_vec2_store(obj->_velocity._arr, a_velocity);
  	obj->_omega = a_omega;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)a_obj;
  	obj->_color_bits |= color_bit;
  }
  break;
 }

 switch(m->_b._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)b_obj;
  	obj->_color_bits |= color_bit;
  	oxb_vec2_store(obj->_velocity._arr, b_velocity);
  	obj->_omega = b_omega;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)b_obj;
  	obj->_color_bits |= color_bit;
  }
  break;
 }

}


void oxb_solver2d_contact_presolve(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool, const oxb_solver2d_input *oxb_restrict const input) {
 bool both_kinematics = false;
 bool both_sleeping = true;
 
 void *a_obj;
 oxb_vec2 a_velocity;
 float a_inv_mass;
 float a_inv_inertia;
 float a_omega;

 void *b_obj;
 oxb_vec2 b_velocity;
 float b_inv_mass;
 float b_inv_inertia;
 float b_omega;
 
 (void)input;

 switch(m->_a._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	a_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)a_obj;
  	a_velocity = oxb_vec2_load(obj->_velocity._arr);
  	a_inv_mass = obj->_inv_mass;
  	a_inv_inertia = obj->_inv_rotational_inertia;
  	a_omega = obj->_omega;
  	both_sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;  	
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	a_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)a_obj;
   switch(m->_b._type) {
   	case OXB_POOL2D_TYPE_DYNAMIC_2D:
    	a_velocity = oxb_vec2_splat(0.0f);
  	  a_inv_mass = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_KINEMATIC_2D:
   	 both_kinematics = true;
    	a_velocity = oxb_vec2_splat(0.0f);
  	  a_inv_mass = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_STATIC_2D:
    	a_velocity = oxb_vec2_load(obj->_velocity._arr);
  	  a_inv_mass = 1.0f;
   	break;
   }
   a_inv_inertia = 0.0f;
  	a_omega = 0.0f;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	a_obj = NULL;
  	a_velocity = oxb_vec2_splat(0.0f);
  	a_inv_mass = 0.0f;
  	a_inv_inertia = 0.0f;
  	a_omega = 0.0f;
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }

 switch(m->_b._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	b_obj = oxb_pool2d_get(pool, m->_b);
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)b_obj;
  	b_velocity = oxb_vec2_load(obj->_velocity._arr);
  	b_inv_mass = obj->_inv_mass;
  	b_inv_inertia = obj->_inv_rotational_inertia;
  	b_omega = obj->_omega;
  	both_sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	b_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)b_obj;
   switch(m->_a._type) {
   	case OXB_POOL2D_TYPE_DYNAMIC_2D:
    	b_velocity = oxb_vec2_splat(0.0f);
  	  b_inv_mass = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_KINEMATIC_2D:
   	 both_kinematics = true;
     b_velocity = oxb_vec2_splat(0.0f);
  	  b_inv_mass = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_STATIC_2D:
    	b_velocity = oxb_vec2_load(obj->_velocity._arr);
  	  b_inv_mass = 1.0f;
   	break;
   }
   b_inv_inertia = 0.0f;
  	b_omega = 0.0f;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	b_obj = NULL;
  	b_velocity = oxb_vec2_splat(0.0f);
  	b_inv_mass = 0.0f;
  	b_inv_inertia = 0.0f;
  	b_omega = 0.0f;
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 
 if(both_sleeping)
  return;

 bool both_dynamic = m->_a._type == m->_b._type && m->_a._type == OXB_POOL2D_TYPE_DYNAMIC_2D;
 if(both_dynamic) {
  oxb_dynamic2d *dynamic_a = (oxb_dynamic2d*)a_obj;
  oxb_dynamic2d *dynamic_b = (oxb_dynamic2d*)b_obj;
  if(dynamic_a->_flags & OXB_FLAG_SLEEP_2D && dynamic_b->_flags & OXB_FLAG_SLEEP_2D)
   return;
 }
 
 if(both_kinematics) {
 	oxb_kinematic2d *obj_a = (oxb_kinematic2d*)a_obj;
 	oxb_kinematic2d *obj_b = (oxb_kinematic2d*)b_obj;
  if(obj_a->_priority < obj_b->_priority) {
  	a_velocity = oxb_vec2_load(obj_a->_velocity._arr);
  	a_inv_mass = 1.0f;
  	a_omega = 1.0f;
  } else if(obj_a->_priority > obj_b->_priority) {
  	b_velocity = oxb_vec2_load(obj_b->_velocity._arr);
  	b_inv_mass = 1.0f;
  	b_omega = 1.0f;
  } else {
  	a_velocity = oxb_vec2_load(obj_a->_velocity._arr);
  	b_velocity = oxb_vec2_load(obj_b->_velocity._arr);
  	a_inv_mass = 0.5f;
  	b_inv_mass = 0.5f;
  	a_omega = 0.5f;
  	b_omega = 0.5f;
  }
 }
 
 for(uint8_t i = 0; i < m->_contact_count; i++) {
 	oxb_contact2d *const contact = m->_contacts + i;

  oxb_vec2 r1 = oxb_vec2_load(contact->_r1._arr);
  oxb_vec2 r2 = oxb_vec2_load(contact->_r2._arr);
  oxb_vec2 normal = oxb_vec2_load(contact->_normal._arr);
  oxb_vec2 tangent = oxb_vec2_rperp(normal);

  float j = contact->_normal_lambda;
  float jt = contact->_tangent_lambda;
	 oxb_vec2 impulse = oxb_vec2_add(oxb_vec2_mul(oxb_vec2_splat(j), normal), oxb_vec2_mul(oxb_vec2_splat(jt), tangent));
 	a_velocity = oxb_vec2_sub(a_velocity, oxb_vec2_mul(oxb_vec2_splat(a_inv_mass), impulse));
	 a_omega -= a_inv_inertia * oxb_vec2_cross(r1, impulse);
		 
	 b_velocity = oxb_vec2_add(b_velocity, oxb_vec2_mul(oxb_vec2_splat(b_inv_mass), impulse));
	 b_omega += b_inv_inertia * oxb_vec2_cross(r2, impulse);
 }


 switch(m->_a._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)a_obj;
  	oxb_vec2_store(obj->_velocity._arr, a_velocity);
   obj->_omega = a_omega;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)a_obj;
  	oxb_vec2_store(obj->_velocity._arr, a_velocity);
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 
 switch(m->_b._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)b_obj;
  	oxb_vec2_store(obj->_velocity._arr, b_velocity);
   obj->_omega = b_omega;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)b_obj;
  	oxb_vec2_store(obj->_velocity._arr, b_velocity);
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}


void oxb_solver2d_contact_solve(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool, const oxb_solver2d_input *oxb_restrict const input) {
 bool both_kinematics = false;
 bool both_sleeping = true;
 
 void *a_obj;
 oxb_vec2 a_velocity;
 float a_inv_mass;
 float a_inv_inertia;
 float a_omega;
 float a_static_friction;
 float a_dynamic_friction;
 
 void *b_obj;
 oxb_vec2 b_velocity;
 float b_inv_mass;
 float b_inv_inertia;
 float b_omega;
 float b_static_friction;
 float b_dynamic_friction;
  
 switch(m->_a._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	a_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)a_obj;
  	a_velocity = oxb_vec2_load(obj->_velocity._arr);
  	a_inv_mass = obj->_inv_mass;
  	a_inv_inertia = obj->_inv_rotational_inertia;
  	a_omega = obj->_omega;
   a_static_friction = obj->_friction._arr[0];
   a_dynamic_friction = obj->_friction._arr[1];
  	both_sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	a_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)a_obj;
   switch(m->_b._type) {
   	case OXB_POOL2D_TYPE_DYNAMIC_2D:
    	a_velocity = oxb_vec2_splat(0.0f);
  	  a_inv_mass = 0.0f;
     a_static_friction = obj->_friction._arr[0];
     a_dynamic_friction = obj->_friction._arr[1];
   	break;
   	case OXB_POOL2D_TYPE_KINEMATIC_2D:
   	 both_kinematics = true;
    	a_velocity = oxb_vec2_splat(0.0f);
  	  a_inv_mass = 0.0f;
     a_static_friction = 0.0f;
     a_dynamic_friction = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_STATIC_2D:
    	a_velocity = oxb_vec2_load(obj->_velocity._arr);
  	  a_inv_mass = 1.0f;
     a_static_friction = 0.0f;
     a_dynamic_friction = 0.0f;
   	break;
   }
   a_inv_inertia = 0.0f;
  	a_omega = 0.0f;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	a_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_static2d *obj = (oxb_static2d*)a_obj;
  	a_velocity = oxb_vec2_splat(0.0f);
  	a_inv_mass = 0.0f;
  	a_inv_inertia = 0.0f;
  	a_omega = 0.0f;
   a_static_friction = obj->_friction._arr[0];
   a_dynamic_friction = obj->_friction._arr[1];
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }

 switch(m->_b._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	b_obj = oxb_pool2d_get(pool, m->_b);
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)b_obj;
  	b_velocity = oxb_vec2_load(obj->_velocity._arr);
  	b_inv_mass = obj->_inv_mass;
  	b_inv_inertia = obj->_inv_rotational_inertia;
  	b_omega = obj->_omega;
   b_static_friction = obj->_friction._arr[0];
   b_dynamic_friction = obj->_friction._arr[1];
  	both_sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	b_obj = oxb_pool2d_get(pool, m->_a);
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)b_obj;
   switch(m->_a._type) {
   	case OXB_POOL2D_TYPE_DYNAMIC_2D:
    	b_velocity = oxb_vec2_splat(0.0f);
  	  b_inv_mass = 0.0f;
     b_static_friction = obj->_friction._arr[0];
     b_dynamic_friction = obj->_friction._arr[1];
   	break;
   	case OXB_POOL2D_TYPE_KINEMATIC_2D:
   	 both_kinematics = true;
    	b_velocity = oxb_vec2_splat(0.0f);
  	  b_inv_mass = 0.0f;
     b_static_friction = 0.0f;
     b_dynamic_friction = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_STATIC_2D:
    	b_velocity = oxb_vec2_load(obj->_velocity._arr);
  	  b_inv_mass = 1.0f;
     b_static_friction = 0.0f;
     b_dynamic_friction = 0.0f;
   	break;
   }
   b_inv_inertia = 0.0f;
  	b_omega = 0.0f;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	b_obj = oxb_pool2d_get(pool, m->_b);
  	oxb_static2d *obj = (oxb_static2d*)b_obj;
  	b_velocity = oxb_vec2_splat(0.0f);
  	b_inv_mass = 0.0f;
  	b_inv_inertia = 0.0f;
  	b_omega = 0.0f;
   b_static_friction = obj->_friction._arr[0];
   b_dynamic_friction = obj->_friction._arr[1];
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 
 if(both_sleeping)
  return;
 
 bool both_dynamic = m->_a._type == m->_b._type && m->_a._type == OXB_POOL2D_TYPE_DYNAMIC_2D;
 if(both_dynamic) {
  oxb_dynamic2d *dynamic_a = (oxb_dynamic2d*)a_obj;
  oxb_dynamic2d *dynamic_b = (oxb_dynamic2d*)b_obj;
  if(dynamic_a->_flags & OXB_FLAG_SLEEP_2D && dynamic_b->_flags & OXB_FLAG_SLEEP_2D)
   return;
 }
 
 if(both_kinematics) {
 	oxb_kinematic2d *obj_a = (oxb_kinematic2d*)a_obj;
 	oxb_kinematic2d *obj_b = (oxb_kinematic2d*)b_obj;
  if(obj_a->_priority < obj_b->_priority) {
  	a_velocity = oxb_vec2_load(obj_a->_velocity._arr);
  	a_inv_mass = 1.0f;
  	a_omega = 1.0f;
  } else if(obj_a->_priority > obj_b->_priority) {
  	b_velocity = oxb_vec2_load(obj_b->_velocity._arr);
  	b_inv_mass = 1.0f;
  	b_omega = 1.0f;
  } else {
  	a_velocity = oxb_vec2_load(obj_a->_velocity._arr);
  	b_velocity = oxb_vec2_load(obj_b->_velocity._arr);
  	a_inv_mass = 0.5f;
  	b_inv_mass = 0.5f;
  	a_omega = 0.5f;
  	b_omega = 0.5f;
  }
 }
 
 float dynamic_friction = oxb_sqrt(a_dynamic_friction * b_dynamic_friction); 
 float static_friction = oxb_sqrt(a_static_friction * b_static_friction);

 for(uint8_t i = 0; i < m->_contact_count; i++) {
 	oxb_contact2d *const contact = m->_contacts + i;
 	
  oxb_vec2 r1 = oxb_vec2_load(contact->_r1._arr);
  oxb_vec2 r2 = oxb_vec2_load(contact->_r2._arr);
  float depth = contact->_depth;

  oxb_vec2 normal = oxb_vec2_load(contact->_normal._arr);
  oxb_vec2 tangent = oxb_vec2_rperp(normal);

	 oxb_vec2 va = oxb_vec2_add(a_velocity, oxb_vec2_scross(a_omega, r1));
  oxb_vec2 vb = oxb_vec2_add(b_velocity, oxb_vec2_scross(b_omega, r2));
	 oxb_vec2 relative_vel = oxb_vec2_sub(vb, va);

		float vn = oxb_vec2_dot(relative_vel, normal);


  float slop = 0.03f;
  float penetration_error = 0.0f;
  
  if(depth < 0.0f) {
   float error = oxb_min((depth + slop), 0.0f);
   penetration_error = -error * input->_bias_coeff;
  }

  /* 
   logarithmic with damping-like behaviour 
   
   it uses natural logarithm because it never exceed linear line throughout its domain
   unlike with log2 graph curve which might inject extra energy.
  */
  penetration_error = oxb_log(penetration_error + 1.0f);
  penetration_error = -(vn - contact->_bias) + penetration_error;
  
  float j;
  {
   float mass = contact->_mass_normal * input->_mass_coeff;
	  j = mass * penetration_error;
   j -= input->_impulse_coeff * contact->_normal_lambda;
  }
  {
		 float accumulated_normal = contact->_normal_lambda;
	 	contact->_normal_lambda = oxb_max(accumulated_normal + j, 0.0f);
		 j = contact->_normal_lambda - accumulated_normal;
  }

	 oxb_vec2 impulse = oxb_vec2_mul(oxb_vec2_splat(j), normal);
	 
 	a_velocity = oxb_vec2_sub(a_velocity, oxb_vec2_mul(oxb_vec2_splat(a_inv_mass), impulse));
	 a_omega -= a_inv_inertia * oxb_vec2_cross(r1, impulse);
		 
	 b_velocity = oxb_vec2_add(b_velocity, oxb_vec2_mul(oxb_vec2_splat(b_inv_mass), impulse));
	 b_omega += b_inv_inertia * oxb_vec2_cross(r2, impulse);
 /*
  tangent / friction impulse
  
   ^
  /|\
   |
   | --- normal direction
   | 
   |       ______ tangent direction
   |       |
   •------------- >
  
  • resist sliding behaviour 
  • always perpendicular or 90°
 */
  
	 va = oxb_vec2_add(a_velocity, oxb_vec2_scross(a_omega, r1));
  vb = oxb_vec2_add(b_velocity, oxb_vec2_scross(b_omega, r2));
	 relative_vel = oxb_vec2_sub(vb, va);

		float vt = oxb_vec2_dot(relative_vel, tangent);
 	j = contact->_mass_tangent * -vt;

  float dynamic_limit = dynamic_friction * contact->_normal_lambda;
  float static_limit = static_friction * contact->_normal_lambda;
  {
   float prev_tangent = contact->_tangent_lambda;
   float target_tangent = prev_tangent + j;

   if(oxb_abs(target_tangent) > static_limit) {
    contact->_tangent_lambda = oxb_min(oxb_max(target_tangent, -dynamic_limit), dynamic_limit);
   } else {
    contact->_tangent_lambda = target_tangent;
   }
   j = contact->_tangent_lambda - prev_tangent;
  }


  impulse = oxb_vec2_mul(oxb_vec2_splat(j), tangent);
 	a_velocity = oxb_vec2_sub(a_velocity, oxb_vec2_mul(oxb_vec2_splat(a_inv_mass), impulse));
	 a_omega -= a_inv_inertia * oxb_vec2_cross(r1, impulse);
		 
	 b_velocity = oxb_vec2_add(b_velocity, oxb_vec2_mul(oxb_vec2_splat(b_inv_mass), impulse));
	 b_omega += b_inv_inertia * oxb_vec2_cross(r2, impulse);
 }


 switch(m->_a._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)a_obj;
  	oxb_vec2_store(obj->_velocity._arr, a_velocity);
   obj->_omega = a_omega;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)a_obj;
  	oxb_vec2_store(obj->_velocity._arr, a_velocity);
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 
 switch(m->_b._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)b_obj;
  	oxb_vec2_store(obj->_velocity._arr, b_velocity);
   obj->_omega = b_omega;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)b_obj;
  	oxb_vec2_store(obj->_velocity._arr, b_velocity);
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}

