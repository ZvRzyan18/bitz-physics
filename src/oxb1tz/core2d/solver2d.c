#include "oxb1tz/core2d/solver2d.h"
#include "oxb1tz/core2d/world2d.h"
#include "oxb1tz/memory.h"
#include "oxb1tz/simd.h"


typedef struct {
 oxb_handle _handle;
 void*      _obj;
 oxb_vec2   _velocity;
 oxb_vec2   _friction;
 float      _omega;
 float      _inv_mass;
 float      _inv_inertia;
 float      _restitution;
 uint32_t   _color;
} oxb_object_tmp;

static void get_object_tmp(oxb_object_tmp *oxb_restrict const tmp, oxb_pool2d *oxb_restrict const pool, const bool invert_mass_sign, bool *oxb_restrict const both_sleeping_state) { 
 bool both_kinematics = false;
 bool both_sleeping = true;
 
 switch(tmp[0]._handle._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	tmp[0]._obj = oxb_pool2d_get(pool, tmp[0]._handle);
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)tmp[0]._obj;
  	tmp[0]._velocity = oxb_vec2_load(obj->_velocity._arr);
  	tmp[0]._omega = obj->_omega;
  	if(invert_mass_sign) {
  	 tmp[0]._inv_mass = -obj->_inv_mass;
  	 tmp[0]._inv_inertia = -obj->_inv_rotational_inertia;
  	} else {
  	 tmp[0]._inv_mass = obj->_inv_mass;
  	 tmp[0]._inv_inertia = obj->_inv_rotational_inertia;
  	}
  	tmp[0]._restitution = obj->_restitution;
  	tmp[0]._friction = oxb_vec2_load(obj->_friction._arr);
  	tmp[0]._color = obj->_color_bits;
  	both_sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	tmp[0]._obj = oxb_pool2d_get(pool, tmp[0]._handle);
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)tmp[0]._obj;
   switch(tmp[1]._handle._type) {
   	case OXB_POOL2D_TYPE_DYNAMIC_2D:
   	 tmp[0]._inv_mass = 0.0f;
  	  tmp[0]._restitution = obj->_restitution;
   	break;
   	case OXB_POOL2D_TYPE_KINEMATIC_2D:
   	 both_kinematics = true;
  	  tmp[0]._inv_mass = 0.0f;
  	  tmp[0]._restitution = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_STATIC_2D:
  	  if(invert_mass_sign) {
  	  tmp[0]._inv_mass = -1.0f;
  	  tmp[0]._inv_inertia = -1.0f;
  	  } else {
  	  tmp[0]._inv_mass = 1.0f;
  	  tmp[0]._inv_inertia = 1.0f;
  	  }
  	  tmp[0]._restitution = 0.0f;
   	break;
   }
  	tmp[0]._velocity = oxb_vec2_load(obj->_velocity._arr);
  	tmp[0]._omega = obj->_omega;

  	tmp[0]._friction = oxb_vec2_load(obj->_friction._arr);
  	tmp[0]._color = obj->_color_bits;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	tmp[0]._obj = oxb_pool2d_get(pool, tmp[0]._handle);
  	oxb_static2d *obj = (oxb_static2d*)tmp[0]._obj;
  	tmp[0]._velocity = oxb_vec2_splat(0.0f);
  	tmp[0]._omega = 0.0f;
  	tmp[0]._inv_mass = 0.0f;
  	tmp[0]._inv_inertia = 0.0f;
  	tmp[0]._restitution = obj->_restitution;
  	tmp[0]._friction = oxb_vec2_load(obj->_friction._arr);
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }


 switch(tmp[1]._handle._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	tmp[1]._obj = oxb_pool2d_get(pool, tmp[1]._handle);
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)tmp[1]._obj;
  	tmp[1]._velocity = oxb_vec2_load(obj->_velocity._arr);
  	tmp[1]._omega = obj->_omega;
   tmp[1]._inv_mass = obj->_inv_mass;
  	tmp[1]._inv_inertia = obj->_inv_rotational_inertia;
  	tmp[1]._restitution = obj->_restitution;
  	tmp[1]._friction = oxb_vec2_load(obj->_friction._arr);
  	tmp[1]._color = obj->_color_bits;
  	both_sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	tmp[1]._obj = oxb_pool2d_get(pool, tmp[1]._handle);
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)tmp[1]._obj;
   switch(tmp[0]._handle._type) {
   	case OXB_POOL2D_TYPE_DYNAMIC_2D:
  	  tmp[1]._inv_mass = 0.0f;
  	  tmp[1]._restitution = obj->_restitution;
   	break;
   	case OXB_POOL2D_TYPE_KINEMATIC_2D:
   	 both_kinematics = true;
  	  tmp[1]._inv_mass = 0.0f;
  	  tmp[1]._restitution = 0.0f;
   	break;
   	case OXB_POOL2D_TYPE_STATIC_2D:
  	  if(invert_mass_sign) {
  	  tmp[1]._inv_mass = -1.0f;
  	  tmp[1]._inv_inertia = -1.0f;
  	  } else {
  	  tmp[1]._inv_mass = 1.0f;
  	  tmp[1]._inv_inertia = 1.0f;
  	  }
  	  tmp[1]._restitution = 0.0f;
   	break;
   }
   tmp[1]._velocity = oxb_vec2_load(obj->_velocity._arr);
  	tmp[1]._omega = obj->_omega;

   tmp[1]._friction = oxb_vec2_load(obj->_friction._arr);
  	tmp[1]._color = obj->_color_bits;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  	tmp[1]._obj = oxb_pool2d_get(pool, tmp[1]._handle);
  	oxb_static2d *obj = (oxb_static2d*)tmp[1]._obj;
  	tmp[1]._velocity = oxb_vec2_splat(0.0f);
  	tmp[1]._omega = 0.0f;
  	tmp[1]._inv_mass = 0.0f;
  	tmp[1]._inv_inertia = 0.0f;
  	tmp[1]._restitution = obj->_restitution;
  	tmp[1]._friction = oxb_vec2_load(obj->_friction._arr);
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
 
 *both_sleeping_state = both_sleeping;

 if(both_kinematics) {
 	oxb_kinematic2d *obj_a = (oxb_kinematic2d*)tmp[0]._obj;
 	oxb_kinematic2d *obj_b = (oxb_kinematic2d*)tmp[1]._obj;
  if(obj_a->_priority < obj_b->_priority) {
  	tmp[0]._velocity = oxb_vec2_load(obj_a->_velocity._arr);
  	tmp[0]._omega = obj_a->_omega;
  	if(invert_mass_sign) {
  		tmp[0]._inv_mass = -1.0f;
  		tmp[0]._inv_inertia = -1.0f;
  	} else {
  		tmp[0]._inv_mass = 1.0f;
  		tmp[0]._inv_inertia = 1.0f;
  	}
  } else if(obj_a->_priority > obj_b->_priority) {
  	tmp[1]._velocity = oxb_vec2_load(obj_b->_velocity._arr);
  	tmp[1]._omega = obj_b->_omega;
  	tmp[1]._inv_mass = 1.0f;
  	tmp[1]._inv_inertia = 1.0f;
  } else {
  	tmp[0]._velocity = oxb_vec2_load(obj_a->_velocity._arr);
  	tmp[0]._omega = obj_a->_omega;
  	if(invert_mass_sign) {
  		tmp[0]._inv_mass = -0.5f;
  		tmp[0]._inv_inertia = -0.5f;
  	} else {
  		tmp[0]._inv_mass = 0.5f;
  		tmp[0]._inv_inertia = 0.5f;
  	}
  	
  	tmp[1]._velocity = oxb_vec2_load(obj_b->_velocity._arr);
  	tmp[1]._omega = obj_b->_omega;
  	tmp[1]._inv_mass = 0.5f;
  	tmp[1]._inv_inertia = 0.5f;
  }
 }
 
}


static void set_object_tmp(oxb_object_tmp *oxb_restrict const tmp) {
 switch(tmp[0]._handle._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)tmp[0]._obj;
  	oxb_vec2_store(obj->_velocity._arr, tmp[0]._velocity);
   obj->_omega = tmp[0]._omega;
   obj->_color_bits =	tmp[0]._color;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)tmp[0]._obj;
  	oxb_vec2_store(obj->_velocity._arr, tmp[0]._velocity);
   obj->_omega = tmp[0]._omega;
   obj->_color_bits =	tmp[0]._color;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }

 switch(tmp[1]._handle._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)tmp[1]._obj;
  	oxb_vec2_store(obj->_velocity._arr, tmp[1]._velocity);
   obj->_omega = tmp[1]._omega;
   obj->_color_bits =	tmp[1]._color;
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)tmp[1]._obj;
  	oxb_vec2_store(obj->_velocity._arr, tmp[1]._velocity);
   obj->_omega = tmp[1]._omega;
   obj->_color_bits =	tmp[1]._color;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}


static void set_object_color_tmp(oxb_object_tmp *oxb_restrict const tmp, const oxb_solver2d_input *oxb_restrict const input) {
 switch(tmp[0]._handle._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)tmp[0]._obj;
   obj->_color_bits =	tmp[0]._color;
   oxb_dynamic2d_updaterest(obj, input->_ctx, input->_dt);
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)tmp[0]._obj;
   obj->_color_bits =	tmp[0]._color;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }

 switch(tmp[1]._handle._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)tmp[1]._obj;
   obj->_color_bits =	tmp[1]._color;
   oxb_dynamic2d_updaterest(obj, input->_ctx, input->_dt);
  }
  break;
  case OXB_POOL2D_TYPE_KINEMATIC_2D:
  {
  	oxb_kinematic2d *obj = (oxb_kinematic2d*)tmp[1]._obj;
   obj->_color_bits =	tmp[1]._color;
  }
  break;
  case OXB_POOL2D_TYPE_STATIC_2D:
  {
  }
  break;
  default:
   assert(0);
   oxb_unreachable();
 }
}



bool oxb_solver2d_contact_sleeping(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool) {
 bool sleeping = true;
 switch(m->_a._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)oxb_pool2d_get(pool, m->_a);
   sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
 }

 switch(m->_b._type) {
  case OXB_POOL2D_TYPE_DYNAMIC_2D:
  {
  	oxb_dynamic2d *obj = (oxb_dynamic2d*)oxb_pool2d_get(pool, m->_b);
   sleeping &= (obj->_flags & OXB_FLAG_SLEEP_2D) != 0;
  }
  break;
 }
 return sleeping;
}

/*
 Projected Gauss Seidel algorithm solver
 with soft constraints
*/
void oxb_solver2d_contact_prepare(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool, const oxb_solver2d_input *oxb_restrict const input) {
 bool both_sleeping;

 oxb_object_tmp obj[2];
 oxb_memory_set(obj, 0, sizeof(oxb_object_tmp) * 2);
 
 obj[0]._handle = m->_a;
 obj[1]._handle = m->_b;
 get_object_tmp(obj, pool, false, &both_sleeping);
  
 if(both_sleeping)
  goto skip_solver;
 
 float inv_mass = obj[0]._inv_mass + obj[1]._inv_mass;
 float e = oxb_min(obj[0]._restitution, obj[1]._restitution);

 
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

   float k_normal = oxb_fma(obj[0]._inv_inertia, (r1_length_squared - rn1_squared), oxb_fma(obj[1]._inv_inertia, (r2_length_squared - rn2_squared), inv_mass));
  	contact->_mass_normal = 1.0f / k_normal;
  }

  {
	  float rt1_squared = oxb_vec2_dot(r1, tangent);
	  float rt2_squared = oxb_vec2_dot(r2, tangent);
	  rt1_squared *= rt1_squared;
	  rt2_squared *= rt2_squared;

   float k_tangent = oxb_fma(obj[0]._inv_inertia, (r1_length_squared - rt1_squared), oxb_fma(obj[1]._inv_inertia, (r2_length_squared - rt2_squared), inv_mass));
	  contact->_mass_tangent = 1.0f / k_tangent;
  }
  /*
   Not correct. but by allowing only the linear relative velocity
   makes it more predictable and less likely to overshoot.
  */
  oxb_vec2 relative_vel = oxb_vec2_sub(obj[1]._velocity, obj[0]._velocity);
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

 uint32_t used_color = obj[0]._color | obj[1]._color;
 
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

 obj[0]._color |= color_bit;
 obj[1]._color |= color_bit;
 set_object_color_tmp(obj, input);
}


void oxb_solver2d_contact_presolve(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool, const oxb_solver2d_input *oxb_restrict const input) {

 (void)input;

 bool both_sleeping;

 oxb_object_tmp obj[2];
 oxb_memory_set(obj, 0, sizeof(oxb_object_tmp) * 2);

 obj[0]._handle = m->_a;
 obj[1]._handle = m->_b;
 get_object_tmp(obj, pool, true, &both_sleeping);
  
 if(both_sleeping)
  return;
 
 for(uint8_t i = 0; i < m->_contact_count; i++) {
 	oxb_contact2d *const contact = m->_contacts + i;

  oxb_vec2 r1 = oxb_vec2_load(contact->_r1._arr);
  oxb_vec2 r2 = oxb_vec2_load(contact->_r2._arr);
  oxb_vec2 normal = oxb_vec2_load(contact->_normal._arr);
  oxb_vec2 tangent = oxb_vec2_rperp(normal);

  float j = contact->_normal_lambda;
  float jt = contact->_tangent_lambda;
  
	 oxb_vec2 impulse = oxb_vec2_add(oxb_vec2_mul(oxb_vec2_splat(j), normal), oxb_vec2_mul(oxb_vec2_splat(jt), tangent));
 	obj[0]._velocity = oxb_vec2_fma(oxb_vec2_splat(obj[0]._inv_mass), impulse, obj[0]._velocity);
	 obj[0]._omega = oxb_fma(obj[0]._inv_inertia, oxb_vec2_cross(r1, impulse), obj[0]._omega);
		 
	 obj[1]._velocity = oxb_vec2_fma(oxb_vec2_splat(obj[1]._inv_mass), impulse, obj[1]._velocity);
	 obj[1]._omega = oxb_fma(obj[1]._inv_inertia, oxb_vec2_cross(r2, impulse), obj[1]._omega);
 }

 set_object_tmp(obj);
}


void oxb_solver2d_contact_solve(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool, const oxb_solver2d_input *oxb_restrict const input) {
 bool both_sleeping;

 oxb_object_tmp obj[2];
 oxb_memory_set(obj, 0, sizeof(oxb_object_tmp) * 2);

 obj[0]._handle = m->_a;
 obj[1]._handle = m->_b;
 get_object_tmp(obj, pool, true, &both_sleeping);
  
 if(both_sleeping)
  return;

 oxb_vec2 friction = oxb_vec2_sqrt(oxb_vec2_mul(obj[0]._friction, obj[1]._friction));
 float static_friction = oxb_vec2_getx(friction);
 float dynamic_friction = oxb_vec2_gety(friction);
 
 for(uint8_t i = 0; i < m->_contact_count; i++) {
 	oxb_contact2d *const contact = m->_contacts + i;
 	
  oxb_vec2 r1 = oxb_vec2_load(contact->_r1._arr);
  oxb_vec2 r2 = oxb_vec2_load(contact->_r2._arr);
  float depth = contact->_depth;

  oxb_vec2 normal = oxb_vec2_load(contact->_normal._arr);
  oxb_vec2 tangent = oxb_vec2_rperp(normal);

	 oxb_vec2 va = oxb_vec2_add(obj[0]._velocity, oxb_vec2_scross(obj[0]._omega, r1));
  oxb_vec2 vb = oxb_vec2_add(obj[1]._velocity, oxb_vec2_scross(obj[1]._omega, r2));
	 oxb_vec2 relative_vel = oxb_vec2_sub(vb, va);

		float vn = oxb_vec2_dot(relative_vel, normal);


  float penetration_error = 0.0f;
  
  if(depth < 0.0f) {
   float error = oxb_min((depth + input->_penetration_slop), 0.0f);
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

 	obj[0]._velocity = oxb_vec2_fma(oxb_vec2_splat(obj[0]._inv_mass), impulse, obj[0]._velocity);
	 obj[0]._omega = oxb_fma(obj[0]._inv_inertia, oxb_vec2_cross(r1, impulse), obj[0]._omega);
		 
	 obj[1]._velocity = oxb_vec2_fma(oxb_vec2_splat(obj[1]._inv_mass), impulse, obj[1]._velocity);
	 obj[1]._omega = oxb_fma(obj[1]._inv_inertia, oxb_vec2_cross(r2, impulse), obj[1]._omega);
	
	
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
  
	 va = oxb_vec2_add(obj[0]._velocity, oxb_vec2_scross(obj[0]._omega, r1));
  vb = oxb_vec2_add(obj[1]._velocity, oxb_vec2_scross(obj[1]._omega, r2));
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
 	obj[0]._velocity = oxb_vec2_fma(oxb_vec2_splat(obj[0]._inv_mass), impulse, obj[0]._velocity);
	 obj[0]._omega = oxb_fma(obj[0]._inv_inertia, oxb_vec2_cross(r1, impulse), obj[0]._omega);
		 
	 obj[1]._velocity = oxb_vec2_fma(oxb_vec2_splat(obj[1]._inv_mass), impulse, obj[1]._velocity);
	 obj[1]._omega = oxb_fma(obj[1]._inv_inertia, oxb_vec2_cross(r2, impulse), obj[1]._omega);
 }
 set_object_tmp(obj);
}


void oxb_solver2d_contact_solve4(oxb_manifold2d *oxb_restrict const m[4], oxb_pool2d *oxb_restrict const pool, const oxb_solver2d_input *oxb_restrict const input) {
 oxb_object_tmp obj[4][2];
 oxb_memory_set(obj, 0, sizeof(oxb_object_tmp) * 4 * 2);
 bool both_sleeping = false;
 
 oxb_manifold2d_batch4 batch;
 oxb_memory_set(&batch, 0, sizeof(oxb_manifold2d_batch4));

 for(size_t i = 0; i < 4; i++) {
  obj[i][0]._handle = m[i]->_a;
  obj[i][1]._handle = m[i]->_b;
  get_object_tmp(obj[i], pool, true, &both_sleeping);

  oxb_vec2 friction = oxb_vec2_sqrt(oxb_vec2_mul(obj[i][0]._friction, obj[i][1]._friction));
  float static_friction = oxb_vec2_getx(friction);
  float dynamic_friction = oxb_vec2_gety(friction);

  batch._static_friction = oxb_vec4_set(batch._static_friction, i, static_friction);
  batch._dynamic_friction = oxb_vec4_set(batch._dynamic_friction, i, dynamic_friction);

  batch._velocity_x[0] = oxb_vec4_set(batch._velocity_x[0], i, oxb_vec2_getx(obj[i][0]._velocity));
  batch._velocity_y[0] = oxb_vec4_set(batch._velocity_y[0], i, oxb_vec2_gety(obj[i][0]._velocity));

  batch._velocity_x[1] = oxb_vec4_set(batch._velocity_x[1], i, oxb_vec2_getx(obj[i][1]._velocity));
  batch._velocity_y[1] = oxb_vec4_set(batch._velocity_y[1], i, oxb_vec2_gety(obj[i][1]._velocity));

  batch._omega[0] = oxb_vec4_set(batch._omega[0], i, obj[i][0]._omega);
  batch._omega[1] = oxb_vec4_set(batch._omega[1], i, obj[i][1]._omega);

  batch._inv_mass[0] = oxb_vec4_set(batch._inv_mass[0], i, obj[i][0]._inv_mass);
  batch._inv_mass[1] = oxb_vec4_set(batch._inv_mass[1], i, obj[i][1]._inv_mass);

  batch._inv_inertia[0] = oxb_vec4_set(batch._inv_inertia[0], i, obj[i][0]._inv_inertia);
  batch._inv_inertia[1] = oxb_vec4_set(batch._inv_inertia[1], i, obj[i][1]._inv_inertia);
  for(size_t j = 0; j < m[i]->_contact_count; j++) {
   oxb_contact2d *const contact = m[i]->_contacts + j;
   batch._r1_x[j] = oxb_vec4_set(batch._r1_x[j], i, contact->_r1._arr[0]);
   batch._r1_y[j] = oxb_vec4_set(batch._r1_y[j], i, contact->_r1._arr[1]);
   batch._r2_x[j] = oxb_vec4_set(batch._r2_x[j], i, contact->_r2._arr[0]);
   batch._r2_y[j] = oxb_vec4_set(batch._r2_y[j], i, contact->_r2._arr[1]);

   batch._normal_x[j] = oxb_vec4_set(batch._normal_x[j], i, contact->_normal._arr[0]);
   batch._normal_y[j] = oxb_vec4_set(batch._normal_y[j], i, contact->_normal._arr[1]);

   batch._depth[j] = oxb_vec4_set(batch._depth[j], i, contact->_depth);
   batch._bias[j] = oxb_vec4_set(batch._bias[j], i, contact->_bias);

   batch._mass_normal[j] = oxb_vec4_set(batch._mass_normal[j], i, contact->_mass_normal);
   batch._mass_tangent[j] = oxb_vec4_set(batch._mass_tangent[j], i, contact->_mass_tangent);

   batch._normal_lambda[j] = oxb_vec4_set(batch._normal_lambda[j], i, contact->_normal_lambda);
   batch._tangent_lambda[j] = oxb_vec4_set(batch._tangent_lambda[j], i, contact->_tangent_lambda);

  }  
 }
 
 oxb_contact2d_solve4(&batch, oxb_vec4_splat(input->_bias_coeff), oxb_vec4_splat(input->_mass_coeff), oxb_vec4_splat(input->_impulse_coeff), oxb_vec4_splat(input->_penetration_slop));

 for(size_t i = 0; i < 4; i++) {

  obj[i][0]._velocity = oxb_vec2_setx(obj[i][0]._velocity, oxb_vec4_get(batch._velocity_x[0], i));
  obj[i][0]._velocity = oxb_vec2_sety(obj[i][0]._velocity, oxb_vec4_get(batch._velocity_y[0], i));

  obj[i][1]._velocity = oxb_vec2_setx(obj[i][1]._velocity, oxb_vec4_get(batch._velocity_x[1], i));
  obj[i][1]._velocity = oxb_vec2_sety(obj[i][1]._velocity, oxb_vec4_get(batch._velocity_y[1], i));
  
  obj[i][0]._omega = oxb_vec4_get(batch._omega[0], i);
  obj[i][1]._omega = oxb_vec4_get(batch._omega[1], i);
  for(size_t j = 0; j < m[i]->_contact_count; j++) {
   oxb_contact2d *const contact = m[i]->_contacts + j;

   contact->_normal_lambda = oxb_vec4_get(batch._normal_lambda[j], i);
   contact->_tangent_lambda = oxb_vec4_get(batch._tangent_lambda[j], i);
  }
 	
 	set_object_tmp(obj[i]);
 }
}


