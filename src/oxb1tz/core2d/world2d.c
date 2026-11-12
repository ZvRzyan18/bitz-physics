#include "oxb1tz/core2d/world2d.h"
#include "oxb1tz/memory.h"


oxb_noinline static void broadphase_query(oxb_handle ha, oxb_handle hb, void *w) {
 oxb_world2d *world = (oxb_world2d*)w;
 
 /* skip : static vs static */
 const bool a_static = ha._type == OXB_POOL2D_TYPE_STATIC_2D;
 const bool b_static = hb._type == OXB_POOL2D_TYPE_STATIC_2D;
 if(a_static && b_static)
  return;

 oxb_collider2d a, b;
 oxb_pool2d_collider(&world->_pool2d, &a, ha);
 oxb_pool2d_collider(&world->_pool2d, &b, hb);

 if((a._aabb._max._arr[0] < b._aabb._min._arr[0] || a._aabb._min._arr[0] > b._aabb._max._arr[0] || a._aabb._max._arr[1] < b._aabb._min._arr[1] || a._aabb._min._arr[1] > b._aabb._max._arr[1]))
  return;

 
 oxb_contacthash2d_node *node = oxb_contacthash2d_get(&world->_contacts, ha, hb);
 
 /* skip duplicates */
 if(node != NULL) {
  if(node->_manifold._flags & OXB_MANIFOLD2D_QUERIED) {
   return;
  }
 }

 bool both_dynamic = ha._type == hb._type && ha._type == OXB_POOL2D_TYPE_DYNAMIC_2D;
 bool reset_frame = world->_island_frame == 0;
 oxb_dynamic2d *obj_a = NULL;
 oxb_dynamic2d *obj_b = NULL;
 if(both_dynamic) {
  obj_a = (oxb_dynamic2d*)oxb_pool2d_get(&world->_pool2d, ha);
  obj_b = (oxb_dynamic2d*)oxb_pool2d_get(&world->_pool2d, hb);
  if((obj_a->_flags & OXB_FLAG_SLEEP_2D) && (obj_b->_flags & OXB_FLAG_SLEEP_2D) && !reset_frame && node != NULL) {
   oxb_pool_id a_island_id = oxb_island2d_find(&world->_island, obj_a->_island_id);
   oxb_pool_id b_island_id = oxb_island2d_find(&world->_island, obj_b->_island_id);
  
   bool is_equal_island = oxb_pool_id_isequal(a_island_id, b_island_id);

   if(is_equal_island && oxb_island2d_sleeping(&world->_island, a_island_id)) {
    node->_manifold._flags |= OXB_MANIFOLD2D_QUERIED;
    oxb_solver2d_contact_prepare(&node->_manifold, &world->_pool2d, &world->_input);
    return;
   }
  }
 }
 
 /* narrowphase */
 oxb_manifold2d m;
 oxb_memory_set(&m, 0x00, sizeof(oxb_manifold2d));

 bool result = oxb_manifold2d_object_vs_object[a._flags][b._flags](&m, &a, &b);
 if(!result)
  return;
 
 if(node == NULL) {
  m._flags |= OXB_MANIFOLD2D_QUERIED;
  oxb_solver2d_contact_prepare(&m, &world->_pool2d, &world->_input);
  oxb_contacthash2d_insert(&world->_contacts, &m);
 } else {
  node->_manifold._flags |= OXB_MANIFOLD2D_QUERIED;
  oxb_manifold2d_combine(&node->_manifold, &m);
  oxb_solver2d_contact_prepare(&node->_manifold, &world->_pool2d, &world->_input);
 }
 
 /* DSU merging */
 if(both_dynamic) {
  bool resting_a = (obj_a->_flags & OXB_FLAG_REST_2D) != 0;
  bool resting_b = (obj_b->_flags & OXB_FLAG_REST_2D) != 0;
  if(oxb_pool_id_isnull(obj_a->_island_id)) {
   obj_a->_island_id = oxb_island2d_acquire(&world->_island, resting_a);
  }
  if(oxb_pool_id_isnull(obj_b->_island_id)) {
   obj_b->_island_id = oxb_island2d_acquire(&world->_island, resting_b);
  }
  oxb_island2d_merge(&world->_island, obj_a->_island_id, obj_b->_island_id);
  oxb_container_id group_id = oxb_island2d_groupid(&world->_island, obj_a->_island_id);
  oxb_island2d_group *group = oxb_container_get(&world->_island._group_container, group_id);
  group->_sleeping &= resting_a;
  group->_sleeping &= resting_b;
 }
}





oxb_noinline static void single_thread_pipeline(oxb_world2d *oxb_restrict const world, const float dt) {
 float divided_dt = dt / (float)world->_substeps;

 /* collisionphase */

 if(oxb_bvh2d_shouldrebuild(&world->_bvh2d, 5)) 
  oxb_bvh2d_mediansplit(&world->_bvh2d, &world->_pool2d);

 oxb_bvh2d_traversepairs(&world->_bvh2d, broadphase_query, world);

 float damping_ratio = 2.6f;
 float hertz = 9.33f;

 float soft_dt = dt;
 float omega = hertz * 6.28318530f;
  
	const float a1 = 2.0f * damping_ratio + omega * soft_dt;
	const float a2 = soft_dt * omega * a1;
	const float a3 = 1.0f / (1.0f + a2);
	world->_input._bias_coeff = omega / a1;
	world->_input._mass_coeff = a2 * a3;
	world->_input._impulse_coeff = a3;
	world->_input._dt = soft_dt;
	world->_input._inv_dt = 1.0f / soft_dt;
	world->_input._ctx = (void*)world;


 bool reset_frame = world->_island_frame == 0;
 if(reset_frame) {
  world->_island_frame = 0;
  oxb_island2d_reset(&world->_island);
 }




 for(size_t i = 0; i < 33; i++) {
  oxb_queue_reset(&world->_color_group[i]);
 }
 
 
 
 {/* solver*/

 for(size_t i = 0; i < world->_pool2d._containers[OXB_POOL2D_TYPE_JOINT_2D]._size; i++) {
  oxb_joint2d *joint = ((oxb_joint2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_JOINT_2D]._bytes) + i;
  oxb_joint2d_presolve(joint, &world->_input);
 }

 for(size_t i = 0; i < world->_pool2d._containers[OXB_POOL2D_TYPE_JOINT_2D]._size; i++) {
  oxb_joint2d *joint = ((oxb_joint2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_JOINT_2D]._bytes) + i;
  oxb_joint2d_solve(joint, &world->_input);
 }


 /* pre solve, vut with single pass and many process. */
 for(size_t i = 0; i < world->_contacts._pool._size;) {
 	oxb_contacthash2d_node *node = ((oxb_contacthash2d_node*)world->_contacts._pool._bytes) + i;
  if(node->_manifold._flags & OXB_MANIFOLD2D_QUERIED) {
   node->_manifold._flags &= ~OXB_MANIFOLD2D_QUERIED;
   
   if(!oxb_solver2d_contact_sleeping(&node->_manifold, &world->_pool2d)) {
    oxb_solver2d_contact_presolve(&node->_manifold, &world->_pool2d, &world->_input);
    size_t current_color = node->_manifold._flags & OXB_MANIFOLD2D_COLOR;
    if(node->_manifold._flags & OXB_MANIFOLD2D_SOLO)
     current_color = 32;
    oxb_queue_push(&world->_color_group[current_color], &node->_allocation);
   }
   
   i++;
  } else {
   oxb_contacthash2d_remove(&world->_contacts, node);
  }
 }


 /* parallel solver */
 for(size_t i = 0; i < 32; i++) {
  oxb_queue *current_group = &world->_color_group[i];
  size_t current_size = current_group->_size;
  size_t j = 0;
  for(; (j+4) < current_size; j+=4) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node1 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_contacthash2d_node *node2 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+1]);
   oxb_contacthash2d_node *node3 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+2]);
   oxb_contacthash2d_node *node4 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+3]);
   oxb_manifold2d *manifolds[4] = {&node1->_manifold, &node2->_manifold, &node3->_manifold, &node4->_manifold};
   oxb_solver2d_contact_solve4(manifolds, &world->_pool2d, &world->_input);
  }
  for(; j < current_size; j++) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
  }
 }
 {
  oxb_queue *current_group = &world->_color_group[32];
  size_t current_size = current_group->_size;
  for(size_t j = 0; j < current_size; j++) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
  }
 }
/*
 for(size_t i = 0; i < world->_contacts._pool._size; i++) {
 	oxb_contacthash2d_node *node = ((oxb_contacthash2d_node*)world->_contacts._pool._bytes) + i;
  oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
 }
*/



 if(reset_frame) {
  size_t i = 0;
  for(; (i+4) <= world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._size; i+=4) {
   oxb_dynamic2d *dynamic_body = ((oxb_dynamic2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._bytes) + i;

   oxb_dynamic2d_updaterest(&dynamic_body[0], world, dt);
   oxb_dynamic2d_updaterest(&dynamic_body[1], world, dt);
   oxb_dynamic2d_updaterest(&dynamic_body[2], world, dt);
   oxb_dynamic2d_updaterest(&dynamic_body[3], world, dt);

   oxb_dynamic2d_integrate4(dynamic_body, world, divided_dt);
 
   for(size_t j = 0; j < 4; j++) {
    oxb_bvh2d_update(&world->_bvh2d, dynamic_body[j]._bvh_id, dynamic_body[j]._aabb);
 
    bool resting = (dynamic_body[j]._flags & OXB_FLAG_REST_2D) != 0;
    dynamic_body[j]._island_id = oxb_island2d_acquire(&world->_island, resting);
   }
  }
  for(; i < world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._size; ++i) {
   oxb_dynamic2d *dynamic_body = ((oxb_dynamic2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._bytes) + i;

   oxb_dynamic2d_updaterest(dynamic_body, world, dt);

   oxb_dynamic2d_integrate(dynamic_body, world, divided_dt);
   oxb_bvh2d_update(&world->_bvh2d, dynamic_body->_bvh_id, dynamic_body->_aabb);
   bool resting = (dynamic_body->_flags & OXB_FLAG_REST_2D) != 0;
   dynamic_body->_island_id = oxb_island2d_acquire(&world->_island, resting);
  
  }
 } else { /* if reset_frame */
  size_t i = 0;
  for(; (i+4) <= world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._size; i+=4) {
   oxb_dynamic2d *dynamic_body = ((oxb_dynamic2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._bytes) + i;

   oxb_dynamic2d_updaterest(&dynamic_body[0], world, dt);
   oxb_dynamic2d_updaterest(&dynamic_body[1], world, dt);
   oxb_dynamic2d_updaterest(&dynamic_body[2], world, dt);
   oxb_dynamic2d_updaterest(&dynamic_body[3], world, dt);

   oxb_dynamic2d_integrate4(dynamic_body, world, divided_dt);
   for(size_t j = 0; j < 4; j++) {
    oxb_bvh2d_update(&world->_bvh2d, dynamic_body[j]._bvh_id, dynamic_body[j]._aabb);
    oxb_dynamic2d_updatesleep(&dynamic_body[j], world);
   }
  }
  for(; i < world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._size; ++i) {
   oxb_dynamic2d *dynamic_body = ((oxb_dynamic2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._bytes) + i;

   oxb_dynamic2d_updaterest(dynamic_body, world, dt);
   
   oxb_dynamic2d_integrate(dynamic_body, world, divided_dt);
   oxb_bvh2d_update(&world->_bvh2d, dynamic_body->_bvh_id, dynamic_body->_aabb);
   oxb_dynamic2d_updatesleep(dynamic_body, world);
  }
 } /* if reset_frame */



 /* parallel solver */
 for(size_t i = 0; i < 32; i++) {
  oxb_queue *current_group = &world->_color_group[i];
  size_t current_size = current_group->_size;
  size_t j = 0;
  for(; (j+4) < current_size; j+=4) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node1 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_contacthash2d_node *node2 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+1]);
   oxb_contacthash2d_node *node3 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+2]);
   oxb_contacthash2d_node *node4 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+3]);
   oxb_manifold2d *manifolds[4] = {&node1->_manifold, &node2->_manifold, &node3->_manifold, &node4->_manifold};
   oxb_solver2d_contact_solve4(manifolds, &world->_pool2d, &world->_input);
  }
  for(; j < current_size; j++) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
  }
 }
 {
  oxb_queue *current_group = &world->_color_group[32];
  size_t current_size = current_group->_size;
  for(size_t j = 0; j < current_size; j++) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
  }
 }
/*
 for(size_t i = 0; i < world->_contacts._pool._size; i++) {
 	oxb_contacthash2d_node *node = ((oxb_contacthash2d_node*)world->_contacts._pool._bytes) + i;
  oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
 }
*/

 }
 
 
 
 
 for(size_t iteration = 1; iteration < world->_substeps; iteration++) {


 for(size_t i = 0; i < world->_pool2d._containers[OXB_POOL2D_TYPE_JOINT_2D]._size; i++) {
  oxb_joint2d *joint = ((oxb_joint2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_JOINT_2D]._bytes) + i;
  oxb_joint2d_presolve(joint, &world->_input);
 }

 for(size_t i = 0; i < world->_pool2d._containers[OXB_POOL2D_TYPE_JOINT_2D]._size; i++) {
  oxb_joint2d *joint = ((oxb_joint2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_JOINT_2D]._bytes) + i;
  oxb_joint2d_solve(joint, &world->_input);
 }
 
 
 for(size_t i = 0; i < world->_contacts._pool._size; i++) {
 	oxb_contacthash2d_node *node = ((oxb_contacthash2d_node*)world->_contacts._pool._bytes) + i;
  oxb_solver2d_contact_presolve(&node->_manifold, &world->_pool2d, &world->_input);
 }


 /* parallel solver */
 for(size_t i = 0; i < 32; i++) {
  oxb_queue *current_group = &world->_color_group[i];
  size_t current_size = current_group->_size;
  size_t j = 0;
  for(; (j+4) < current_size; j+=4) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node1 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_contacthash2d_node *node2 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+1]);
   oxb_contacthash2d_node *node3 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+2]);
   oxb_contacthash2d_node *node4 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+3]);
   oxb_manifold2d *manifolds[4] = {&node1->_manifold, &node2->_manifold, &node3->_manifold, &node4->_manifold};
   oxb_solver2d_contact_solve4(manifolds, &world->_pool2d, &world->_input);
  }
  for(; j < current_size; j++) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
  }
 }
 {
  oxb_queue *current_group = &world->_color_group[32];
  size_t current_size = current_group->_size;
  for(size_t j = 0; j < current_size; j++) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
  }
 }
/*
 for(size_t i = 0; i < world->_contacts._pool._size; i++) {
 	oxb_contacthash2d_node *node = ((oxb_contacthash2d_node*)world->_contacts._pool._bytes) + i;
  oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
 }
*/

 size_t i = 0;
 for(; (i+4) <= world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._size; i+=4) {
  oxb_dynamic2d *dynamic_body = ((oxb_dynamic2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._bytes) + i;
  oxb_dynamic2d_integrate4(dynamic_body, world, divided_dt);
 }
 
 for(; i < world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._size; ++i) {
  oxb_dynamic2d *dynamic_body = ((oxb_dynamic2d*)world->_pool2d._containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._bytes) + i;
  oxb_dynamic2d_integrate(dynamic_body, world, divided_dt);
 }

 /* parallel solver */
 for(size_t i = 0; i < 32; i++) {
  oxb_queue *current_group = &world->_color_group[i];
  size_t current_size = current_group->_size;
  size_t j = 0;
  for(; (j+4) < current_size; j+=4) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node1 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_contacthash2d_node *node2 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+1]);
   oxb_contacthash2d_node *node3 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+2]);
   oxb_contacthash2d_node *node4 = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j+3]);
   oxb_manifold2d *manifolds[4] = {&node1->_manifold, &node2->_manifold, &node3->_manifold, &node4->_manifold};
   oxb_solver2d_contact_solve4(manifolds, &world->_pool2d, &world->_input);
  }
  for(; j < current_size; j++) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
  }
 }
 {
  oxb_queue *current_group = &world->_color_group[32];
  size_t current_size = current_group->_size;
  for(size_t j = 0; j < current_size; j++) {
   oxb_container_id *current_id = (oxb_container_id*)oxb_queue_front(current_group);
   
   oxb_contacthash2d_node *node = (oxb_contacthash2d_node*)oxb_container_get(&world->_contacts._pool, current_id[j]);
   oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
  }
 }
/*
 for(size_t i = 0; i < world->_contacts._pool._size; i++) {
 	oxb_contacthash2d_node *node = ((oxb_contacthash2d_node*)world->_contacts._pool._bytes) + i;
  oxb_solver2d_contact_solve(&node->_manifold, &world->_pool2d, &world->_input);
 }
*/


 } /* iteration */


 world->_island_frame = (world->_island_frame + 1) & 127;
}


















oxb_world2d* oxb_world2d_create(const oxb_world2d_info *oxb_restrict const info) { 
 oxb_world2d *world = (oxb_world2d*)oxb_aligned_alloc((oxb_allocator*)&info->_allocator, sizeof(oxb_world2d));
 if(oxb_unlikely(!world)) {
  OXB_PANIC("oxb_world2d_create() : allocation failed. ");
 }
 oxb_memory_set(world, 0, sizeof(oxb_world2d));

 world->_allocator = info->_allocator;
 world->_gravity = info->_gravity;
 world->_substeps = info->_substeps;
 world->_linear_rest_threshold = info->_linear_rest_threshold;
 world->_angular_rest_threshold = info->_angular_rest_threshold;
 world->_rest_time_threshold = info->_rest_time_threshold;
 world->_input._penetration_slop = info->_penetration_slop;
 world->_max_omega = info->_max_omega;
 
 assert(info->_substeps != 0);
 
 size_t initial_reserves[5] = {
 	info->_pool2d._dynamic_reserve,
 	info->_pool2d._kinematic_reserve,
 	info->_pool2d._static_reserve,
 	info->_pool2d._sensor_reserve,
 	info->_pool2d._joint_reserve
 };
 oxb_pool2d_create(&world->_pool2d, &world->_allocator, initial_reserves);
 
 oxb_contacthash2d_create(&world->_contacts, &world->_allocator, info->_contacthash2d._buckets_reserve);
 oxb_bvh2d_create(&world->_bvh2d, &world->_allocator, info->_bvh2d._node_reserve, info->_bvh2d._bump_reserve, info->_bvh2d._fat_margin);
 oxb_island2d_create(&world->_island, &world->_allocator, 5, 5);
 
 for(size_t i = 0; i < 33; i++) {
  oxb_queue_create(&world->_color_group[i], &world->_allocator, sizeof(oxb_container_id), 8);
 }
 
 world->_pool2d._world = (void*)world;
 return world;
}


void oxb_world2d_destroy(oxb_world2d *oxb_restrict const world) {
 oxb_bvh2d_destroy(&world->_bvh2d);
 oxb_contacthash2d_destroy(&world->_contacts);
 oxb_pool2d_destroy(&world->_pool2d);
 oxb_island2d_destroy(&world->_island);

 for(size_t i = 0; i < 33; i++) {
  oxb_queue_destroy(&world->_color_group[i]);
 }
 oxb_aligned_free(&world->_allocator, world);
}


oxb_pool2d* oxb_world2d_getpool2d(oxb_world2d *oxb_restrict const world) {
 return &world->_pool2d;
}

oxb_handle oxb_world2d_querypoint(oxb_world2d *oxb_restrict const world, oxb_vec2 point) {
	return oxb_bvh2d_querypoint(&world->_bvh2d, &world->_pool2d, point);
}

void oxb_world2d_step(oxb_world2d *oxb_restrict const world, const float dt) {
 single_thread_pipeline(world, dt);
}

