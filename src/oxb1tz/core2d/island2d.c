#include "oxb1tz/core2d/island2d.h"

/*
 Disjoint Set Union Algorithm
 a fast graph merge with path compression
*/

void oxb_island2d_create(oxb_island2d *oxb_restrict const island, oxb_allocator *oxb_restrict const alloc, const size_t pool_reserve, const size_t container_reserve) {
 oxb_pool_create(&island->_node_pool, alloc, sizeof(oxb_island2d_node), pool_reserve);
 oxb_container_create(&island->_group_container, alloc, sizeof(oxb_island2d_group), container_reserve);
}


void oxb_island2d_destroy(oxb_island2d *oxb_restrict const island) {
 oxb_pool_destroy(&island->_node_pool);
 oxb_container_destroy(&island->_group_container);
}


oxb_pool_id oxb_island2d_acquire(oxb_island2d *oxb_restrict const island, const bool resting) {
 oxb_pool_id id = oxb_pool_acquire(&island->_node_pool);
 oxb_island2d_node *node = oxb_pool_get(&island->_node_pool, id);

 oxb_container_id island_id = oxb_container_acquire(&island->_group_container);
 oxb_island2d_group *group = oxb_container_get(&island->_group_container, island_id);
 group->_member_count = 1;
 group->_allocation = island_id;
 group->_sleeping = resting;
 group->_leader = id;
 
 node->_allocation = id;
 node->_leader = id;
 node->_island = island_id;
 return id;
}


oxb_pool_id oxb_island2d_find(oxb_island2d *oxb_restrict const island, const oxb_pool_id h) {
 oxb_island2d_node *node = oxb_pool_get(&island->_node_pool, h);
 if(!oxb_pool_id_isequal(node->_leader, h)) 
  node->_leader = oxb_island2d_find(island, node->_leader);
 return node->_leader;
}


void oxb_island2d_merge(oxb_island2d *oxb_restrict const island, const oxb_pool_id a, const oxb_pool_id b) {
 oxb_pool_id ha = oxb_island2d_find(island, a);
 oxb_pool_id hb = oxb_island2d_find(island, b);

 oxb_island2d_node *root_a = (oxb_island2d_node*)oxb_pool_get(&island->_node_pool, ha);
 oxb_island2d_node *root_b = (oxb_island2d_node*)oxb_pool_get(&island->_node_pool, hb);

 if(oxb_pool_id_isequal(ha, hb))
  return;

 oxb_island2d_group *group_a = (oxb_island2d_group*)oxb_container_get(&island->_group_container, root_a->_island);
 oxb_island2d_group *group_b = (oxb_island2d_group*)oxb_container_get(&island->_group_container, root_b->_island);

 if(group_a->_member_count < group_b->_member_count) {
  root_a->_leader = root_b->_allocation;
  root_a->_island = OXB_CONTAINER_NULL_ID;
  group_b->_sleeping &= group_a->_sleeping;
  group_b->_member_count += group_a->_member_count;
  oxb_container_release(&island->_group_container, group_a->_allocation);
 } else {
  root_b->_leader = root_a->_allocation;
  root_b->_island = OXB_CONTAINER_NULL_ID;
  group_a->_sleeping &= group_b->_sleeping;
  group_a->_member_count += group_b->_member_count;
  oxb_container_release(&island->_group_container, group_b->_allocation);
 }
}


void oxb_island2d_reset(oxb_island2d *oxb_restrict const island) {
 oxb_pool_reset(&island->_node_pool);
 oxb_container_reset(&island->_group_container);
}



bool oxb_island2d_sleeping(oxb_island2d *oxb_restrict const island, const oxb_pool_id id) {
 oxb_island2d_group *group = (oxb_island2d_group*)oxb_container_get(&island->_group_container, oxb_island2d_groupid(island, id));
 return group->_sleeping;
}


oxb_container_id oxb_island2d_groupid(oxb_island2d *oxb_restrict const island, const oxb_pool_id id) {
 oxb_pool_id leader = oxb_island2d_find(island, id);
 oxb_island2d_node *node = (oxb_island2d_node*)oxb_pool_get(&island->_node_pool, leader);
 assert(!oxb_container_id_isnull(node->_island));
 return node->_island;
}

