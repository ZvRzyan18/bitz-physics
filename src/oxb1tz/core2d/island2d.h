#ifndef OXB_ISLAND2D_H
#define OXB_ISLAND2D_H

#include "oxb1tz/pool.h"
#include "oxb1tz/container.h"

/*
 island is only used for batch sleeping
*/

typedef struct {
 oxb_container_id _island;
 oxb_pool_id      _allocation;
 oxb_pool_id      _leader; 
} oxb_island2d_node;


typedef struct {
 oxb_container_id _allocation;
 oxb_pool_id      _leader;
 uint32_t         _member_count;
 bool             _sleeping;
} oxb_island2d_group;


typedef struct {
 oxb_pool      _node_pool;
 oxb_container _group_container;
} oxb_island2d;


void oxb_island2d_create(oxb_island2d *oxb_restrict const island, oxb_allocator *oxb_restrict const alloc, const size_t pool_reserve, const size_t container_reserve);
void oxb_island2d_destroy(oxb_island2d *oxb_restrict const island);
oxb_pool_id oxb_island2d_acquire(oxb_island2d *oxb_restrict const island, const bool resting);

oxb_pool_id oxb_island2d_find(oxb_island2d *oxb_restrict const island, const oxb_pool_id h);
void oxb_island2d_merge(oxb_island2d *oxb_restrict const island, const oxb_pool_id a, const oxb_pool_id b);
void oxb_island2d_reset(oxb_island2d *oxb_restrict const island);
bool oxb_island2d_sleeping(oxb_island2d *oxb_restrict const island, const oxb_pool_id id);
oxb_container_id oxb_island2d_groupid(oxb_island2d *oxb_restrict const island, const oxb_pool_id id);

#endif

