#ifndef OXB_BVH2D_H
#define OXB_BVH2D_H

#include "oxb1tz/core2d/object2d.h"
#include "oxb1tz/core2d/pool2d.h"
#include "oxb1tz/pool.h"
#include "oxb1tz/bump.h"

typedef struct {
 oxb_aabb2d  _aabb;
 oxb_pool_id _left;
 oxb_pool_id _right;
 oxb_pool_id _parent;
 oxb_pool_id _allocation;
 oxb_handle  _body;
} oxb_bvh2d_node;

typedef struct {
 uint32_t _node_visits;
 uint32_t _max_depth;
 uint32_t _pair_test;
} oxb_bvh2d_traversalinfo;


typedef struct {
 oxb_pool    _pool;
 oxb_bump    _bump;
 oxb_pool_id _root;
 float       _aabb_margin;
 uint32_t    _leaf_size;
 uint8_t     _frame_tolerance;
} oxb_bvh2d;


void oxb_bvh2d_create(oxb_bvh2d *oxb_restrict const bvh, oxb_allocator *oxb_restrict const alloc, const size_t pool_size, const size_t bump_size, const float aabb_margin);
void oxb_bvh2d_destroy(oxb_bvh2d *oxb_restrict const bvh);
void oxb_bvh2d_update(oxb_bvh2d *oxb_restrict const bvh, const oxb_pool_id id, const oxb_aabb2d fit);
oxb_pool_id oxb_bvh2d_insert(oxb_bvh2d *oxb_restrict const bvh, const oxb_aabb2d fit, const oxb_handle body_id);
void oxb_bvh2d_remove(oxb_bvh2d *oxb_restrict const bvh, const oxb_pool_id id);
void oxb_bvh2d_traversepairs(oxb_bvh2d *oxb_restrict const bvh, void (*func)(oxb_handle, oxb_handle, void*), void *ptr);
void oxb_bvh2d_traversebody(oxb_bvh2d *oxb_restrict const bvh, void (*func)(oxb_handle, oxb_handle, void*), void *ptr, const oxb_pool_id id);
void oxb_bvh2d_mediansplit(oxb_bvh2d *oxb_restrict const bvh, oxb_pool2d *oxb_restrict pool2d);
bool oxb_bvh2d_shouldrebuild(oxb_bvh2d *oxb_restrict const bvh, const uint8_t frame_tolerance);

#endif


