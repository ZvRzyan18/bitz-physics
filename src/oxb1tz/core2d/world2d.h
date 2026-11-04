#ifndef OXB_WORLD2D_H
#define OXB_WORLD2D_H

#include "oxb1tz/core2d.h"
#include "oxb1tz/core2d/pool2d.h"
#include "oxb1tz/core2d/contacthash2d.h"
#include "oxb1tz/core2d/bvh2d.h"
#include "oxb1tz/core2d/solver2d.h"
#include "oxb1tz/core2d/island2d.h"


struct oxb_world2d {
 oxb_allocator      _allocator;
 oxb_pool2d         _pool2d;
 oxb_contacthash2d  _contacts;
 oxb_bvh2d          _bvh2d;
 oxb_solver2d_input _input;
 oxb_island2d       _island;
 
 oxb_vec2           _gravity;
  
 
 uint32_t           _colors[32];
 uint32_t           _solo_manifolds;
 
 uint8_t            _substeps;
 uint8_t            _island_frame;
 
};

oxb_world2d* oxb_world2d_create(const oxb_world2d_info *oxb_restrict const info);
void oxb_world2d_destroy(oxb_world2d *oxb_restrict const world);
oxb_pool2d* oxb_world2d_getpool2d(oxb_world2d *oxb_restrict const world);
void oxb_world2d_step(oxb_world2d *oxb_restrict const world, const float dt);

#endif
