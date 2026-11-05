#ifndef OXB_JOINT2D_H
#define OXB_JOINT2D_H

#include "oxb1tz/vector.h"
#include "oxb1tz/core2d.h"


typedef struct {
 uint8_t _flags;
 union {
  
  struct {
   float         _inv_mass[4];
   oxb_packedv2f _point;
   oxb_packedv2f _anchor;
   oxb_packedv2f _global_anchor;
   oxb_packedv2f _rotate;
   oxb_packedv2f _accumulated_lambda;
   float         _max_force;
   oxb_handle _object;
  } _mouse;
  
 } _type;
} oxb_joint2d;


void oxb_joint2d_init(oxb_joint2d *oxb_restrict const joint, const oxb_object2d_info *oxb_restrict const info);
void oxb_joint2d_presolve(oxb_joint2d *oxb_restrict const joint, const void *oxb_restrict const _input);
void oxb_joint2d_solve(oxb_joint2d *oxb_restrict const joint, const void *oxb_restrict const _input);

#endif


