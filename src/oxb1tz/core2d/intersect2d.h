#ifndef OXB_INTERSECT2D_H
#define OXB_INTERSECT2D_H

#include "oxb1tz/core2d/object2d.h"

typedef bool (*intersect2d_point_func)(oxb_vec2 a, const oxb_collider2d *oxb_restrict const b);

extern const intersect2d_point_func oxb_intersect2d_point_vs_object[2];

bool oxb_intersect2d_point_vs_box(oxb_vec2 a, const oxb_collider2d *oxb_restrict const b);
bool oxb_intersect2d_point_vs_circle(oxb_vec2 a, const oxb_collider2d *oxb_restrict const b);

#endif

