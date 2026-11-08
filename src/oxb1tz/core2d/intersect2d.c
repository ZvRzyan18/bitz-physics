#include "oxb1tz/core2d/intersect2d.h"
#include "oxb1tz/mask.h"

const intersect2d_point_func oxb_intersect2d_point_vs_object[2] = {
 oxb_intersect2d_point_vs_box,
 oxb_intersect2d_point_vs_circle,
};

/*
 this file is only for point and ray intersection.
*/
bool oxb_intersect2d_point_vs_box(oxb_vec2 a, const oxb_collider2d *oxb_restrict const b) {
 assert((b->_flags & OXB_SHAPE_MASK_2D) == OXB_SHAPE_BOX_2D);
 
 oxb_vec2 b_position = oxb_vec2_load(b->_position._arr);
 oxb_complex b_rotation = oxb_vec2_load(b->_rotation._arr);
 oxb_vec2 b_half_size = oxb_vec2_load(b->_shape._box._half_size._arr);
 
 oxb_vec2 relative = oxb_vec2_sub(a, b_position);

 oxb_vec2 local = oxb_complex_mul(relative, oxb_complex_conj(b_rotation));
 local = oxb_vec2_abs(local);
 return oxb_mask2_all(oxb_mask2_flte(local, b_half_size));
}


bool oxb_intersect2d_point_vs_circle(oxb_vec2 a, const oxb_collider2d *oxb_restrict const b) {
 assert((b->_flags & OXB_SHAPE_MASK_2D) == OXB_SHAPE_CIRCLE_2D);
 oxb_vec2 b_position = oxb_vec2_load(b->_position._arr);
 float b_radius = b->_shape._circle._radius;

 oxb_vec2 distance = oxb_vec2_sub(a, b_position);
 float distance_squared = oxb_vec2_dot(distance, distance);
 float radius_squared = b_radius * b_radius;
 return distance_squared <= radius_squared;
}

