#ifndef OXB_MANIFOLD2D_H
#define OXB_MANIFOLD2D_H

#include "oxb1tz/core2d/object2d.h"

#define OXB_MANIFOLD2D_QUERIED 0b10000000
#define OXB_MANIFOLD2D_SOLO    0b01000000
#define OXB_MANIFOLD2D_COLOR   0b00011111

/*
 feature pair of contact id.
 this uses to be able to make contact persistent across multiple frames
 and make it able to reorder the contacts as needed.
 
 it uses bitfields to minimize memory consumption,
 since 2 ^ 4 = 16, its more than enough for the ids
*/
typedef union {
 struct {
  unsigned char _in_edge1  : 4;
  unsigned char _out_edge1 : 4;
  unsigned char _in_edge2  : 4;
  unsigned char _out_edge2 : 4;
 };
 uint16_t _value;
} oxb_contact2d_id;

typedef struct {
 oxb_packedv2f    _r1, _r2;
 oxb_packedv2f    _normal;
 float            _depth;
 float            _bias;
 float            _mass_normal;
 float            _mass_tangent;
 float            _normal_lambda;
 float            _tangent_lambda;
 oxb_contact2d_id _id;
} oxb_contact2d;

typedef struct {
 oxb_contact2d _contacts[2];
 oxb_handle    _a;
 oxb_handle    _b;
 uint8_t       _contact_count;
 uint8_t       _flags;
} oxb_manifold2d;

typedef bool (*manifold2d_contact_func)(oxb_manifold2d *oxb_restrict const out, const oxb_collider2d *oxb_restrict const a, const oxb_collider2d *oxb_restrict const b);

extern const manifold2d_contact_func oxb_manifold2d_object_vs_object[2][2];

void oxb_manifold2d_combine(oxb_manifold2d *oxb_restrict const out, const oxb_manifold2d *oxb_restrict const m);
bool oxb_manifold2d_box_vs_box(oxb_manifold2d *oxb_restrict const out, const oxb_collider2d *oxb_restrict const a, const oxb_collider2d *oxb_restrict const b);
bool oxb_manifold2d_box_vs_circle(oxb_manifold2d *oxb_restrict const out, const oxb_collider2d *oxb_restrict const a, const oxb_collider2d *oxb_restrict const b);
bool oxb_manifold2d_circle_vs_box(oxb_manifold2d *oxb_restrict const out, const oxb_collider2d *oxb_restrict const a, const oxb_collider2d *oxb_restrict const b);
bool oxb_manifold2d_circle_vs_circle(oxb_manifold2d *oxb_restrict const out, const oxb_collider2d *oxb_restrict const a, const oxb_collider2d *oxb_restrict const b);

#endif

