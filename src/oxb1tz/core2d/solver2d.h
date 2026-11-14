#ifndef OXB_SOLVER2D_H
#define OXB_SOLVER2D_H

#include "oxb1tz/core2d/manifold2d.h"
#include "oxb1tz/core2d/pool2d.h"

typedef struct {
 void *_ctx;
 float _impulse_coeff;
 float _mass_coeff;
 float _bias_coeff;
 float _inv_dt;
 float _dt;
 float _penetration_slop;
} oxb_solver2d_input;

bool oxb_solver2d_contact_sleeping(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool);
void oxb_solver2d_contact_prepare(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool, const oxb_solver2d_input *oxb_restrict const input);

void oxb_solver2d_contact_presolve(oxb_manifold2d *oxb_restrict const m, oxb_pool2d *oxb_restrict const pool, const oxb_solver2d_input *oxb_restrict const input);

/* iterator */
void oxb_solver2d_contact_presolve_iterate(void *oxb_restrict const ptr, const size_t begin, const size_t end, const oxb_solver2d_input *oxb_restrict const input);
void oxb_solver2d_contact_solve_iterate(void *oxb_restrict const ptr, const size_t begin, const size_t end, const oxb_solver2d_input *oxb_restrict const input);

#endif

