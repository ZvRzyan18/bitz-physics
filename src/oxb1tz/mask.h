#ifndef OXB_MASK_H
#define OXB_MASK_H

#include "oxb1tz/vector.h"

/*
 TODO : unit test?
*/
#ifdef OXB_NEON
typedef uint32x4_t oxb_mask4;
#else
typedef struct {
 uint32_t _arr[4];
} oxb_mask4;
#endif

#ifdef OXB_NEON
#define OXB_MASK4_INIT(a, b, c, d) ((uint32x4_t){a, b, c, d})
#else
#define OXB_MASK4_INIT(a, b, c, d) ((oxb_mask4){a, b, c, d})
#endif

/**********************************************
*                   Vector 4
**********************************************/
oxb_inline oxb_mask4 oxb_mask4_splat(uint32_t x) {
#ifdef OXB_NEON
 return vdupq_n_u32(x);
#else
 oxb_mask4 out;
 out._arr[0] = x;
 out._arr[1] = x;
 out._arr[2] = x;
 out._arr[3] = x;
 return out;
#endif
}

oxb_inline uint32_t oxb_mask4_get(const oxb_mask4 m, const size_t s) {
#ifdef OXB_NEON
 switch(s) {
  case 0:
   return vgetq_lane_u32(m, 0);
  break;
  case 1:
   return vgetq_lane_u32(m, 1);
  break;
  case 2:
   return vgetq_lane_u32(m, 2);
  break;
  case 3:
   return vgetq_lane_u32(m, 3);
  break;
 }
 assert(0);
 oxb_unreachable();
 return 0; /* return something to remove a warning */
#else
 assert(s < 4);
 return m._arr[s];
#endif
}

oxb_inline oxb_mask4 oxb_mask4_set(oxb_mask4 m, const size_t s, const uint32_t x) {
#ifdef OXB_NEON
 switch(s) {
  case 0:
   return vsetq_lane_u32(x, m, 0);
  break;
  case 1:
   return vsetq_lane_u32(x, m, 1);
  break;
  case 2:
   return vsetq_lane_u32(x, m, 2);
  break;
  case 3:
   return vsetq_lane_u32(x, m, 3);
  break;
 }
 assert(0);
 oxb_unreachable();
 return oxb_mask4_splat(0); /* return something to remove a warning */
#else
 assert(s < 4);
 m._arr[s] = x;
 return m;
#endif
}


oxb_inline oxb_mask4 oxb_mask4_fromvec4(const oxb_vec4 x) {
#ifdef OXB_NEON
 return vreinterpretq_u32_f32(x);
#else
 return *((oxb_mask4*)&x);
#endif
}

oxb_inline oxb_vec4 oxb_mask4_tovec4(const oxb_mask4 x) {
#ifdef OXB_NEON
 return vreinterpretq_f32_u32(x);
#else
 return *((oxb_vec4*)&x);
#endif
}

oxb_inline oxb_mask4 oxb_mask4_and(const oxb_mask4 a, const oxb_mask4 b) {
#ifdef OXB_NEON
 return vandq_u32(a, b);
#else
 oxb_mask4 out;
 out._arr[0] = a._arr[0] & b._arr[0];
 out._arr[1] = a._arr[1] & b._arr[1];
 out._arr[2] = a._arr[2] & b._arr[2];
 out._arr[3] = a._arr[3] & b._arr[3];
 return out;
#endif
}


oxb_inline oxb_mask4 oxb_mask4_equal(const oxb_mask4 a, const oxb_mask4 b) {
#ifdef OXB_NEON
 return vceqq_u32(a, b);
#else
 oxb_mask4 out;
 out._arr[0] = a._arr[0] == b._arr[0] ? 0xFFFFFFFF : 0;
 out._arr[1] = a._arr[1] == b._arr[1] ? 0xFFFFFFFF : 0;
 out._arr[2] = a._arr[2] == b._arr[2] ? 0xFFFFFFFF : 0;
 out._arr[3] = a._arr[3] == b._arr[3] ? 0xFFFFFFFF : 0;
 return out;
#endif
}


#endif

