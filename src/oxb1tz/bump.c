#include "oxb1tz/bump.h"
#include "oxb1tz/math.h"
#include "oxb1tz/memory.h"


oxb_cold oxb_noinline static void resize(oxb_bump *oxb_restrict const bump, size_t request_size) {
 oxb_bump new_bump;
 new_bump._allocator = bump->_allocator;
 new_bump._forward_offset = bump->_forward_offset;
 if(oxb_likely(request_size > 0)) {
  new_bump._max_size = bump->_max_size + request_size;
 } else {
  new_bump._max_size = bump->_max_size << 1;
 }
 new_bump._bytes = (uint8_t*)oxb_aligned_alloc(new_bump._allocator, new_bump._max_size);

 if(oxb_unlikely(!new_bump._bytes)) {
  OXB_PANIC("bump allocation failed.");
 }

 assert((((uintptr_t)new_bump._bytes) % bump->_allocator->_alignment) == 0);

 oxb_memory_copy(new_bump._bytes, bump->_bytes, bump->_forward_offset);
 oxb_bump_destroy(bump);
 *bump = new_bump;
}


void oxb_bump_create(oxb_bump *oxb_restrict const bump, oxb_allocator *oxb_restrict const alloc, const size_t reserve) {
 assert(reserve != 0);
 
 bump->_allocator = alloc;
 bump->_forward_offset = 0;
 bump->_max_size = reserve;
 bump->_bytes = (uint8_t*)oxb_aligned_alloc(bump->_allocator, bump->_max_size);
 if(oxb_unlikely(!bump->_bytes)) {
  OXB_PANIC("bump allocation failed.");
 }
 assert((((uintptr_t)bump->_bytes) % bump->_allocator->_alignment) == 0);
}


void oxb_bump_destroy(oxb_bump *oxb_restrict const bump) {
 oxb_aligned_free(bump->_allocator, bump->_bytes);
}



size_t oxb_bump_acquire(oxb_bump *oxb_restrict const bump, const size_t size, const size_t alignment) {
 assert(alignment != 0);
 
 size_t requested = size + alignment;
 size_t extra_bytes = bump->_max_size - bump->_forward_offset;
 if(requested > extra_bytes) {
  if(oxb_unlikely(requested > (bump->_max_size << 1))) {
   /* 
    the requested size is currently larger than double max_size 
    so request it instead
   */
   resize(bump, requested);
  } else {
   /* no request size, default, double the max size */
   resize(bump, 0);
  }
 }
 size_t offset = (size_t)oxb_alignto((uintptr_t)bump->_forward_offset, alignment);
 bump->_forward_offset = offset + size;
 return offset;
}



void oxb_bump_point(oxb_bump *oxb_restrict const bump, const size_t offset) {
 /* catch overflow offset */
 assert(offset < bump->_max_size);
 bump->_forward_offset = offset;
}


