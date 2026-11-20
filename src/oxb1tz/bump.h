#ifndef OXB_BUMP_H
#define OXB_BUMP_H

#include "oxb1tz/types.h"
#include "oxb1tz/hint.h"

typedef struct {
 oxb_allocator *_allocator;
 uint8_t       *_bytes;
 size_t        _forward_offset;
 size_t        _max_size;
} oxb_bump;

void oxb_bump_create(oxb_bump *oxb_restrict const bump, oxb_allocator *oxb_restrict const alloc, const size_t reserve);
void oxb_bump_destroy(oxb_bump *oxb_restrict const bump);
size_t oxb_bump_acquire(oxb_bump *oxb_restrict const bump, const size_t size, const size_t alignment);
void oxb_bump_point(oxb_bump *oxb_restrict const bump, const size_t offset);

oxb_inline void* oxb_bump_get(oxb_bump *oxb_restrict const bump, size_t offset) {
 /* catch overflow access */
 assert(offset < bump->_max_size);
 
 /* catch unallocated access */
 assert(offset < bump->_forward_offset);
 
 return (void*)(&bump->_bytes[offset]);
}


#endif


