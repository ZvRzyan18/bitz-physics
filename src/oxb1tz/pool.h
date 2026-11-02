#ifndef OXB_POOL_H
#define OXB_POOL_H

#include "oxb1tz/hint.h"
#include "oxb1tz/types.h"

typedef struct {
 oxb_id _val;
} oxb_pool_id;

typedef struct {
 oxb_allocator *_allocator;
 uint8_t       *_bytes;
 oxb_pool_id   *_free_list;
 size_t        _max_size;
 size_t        _size;
 size_t        _free_count;
 size_t        _forward_index;
 uint16_t      _stride;
} oxb_pool;

extern const oxb_pool_id OXB_POOL_NULL_ID;
oxb_inline bool oxb_pool_id_isequal(oxb_pool_id a, oxb_pool_id b) {
 return a._val == b._val;
}
oxb_inline bool oxb_pool_id_notequal(oxb_pool_id a, oxb_pool_id b) {
 return a._val != b._val;
}
oxb_inline bool oxb_pool_id_isnull(oxb_pool_id a) {
 return oxb_pool_id_isequal(a, OXB_POOL_NULL_ID);
}

void oxb_pool_create(oxb_pool *oxb_restrict const pool, oxb_allocator *oxb_restrict const alloc, const uint16_t stride, const size_t reserve);
void oxb_pool_destroy(oxb_pool *oxb_restrict const pool);
oxb_pool_id oxb_pool_acquire(oxb_pool *oxb_restrict const pool);
void oxb_pool_release(oxb_pool *oxb_restrict const pool, const oxb_pool_id id);
void oxb_pool_reset(oxb_pool *oxb_restrict const pool);

oxb_inline void* oxb_pool_get(oxb_pool *oxb_restrict const pool, const oxb_pool_id id) {
 /* catch null id */
 assert(!oxb_pool_id_isnull(id));
 
 /* catch overflow id */
 assert(id._val < pool->_max_size);

 return (void*)(pool->_bytes + (pool->_stride * id._val));
}

#endif

