#include "oxb1tz/pool.h"
#include "oxb1tz/memory.h"
#include "oxb1tz/math.h"

const oxb_pool_id OXB_POOL_NULL_ID = {._val = OXB_ID_MAX};
/*
 forward pointer based pool
 
 acquire O(1), it becomes O(n) when resizing
 release O(1)
 reset   O(1)
 
*/
oxb_cold oxb_noinline static void resize(oxb_pool *oxb_restrict const pool) {
 oxb_pool new_pool;
 new_pool._max_size = pool->_max_size << 1;
 new_pool._size = pool->_size;
 new_pool._stride = pool->_stride;
 new_pool._allocator = pool->_allocator;

 new_pool._forward_index = pool->_forward_index;
 new_pool._free_count = pool->_free_count;


 const size_t max_size = new_pool._max_size;
 const size_t free_list_size = max_size * sizeof(oxb_pool_id);
 const size_t bytes_size = max_size * new_pool._stride;
 
 new_pool._bytes = (uint8_t*)oxb_aligned_alloc(new_pool._allocator, free_list_size + bytes_size + new_pool._allocator->_alignment);
 if(oxb_unlikely(!new_pool._bytes)) {
  OXB_PANIC("pool::resize() : allocation failed.");
 }
 new_pool._free_list = (oxb_pool_id*)oxb_alignto((uintptr_t)(new_pool._bytes) + bytes_size, new_pool._allocator->_alignment);

 assert((((uintptr_t)new_pool._bytes) % new_pool._allocator->_alignment) == 0);
 assert((((uintptr_t)new_pool._free_list) % new_pool._allocator->_alignment) == 0);


 const size_t prev_free_list_size = pool->_size * sizeof(oxb_pool_id);
 const size_t prev_bytes_size = pool->_size * new_pool._stride;

 oxb_memory_copy(new_pool._bytes, pool->_bytes, prev_bytes_size);
 oxb_memory_copy(new_pool._free_list, pool->_free_list, prev_free_list_size);


 oxb_pool_destroy(pool);
 *pool = new_pool;
}


void oxb_pool_create(oxb_pool *oxb_restrict const pool, oxb_allocator *oxb_restrict const alloc, const uint16_t stride, const size_t reserve) {
 assert(reserve != 0);
 assert(stride != 0);

 pool->_max_size = reserve;
 pool->_size = 0;
 pool->_stride = stride;
 pool->_allocator = alloc;
 
 pool->_forward_index = 0;
 pool->_free_count = 0;

 const size_t max_size = pool->_max_size;
 const size_t free_list_size = max_size * sizeof(oxb_pool_id);
 const size_t bytes_size = max_size * stride;
 
 pool->_bytes = (uint8_t*)oxb_aligned_alloc(pool->_allocator, free_list_size + bytes_size + pool->_allocator->_alignment);
 if(oxb_unlikely(!pool->_bytes)) {
  OXB_PANIC("oxb_pool_create() : allocation failed.");
 }
 
 pool->_free_list = (oxb_pool_id*)oxb_alignto((uintptr_t)(pool->_bytes) + bytes_size, pool->_allocator->_alignment);

 assert((((uintptr_t)pool->_bytes) % pool->_allocator->_alignment) == 0);
 assert((((uintptr_t)pool->_free_list) % pool->_allocator->_alignment) == 0);
}


void oxb_pool_destroy(oxb_pool *oxb_restrict const pool) {
 oxb_aligned_free(pool->_allocator, pool->_bytes);
}


oxb_pool_id oxb_pool_acquire(oxb_pool *oxb_restrict const pool) {
 if(pool->_free_count > 0) {
  pool->_size++;
  return pool->_free_list[--pool->_free_count];
 } else {
  if(pool->_size >= pool->_max_size)
   resize(pool);
  pool->_size++;
  oxb_pool_id id;
  id._val = (oxb_id)pool->_forward_index++;
  return id;
 }
 oxb_unreachable();
 return OXB_POOL_NULL_ID;
}


void oxb_pool_release(oxb_pool *oxb_restrict const pool, const oxb_pool_id id) {
 /* catch overflow id */
 assert(id._val < pool->_max_size);

 /* catch null id */
 assert(!oxb_pool_id_isnull(id));

 /* releasing when the pool is empty is not allowed */
 assert(pool->_size != 0);

#ifndef NDEBUG
 /* invalidate the memory */
 oxb_memory_set(&pool->_bytes[id._val * pool->_stride], 0xFF, pool->_stride);
#endif
 pool->_free_list[pool->_free_count++] = id;
 pool->_size--;
}


void oxb_pool_reset(oxb_pool *oxb_restrict const pool) {
 pool->_free_count = 0;
 pool->_forward_index = 0;
 pool->_size = 0;
}


