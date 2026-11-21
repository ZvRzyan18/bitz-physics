#include "oxb1tz/container.h"
#include "oxb1tz/memory.h"
#include "oxb1tz/math.h"

static_assert(sizeof(oxb_container_id) == sizeof(oxb_id), "not expected size");
const oxb_container_id OXB_CONTAINER_NULL_ID = {._val = OXB_ID_MAX};


oxb_inline void memswap(void *oxb_restrict const a, void *oxb_restrict const b, const size_t stride) {
 assert(stride < 512);
 
 uint8_t tmp_buff[512]; /* more portable than vla */
 oxb_memory_copy(tmp_buff, a, stride);
 oxb_memory_copy(a, b, stride);
 oxb_memory_copy(b, tmp_buff, stride);
}


oxb_cold oxb_noinline static void resize(oxb_container *oxb_restrict const c) {
 oxb_container new_container;
 new_container._allocator = c->_allocator;
	new_container._max_size = c->_max_size * 2;
	new_container._size = c->_size;
	new_container._stride = c->_stride;

	new_container._free_count = c->_free_count;
	new_container._forward_index = c->_forward_index;


 const size_t max_size = new_container._max_size;
	const size_t bytes_size = max_size * new_container._stride;
	const size_t helper_size = sizeof(oxb_container_id) * max_size;


	const size_t luts_size = helper_size * 3 + bytes_size;
 uint8_t *ptr = (uint8_t*)oxb_aligned_alloc(new_container._allocator, luts_size + new_container._allocator->_alignment * 3);

 if(oxb_unlikely(!ptr)) {
  OXB_PANIC("container::resize() : allocation failed.");
 }
 
 new_container._id_to_index = (oxb_container_id*)ptr;
 ptr = ((uint8_t*)new_container._id_to_index) + helper_size;
 new_container._index_to_id = (oxb_container_id*)oxb_alignto((uintptr_t)ptr, new_container._allocator->_alignment);
 ptr = ((uint8_t*)new_container._index_to_id) + helper_size;
 new_container._free_list = (oxb_container_id*)oxb_alignto((uintptr_t)ptr, new_container._allocator->_alignment);
 ptr = ((uint8_t*)new_container._free_list) + helper_size;
 new_container._bytes = (uint8_t*)oxb_alignto((uintptr_t)ptr, new_container._allocator->_alignment);


#ifndef NDEBUG
 oxb_memory_set(new_container._bytes, 0xFF, bytes_size);
 oxb_memory_set(new_container._free_list, 0xFF, helper_size);
 oxb_memory_set(new_container._index_to_id, 0xFF, helper_size);
 oxb_memory_set(new_container._id_to_index, 0xFF, helper_size);
#endif

 assert((((uintptr_t)new_container._bytes) % new_container._allocator->_alignment) == 0);
 assert((((uintptr_t)new_container._id_to_index) % new_container._allocator->_alignment) == 0);
 assert((((uintptr_t)new_container._index_to_id) % new_container._allocator->_alignment) == 0);
 assert((((uintptr_t)new_container._free_list) % new_container._allocator->_alignment) == 0);

 const size_t prev_bytes_size = c->_size * c->_stride;
 const size_t prev_helper_size = c->_size * sizeof(oxb_container_id);
 
 oxb_memory_copy(new_container._bytes, c->_bytes, prev_bytes_size);
 oxb_memory_copy(new_container._free_list, c->_free_list, prev_helper_size);
 oxb_memory_copy(new_container._index_to_id, c->_index_to_id, prev_helper_size);
 oxb_memory_copy(new_container._id_to_index, c->_id_to_index, prev_helper_size);

 oxb_container_destroy(c);
 *c = new_container;
}

/*

 lets say we have 
 
 Elements [A, E]
 Indices  [0, 4]
 
 array         ids
 •---•  •---•
 | A |  | 0 |   0
 •---•  •---•
 | B |  | 1 |   1
 •---•  •---•
 | C |  | 2 |   2
 •---•  •---•
 | D |  | 3 |   3
 •---•  •---•
 | E |  | 4 |   4
 •---•  •---•

 
 Remove C.
 
 'C' is accessible through id 2, if we remove it it will become.
 (swapping both array and indices lut)
 
 array         ids
 •---•  •---•
 | A |  | 0 |   0
 •---•  •---•
 | B |  | 1 |   1
 •---•  •---•
 | E |  | 4 |   2
 •---•  •---•
 | D |  | 3 |   3
 •---•  •---•
 |nul|  | 2 |   4
 •---•  •---•

 
 the id 2 points to the 4'th index, and the data 'E' can still be accessible via
 id 4. even if we move the element at the center, the ids are still mapped to its actual value.
 
 and the id can easily be reused by using free list
*/
void oxb_container_create(oxb_container *oxb_restrict const c, oxb_allocator *oxb_restrict const allocator, const uint16_t stride, const size_t reserve) {
 assert(reserve != 0);
 assert(stride != 0);
 
 c->_allocator = allocator;
	c->_max_size = reserve;
	c->_size = 0;
	c->_stride = stride;
	
	c->_free_count = 0;
	c->_forward_index = 0;

 const size_t max_size = c->_max_size;
	const size_t bytes_size = max_size * c->_stride;
	const size_t helper_size = sizeof(oxb_container_id) * max_size;
	
	const size_t luts_size = helper_size * 3 + bytes_size;
 uint8_t *ptr = (uint8_t*)oxb_aligned_alloc(allocator, luts_size + allocator->_alignment * 3);
 
 if(oxb_unlikely(!ptr)) {
  OXB_PANIC("oxb_container_create() : allocation failed.");
 }
  
 c->_id_to_index = (oxb_container_id*)ptr;
 ptr = ((uint8_t*)c->_id_to_index) + helper_size;
 c->_index_to_id = (oxb_container_id*)oxb_alignto((uintptr_t)ptr, allocator->_alignment);
 ptr = ((uint8_t*)c->_index_to_id) + helper_size;
 c->_free_list = (oxb_container_id*)oxb_alignto((uintptr_t)ptr, allocator->_alignment);
 ptr = ((uint8_t*)c->_free_list) + helper_size;
 c->_bytes = (uint8_t*)oxb_alignto((uintptr_t)ptr, allocator->_alignment);

#ifndef NDEBUG
 oxb_memory_set(c->_bytes, 0xFF, bytes_size);
 oxb_memory_set(c->_free_list, 0xFF, helper_size);
 oxb_memory_set(c->_index_to_id, 0xFF, helper_size);
 oxb_memory_set(c->_id_to_index, 0xFF, helper_size);
#endif
 assert((((uintptr_t)c->_bytes) % allocator->_alignment) == 0);
 assert((((uintptr_t)c->_id_to_index) % allocator->_alignment) == 0);
 assert((((uintptr_t)c->_index_to_id) % allocator->_alignment) == 0);
 assert((((uintptr_t)c->_free_list) % allocator->_alignment) == 0);
}

void oxb_container_destroy(oxb_container *oxb_restrict const c) {
 oxb_aligned_free(c->_allocator, c->_id_to_index);
}

/*
 just return last id, it always points to the last element.

 we can free list to reuse the id that's been freed previously
*/
oxb_container_id oxb_container_acquire(oxb_container *oxb_restrict const c) {
 oxb_container_id id;

 if(c->_free_count != 0) {
  id = c->_free_list[--c->_free_count];
 } else {
  if(c->_forward_index >= c->_max_size)
   resize(c);
  id._val = (oxb_id)c->_forward_index++;
 }
 const size_t index = c->_size++;
 c->_id_to_index[id._val]._val = (oxb_id)index;
 c->_index_to_id[index] = id;
 return id;
}

/*
 swap & pop, this creates no gap and as fast as array iteration
 and retrieve the id to the free list
*/
void oxb_container_release(oxb_container *oxb_restrict const c, const oxb_container_id id) {
 /* catch overflow id */
 assert(id._val < c->_max_size);

 /* catch null */
 assert(!oxb_container_id_isnull(id));
 assert(!oxb_container_id_isnull(c->_id_to_index[id._val]));
 assert(!oxb_container_id_isnull(c->_index_to_id[c->_id_to_index[id._val]._val]));

 /* not allowed when empty */
 assert(c->_size != 0);
 
 
 const size_t current_index = c->_id_to_index[id._val]._val;
 assert(current_index < c->_size);
 const size_t last_index = c->_size - 1;
 const oxb_container_id last_id = c->_index_to_id[last_index];

 if(current_index != last_index) {
  oxb_memory_copy(c->_bytes + ((size_t)current_index * (size_t)c->_stride), c->_bytes + ((size_t)last_index * (size_t)c->_stride), c->_stride);
  c->_id_to_index[last_id._val]._val = (oxb_id)current_index;
  c->_index_to_id[current_index] = last_id;
 }
#ifndef NDEBUG
 /*
  debug, to possibly track invalid id memory 
  ans it also initialize the data into 0xFF per bytes
  making it invalid even if you do use it.
 */
 c->_id_to_index[id._val] = OXB_CONTAINER_NULL_ID;
 c->_index_to_id[last_index] = OXB_CONTAINER_NULL_ID;
 oxb_memory_set(&c->_bytes[last_index * c->_stride], 0xFF, c->_stride);
#endif
 c->_free_list[c->_free_count++] = id;
 c->_size--;
}


void oxb_container_reset(oxb_container *oxb_restrict const c) {
 c->_size = 0;
 c->_free_count = 0;
 c->_forward_index = 0;
}


void oxb_container_insertionsort(oxb_container *oxb_restrict const c, bool (*should_swap)(void *a, void * b)) {
 for(int i = 1; i < (int)c->_size; i++) {
  int j = i;
  int jm1 = j - 1;
  void *j_data = c->_bytes + (j * c->_stride);
  void *jm1_data = c->_bytes + (jm1 * c->_stride);
  
  while (j > 0 && should_swap(jm1_data, j_data)) {
   memswap(j_data, jm1_data, c->_stride);
   oxb_container_id j_id = c->_index_to_id[j];
   oxb_container_id jm1_id = c->_index_to_id[jm1];
   
   memswap(&c->_index_to_id[j], &c->_index_to_id[jm1], sizeof(oxb_container_id));
   memswap(&c->_id_to_index[j_id._val], &c->_id_to_index[jm1_id._val], sizeof(oxb_container_id));
   j--;
  }
 }
}

