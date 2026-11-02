#include "oxb1tz/core2d/contacthash2d.h"
#include "oxb1tz/memory.h"
#include "oxb1tz/math.h"


/*
 Robert Jenkins' 32 bit integer hash function
 src : https://gist.github.com/badboy/6267743
*/
oxb_const oxb_inline uint32_t hash(uint32_t a) {
 a = (a + 0x7ed55d16) + (a << 12);
 a = (a ^ 0xc761c23c) ^ (a >> 19);
 a = (a + 0x165667b1) + (a << 5);
 a = (a + 0xd3a2646c) ^ (a << 9);
 a = (a + 0xfd7046c5) + (a << 3);
 a = (a ^ 0xb55a4f09) ^ (a >> 16);
 return a;
}

/*
 symetrical hash function
 
 hash_combine(a, b)
 must be equal to :
 hash_combine(b, a)
*/
oxb_hot oxb_noinline static uint32_t hash_combine(oxb_handle a, oxb_handle b) { 
 bool sswap = a._type < b._type || (a._type == b._type && a._id < b._id);
 if(sswap) {
  oxb_handle tmp = b;
  b = a;
  a = tmp;
 }
 uint32_t out = 0x316312AC;
 out = hash(out ^ a._type);
 out = hash(out ^ b._type);
 out = hash(out ^ a._id);
 out = hash(out ^ b._id);
 return out;
}

/*
 resize insert
 O(1) -> must be unique because i never check for duplicate
*/
oxb_hot oxb_noinline static void insert_unique(oxb_contacthash2d *oxb_restrict const c, const oxb_manifold2d *oxb_restrict const m) {
 const size_t hash_index = (size_t)hash_combine(m->_a, m->_b) & c->_buckets_mask;

 oxb_contacthash2d_node *start_node = NULL;
 oxb_container_id next_index; 

	next_index = oxb_container_acquire(&c->_pool);
	
 start_node = (oxb_contacthash2d_node*)oxb_container_get(&c->_pool, next_index);
	start_node->_next = c->_buckets[hash_index];
	start_node->_prev = OXB_CONTAINER_NULL_ID;
	start_node->_allocation = next_index;
 start_node->_manifold = *m;

 if(!oxb_container_id_isnull(c->_buckets[hash_index])) {
 	start_node = (oxb_contacthash2d_node*)oxb_container_get(&c->_pool, c->_buckets[hash_index]);
	 start_node->_prev = next_index;
 }
 c->_buckets[hash_index] = next_index;
}





void oxb_contacthash2d_create(oxb_contacthash2d *oxb_restrict const c, oxb_allocator *oxb_restrict const alloc, const size_t reserve) {
 c->_buckets_size = oxb_nextp2(reserve);
 c->_buckets_mask = c->_buckets_size - 1;
 oxb_container_create(&c->_pool, alloc, sizeof(oxb_contacthash2d_node), c->_buckets_size);

 c->_buckets = (oxb_container_id*)oxb_aligned_alloc(c->_pool._allocator, sizeof(oxb_container_id) * c->_buckets_size);
 if(oxb_unlikely(!c->_buckets))
  OXB_PANIC("contact hash2d allocation failed.");
 oxb_memory_set(c->_buckets, 0xFF, sizeof(oxb_container_id) * c->_buckets_size);
}


void oxb_contacthash2d_destroy(oxb_contacthash2d *oxb_restrict const c) {
 oxb_aligned_free(c->_pool._allocator, c->_buckets);
 oxb_container_destroy(&c->_pool);
}

/*
 insertion :
 O(1) -> must be unique because i never check for duplicate
 O(n) -> for resize or rehash
*/
void oxb_contacthash2d_insert(oxb_contacthash2d *oxb_restrict const c, const oxb_manifold2d *oxb_restrict const m) { 
 if(c->_pool._size >= c->_pool._max_size) {
  oxb_contacthash2d new_table;
  oxb_contacthash2d_create(&new_table, c->_pool._allocator, c->_pool._max_size << 1);
  oxb_contacthash2d_node *nodes = (oxb_contacthash2d_node*)c->_pool._bytes;
  for(size_t i = 0; i < c->_pool._size; i++) {
   insert_unique(&new_table, &nodes[i]._manifold);
  }
  oxb_contacthash2d_destroy(c);
  *c = new_table;
  c->_pool = new_table._pool;
  c->_buckets = new_table._buckets;
  c->_buckets_size = new_table._buckets_size;
  c->_buckets_mask = new_table._buckets_mask;
 }
 const size_t hash_index = (size_t)hash_combine(m->_a, m->_b) & c->_buckets_mask;

 oxb_contacthash2d_node *start_node = NULL;
 oxb_container_id next_index; 

	next_index = oxb_container_acquire(&c->_pool);
	
 start_node = (oxb_contacthash2d_node*)oxb_container_get(&c->_pool, next_index);
	start_node->_next = c->_buckets[hash_index];
	start_node->_prev = OXB_CONTAINER_NULL_ID;
	start_node->_allocation = next_index;
 start_node->_manifold = *m;

 if(!oxb_container_id_isnull(c->_buckets[hash_index])) {
 	start_node = (oxb_contacthash2d_node*)oxb_container_get(&c->_pool, c->_buckets[hash_index]);
	 start_node->_prev = next_index;
 }
 c->_buckets[hash_index] = next_index;
}

/*
 searching / lookup :
 
 O(1) -> average cases
 O(b) -> b is a number of element in a bucket
*/
oxb_contacthash2d_node* oxb_contacthash2d_get(oxb_contacthash2d *oxb_restrict const c, const oxb_handle a, const oxb_handle b) {
 const size_t hash_index = (size_t)hash_combine(a, b) & c->_buckets_mask;
 oxb_contacthash2d_node *start_node = NULL;
 oxb_container_id next_index = c->_buckets[hash_index];
 while(!oxb_container_id_isnull(next_index)) {
 	start_node = (oxb_contacthash2d_node*)oxb_container_get(&c->_pool, next_index);
 	if((oxb_handle_isequal(start_node->_manifold._a, a) && oxb_handle_isequal(start_node->_manifold._b, b)) || (oxb_handle_isequal(start_node->_manifold._a, b) && oxb_handle_isequal(start_node->_manifold._b, a))) {
	  return start_node;
 	}
		next_index = start_node->_next;
 }
 return NULL;
}

/*
 removal 
 
 O(1) -> constant time, just node rewring
*/
void oxb_contacthash2d_remove(oxb_contacthash2d *oxb_restrict const c, const oxb_contacthash2d_node *oxb_restrict const n) {
 if(!oxb_container_id_isnull(n->_prev)) {
  ((oxb_contacthash2d_node*)oxb_container_get(&c->_pool, n->_prev))->_next = n->_next;
 } else {
  const size_t hash_index = (size_t)hash_combine(n->_manifold._a, n->_manifold._b) & c->_buckets_mask;
  c->_buckets[hash_index] = n->_next;
 }
 if(!oxb_container_id_isnull(n->_next)) {
  ((oxb_contacthash2d_node*)oxb_container_get(&c->_pool, n->_next))->_prev = n->_prev;
 }
 oxb_container_release(&c->_pool, n->_allocation);
}

