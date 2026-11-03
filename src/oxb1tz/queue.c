#include "oxb1tz/queue.h"
#include "oxb1tz/memory.h"

static void resize(oxb_queue *oxb_restrict const c) {
 oxb_queue new_container;
 new_container._allocator = c->_allocator;
 new_container._max_size = c->_max_size << 1;
 new_container._size = c->_size;
 new_container._front = 0;
 new_container._stride = c->_stride;
 new_container._bytes = (uint8_t*)oxb_aligned_alloc(new_container._allocator, (size_t)new_container._stride * new_container._max_size);
 if(oxb_unlikely(!new_container._bytes)) {
  OXB_PANIC("queue allocation failed.");
 }
 for(size_t i = 0; i < c->_size; i++) {
  oxb_memory_copy(&new_container._bytes[i * c->_stride], &c->_bytes[((c->_front + i) % c->_max_size) * c->_stride], c->_stride);
 }
 oxb_queue_destroy(c);
 *c = new_container;
}

void oxb_queue_create(oxb_queue *oxb_restrict const c, oxb_allocator *oxb_restrict const alloc, const uint16_t stride, const size_t reserve) {
 assert(reserve != 0);
 c->_allocator = alloc;
 c->_max_size = reserve;
 c->_size = 0;
 c->_front = 0;
 c->_stride = stride;
 c->_bytes = (uint8_t*)oxb_aligned_alloc(c->_allocator, (size_t)stride * reserve);

 if(oxb_unlikely(!c->_bytes)) {
  OXB_PANIC("queue allocation failed.");
 }
}


void oxb_queue_destroy(oxb_queue *oxb_restrict const c) {
 oxb_aligned_free(c->_allocator, c->_bytes);
}


void oxb_queue_push(oxb_queue *oxb_restrict const c, const void *oxb_restrict const data) {
 if(c->_size >= c->_max_size) {
  resize(c);
 }
 size_t rear = (c->_front + c->_size) % c->_max_size;
 oxb_memory_copy(&c->_bytes[rear * (size_t)c->_stride], data, c->_stride);
 c->_size++;
}

void oxb_queue_pop(oxb_queue *oxb_restrict const c) {
 assert(c->_size != 0);
 c->_front = (c->_front + 1) % c->_max_size;
 c->_size--;
}

void oxb_queue_reset(oxb_queue *oxb_restrict const c) {
 c->_front = 0;
 c->_size = 0;
}
