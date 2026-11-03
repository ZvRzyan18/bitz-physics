#ifndef OXB_QUEUE_H
#define OXB_QUEUE_H

#include "oxb1tz/types.h"
#include "oxb1tz/hint.h"

typedef struct {
 oxb_allocator *_allocator;
 uint8_t       *_bytes;
 size_t        _max_size;
 size_t        _front;
 size_t        _size;
 uint16_t      _stride;
} oxb_queue;

void oxb_queue_create(oxb_queue *oxb_restrict const c, oxb_allocator *oxb_restrict const alloc, const uint16_t stride, const size_t reserve);
void oxb_queue_destroy(oxb_queue *oxb_restrict const c);
void oxb_queue_push(oxb_queue *oxb_restrict const c, const void *oxb_restrict const data);
void oxb_queue_pop(oxb_queue *oxb_restrict const c);
void oxb_queue_reset(oxb_queue *oxb_restrict const c);

oxb_inline void* oxb_queue_front(oxb_queue *oxb_restrict const c) {
 assert(c->_size > 0);
 return (void*)(&c->_bytes[c->_front * (size_t)c->_stride]);
}

#endif


