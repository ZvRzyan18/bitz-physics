#include "container.h"
#include "oxb1tz/container.h"
#include "oxb1tz/hint.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef OXB_TEST

/*
static void is_valid(oxb_container *const c) {
 for(size_t i = 0; i < c->_size; i++) {
  assert(c->_id_to_index[c->_index_to_id[i]._val]._val == i);
 }

 for(size_t id = 0; id < c->_size; id++) {
  assert(c->_index_to_id[c->_id_to_index[id]._val]._val == id);
 }
}
*/


typedef struct test_allocation test_allocation;
struct test_allocation {
 void*            _ptr;
 size_t           _size;
 test_allocation* _next;
};

static test_allocation* allocations = NULL;

static size_t total_allocations = 0;
static size_t total_frees = 0;
static size_t current_bytes = 0;
static size_t highest_bytes = 0;
static size_t current_allocation = 0;

static void* debug_malloc(size_t size) {
 void* ptr = malloc(size);

 if(!ptr) {
  assert(0 && "failed alloc.");
  return NULL;
 }

 test_allocation* record = (test_allocation*)malloc(sizeof(test_allocation));

 if(!record) {
  free(ptr);
  assert(0);
  return NULL;
 }
 record->_ptr = ptr;
 record->_size = size;
 record->_next = allocations;
 allocations = record;

 ++total_allocations;
 current_bytes += size;

 if(current_bytes > highest_bytes) {
  highest_bytes = current_bytes;
 }
 current_allocation++;
 return ptr;
}

static void debug_free(void* ptr) {
 if(!ptr) {
  assert(0 && "null free.");
  return;
 }

 test_allocation** link = &allocations;
 size_t i = 0;
 while(*link && (*link)->_ptr != ptr) {
  assert(i < current_allocation);
  link = &(*link)->_next;
  i++;
 }

 if(!*link) {
  assert(0 && "invalid memory pointer.");
  return;
 }

 test_allocation* record = *link;
 *link = record->_next;

 current_bytes -= record->_size;
 ++total_frees;

 free(record->_ptr);
 free(record);
 
 current_allocation--;
}

static void debug_malloc_report() {
 printf("<<--     MEMORY    -->>\n");
 printf("total_allocations : %zu\n", total_allocations);
 printf("total_frees       : %zu\n", total_frees);
 printf("current bytes     : %zu\n", current_bytes);
 printf("highest_bytes     : %zu\n", highest_bytes);
 fflush(stdout);
 test_allocation* record = allocations;
 size_t i = 0;
 while(record) {
  assert(i < current_allocation);
  printf("[LEAK] : ptr=%p size=%zu bytes\n", record->_ptr, record->_size);
  fflush(stdout);
  record = record->_next;
  i++;
 }
 fflush(stdout);
}




static void is_valid(const oxb_container *const c) {
 assert(c->_size <= c->_forward_index);
 assert(c->_forward_index <= c->_max_size);
 assert(c->_free_count <= c->_forward_index);
 assert(c->_size + c->_free_count == c->_forward_index);
    
 for(size_t i = 0; i < c->_size; i++) {
  const oxb_container_id id = c->_index_to_id[i];
  assert(id._val < c->_forward_index);
  assert(c->_id_to_index[id._val]._val == i);
 }
}


void oxb_container_test() {
 oxb_allocator alloc;
 alloc._alloc = debug_malloc;
 alloc._free = debug_free;
 alloc._alignment = 16;
 
 oxb_container test;
 oxb_container_create(&test, &alloc, sizeof(int), 50);
 printf("--acquire test\n");
 fflush(stdout);
 
 oxb_container_id ids[100];
 for(int i = 0; i < 100; i++) {
  ids[i] = oxb_container_acquire(&test);
  int *v = (int*)oxb_container_get(&test, ids[i]);
  *v = i;
  is_valid(&test);
 }


 printf("--get test\n");
 fflush(stdout);
 
 for(int i = 0; i < 100; i++) {
  int *v = (int*)oxb_container_get(&test, ids[i]);
  assert(*v == i);
 }
 
 
 printf("--release test\n");
 fflush(stdout);
 
 //release first half
 for(int i = 0; i < 50; i++) {
  oxb_container_release(&test, ids[i]);
  is_valid(&test);
 }

 //make sure that the elements inside are not corrupted
 for(int i = 50; i < 100; i++) {
  int *v = (int*)oxb_container_get(&test, ids[i]);
  assert(*v == i);
 }
 
 //release the second half
 for(int i = 50; i < 100; i++) {
  oxb_container_release(&test, ids[i]);
  is_valid(&test);
 }

 //destroy
 oxb_container_destroy(&test);
 
 debug_malloc_report();
}


#endif


