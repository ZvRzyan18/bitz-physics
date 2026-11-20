#include "memory.h"
#include "oxb1tz/memory.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef OXB_TEST

void oxb_memory_test() {
 uint8_t *bytes = (uint8_t*)malloc(1024);
 uint8_t *dst = (uint8_t*)malloc(1024);
 
 printf("--memory set test\n");
 fflush(stdout);
 
 oxb_memory_set(bytes, 0, 1024);
 oxb_memory_set(bytes + 10, 3, 512);
 
 for(size_t i = 0; i < 10; i++) {
  assert(bytes[i] == 0);
 }
 for(size_t i = 10; i < 10+512; i++) {
  assert(bytes[i] == 3);
 }
 for(size_t i = 10+512; i < 1024; i++) {
  assert(bytes[i] == 0);
 }
 
 printf("--memory copy test\n");
 fflush(stdout);
 oxb_memory_copy(dst, bytes, 1024);

 for(size_t i = 0; i < 10; i++) {
  assert(dst[i] == 0);
 }
 for(size_t i = 10; i < 10+512; i++) {
  assert(dst[i] == 3);
 }
 for(size_t i = 10+512; i < 1024; i++) {
  assert(dst[i] == 0);
 }
 
 
 free(bytes);
 free(dst);
}

#endif
