#include "oxb1tz/memory.h"
#include "oxb1tz/math.h"

/*
 aligned memory allocation
*/
void* oxb_aligned_alloc(oxb_allocator *oxb_restrict const allocator, const size_t bytes_size) {
	void *p1, **p2;
	size_t padd = allocator->_alignment - 1 + sizeof(void*);
 p1 = allocator->_alloc(bytes_size + padd);
 if(oxb_unlikely(!p1)) {
  return NULL;
 }
 p2 = (void**)(oxb_alignto((uintptr_t)p1 + padd, allocator->_alignment));
 p2[-1] = p1;
 return p2;
}
/*
 aligned memory deallocation
*/
void oxb_aligned_free(oxb_allocator *oxb_restrict const allocator, void *oxb_restrict const ptr) {
 if(oxb_unlikely(!ptr)) {
  OXB_PANIC("oxb_aligned_free() : null deallocation.");
 }
 allocator->_free(((void**)ptr)[-1]);
}




/*
 compiler automatically inline these static functions
 no need custom assembly functions.
*/
#ifdef OXB_NEON
/**********************************************
*             arm64/arm32 optimized copy
**********************************************/
static void __neon_copy_small(uint8_t *oxb_restrict dst_bytes, const uint8_t *oxb_restrict src_bytes, size_t n) {
 /* 16 bytes loop copy */
 while(n >= 16) {
  vst1q_u8(dst_bytes, vld1q_u8(src_bytes));
  dst_bytes += 16; src_bytes += 16; n -= 16; 
 }
 while(n--) *(dst_bytes++) = *(src_bytes++);
}

static void __neon_copy_mid(uint8_t *oxb_restrict dst_bytes, const uint8_t *oxb_restrict src_bytes, size_t n) {
 /* 128 bytes loop copy */
 while(n >= 128) {
  uint8x16_t v1 =	vld1q_u8(src_bytes);
  uint8x16_t v2 =	vld1q_u8(src_bytes + 16);
  uint8x16_t v3 =	vld1q_u8(src_bytes + 32);
  uint8x16_t v4 =	vld1q_u8(src_bytes + 48);

  vst1q_u8(dst_bytes, v1);
  vst1q_u8(dst_bytes + 16, v2);
  vst1q_u8(dst_bytes + 32, v3);
  vst1q_u8(dst_bytes + 48, v4);

  uint8x16_t v5 =	vld1q_u8(src_bytes + 64);
  uint8x16_t v6 =	vld1q_u8(src_bytes + 80);
  uint8x16_t v7 =	vld1q_u8(src_bytes + 96);
  uint8x16_t v8 =	vld1q_u8(src_bytes + 112);

  vst1q_u8(dst_bytes + 64, v5);
  vst1q_u8(dst_bytes + 80, v6);
  vst1q_u8(dst_bytes + 96, v7);
  vst1q_u8(dst_bytes + 112, v8);
  
  dst_bytes += 128; src_bytes += 128; n -= 128; 
 }
 
 /* 64 bytes loop copy */
 while(n >= 64) {
  uint8x16_t v1 =	vld1q_u8(src_bytes);
  uint8x16_t v2 =	vld1q_u8(src_bytes + 16);
  uint8x16_t v3 =	vld1q_u8(src_bytes + 32);
  uint8x16_t v4 =	vld1q_u8(src_bytes + 48);

  vst1q_u8(dst_bytes, v1);
  vst1q_u8(dst_bytes + 16, v2);
  vst1q_u8(dst_bytes + 32, v3);
  vst1q_u8(dst_bytes + 48, v4);

  dst_bytes += 64; src_bytes += 64; n -= 64; 
 }
 /* 16 bytes loop copy */
 while(n >= 16) {
  vst1q_u8(dst_bytes, vld1q_u8(src_bytes));
  dst_bytes += 16; src_bytes += 16; n -= 16; 
 }
 /* 1 byte loop copy */
 while(n--) *(dst_bytes++) = *(src_bytes++);
}


static void __neon_copy_large(uint8_t *oxb_restrict dst_bytes, const uint8_t *oxb_restrict src_bytes, size_t n) {
 while(n >= 256) {
  /* fetch and store first 128 */
  uint8x16_t v1 =	vld1q_u8(src_bytes);
  uint8x16_t v2 =	vld1q_u8(src_bytes + 16);
  uint8x16_t v3 =	vld1q_u8(src_bytes + 32);
  uint8x16_t v4 =	vld1q_u8(src_bytes + 48);

  uint8x16_t v5 =	vld1q_u8(src_bytes + 64);
  uint8x16_t v6 =	vld1q_u8(src_bytes + 80);
  uint8x16_t v7 =	vld1q_u8(src_bytes + 96);
  uint8x16_t v8 =	vld1q_u8(src_bytes + 112);

  vst1q_u8(dst_bytes, v1);
  vst1q_u8(dst_bytes + 16, v2);
  vst1q_u8(dst_bytes + 32, v3);
  vst1q_u8(dst_bytes + 48, v4);

  vst1q_u8(dst_bytes + 64, v5);
  vst1q_u8(dst_bytes + 80, v6);
  vst1q_u8(dst_bytes + 96, v7);
  vst1q_u8(dst_bytes + 112, v8);


  /* fetch and store second 128 */
  uint8x16_t v9 =	vld1q_u8(src_bytes + 128);
  uint8x16_t v10 =	vld1q_u8(src_bytes + 16 + 128);
  uint8x16_t v11 =	vld1q_u8(src_bytes + 32 + 128);
  uint8x16_t v12 =	vld1q_u8(src_bytes + 48 + 128);

  uint8x16_t v13 =	vld1q_u8(src_bytes + 64 + 128);
  uint8x16_t v14=	vld1q_u8(src_bytes + 80 + 128);
  uint8x16_t v15 =	vld1q_u8(src_bytes + 96 + 128);
  uint8x16_t v16 =	vld1q_u8(src_bytes + 112 + 128);


  vst1q_u8(dst_bytes + 128, v9);
  vst1q_u8(dst_bytes + 16 + 128, v10);
  vst1q_u8(dst_bytes + 32 + 128, v11);
  vst1q_u8(dst_bytes + 48 + 128, v12);

  vst1q_u8(dst_bytes + 64 + 128, v13);
  vst1q_u8(dst_bytes + 80 + 128, v14);
  vst1q_u8(dst_bytes + 96 + 128, v15);
  vst1q_u8(dst_bytes + 112 + 128, v16);
  
  dst_bytes += 256; src_bytes += 256; n -= 256;
 }
 
 /* 128 bytes loop copy */
 while(n >= 128) {
  uint8x16_t v1 =	vld1q_u8(src_bytes);
  uint8x16_t v2 =	vld1q_u8(src_bytes + 16);
  uint8x16_t v3 =	vld1q_u8(src_bytes + 32);
  uint8x16_t v4 =	vld1q_u8(src_bytes + 48);

  uint8x16_t v5 =	vld1q_u8(src_bytes + 64);
  uint8x16_t v6 =	vld1q_u8(src_bytes + 80);
  uint8x16_t v7 =	vld1q_u8(src_bytes + 96);
  uint8x16_t v8 =	vld1q_u8(src_bytes + 112);

  vst1q_u8(dst_bytes, v1);
  vst1q_u8(dst_bytes + 16, v2);
  vst1q_u8(dst_bytes + 32, v3);
  vst1q_u8(dst_bytes + 48, v4);

  vst1q_u8(dst_bytes + 64, v5);
  vst1q_u8(dst_bytes + 80, v6);
  vst1q_u8(dst_bytes + 96, v7);
  vst1q_u8(dst_bytes + 112, v8);
  
  dst_bytes += 128; src_bytes += 128; n -= 128; 
 }
 /* 64 bytes loop copy */
 while(n >= 64) {
  uint8x16_t v1 =	vld1q_u8(src_bytes);
  uint8x16_t v2 =	vld1q_u8(src_bytes + 16);
  uint8x16_t v3 =	vld1q_u8(src_bytes + 32);
  uint8x16_t v4 =	vld1q_u8(src_bytes + 48);

  vst1q_u8(dst_bytes, v1);
  vst1q_u8(dst_bytes + 16, v2);
  vst1q_u8(dst_bytes + 32, v3);
  vst1q_u8(dst_bytes + 48, v4);
  
  dst_bytes += 64; src_bytes += 64; n -= 64; 
 }
 /* 16 bytes loop copy */
 while(n >= 16) {
  vst1q_u8(dst_bytes, vld1q_u8(src_bytes));
  dst_bytes += 16; src_bytes += 16; n -= 16; 
 }
 /* 1 byte loop copy */
 while(n--) *(dst_bytes++) = *(src_bytes++);
}


void oxb_memory_copy(void *oxb_restrict dst, const void *oxb_restrict src, size_t n) {
 uint8_t *dst_bytes = (uint8_t*)dst;
 const uint8_t *src_bytes = (const uint8_t*)src;
 
 /* specialized copy for different ranges */
 if(n <= 1024) {
  __neon_copy_small(dst_bytes, src_bytes, n);
 } else if(n <= (1024 * 1024)) {
  __neon_copy_mid(dst_bytes, src_bytes, n);
 } else {
  __neon_copy_large(dst_bytes, src_bytes, n);
 }
}

#else 

oxb_hot oxb_inline void __copy4(uint8_t *oxb_restrict dst, const uint8_t *oxb_restrict src) {
 dst[0] = src[0];
 dst[1] = src[1];
 dst[2] = src[2];
 dst[3] = src[3];
}

void oxb_memory_copy(void *oxb_restrict dst, const void *oxb_restrict src, size_t n) {
 uint8_t *dst_bytes = (uint8_t*)dst;
 const uint8_t *src_bytes = (const uint8_t*)src;

 while(n >= 16) {
  __copy4(dst_bytes, src_bytes);
  __copy4(dst_bytes + 4, src_bytes + 4);
  __copy4(dst_bytes + 8, src_bytes + 8);
  __copy4(dst_bytes + 12, src_bytes + 12);
  dst_bytes += 16; src_bytes += 16; n -= 16; 
 }

 while(n >= 8) {
  __copy4(dst_bytes, src_bytes);
  __copy4(dst_bytes + 4, src_bytes + 4);
  dst_bytes += 8; src_bytes += 8; n -= 8; 
 }

 while(n >= 4) {
  __copy4(dst_bytes, src_bytes);
  dst_bytes += 4; src_bytes += 4; n -= 4; 
 }

 while(n--) *(dst_bytes++) = *(src_bytes++);
}

#endif





#ifdef OXB_NEON
/**********************************************
*     arm64/arm32 optimized memory initialize
**********************************************/

void oxb_memory_set(void *oxb_restrict dst, const uint8_t src, size_t n) {
 uint8_t *dst_bytes = (uint8_t*)dst;
 const uint8x16_t src_data = vdupq_n_u8(src);

 while(n >= 128) {
  vst1q_u8(dst_bytes, src_data);
  vst1q_u8(dst_bytes + 16, src_data);
  vst1q_u8(dst_bytes + 32, src_data);
  vst1q_u8(dst_bytes + 48, src_data);

  vst1q_u8(dst_bytes + 64, src_data);
  vst1q_u8(dst_bytes + 80, src_data);
  vst1q_u8(dst_bytes + 96, src_data);
  vst1q_u8(dst_bytes + 112, src_data);
  
  dst_bytes += 128;
  n -= 128; 
 }

 while(n >= 64) {
  vst1q_u8(dst_bytes, src_data);
  vst1q_u8(dst_bytes + 16, src_data);
  vst1q_u8(dst_bytes + 32, src_data);
  vst1q_u8(dst_bytes + 48, src_data);
  
  dst_bytes += 64;
  n -= 64; 
 }

 while(n >= 16) {
  vst1q_u8(dst_bytes, src_data);
  dst_bytes += 16;
  n -= 16; 
 }
 
 while(n--)
  *(dst_bytes++) = src;
}

#else

oxb_hot oxb_inline void __splat4(uint8_t *oxb_restrict dst, const uint8_t src) {
 dst[0] = src;
 dst[1] = src;
 dst[2] = src;
 dst[3] = src;
}

void oxb_memory_set(void *oxb_restrict dst, const uint8_t src, size_t n) {
 uint8_t *dst_bytes = (uint8_t*)dst;

 while(n >= 16) {
  __splat4(dst_bytes, src);
  __splat4(dst_bytes + 4, src);
  __splat4(dst_bytes + 8, src);
  __splat4(dst_bytes + 12, src);
  dst_bytes += 16; n -= 16; 
 }

 while(n >= 8) {
  __splat4(dst_bytes, src);
  __splat4(dst_bytes + 4, src);
  dst_bytes += 8; n -= 8; 
 }

 while(n >= 4) {
  __splat4(dst_bytes, src);
  dst_bytes += 4; n -= 4; 
 }

 while(n--) *(dst_bytes++) = src;
}

#endif

