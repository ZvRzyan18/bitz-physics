#ifndef OXB_HINT_H
#define OXB_HINT_H

#include <stddef.h>
#include <stdint.h>

#include <assert.h>

/*
 restrict keyword
*/
#ifdef __cplusplus
#define oxb_restrict __restrict
#else
#define oxb_restrict restrict
#endif


/*
 hints
*/
#if defined(__clang__) || defined(__GNUC__)

#define oxb_likely(x)   __builtin_expect(!!(x), 1)
#define oxb_unlikely(x) __builtin_expect(!!(x), 0)
#define oxb_inline __attribute__((always_inline, hot)) static inline 
#define oxb_noinline __attribute__((noinline))
#define oxb_unreachable() __builtin_unreachable()
#define oxb_const __attribute__((const))
#define oxb_pure __attribute__((pure))
#define oxb_hot __attribute__((hot))
#define oxb_cold __attribute__((cold))
#define oxb_prefetch(v) __builtin_prefetch(v, 1, 3)
#define oxb_compiler_memory_barrier() __asm__ volatile("" ::: "memory");

#elif defined(_MSC_VER)
#include <intrin.h>


#define oxb_likely(x) (x)
#define oxb_unlikely(x) (x)
#define oxb_inline __forceinline static inline 
#define oxb_noinline __declspec(noinline)
#define oxb_unreachable() __assume(0)
#define oxb_const 
#define oxb_pure
#define oxb_hot 
#define oxb_cold 
#define oxb_prefetch(v)
#define oxb_compiler_memory_barrier() _ReadWriteBarrier()

#else
/*
 fallback.
 hints are just optional
*/
#define oxb_likely(x) (x)
#define oxb_unlikely(x) (x)
#define oxb_inline static inline
#define oxb_noinline
#define oxb_unreachable() 
#define oxb_const 
#define oxb_pure
#define oxb_hot 
#define oxb_cold 
#define oxb_prefetch(v)
#define oxb_compiler_memory_barrier() 

#endif


/*
 simd support
*/
#if (defined(__ARM_NEON__) || defined(__ARM_NEON)) && defined(OXB_SIMD)
#include <arm_neon.h>
#define OXB_NEON

#if !defined(__aarch64__) && !defined(__ARM_FEATURE_FMA)
#define OXB_NEON_32
#define oxb_neon_fma vmla_f32
#define oxb_neon_fmaq vmlaq_f32
#else
#define OXB_NEON_64
#define oxb_neon_fma vfma_f32
#define oxb_neon_fmaq vfmaq_f32
#endif

#endif



#ifdef __cplusplus
extern "C" {
#endif
extern void oxb_test();
#ifdef __cplusplus
}
#endif


#ifndef OXB_PANIC 
#include <stdio.h>
#include <stdlib.h>
#define OXB_PANIC(x) do {\
   fprintf(stderr, "ERROR : %s", x); \
   abort(); \
  } while(0)
#endif




#endif


