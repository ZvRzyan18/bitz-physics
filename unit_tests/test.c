#include "oxb1tz/hint.h"
#include "vector.h"
#include "complex.h"
#include "matrix.h"
#include "container.h"
#include "pool.h"
#include "memory.h"
#include "test_math.h"


#ifdef OXB_TEST

void oxb_test() {
 printf("OxB1TZ UNIT TEST STARTING....\n\n");

 printf("<vector.h> test...\n");
 oxb_vector_test();
 printf("<vector.h> passed...\n\n");

 printf("<complex.h> test...\n");
 oxb_complex_test();
 printf("<complex.h> passed...\n\n");

 printf("<matrix.h> test...\n");
 oxb_matrix_test();
 printf("<matrix.h> passed...\n\n");

 printf("<container.h> test...\n");
 oxb_container_test();
 printf("<container.h> passed...\n\n");

 printf("<pool.h> test...\n");
 oxb_pool_test();
 printf("<pool.h> passed...\n\n");

 printf("<memory.h> test...\n");
 oxb_memory_test();
 printf("<memory.h> passed...\n\n");

 printf("<math.h> test...\n");
 oxb_math_test();
 printf("<math.h> passed...\n\n");
}

#endif

