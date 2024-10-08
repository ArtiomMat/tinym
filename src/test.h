/* test.h : Testing utilities for the modules in the VM. */

#ifndef TEST_H
#define TEST_H

/* I can't be bothered to make a dynamic array, so init_test() allocates this many cells. */
#define MAX_TESTS 64

#define ADD_TEST(T) add_test(T, #T "()")
/* MSG should give what you hoped for */
#define HOPE_THAT(COND, MSG) _hope_that(COND, __FILE__, __LINE__, MSG)

void init_tests(void);
void free_tests(void);
/* Returns 1 if test added, 0 if no more space. */
int add_test(void (*t)(void), const char* name);
void run_tests(void);

/* MSG should give what you hoped for */
void hope_that(int cond, const char* msg);
/* Don't use it's made for HOPE_THAT() macro. */
void _hope_that(int cond, const char* file, int line, const char* msg);

#endif
