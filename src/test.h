/* test.h : Testing utilities for the modules in the VM. */

#ifndef TEST_H
#define TEST_H

/* I can't be bothered to make a dynamic array, so init_test() allocates this many cells. */
#define MAX_TESTS 64

void init_tests(void);
void free_tests(void);
/* Returns 1 if test added, 0 if no more space. */
int add_test(void (*t)(void), const char* name);
void run_tests(void);

void hope_that(int cond, const char* msg);

#define hope_not(cond, msg) hope_that(!cond, msg)

#endif
