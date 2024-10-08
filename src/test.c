#include "test.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

static struct {
  const char* name;
  void (*t)(void);
}* tests = NULL;
static uint16_t test_i;
static uint16_t tests_n = 0;

void init_tests(void) {
  tests = malloc(sizeof (*tests) * MAX_TESTS);
}

void free_tests(void) {
  free(tests);
  tests_n = 0;
}

/* Returns 1 if test added, 0 if no more space. */
int add_test(void (*t)(void), const char* name) {
  if (tests_n == MAX_TESTS) {
    return 0;
  }

  tests[tests_n].name = name;
  tests[tests_n].t = t;

  ++tests_n;
  return 1;
}

void run_tests(void) {
  for (test_i = 0; test_i < tests_n; ++test_i) {
    printf("* running '%s'...", tests[test_i].name);
    tests[test_i].t();
    puts("SUCCESS");
  }
  puts("\nall tests succeeeded.\n");
}

void hope_that(int cond, const char* msg) {
  _hope_that(cond, NULL, 0, msg);
}

void _hope_that(int cond, const char* file, int line, const char* msg) {
  if (cond) {
    return;
  }

  puts("FAILED");

  if (file != NULL) { /* Comes from HOPE_THAT() macro and not hope_that() */
    printf("%s:%d: ", file, line);
  }

  printf("hoped that '%s'\n\nfailed test %hu/%hu.\n", msg, test_i + 1, tests_n);
  exit(1);
}
