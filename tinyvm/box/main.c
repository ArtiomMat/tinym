#include "sand/cpu8086.h"
#include "os.h"
#include "test.h"
#include "util.h"

#include <assert.h>
#include <string.h>
#include <stdio.h>


static void add_all_tests(void) {
  add_mem_tests();
  add_cpu8086_tests();
  add_os_tests();
}

int main(int _args_n, const char** _args) {
  unsigned i;
  mem_t mem;
  cpu8086_t cpu;

  args_n = _args_n;
  args = _args;

  i = find_arg("--test");
  if (i) {
    ++i;
    if (i >= args_n) {
      puts("--test must be followed by: all/os/cpu8086/mem");
      return 1;
    }
    init_tests();
    
    if (!strcmp(args[i], "all")) {
      add_all_tests();
    }
    else if (!strcmp(args[i], "cpu8086")) {
      add_cpu8086_tests();
    }
    else if (!strcmp(args[i], "os")) {
      add_os_tests();
    }
    else if (!strcmp(args[i], "mem")) {
      add_mem_tests();
    }

    run_tests();
    free_tests();
  }

  i = find_arg("--boot-sector");
  if (i) {
    
  }

  if (!init_os()) {
    puts("main(): init_os() failed.");
    return 1;
  }
  
  init_mem8086(&mem, 0);
  reset_cpu8086(&cpu, &mem);

  while (1) {
    i = cycle_cpu8086(&cpu);
    if (i) {
      printf("Error: %u.\n", i);
      return 1;
    }
  }

  /* printf("0xC0020 is ROM: %i.\n", ) */
  
  return 0;
}
